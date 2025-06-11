import torch
import numpy as np
import os
import pickle
import argparse
import matplotlib.pyplot as plt
from copy import deepcopy
from tqdm import tqdm
from einops import rearrange


from PIL import Image
from constants import DT
from constants import PUPPET_GRIPPER_JOINT_OPEN
from utils import load_data # data functions
from utils import sample_box_pose, sample_insertion_pose # robot functions
from utils import compute_dict_mean, set_seed, detach_dict # helper functions
from policy import ACTPolicy, CNNMLPPolicy
from visualize_episodes import save_videos

import rospy
from act_dp_service.srv import get_action, get_actionResponse
from std_msgs.msg import Float64MultiArray

exp_weight = 0.1
is_degree = False



def make_policy(policy_class, policy_config):
    if policy_class == 'ACT':
        policy = ACTPolicy(policy_config)
    elif policy_class == 'CNNMLP':
        policy = CNNMLPPolicy(policy_config)
    else:
        raise NotImplementedError
    return policy


def make_optimizer(policy_class, policy):
    if policy_class == 'ACT':
        optimizer = policy.configure_optimizers()
    elif policy_class == 'CNNMLP':
        optimizer = policy.configure_optimizers()
    else:
        raise NotImplementedError
    return optimizer


def get_image(image):
     
    image = rearrange(image, 'h w c -> c h w')  # 形状变为 (c, h, w)

    # 转换为 PyTorch 张量，归一化，并移动到 GPU 上
    image = torch.from_numpy(image / 255.0).float().cuda().unsqueeze(0)  # 添加批量维度

    return image

def inference(raw_data):
    global t
    global all_actions
    
    cur_joint_pos = np.array(raw_data.sensor.joint_pos.data).reshape(-1)
    reset = bool(raw_data.sensor.reset.data)

    # 2. 解码图像
    rgb_flat = np.frombuffer(bytes(raw_data.sensor.rgb_data.data), dtype=np.uint8)
    height = raw_data.sensor.rgb_data.layout.dim[0].size
    width = raw_data.sensor.rgb_data.layout.dim[1].size
    channels = raw_data.sensor.rgb_data.layout.dim[2].size
    rgb_img = rgb_flat.reshape((height, width, channels))

    policy.eval()
    # 3. 处理并返回
    with torch.inference_mode():
        # TODO： cur_image和cur_qpos从ROS服务器接收
        obs_image = rgb_img
        obs_qpos = cur_joint_pos

        qpos_numpy = np.array(obs_qpos)

        if is_degree:
            qpos_numpy = qpos_numpy / np.pi *180
            qpos_numpy[-1] =  obs_qpos[-1] / 0.04

        qpos = pre_process(qpos_numpy)
        qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
        qpos_history[:, t] = qpos
        curr_image = get_image(obs_image)
        curr_image = curr_image.unsqueeze(1)
        # print(curr_image.shape)
        # print(qpos.shape, curr_image.shape)
        ### query policy
        if config['policy_class'] == "ACT":
            if t % query_frequency == 0:
                all_actions = policy(qpos, curr_image)
            if temporal_agg:
                all_time_actions[[t], t:t+num_queries] = all_actions[:,:num_queries]
                actions_for_curr_step = all_time_actions[:, t]
                actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                actions_for_curr_step = actions_for_curr_step[actions_populated]
                exp_weights = np.exp(-exp_weight * np.arange(len(actions_for_curr_step)))
                exp_weights = exp_weights / exp_weights.sum()
                exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
                raw_action = (actions_for_curr_step * exp_weights).sum(dim=0, keepdim=True)
            else:
                raw_action = all_actions[:, t % query_frequency]
        elif config['policy_class'] == "CNNMLP":
            raw_action = policy(qpos, curr_image)
        else:
            raise NotImplementedError

        ### post-process actions
        raw_action = raw_action.squeeze(0).cpu().numpy()
        action = post_process(raw_action)
        target_qpos = action

        #TODO 仿真环境中机械臂采取target_qos
        t = t + 1

        ### for visualization
        qpos_list.append(qpos_numpy)
        target_qpos_list.append(target_qpos)

        if is_degree:
            temp = target_qpos[-1]
            target_qpos = target_qpos / 180 * np.pi
            target_qpos[-1] =  temp * 0.04

        print(target_qpos)
        a = Float64MultiArray(data = target_qpos.tolist()) # 
        return get_actionResponse(a)   

   

def debug():
    import debugpy

    # 监听任意 IP 的 5678 端口
    debugpy.listen(("0.0.0.0", 5678))  # 或者 ("localhost", 5678)

    print("Waiting for debugger attach...")
    debugpy.wait_for_client()  # 程序会在此阻塞，直到 VSCode 连接上

# debug()
parser = argparse.ArgumentParser()
parser.add_argument('--eval', action='store_true')
parser.add_argument('--onscreen_render', action='store_true')
parser.add_argument('--ckpt_dir', action='store', type=str, help='ckpt_dir', required=True)
parser.add_argument('--policy_class', action='store', type=str, help='policy_class, capitalize', required=True)
parser.add_argument('--task_name', action='store', type=str, help='task_name', required=True)
parser.add_argument('--batch_size', action='store', type=int, help='batch_size', required=True)
parser.add_argument('--seed', action='store', type=int, help='seed', required=True)
parser.add_argument('--num_epochs', action='store', type=int, help='num_epochs', required=True)
parser.add_argument('--lr', action='store', type=float, help='lr', required=True)

# for ACT
parser.add_argument('--kl_weight', action='store', type=int, help='KL Weight', required=False)
parser.add_argument('--chunk_size', action='store', type=int, help='chunk_size', required=False)
parser.add_argument('--hidden_dim', action='store', type=int, help='hidden_dim', required=False)
parser.add_argument('--dim_feedforward', action='store', type=int, help='dim_feedforward', required=False)
parser.add_argument('--temporal_agg', action='store_true')
parser.add_argument('--ckpt_path', type=str,default="policy_best.ckpt")

# eval_bc() 

args = vars(parser.parse_args())
# command line parameters
is_eval = args['eval']
ckpt_dir = args['ckpt_dir']
policy_class = args['policy_class']
onscreen_render = args['onscreen_render']
task_name = args['task_name']
batch_size_train = args['batch_size']
batch_size_val = args['batch_size']
num_epochs = args['num_epochs']

# get task parameters
is_sim = task_name[:4] == 'sim_'
if is_sim:
    from constants import SIM_TASK_CONFIGS
    task_config = SIM_TASK_CONFIGS[task_name]
else:
    from aloha_scripts.constants import TASK_CONFIGS # TODO
    task_config = TASK_CONFIGS[task_name]
dataset_dir = task_config['dataset_dir']
num_episodes = task_config['num_episodes']
episode_len = task_config['episode_len']
camera_names = task_config['camera_names']

# fixed parameters
state_dim = 8
lr_backbone = 1e-5
backbone = 'resnet18'
if policy_class == 'ACT':
    enc_layers = 4
    dec_layers = 7
    nheads = 8
    policy_config = {'lr': args['lr'],
                        'num_queries': args['chunk_size'],
                        'kl_weight': args['kl_weight'],
                        'hidden_dim': args['hidden_dim'],
                        'dim_feedforward': args['dim_feedforward'],
                        'lr_backbone': lr_backbone,
                        'backbone': backbone,
                        'enc_layers': enc_layers,
                        'dec_layers': dec_layers,
                        'nheads': nheads,
                        'camera_names': camera_names,
                        }
elif policy_class == 'CNNMLP':
    policy_config = {'lr': args['lr'], 'lr_backbone': lr_backbone, 'backbone' : backbone, 'num_queries': 1,
                        'camera_names': camera_names,}
else:
    raise NotImplementedError

config = {
    'num_epochs': num_epochs,
    'ckpt_dir': ckpt_dir,
    'episode_len': episode_len,
    'state_dim': state_dim,
    'lr': args['lr'],
    'policy_class': policy_class,
    'onscreen_render': onscreen_render,
    'policy_config': policy_config,
    'task_name': task_name,
    'seed': args['seed'],
    'temporal_agg': args['temporal_agg'],
    'camera_names': camera_names,
    'real_robot': not is_sim
}

ckpt_name = args['ckpt_path']

ckpt_dir = config['ckpt_dir']
state_dim = config['state_dim']
policy_class = config['policy_class']
policy_config = config['policy_config']
max_timesteps = config['episode_len']
temporal_agg = config['temporal_agg']

# load policy and stats
ckpt_path = os.path.join(ckpt_dir, ckpt_name)
policy = make_policy(policy_class, policy_config)
loading_status = policy.load_state_dict(torch.load(ckpt_path))
print(loading_status)
policy.cuda()
policy.eval()
print(f'Loaded: {ckpt_path}')
stats_path = os.path.join(ckpt_dir, f'dataset_stats.pkl')
with open(stats_path, 'rb') as f:
    stats = pickle.load(f)

pre_process = lambda s_qpos: (s_qpos - stats['qpos_mean']) / stats['qpos_std']
post_process = lambda a: a * stats['action_std'] + stats['action_mean']

# load environment

query_frequency = policy_config['num_queries'] # num_queries == chunk_size
if temporal_agg: # temporal aggregation
    query_frequency = 1
    num_queries = policy_config['num_queries']

max_timesteps = int(2*max_timesteps * 1) # may increase for real-world tasks


if temporal_agg:
    all_time_actions = torch.zeros([max_timesteps, max_timesteps+num_queries, state_dim]).cuda()

qpos_history = torch.zeros((1, max_timesteps, state_dim)).cuda()
image_list = [] # for visualization
qpos_list = []
target_qpos_list = []
all_actions = []
t = 0
rospy.init_node('policy_network_node')  
s = rospy.Service('sensor_processing', get_action, inference)
rospy.spin()   # 就像订阅者示例一样，rospy.spin()使代码不会退出，直到服务关闭；