import torch
import numpy as np
import os
import pickle
import argparse
import matplotlib.pyplot as plt
from copy import deepcopy
from tqdm import tqdm
from einops import rearrange

from utils import compute_dict_mean, set_seed, detach_dict # helper functions
from policy import ACTPolicy, CNNMLPPolicy
from utils import get_norm_stats
from torch.utils.data import DataLoader
import h5py


import IPython
e = IPython.embed

class EpisodicDataset(torch.utils.data.Dataset):
    def __init__(self, episode_id, dataset_dir, camera_names, norm_stats):
        super(EpisodicDataset).__init__()
        self.episode_id = episode_id
        self.episode_len = None
        self.dataset_dir = dataset_dir
        self.camera_names = camera_names
        self.norm_stats = norm_stats
        self.is_sim = None
        self.__getitem__(0)

    def __len__(self):
        return self.episode_len

    def __getitem__(self, index):
        episode_id = self.episode_id
        dataset_path = os.path.join(self.dataset_dir, f'episode_{episode_id}.hdf5')
        with h5py.File(dataset_path, 'r') as root:
            original_action_shape = root['/action'].shape
            episode_len = original_action_shape[0]
            if self.episode_len is None:
                self.episode_len = episode_len
            start_ts = index
            # get observation at start_ts only
            qpos = root['/observations/qpos'][start_ts]
            # qvel = root['/observations/qvel'][start_ts]
            image_dict = dict()
            for cam_name in self.camera_names:
                image_dict[cam_name] = root[f'/observations/images/{cam_name}'][start_ts]
            # get all actions after and including start_ts
            action = root['/action'][start_ts:]
            action_len = episode_len - start_ts

        padded_action = np.zeros(original_action_shape, dtype=np.float32)
        padded_action[:action_len] = action
        is_pad = np.zeros(episode_len)
        is_pad[action_len:] = 1

        # new axis for different cameras
        all_cam_images = []
        for cam_name in self.camera_names:
            all_cam_images.append(image_dict[cam_name])
        all_cam_images = np.stack(all_cam_images, axis=0)

        # construct observations
        image_data = torch.from_numpy(all_cam_images)
        qpos_data = torch.tensor(qpos, dtype=torch.float32)
        action_data = torch.tensor(padded_action, dtype=torch.float32)
        is_pad = torch.from_numpy(is_pad).bool()

        # channel last
        image_data = torch.einsum('k h w c -> k c h w', image_data)

        # normalize image and change dtype to float
        image_data = image_data / 255.0
        action_data = (action_data - self.norm_stats["action_mean"]) / self.norm_stats["action_std"]
        qpos_data = (qpos_data - self.norm_stats["qpos_mean"]) / self.norm_stats["qpos_std"]

        return image_data, qpos_data, action_data, is_pad
    
def load_data(dataset_dir, num_episodes, camera_names, batch_size_train, batch_size_val,episode_id):
    print(f'\nData from: {dataset_dir}\n')
    # obtain train test split

    # obtain normalization stats for qpos and action
    norm_stats = get_norm_stats(dataset_dir, num_episodes)

    # construct dataset and dataloader
    val_dataset = EpisodicDataset(episode_id, dataset_dir, camera_names, norm_stats)
    val_dataloader = DataLoader(val_dataset, batch_size=batch_size_val, shuffle=True, pin_memory=True, num_workers=4, prefetch_factor=1)

    return val_dataloader

def main(args):
    set_seed(1)
    # command line parameters
    is_eval = args['eval']
    is_open_loop = args['open_loop']
    ckpt_dir = args['ckpt_dir']
    policy_class = args['policy_class']
    onscreen_render = args['onscreen_render']
    task_name = args['task_name']
    batch_size_train = args['batch_size']
    # batch_size_val = args['batch_size']
    batch_size_val = 1
    num_epochs = args['num_epochs']
    episode_id = args['episode_id']


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

    val_dataloader = load_data(dataset_dir, num_episodes, camera_names, batch_size_train, batch_size_val,episode_id)

    if is_eval:
        ckpt_names = [f'policy_best.ckpt']
        for ckpt_name in ckpt_names:
            eval_open_loop(config, ckpt_name,val_dataloader=val_dataloader,episode_id=episode_id)
            exit()



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


def get_image(ts, camera_names):
    curr_images = []
    for cam_name in camera_names:
        curr_image = rearrange(ts.observation['images'][cam_name], 'h w c -> c h w')
        curr_images.append(curr_image)
    curr_image = np.stack(curr_images, axis=0)
    curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)
    return curr_image

def eval_open_loop(config,ckpt_name,val_dataloader,episode_id):
    set_seed(1000)
    ckpt_dir = config['ckpt_dir']
    state_dim = config['state_dim']
    real_robot = config['real_robot']
    policy_class = config['policy_class']
    onscreen_render = config['onscreen_render']
    policy_config = config['policy_config']
    camera_names = config['camera_names']
    max_timesteps = config['episode_len']
    task_name = config['task_name']
    temporal_agg = config['temporal_agg']
    onscreen_cam = 'angle'

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

    # load the val dataset
    with torch.inference_mode():
        policy.eval()
        episode_losses = []
        for batch_idx, data in tqdm(enumerate(val_dataloader), total=len(val_dataloader), desc="Evaluating"):
            forward_dict = forward_pass(data, policy)
            l1_loss = forward_dict['l1']
            episode_losses.append(l1_loss.item()) 
            
    # 计算平均 loss 并打印
    avg_loss = sum(episode_losses) / len(episode_losses)
    print(f"Average L1 Loss on validation set: {avg_loss:.4f}")

    save_dir = "./open_loop_loss"
    save_path = os.path.join(save_dir, f"{episode_id}_loss={avg_loss:.4f}.png")
    # 绘制 loss 曲线
    plt.figure(figsize=(8, 5))
    plt.plot(episode_losses, label='L1 Loss per Batch')
    plt.xlabel("Time Step")
    plt.ylabel("L1 Loss")
    plt.title("Validation L1 Loss over Time")
    plt.ylim(0, 1)  # 设置纵轴范围为0到1
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_path)
    # plt.show()




def forward_pass(data, policy):
    image_data, qpos_data, action_data, is_pad = data
    image_data, qpos_data, action_data, is_pad = image_data.cuda(), qpos_data.cuda(), action_data.cuda(), is_pad.cuda()
    return policy(qpos_data, image_data, action_data, is_pad) # TODO remove None


def debug():
    import debugpy

    # 监听任意 IP 的 5678 端口
    debugpy.listen(("0.0.0.0", 5678))  # 或者 ("localhost", 5678)

    print("Waiting for debugger attach...")
    debugpy.wait_for_client()  # 程序会在此阻塞，直到 VSCode 连接上

if __name__ == '__main__':
    from detr.main import get_args_parser
    parser = get_args_parser()
    
    # debug()
    main(vars(parser.parse_args()))
