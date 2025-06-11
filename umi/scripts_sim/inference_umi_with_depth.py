import torch
import numpy as np
import os, sys
import hydra
sys.path.append(os.getcwd())
os.environ['HF_ENDPOINT'] = 'https://hf-mirror.com'
from collections import deque
from std_msgs.msg import Float64MultiArray

from diffusion_policy.workspace.base_workspace import BaseWorkspace
from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.pytorch_util import dict_apply
from umi.common.pose_util import mat_to_pose, pose10d_to_mat
from diffusion_policy.common.pose_repr_util import convert_pose_mat_rep

from PIL import Image

import rospy
from act_dp_service.srv import get_action, get_actionResponse
from std_msgs.msg import Float64MultiArray

from scipy.spatial.transform import Rotation as R
import numpy as np
from umi.common.pose_util import pose_to_mat, mat_to_pose10d
from scipy.spatial.transform import Rotation
from collections import defaultdict

from scipy.spatial.transform import Rotation
import argparse
from depth_anything_v2.dpt import DepthAnythingV2
import cv2

def pose_to_matrix(pose, rot_type='rotvec'):
    """
    将 6D 位姿 (平移 + 旋转向量 或 四元数) 转换为 4×4 齐次变换矩阵。

    参数:
        pose (array-like): 
            - 如果 rot_type='rotvec'，则为长度为 6 的数组，前三个是平移 (x, y, z)，后三个是旋转向量。
            - 如果 rot_type='quat'，则为长度为 7 的数组，前三个是平移 (x, y, z)，后四个是四元数 (w, x, y, z)。
        rot_type (str): 旋转的表示类型，'rotvec' 或 'quat'。

    返回:
        np.ndarray: 4×4 齐次变换矩阵。
    """
    translation = np.array(pose[:3])

    if rot_type == 'rotvec':
        rot_vec = np.array(pose[3:])
        rot_mat = Rotation.from_rotvec(rot_vec).as_matrix()
    elif rot_type == 'quat':
        quat_wxyz = np.array(pose[3:])
        quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
        rot_mat = Rotation.from_quat(quat_xyzw).as_matrix()
    else:
        raise ValueError(f"Unsupported rot_type: {rot_type}")

    T = np.eye(4)
    T[:3, :3] = rot_mat
    T[:3, 3] = translation

    return T


def matrix_to_pose(T):
    """
    将 4x4 齐次变换矩阵转换为 3D 位置 (x, y, z) 和四元数 (w, x, y, z).

    参数:
        T (np.ndarray): 4x4 齐次变换矩阵.

    返回:
        tuple: (pos, quat)
            - pos (np.ndarray): 长度为 3 的平移向量 (x, y, z).
            - quat (np.ndarray): 长度为 4 的四元数 (w, x, y, z).
    """
    # 提取平移部分
    pos = T[:3, 3]

    # 提取旋转矩阵部分
    rot_mat = T[:3, :3]

    # 旋转矩阵转换为四元数 (默认返回 [x, y, z, w])
    rot = Rotation.from_matrix(rot_mat)
    quat = rot.as_quat()

    # 转换为 (w, x, y, z) 格式
    quat_wxyz = np.array([quat[3], quat[0], quat[1], quat[2]])

    return np.concatenate([pos, quat_wxyz])

def relative_to_absolute_pose(cur_pose,target_rel_pos):
    '''
        cur_pose: 当前时刻世界坐标系下的位姿, 默认的格式是 [x, y, z, w,x,y,z, gripper_width]
        target_pos: 相对目标位姿, 默认的格式是 [x, y, z, rot_x,rot_y,rot_z, gripper_width]
        
    '''
    gripper_width = target_rel_pos[-1]
    target_rot_mat = pose_to_matrix(target_rel_pos[:6],rot_type='rotvec')
    cur_rot_mat = pose_to_matrix(cur_pose[:7],rot_type='quat')

    world_rot_mat = cur_rot_mat @ target_rot_mat   # 左乘
    ee_pose = matrix_to_pose(world_rot_mat) # 转为 [x,y,z,w,x,y,z] 位置 + 四元数

    target_action = np.concatenate( [ee_pose, [gripper_width]] )
    return target_action

def quaternion_to_rotvec(quat):
    """
    将四元数 (w, x, y, z) 转换为旋转向量 (Rodrigues 旋转公式).
    
    参数:
        quat (array-like): 长度为 4 的数组，表示四元数 (w, x, y, z).
    
    返回:
        np.ndarray: 旋转向量 (3D).
    """
    # scipy 需要 [x, y, z, w] 格式，因此调整顺序
    rot = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])
    return rot.as_rotvec()

def resize_and_center_crop(image: Image.Image, target_size=224):
    # 获取原始尺寸
    width, height = image.size

    # 计算缩放比例，使短边变为 target_size
    scale = target_size / min(width, height)
    new_width = int(width * scale)
    new_height = int(height * scale)

    # 先等比例缩放
    image = image.resize((new_width, new_height), Image.Resampling.LANCZOS)

    # 计算中心裁剪区域
    left = (new_width - target_size) // 2
    top = (new_height - target_size) // 2
    right = left + target_size
    bottom = top + target_size

    # 进行中心裁剪
    image = image.crop((left, top, right, bottom))

    return image

def process_data(
        raw_data_dict,
        num_robot = 1,
        obs_pose_repr = 'relative'):
    """
        raw_data_dict: 包含图像、位姿等数据的字典
        num_robot: 机器人的数量, 单臂为1
        obs_pose_repr: 位姿的表示，默认使用相对于初始位置的位姿
    """
    data = raw_data_dict
    obs_dict = dict()
   
    rgb_key = ['camera0_rgb','depth0_rgb']
    # Step1: 处理 RGB 图像
    # move channel last to channel first
    # T,H,W,C
    # convert uint8 image to float32
    for key in rgb_key:
        obs_dict[key] = np.moveaxis(data[key], -1, -3).astype(np.float32) / 255.

    # Step2: 处理其他数据
    for key in raw_data_dict:
        if key in rgb_key: # RGB 图像原来就是 Uint 类型
            continue
        obs_dict[key] = data[key].astype(np.float32)
    
    # Step3: 产生两个机器手之间的相对位姿，没有就不作用
    # generate relative pose between two ees
    for robot_id in range(num_robot):
        # convert pose to mat
        pose_mat = pose_to_mat(np.concatenate([
            obs_dict[f'robot{robot_id}_eef_pos'],
            obs_dict[f'robot{robot_id}_eef_rot_axis_angle']
        ], axis=-1))
        for other_robot_id in range(num_robot):
            if robot_id == other_robot_id:
                continue
            if not f'robot{robot_id}_eef_pos_wrt{other_robot_id}' in self.lowdim_keys:
                continue
            other_pose_mat = pose_to_mat(np.concatenate([
                obs_dict[f'robot{other_robot_id}_eef_pos'],
                obs_dict[f'robot{other_robot_id}_eef_rot_axis_angle']
            ], axis=-1))
            rel_obs_pose_mat = convert_pose_mat_rep(
                pose_mat,
                base_pose_mat=other_pose_mat[-1],
                pose_rep='relative',
                backward=False)
            rel_obs_pose = mat_to_pose10d(rel_obs_pose_mat)
            obs_dict[f'robot{robot_id}_eef_pos_wrt{other_robot_id}'] = rel_obs_pose[:,:3]
            obs_dict[f'robot{robot_id}_eef_rot_axis_angle_wrt{other_robot_id}'] = rel_obs_pose[:,3:]
            
    # Step4: 产生相对于初始状态的末端位姿
    # generate relative pose with respect to episode start
    for robot_id in range(num_robot):
        # convert pose to mat
        pose_mat = pose_to_mat(np.concatenate([
            obs_dict[f'robot{robot_id}_eef_pos'],
            obs_dict[f'robot{robot_id}_eef_rot_axis_angle']
        ], axis=-1))
        
        # get start pose
        start_pose = obs_dict[f'robot{robot_id}_demo_start_pose'][0]
        start_pose_mat = pose_to_mat(start_pose)
        rel_obs_pose_mat = convert_pose_mat_rep(
            pose_mat,
            base_pose_mat=start_pose_mat,
            pose_rep='relative',
            backward=False)
        
        rel_obs_pose = mat_to_pose10d(rel_obs_pose_mat)
        # obs_dict[f'robot{robot_id}_eef_pos_wrt_start'] = rel_obs_pose[:,:3]
        obs_dict[f'robot{robot_id}_eef_rot_axis_angle_wrt_start'] = rel_obs_pose[:,3:]

    del_keys = list()
    for key in obs_dict:
        if key.endswith('_demo_start_pose') or key.endswith('_demo_end_pose'):
            del_keys.append(key)
    for key in del_keys:
        del obs_dict[key]

    # Step5: 产生观测数据以及 action
    for robot_id in range(num_robot):
        # convert pose to mat
        pose_mat = pose_to_mat(np.concatenate([
            obs_dict[f'robot{robot_id}_eef_pos'],
            obs_dict[f'robot{robot_id}_eef_rot_axis_angle']
        ], axis=-1))
        
        # solve relative obs
        obs_pose_mat = convert_pose_mat_rep(
            pose_mat, 
            base_pose_mat=pose_mat[-1],
            pose_rep= obs_pose_repr,
            backward=False)
       
        # convert pose to pos + rot6d representation
        obs_pose = mat_to_pose10d(obs_pose_mat)
    
        # generate data
        obs_dict[f'robot{robot_id}_eef_pos'] = obs_pose[:,:3]
        obs_dict[f'robot{robot_id}_eef_rot_axis_angle'] = obs_pose[:,3:]
        
    
    # torch_data = {
    #     'obs': dict_apply(obs_dict, torch.from_numpy),
    # }
    return dict_apply(obs_dict, lambda x: torch.from_numpy(x).unsqueeze(0).to(device))



class FrameBuffer:
    def __init__(self, buffer_size=5):
        """
        初始化帧缓存
        :param buffer_size: 缓存的帧数
        """
        self.buffer_size = buffer_size
        self.buffer = deque(maxlen=buffer_size)
        self.initialized = False  # 记录是否已初始化

    def update(self, new_frame):
        """ 更新缓冲区，如果是第一帧，则填充整个 buffer """
        if not self.initialized:
            self.buffer.extend([new_frame] * self.buffer_size)  # 用第一帧填充整个 buffer
            self.initialized = True
        else:
            self.buffer.append(new_frame)  # 追加新帧，自动丢弃最旧帧

    def get_previous_frame(self):
        """ 获取上一帧数据（如果存在） """
        return self.buffer[-1] if self.buffer else None

    def get_all_frames(self):
        """ 获取整个 buffer 的所有数据 """
        return list(self.buffer)
    

def debug():
    import debugpy

    # 启动调试器，指定调试端口
    debugpy.listen(5678)

    print("Waiting for debugger to attach...")

    # 在这里设置断点
    debugpy.wait_for_client()

def load_model(ckpt_path):
    """
    加载训练好的模型
    :param ckpt_path: 模型检查点文件路径
    :return: 加载好的模型
    """
    import dill
    payload = torch.load(open(ckpt_path, 'rb'), pickle_module=dill)
    cfg = payload['cfg']
    cls = hydra.utils.get_class(cfg._target_)
    print("model_name:", cfg.policy.obs_encoder.model_name)
    print("dataset_path:", cfg.task.dataset.dataset_path)

    n_action_steps = cfg['n_action_steps']
    num_inference_steps = cfg['policy']['num_inference_steps']

    workspace = cls(cfg)
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)
    print("workspace:", workspace)

    policy = workspace.model
    if cfg.training.use_ema:
        policy = workspace.ema_model

    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    policy.eval().to(device)
    # import ipdb; ipdb.set_trace()
    # policy.num_inference_steps = 2
    # policy.n_action_steps = policy.horizon - policy.n_obs_steps + 1
    return policy, device, policy.num_inference_steps, n_action_steps

def load_depth_anythong_v2(raw_img, encoder = 'vitl'): # 'vits', 'vitb', 'vitg'
    """
    加载训练好的模型
    :param ckpt_path: 模型检查点文件路径
    :return: depth-img
    """
    DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
    }

    model = DepthAnythingV2(**model_configs[encoder])
    model.load_state_dict(torch.load(f'./ckpt/depth_anything_v2_{encoder}.pth', map_location='cpu'))
    model = model.to(DEVICE).eval()
    depth_img = model.infer_image(raw_img)
    return depth_img


def inference(policy, obs_dict, device):
    """
    模型推理
    :param policy: 加载好的模型
    :param obs_dict: 预处理后的观测数据
    :param device: 
    :return: 推理得到的动作
    """
    with torch.no_grad():
        # import ipdb; ipdb.set_trace()
        result = policy.predict_action(obs_dict)
        action = result['action'][0]
        action = action.detach().to('cpu').numpy()
    return action

    # return torch.cat([obs_dict['robot0_eef_pos'][0][-1], obs_dict['robot0_eef_rot_axis_angle'][0][-1],obs_dict['robot0_gripper_width'][0][-1]]).unsqueeze(0).detach().to('cpu').numpy()


def postprocess_action(action):
    """
    后处理动作数据
    :param action: 推理得到的动作
    :return: 后处理后的动作
    : 返回
    """
    action_pose10d = action[:, :9]
    action_grip = action[:, 9:]
    action_pose = mat_to_pose(pose10d_to_mat(action_pose10d))
    action = np.concatenate([action_pose, action_grip], axis=-1)
    return action    


def main(raw_data):
    """
    主函数，完成模型加载、推理和结果处理
    :param ckpt_path: 模型检查点文件路径
    """
    if len(action_buffer) != 0:
        target_action = action_buffer.popleft()
    else:
        # 输入是世界坐标系下的绝对 pose + 第 0 时刻的 pose +  RGB 图像
        init_ee_pose = np.array(raw_data.sensor.init_ee_pose.data)  # wxyz 格式的四元数表示旋转
        rot_vec = quaternion_to_rotvec(init_ee_pose[3:7])
        init_ee_pose = np.concatenate([init_ee_pose[:3], rot_vec])  # shape: (6,)

        cur_ee_pose = np.array(raw_data.sensor.ee_pose.data) # wxyz 格式的四元数表示旋转
        gripper_width = raw_data.sensor.gripper_width.data
        reset = bool(raw_data.sensor.reset.data)

        # 2. 解码图像
        rgb_flat = np.frombuffer(bytes(raw_data.sensor.rgb_data.data), dtype=np.uint8)
        height = raw_data.sensor.rgb_data.layout.dim[0].size
        width = raw_data.sensor.rgb_data.layout.dim[1].size
        channels = raw_data.sensor.rgb_data.layout.dim[2].size
        rgb_img = rgb_flat.reshape((height, width, channels))

        resized_img_array = np.array(resize_and_center_crop(Image.fromarray(rgb_img), target_size=img_size))
        # 3. 解码深度图像
        if parsed_args.simple_render:
            depth_flat = np.frombuffer(bytes(raw_data.sensor.depth_data.data), dtype=np.uint8)
        else:
            depth_flat = np.frombuffer(bytes(load_depth_anythong_v2(rgb_img)), dtype=np.float32)
        depth_img = depth_flat.reshape((height, width, channels))
        resized_depth_array = np.array(resize_and_center_crop(Image.fromarray(depth_img), target_size=img_size))
        
        obs_raw_data = {
            'robot0_eef_pos': cur_ee_pose[:3].astype(np.float32),
            'robot0_eef_rot_axis_angle': quaternion_to_rotvec(cur_ee_pose[3:]).astype(np.float32), # 四元数转旋转向量
            'robot0_gripper_width': np.array([gripper_width]),
            'robot0_demo_start_pose': init_ee_pose, # 四元数转旋转向量
            'camera0_rgb': resized_img_array.astype(np.float32),
            'depth0_rgb': resized_depth_array.astype(np.float32)
        }
        
        # 更新 buffer
        obs_buffer.update(obs_raw_data)  
        # 合并之前的观测
        history_obs = obs_buffer.get_all_frames()
        all_obs = defaultdict(list)
        for d in history_obs:
            for key, value in d.items():
                all_obs[key].append(value)

        # 转换为 numpy 数组
        all_obs = {key: np.array(value) for key, value in all_obs.items()}

        # 预处理数据
        obs_dict = process_data(all_obs)

        # 两张图片只使用一张
        obs_dict['camera0_rgb'] = obs_dict['camera0_rgb'][0][-1].unsqueeze(0).unsqueeze(0)
        obs_dict['depth0_rgb'] = obs_dict['depth0_rgb'][0][-1].unsqueeze(0).unsqueeze(0)


        # 模型推理
        actions = inference(policy, obs_dict, device)


        # 后处理动作数据
        actions = postprocess_action(actions) # 旋转向量   action 代表相对于当前时刻的位姿
        
        for i in range(n_action_steps):
            # 将相对位姿转为世界坐标下的绝对位姿
            abs_pose = relative_to_absolute_pose(
                cur_pose = cur_ee_pose,
                target_rel_pos = actions[i]
            )
            if i == 0:
                target_action = abs_pose
            else:
                action_buffer.append(abs_pose)
                

    # 将结果返回给仿真环境
    # 这里需要根据实际的仿真环境接口进行修改
    print("actions:", target_action)
    print("actions shape:", target_action.shape)

    # 暂时只返回一帧的
    a = Float64MultiArray(data = target_action.flatten().tolist()) # 
    return get_actionResponse(a)   

def load_data_from_file(file_path):
    """
    从文件中加载数据
    :param file_path: 数据文件路径
    :return: numpy 数组
    """
    return np.loadtxt(file_path)

 
args = argparse.ArgumentParser()
args.add_argument("--ckpt_path",type=str,required=True,help="The path to your saved ckpt file")
args.add_argument("--n_obs_steps",type=int,default=2)
args.add_argument("--n_action_steps",type=int,default=4,help="How many of predicted actions will be executed before next inference")    
args.add_argument("--img_size",type=int,default=224,help="the image size during inference")
args.add_argument("--simple_render",type=bool,default=True,help="the render method during inference")
parsed_args = args.parse_args()
# debug()

ckpt_path = parsed_args.ckpt_path

n_obs_steps = parsed_args.n_obs_steps
n_action_steps = parsed_args .n_action_steps

policy, device, num_inference_steps, n_action_steps = load_model(ckpt_path)


obs_buffer = FrameBuffer(buffer_size=n_obs_steps) # 存储历史观测数据
action_buffer =  deque(maxlen=n_action_steps) # 存储输出的动作，因为一次可能输出多个动作


img_size = 224 

rospy.init_node('policy_network_node')  
s = rospy.Service('sensor_processing', get_action, main)
rospy.spin()   # 就像订阅者示例一样，rospy.spin()使代码不会退出，直到服务关闭；
