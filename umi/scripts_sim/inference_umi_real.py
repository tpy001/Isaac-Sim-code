import torch
import numpy as np
import os, sys
import hydra
sys.path.append(os.getcwd())
os.environ['HF_ENDPOINT'] = 'https://hf-mirror.com'
from collections import deque

from diffusion_policy.workspace.base_workspace import BaseWorkspace
from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.pytorch_util import dict_apply
from umi.common.pose_util import mat_to_pose, pose10d_to_mat,mat_to_pos_rot
from diffusion_policy.common.pose_repr_util import convert_pose_mat_rep

from PIL import Image


from scipy.spatial.transform import Rotation as R
import numpy as np
from umi.common.pose_util import pose_to_mat, mat_to_pose10d
from scipy.spatial.transform import Rotation
from collections import defaultdict

from scipy.spatial.transform import Rotation
import argparse

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


def matrix_to_pose(T,rot_type='quat'):
    """
    将 4x4 齐次变换矩阵转换为 3D 位置 (x, y, z) 和四元数 (w, x, y, z) 或 旋转向量
    """
    # 提取平移部分
    pos = T[:3, 3]

    # 提取旋转矩阵部分
    rot_mat = T[:3, :3]

    # 旋转矩阵转换为四元数 (默认返回 [x, y, z, w])
    rot = Rotation.from_matrix(rot_mat)
    if rot_type == 'quat':
        quat = rot.as_quat()

        # 转换为 (w, x, y, z) 格式
        quat_wxyz = np.array([quat[3], quat[0], quat[1], quat[2]])

        return np.concatenate([pos, quat_wxyz])
    elif rot_type == 'rotvec':
        rot_vec =  rot.as_rotvec()
        return np.concatenate([pos, rot_vec])

def relative_to_absolute_pose(cur_pose,target_rel_pos):
    '''
        cur_pose: 当前时刻世界坐标系下的位姿, 位置+旋转向量
        target_pos: 相对目标位姿, 默认的格式是 [x, y, z, rot_x,rot_y,rot_z, gripper_width]
        输出: 
            target_action: 绝对位置 + 四元数  wxyz格式
    '''
    gripper_width = target_rel_pos[-1]
    target_rot_mat = pose_to_matrix(target_rel_pos[:6],rot_type='rotvec')
    cur_rot_mat = pose_to_matrix(cur_pose[:6],rot_type='rotvec')

    world_rot_mat = cur_rot_mat @ target_rot_mat   # 左乘
    ee_pose = matrix_to_pose(world_rot_mat,rot_type="rotvec") # 转为 [x,y,z,w,x,y,z] 位置 + 四元数

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
   
    rgb_key = 'camera0_rgb'
    # Step1: 处理 RGB 图像
    # move channel last to channel first
    # T,H,W,C
    # convert uint8 image to float32
    obs_dict[rgb_key] = np.moveaxis(data[rgb_key], -1, -3).astype(np.float32) / 255.

    # Step2: 处理其他数据
    for key in raw_data_dict:
        if key == rgb_key: # RGB 图像原来就是 Uint 类型
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


def main(raw_data,my_chain):
    """
    主函数，完成模型加载、推理和结果处理
    :param ckpt_path: 模型检查点文件路径
    """
    if len(action_buffer) != 0:
        output_action = action_buffer.popleft()
    else:
        # 1: 计算t0时刻、当前时刻末端位姿
        initial_joint_angles = np.deg2rad(raw_data['initial_joint_angles'])
        current_joint_angles= np.deg2rad(raw_data['current_joint_angles'])

        initial_mat = my_chain.forward_kinematics(np.concatenate( [[0],initial_joint_angles])) # 计算 t0时刻末端位姿（齐次矩阵）
        current_mat = my_chain.forward_kinematics(np.concatenate( [[0], current_joint_angles]) ) # 当前时刻末端位姿（齐次矩阵）
        
        init_position, init_orientation = mat_to_pos_rot(initial_mat) # 末端位姿转为 position + 旋转向量
        init_orientation = init_orientation.as_rotvec()

        cur_position,cur_orientation = mat_to_pos_rot(current_mat)  # 末端位姿转为 position + 旋转向量
        cur_orientation = cur_orientation.as_rotvec()

        gripper_width = raw_data['gripper_width']


        # 2. 解码图像
        img = raw_data['bgr_data']

        # 3. 打包所有数据
        obs_raw_data = {
            'robot0_eef_pos': cur_position.astype(np.float32),
            'robot0_eef_rot_axis_angle': cur_orientation.astype(np.float32), 
            'robot0_gripper_width': np.array([gripper_width]),
            'robot0_demo_start_pose': np.concatenate([init_position,init_orientation]), 
            'camera0_rgb': img.astype(np.float32)
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

        # 模型推理
        actions = inference(policy, obs_dict, device)


        # 后处理动作数据
        actions = postprocess_action(actions) # 位置 + 旋转向量； action 代表相对于当前时刻的位姿
        
        for i in range(n_action_steps):
            # 将相对位姿转为世界坐标下的绝对位姿
            abs_pose = relative_to_absolute_pose(  
                cur_pose = np.concatenate([cur_position,cur_orientation]), # 位置 + 旋转向量
                target_rel_pos = actions[i]
            )
            # 将末端位姿转为关节角度
            target_joint_pos = my_chain.inverse_kinematics(
                abs_pose[:3],
                R.from_rotvec(abs_pose[3:6]).as_matrix(),
                orientation_mode='all',
                initial_position=np.concatenate( [[0], current_joint_angles]),
                regularization_parameter = 0.05 # 正则化参数
            )[1:]
            
            target_action = np.rad2deg(target_joint_pos)
            target_action = np.concatenate([target_action,[abs_pose[-1]]])

            if i == 0:
                output_action = target_action
            else:
                action_buffer.append(target_action)
                

    # 将结果返回给仿真环境
    # 这里需要根据实际的仿真环境接口进行修改
    print("actions:", output_action)
    print("actions shape:", output_action.shape)

    # 暂时只返回一帧的
    return output_action

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
args.add_argument("--urdf_path",type=str,help="the path to urdf file")

parsed_args = args.parse_args()
debug()

ckpt_path = parsed_args.ckpt_path

n_obs_steps = parsed_args.n_obs_steps
n_action_steps = parsed_args .n_action_steps
urdf_path = parsed_args.urdf_path
img_size = parsed_args.img_size 

policy, device, num_inference_steps, n_action_steps = load_model(ckpt_path) # 加载预训练的权重


obs_buffer = FrameBuffer(buffer_size=n_obs_steps) # 存储历史观测数据
action_buffer =  deque(maxlen=n_action_steps) # 存储输出的动作，因为一次可能输出多个动作


# 创建 ik 控制器
import ikpy.chain
my_chain = ikpy.chain.Chain.from_urdf_file(urdf_path, base_elements=['base_link'])


raw_data = {
    "initial_joint_angles": np.array([0.0, -36.0, 0.0, -126.5, 0.0, 90.0, 0.0]),  # 数据采集时，记录的 t0 时刻的关节角度
    "current_joint_angles": np.array([0.28, -39.34, -0.164, -127.56, 0.06, 87.09, -0.10]),  # 当前时刻的关节角度，随机生成7个在-180到180之间的值
    # "bgr_data": np.random.randint(low=0, high=256, size=(224, 224, 3)),  # bgr 格式的图像，随机生成224x224x3的数组
    "bgr_data": np.array(Image.open("image.png")),
    "gripper_width": 0.8  # 夹爪宽度，已经归一化到 [0,1]
}
# 输出是角度
action = main(raw_data,my_chain) # 获取 action， action 为 7DOF 关节角度 + 夹爪宽度

# joint_angle = action[:7]
# gripper_width = action[-1]
