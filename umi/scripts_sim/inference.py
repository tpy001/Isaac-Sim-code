import torch
import numpy as np
import os, sys
import hydra
sys.path.append(os.getcwd())
os.environ['HF_ENDPOINT'] = 'https://hf-mirror.com'

from diffusion_policy.workspace.base_workspace import BaseWorkspace
from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.pytorch_util import dict_apply
from umi.common.pose_util import mat_to_pose, pose10d_to_mat


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
    workspace = cls(cfg)
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)
    print("workspace:", workspace)

    policy = workspace.model
    if cfg.training.use_ema:
        policy = workspace.ema_model

    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    policy.eval().to(device)
    # import ipdb; ipdb.set_trace()
    policy.num_inference_steps = 2
    # policy.n_action_steps = policy.horizon - policy.n_obs_steps + 1
    return policy, device, policy.num_inference_steps


def preprocess_obs(obs_dict_np, device):
    """
    预处理观测数据
    :param obs_dict_np: 包含观测数据的字典，numpy数组格式
    img (16, 3, 224, 224) 16是infer step
    robot_eef_pose (16, 6)
    gripper_width (16, 1)
    :param device: 
    :return: 预处理后的观测数据，torch.Tensor格式
    """
    # import ipdb; ipdb.set_trace()
    obs_dict = dict_apply(obs_dict_np, lambda x: torch.from_numpy(x).unsqueeze(0).to(device))
    return obs_dict


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
        action = result['action'][0].detach().to('cpu').numpy()
    return action


def postprocess_action(action):
    """
    后处理动作数据
    :param action: 推理得到的动作
    :return: 后处理后的动作
    """
    action_pose10d = action[:, :9]
    action_grip = action[:, 9:]
    action_pose = mat_to_pose(pose10d_to_mat(action_pose10d))
    action = np.concatenate([action_pose, action_grip], axis=-1)
    return action

def get_oba_from_sim():
    pass


def main(ckpt_path):
    """
    主函数，完成模型加载、推理和结果处理
    :param ckpt_path: 模型检查点文件路径
    """


    import debugpy

    # 启动调试器，指定调试端口
    debugpy.listen(5678)

    print("Waiting for debugger to attach...")

    # 在这里设置断点
    debugpy.wait_for_client()


    # 加载模型
    policy, device, n_obs_steps = load_model(ckpt_path)

    # 模拟从仿真环境获取观测数据
    # 这里需要根据实际的仿真环境接口进行修改
    get_oba_from_sim()
    obs_dict_np = {
    # 'action': np.random.rand(16, 10).astype(np.float32),
    'robot0_eef_pos': np.random.rand(n_obs_steps, 3).astype(np.float32),
    'robot0_eef_rot_axis_angle': np.random.rand(n_obs_steps, 6).astype(np.float32),
    'robot0_gripper_width': np.random.rand(n_obs_steps, 1).astype(np.float32),
    'robot0_eef_rot_axis_angle_wrt_start': np.random.rand(n_obs_steps, 6).astype(np.float32),
    'camera0_rgb': np.random.rand(n_obs_steps, 3, 224, 224).astype(np.float32)
}

    # 预处理观测数据
    obs_dict = preprocess_obs(obs_dict_np, device)

    # 模型推理
    action = inference(policy, obs_dict, device)

    # 后处理动作数据
    action = postprocess_action(action)

    # 将结果返回给仿真环境
    # 这里需要根据实际的仿真环境接口进行修改
    print("actions:", action)
    print("actions shape:", action.shape)
    return action


if __name__ == "__main__":
    # ckpt_path = "data/outputs/2025.03.14/12.55.42_train_diffusion_unet_timm_umi/checkpoints/epoch=0110-train_loss=0.019.ckpt"  # 请替换为实际的模型检查点文件路径
    ckpt_path = "scripts_sim/model/latest.ckpt"
    main(ckpt_path)

    