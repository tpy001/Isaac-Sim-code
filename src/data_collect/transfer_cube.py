from src.task.base import BaseTask
import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
import os
from src.utils import quaternion_to_rotvec
from PIL import Image
from .base import BaseDataCollect


class TransferCubeDataCollect(BaseDataCollect):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        
        # 设定随机范围
        x_range = (0.1, 0.2)  # X 轴范围
        y_range = (-0.2,0.2)
        # y_range = (-0.15, 0.15)  # Y 轴范围

        # 随机 yaw 角度（单位：弧度）
        # yaw = np.random.uniform(-np.pi/4, np.pi/4)

        # 欧拉角（roll, pitch, yaw） -> 四元数 [w, x, y, z]
        quat = euler_angles_to_quat([0, 0, 0])  # 只绕 Z 轴旋转

        pos_red = np.array([
            np.random.uniform(*x_range),
            np.random.uniform(*y_range),
            0.12
        ])


        self.cube_red_cfg = {
            "name": "cube",
            "position": pos_red,
            "orientation": quat,
            "prim_path": "/World/Cube",
            # "scale": np.array([0.04, 0.04, 0.04]),
            "scale": np.array([0.05, 0.05, 0.05]),
            "size": 1.0,
            "color": np.array([1, 0, 0]),
        }

        init_pos = {}
        init_orientation = {}
        for arm_side in ["left","right"]:
            states = self.robot.get_ee_pose(arm_side=arm_side)
            init_pos[arm_side] = states[:3]
            init_orientation[arm_side] = states[3:7] 


        meet_xyz = np.array([0.3, 0, 0.7])
        init_left_quat = np.array([-0.707,0,0.707,0])
        meet_left_quat = np.array([0.5, 0.5, -0.5, -0.5])
        meet_right_quat = np.array([0,0.707,0,0.707])

        gripper_pick_quat = np.array([0,1,0,0])


        left_trajectory = [
            {"t": 0, "xyz": init_pos["left"], "quat": init_orientation["left"], "gripper": 1}, # sleep
            {"t": 100, "xyz": meet_xyz + np.array([+0.25, 0, 0]), "quat": meet_left_quat, "gripper": 1}, # approach meet position
            {"t": 130, "xyz": meet_xyz + np.array([0.15, 0, 0]), "quat": meet_left_quat, "gripper": 1}, # move to meet position
            {"t": 155, "xyz": meet_xyz + np.array([0.15, 0, 0]), "quat": meet_left_quat, "gripper": 0}, # close gripper
            {"t": 180, "xyz": meet_xyz + np.array([+0.25, 0, 0]), "quat": init_left_quat, "gripper": 0}, # move left
        ]

        right_trajectory = [
            {"t": 0, "xyz": init_pos["right"], "quat": init_orientation["right"], "gripper": 1}, # sleep
            {"t": 45, "xyz": pos_red + np.array([0, 0, 0.08]), "quat": gripper_pick_quat, "gripper": 1}, # approach the cube
            {"t": 65, "xyz": pos_red + np.array([0, 0, -0.02]), "quat": gripper_pick_quat, "gripper": 1}, # go down
            {"t": 75, "xyz": pos_red + np.array([0, 0, -0.02]), "quat": gripper_pick_quat, "gripper": 0}, # close gripper
            {"t": 100, "xyz": meet_xyz + np.array([-0.2, 0, 0]), "quat": meet_right_quat, "gripper": 0}, # approach meet position
            {"t": 130, "xyz": meet_xyz, "quat": meet_right_quat, "gripper": 0}, # move to meet position
            {"t": 155, "xyz": meet_xyz, "quat": meet_right_quat, "gripper": 1}, # open gripper
            {"t": 180, "xyz": meet_xyz + np.array([-0.1, 0, 0]), "quat": meet_right_quat, "gripper": 1}, # move to right
        ]
        

        self.scenary.add_cube(self.cube_red_cfg)
        self.robot.set_trajectory(left_trajectory,"left")
        self.robot.set_trajectory(right_trajectory,"right")
        self.reset()



    def save_data(
            self,
            output_dir,
            data_list,
            cube_pose1 = None,
            cube_pose2 = None
        ):
        
        cube_pose1=np.concatenate([self.cube_red_cfg['position'], self.cube_red_cfg['orientation']])

        # 创建目录结构
        meta_dir = os.path.join(output_dir, "meta")
        data_dir = os.path.join(output_dir, "data")
        os.makedirs(meta_dir, exist_ok=True)
        os.makedirs(data_dir, exist_ok=True)
        
        ee_pose_list = []
        joint_pos_list = []
        gripper_list = []
        action_ee_pose_list = []
        action_joint_pose_list = []
        action_gripper_list = []

        dof_num = len(data_list[0]['robot']['states']['joint_pos'])
        for t in range(len(data_list)):
            ee_pose = data_list[t]['robot']['states']['ee_pose']
            joint_pos = data_list[t]['robot']['states']['joint_pos']
            action_ee_pose = data_list[t]['action']['ee_pose']
            action_joint_pose = data_list[t]['action']['joint_pos']

            ee_pose_list.append(ee_pose)
            joint_pos_list.append(joint_pos)
            gripper_list.append(np.array( [ joint_pos[dof_num//2-1], joint_pos[-1]  ])) 

            action_ee_pose_list.append(action_ee_pose)
            action_joint_pose_list.append(action_joint_pose)
            action_gripper_list.append(  [ action_joint_pose[dof_num//2-1], action_joint_pose[-1]  ])

            
        #存储机器人初始关节pos数据
        np.savetxt(os.path.join(data_dir, "ee_pose.txt"), ee_pose_list, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "joint_pos.txt"), joint_pos_list, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "gripper_width.txt"), gripper_list, fmt='%.6f')


        # 存储动作
        np.savetxt(os.path.join(data_dir, "action_ee_pose.txt"), action_ee_pose_list, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "action_joint_pos.txt"), action_joint_pose_list, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "action_gripper_width.txt"), action_gripper_list, fmt='%.6f')


        if cube_pose1 is not None:
            np.savetxt(os.path.join(data_dir, "cube_red"), cube_pose1.reshape(1,-1), fmt='%.6f')
        if cube_pose2 is not None:
            np.savetxt(os.path.join(data_dir, "cube_blue"), cube_pose2.reshape(1,-1), fmt='%.6f')
        
        print(f"Data saved successfully in {output_dir}")
