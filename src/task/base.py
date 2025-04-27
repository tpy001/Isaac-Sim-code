import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
import os
from src.utils import quaternion_to_rotvec
from PIL import Image

class BaseTask:
    def __init__(self,
                 scenary,
                 robot,
                 controller,
                 sensors,
                 dataset=None,
                 replay_horizon = 0,
                 replay_trajectory_index = 0):
        self.scenary = scenary
        self.robot = robot
        self.controller = controller
        self.sensors = sensors

        self.reset_needed = False
        self.dataset = dataset
        self.replay_horizon = replay_horizon if replay_horizon != -1 else self.dataset[0]['action'].shape[0]
        self.replay_trajectory_index = replay_trajectory_index

        # build task
        self.build()

    def reset(self):
        self.scenary.reset()
        self.robot.reset()
        self.controller.reset()
        self.sensors.reset()

    def build(self):
        # 1. 加载场景
        self.scenary.load_stage()

        # 2. 创建机器人
        self.robot.spawn(self.scenary.world)

        # 3. 创建传感器
        self.sensors.spawn()

        # 4. 创建控制器    
        self.controller.spawn(self.robot)

        # 5. 重置环境
        self.reset()

    def get_raw_data(self):
        rgb_data = self.sensors.get_data()

        ee_pose = self.robot.get_ee_pose()
        joint_pos = self.robot.get_joint_position()

        init_ee_pose = self.robot.get_init_ee_pose()
        init_joint_pos = self.robot.get_init_joint_pos()

        gripper_width = self.robot.get_gripper_width()

        data = {
            'rgb_data': rgb_data,
            'ee_pose': ee_pose,
            'joint_pos': joint_pos,
            'init_ee_pose': init_ee_pose,
            'init_joint_pos': init_joint_pos,
            'gripper_width': gripper_width
        }
        return data

    def run(self,simulation_app):
        world = self.scenary.world
        warm_up = 100
        i = 0
        interval = 60 // self.sensors.camera_freq
        replay_count = 0
        while simulation_app.is_running():
            # 推进仿真并渲染
            self.scenary.step(render=True)
            if world.is_stopped() and not self.reset_needed:
                self.reset_needed = True
            if i < warm_up:
                i += 1
                continue
            if world.is_playing():
                reset = self.reset_needed
                if self.reset_needed:
                    self.reset()
                    replay_count = 0
                    i = 0
                    self.reset_needed = False

                if i % interval == 0:
                    if replay_count < self.replay_horizon:
                        trajectory_index = self.replay_trajectory_index
                        all_action = self.dataset[trajectory_index]['action']
                        action = all_action[replay_count+1]
                        replay_count += 1

                        # print(action[-1])
                        # if action[-1] < 0.02:
                        #     action[-1]  = 0
                        if replay_count == (self.dataset[trajectory_index]['action'].shape[0] - 1):
                            break
                    else:
                        # 获取传感器数据
                        data = self.get_raw_data()
                        data['reset'] = reset

                        # 获取动作
                        action = self.controller.forward(data)

                    # 控制机器人
                    if isinstance(action, np.ndarray):
                        self.robot.apply_action(action)
                    elif isinstance(action,tuple) and isinstance(action[0],ArticulationAction): # Joint position + gripper width
                        target_action = action[0]
                        gripper_width = action[1]
                        self.robot.apply_action(target_action.joint_positions,target_action.joint_indices)
                        self.robot.apply_gripper_width(gripper_width)

                i = i +1 

        simulation_app.close()

class StackCubeDataCollect(BaseTask):
    def get_next_episode_dir(self,base_dir):
        """在 `base_dir` 下创建一个新的 `epo_xx` 目录，编号递增"""
        os.makedirs(base_dir, exist_ok=True)  # 确保基础目录存在
        existing_dirs = [d for d in os.listdir(base_dir) if d.startswith("epo_") and d[4:].isdigit()]
        existing_dirs.sort(key=lambda x: int(x[4:]))  # 按编号排序

        if existing_dirs:
            last_index = int(existing_dirs[-1][4:])  # 获取最后一个编号
            new_index = last_index + 1
        else:
            new_index = 0  # 第一次运行，从 0 开始

        new_episode_dir = os.path.join(base_dir, f"epo_{new_index}")
        os.makedirs(new_episode_dir)  # 创建新目录
        return new_episode_dir

    def __init__(self,save_dir = "./output_data", *args,**kwargs):

        super().__init__(*args,**kwargs)
        
        self.img_count = 0
        self.save_dir = self.get_next_episode_dir(save_dir)
        os.makedirs(self.save_dir,exist_ok=True)

        # 设定随机范围
        x_range = (0.5, 0.7)  # X 轴范围
        y_range = (-0.3, 0.3)  # Y 轴范围

        # 随机 yaw 角度（单位：弧度）
        yaw = np.random.uniform(-np.pi/4, np.pi/4)

        # 欧拉角（roll, pitch, yaw） -> 四元数 [w, x, y, z]
        quat = euler_angles_to_quat([0, 0, yaw])  # 只绕 Z 轴旋转

        pos_red = np.array([
            np.random.uniform(*x_range),
            np.random.uniform(*y_range),
            0.12
        ])

        pos_green = np.array([
            np.random.uniform(*x_range),
            np.random.uniform(*y_range),
            0.12
        ])
        #保证两个物体不重叠
        while np.linalg.norm(pos_red - pos_green) < 0.2:
            pos_green = np.array([
                np.random.uniform(*x_range),
                np.random.uniform(*y_range),
                0.12
            ])

        self.cube_red_cfg = {
            "name": "cube",
            "position": pos_red,
            "orientation": quat,
            "prim_path": "/World/Cube",
            "scale": np.array([0.05, 0.05, 0.05]),
            "size": 1.0,
            "color": np.array([1, 0, 0]),
        }

        self.cube_green_cfg = {
            "name": "cube2",
            "position": pos_green,
            "orientation": quat,
            "prim_path": "/World/Cube2",
            "scale": np.array([0.05, 0.05, 0.05]),
            "size": 1.0,
            "color": np.array([0, 0, 1]),
        }

        init_pos, init_orientation = self.robot.get_init_ee_pose()
        gripper_quat = np.array(
            [
                -quat[1],
                quat[0],
                quat[3],
                -quat[2],
            ]
        )

        trajectory = [
            {"t": 0, "xyz": init_pos, "quat": init_orientation, "gripper": 1.0},  # Start 
            {"t": 100, "xyz": [pos_red[0], pos_red[1], pos_red[2] + 0.20], "quat": init_orientation, "gripper": 1.0},  # 靠近
            {"t": 140, "xyz": [pos_red[0], pos_red[1], pos_red[2] - 0.01], "quat": gripper_quat, "gripper": 1.0},  # 下沉
            {"t": 170, "xyz": [pos_red[0], pos_red[1], pos_red[2] - 0.01], "quat": gripper_quat, "gripper": 0.1},  # 抓取
            {"t": 230, "xyz": [pos_green[0], pos_green[1], pos_green[2] + 0.20], "quat": gripper_quat, "gripper": 0.1},  # 移动
            {"t": 270, "xyz": [pos_green[0], pos_green[1], pos_green[2] + 0.10], "quat": gripper_quat, "gripper": 0.1},  # 下沉
            {"t": 300, "xyz": [pos_green[0], pos_green[1], pos_green[2] + 0.10], "quat": gripper_quat, "gripper": 1.0},  # 释放
        ]
        

        self.scenary.add_cube(self.cube_red_cfg)
        self.scenary.add_cube(self.cube_green_cfg)
        self.controller.set_trajectory(trajectory)
        self.reset()

    def save_img(self,rgb_data):
        output_dir = self.save_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        camera_dir = os.path.join(output_dir, "camera")
        if not os.path.exists(camera_dir):
            os.makedirs(camera_dir)
        
        img = Image.fromarray(rgb_data) # 展平

        file_path = os.path.join(camera_dir, f"camera_output_{self.img_count}.png")
        self.img_count += 1
        if os.path.exists(file_path):
            print(f"Frame {file_path} already recorded. Skipping save.")
        else:
            img.save(file_path)

    def get_raw_data(self):
        rgb_data = self.sensors.get_data()
        self.save_img(rgb_data)

        ee_pose = self.robot.get_ee_pose()
        joint_pos = self.robot.get_joint_position()

        init_ee_pose = self.robot.get_init_ee_pose()
        init_joint_pos = self.robot.get_init_joint_pos()

        gripper_width = self.robot.get_gripper_width()

        data = {
            'ee_pose': ee_pose,
            'joint_pos': joint_pos,
            'init_ee_pose': init_ee_pose,
            'init_joint_pos': init_joint_pos,
            'gripper_width': gripper_width
        }
        return data

    def run(self,simulation_app):
        world = self.scenary.world
        warm_up = 100
        i = 0
        interval = 60 // self.sensors.camera_freq
        data_list = []
        while simulation_app.is_running():
            # 推进仿真并渲染
            self.scenary.step(render=True)
            if world.is_stopped() and not self.reset_needed:
                self.reset_needed = True
            if i < warm_up:
                i += 1
                continue
            if world.is_playing():
                reset = self.reset_needed
                if self.reset_needed:
                    self.reset()
                    i = 0
                    self.reset_needed = False

                if i % interval == 0:
                    # 获取传感器数据
                    data = self.get_raw_data()
                    data['reset'] = reset
                    data_list.append(data)

                    # 获取动作
                    action = self.controller.forward(data)
                    
                    # 控制机器人
                    if isinstance(action, np.ndarray):
                        self.robot.apply_action(action)
                    elif isinstance(action,tuple) and isinstance(action[0],ArticulationAction): # Joint position + gripper width
                        target_action = action[0]
                        gripper_width = action[1]
                        self.robot.apply_action(target_action.joint_positions,target_action.joint_indices)
                        self.robot.apply_gripper_width(gripper_width)

                    if self.controller.is_done():
                        print("done picking and placing")
                        self.save_data(
                            self.save_dir,
                            data_list,
                            cube_pose1=np.concatenate([self.cube_red_cfg['position'], self.cube_red_cfg['orientation']]),
                            cube_pose2=np.concatenate([self.cube_green_cfg['position'], self.cube_green_cfg['orientation']]),
                        )
                        break
                i = i + 1 

        simulation_app.close()



    def save_data(
            self,
            output_dir,
            data_list,
            cube_pose1 = None,
            cube_pose2 = None
        ):
        
        # 创建目录结构
        meta_dir = os.path.join(output_dir, "meta")
        data_dir = os.path.join(output_dir, "data")
        os.makedirs(meta_dir, exist_ok=True)
        os.makedirs(data_dir, exist_ok=True)
        
        cur_pos_list = []
        cur_orientation_list = []
        gripper_width_list = []
        joint_pos_list = []

        init_joint_pos = data_list[0]["init_joint_pos"]
        init_pos = data_list[0]["ee_pose"][0]
        init_orientation = data_list[0]["ee_pose"][1]
        end_pose = data_list[-1]["ee_pose"][0]
        end_orientation = data_list[-1]["ee_pose"][1]

        for t in range(len(data_list)):
            item = data_list[t]
            cur_pos_list.append(item["ee_pose"][0])
            cur_orientation_list.append(item["ee_pose"][1])
            gripper_width_list.append([item["gripper_width"]])  # 注意要加中括号，保持二维
            joint_pos_list.append(item["joint_pos"])
            
        # 处理旋转表示
        cur_orientation_rotvec = [quaternion_to_rotvec(q) for q in cur_orientation_list]
        init_orientation_rotvec = quaternion_to_rotvec(init_orientation)
        end_orientation_rotvec = quaternion_to_rotvec(end_orientation)

        #存储机器人初始关节pos数据
        np.savetxt(os.path.join(data_dir, "robot0_init_joint_positions.txt"), init_joint_pos, fmt='%.6f')
        #存储机器人关节pos数据
        np.savetxt(os.path.join(data_dir, "robot0_joint_positions.txt"), joint_pos_list, fmt='%.6f')

        # 存储机器人数据
        np.savetxt(os.path.join(data_dir, "robot0_gripper_width.txt"), gripper_width_list, fmt='%.6f')
        
        # 存储轨迹数据
        np.savetxt(os.path.join(data_dir, "robot0_eef_position.txt"), cur_pos_list, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "robot0_eef_rot_axis_angle.txt"), cur_orientation_rotvec, fmt='%.6f')
        
        # 存储初始和结束状态
        np.savetxt(os.path.join(data_dir, "robot0_init_position.txt"), init_pos, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "robot0_init_orientation.txt"), init_orientation_rotvec, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "robot0_end_position.txt"), end_pose, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "robot0_end_orientation.txt"), end_orientation_rotvec, fmt='%.6f')

        np.savetxt(os.path.join(data_dir, "cube_red"), cube_pose1, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "cube_blue"), cube_pose2, fmt='%.6f')
        
        print(f"Data saved successfully in {output_dir}")