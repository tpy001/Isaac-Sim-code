from collections import deque
import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import euler_angles_to_quat
import os
from src.utils import quaternion_to_rotvec,set_prim_transform
from PIL import Image
import random
import cv2

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
        if self.dataset is not None:
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
        rgb_data = self.sensors.get_rgb_data()
        depth_data = self.sensors.get_depth_map()

        ee_pose = self.robot.get_ee_pose()
        joint_pos = self.robot.get_joint_position()

        init_ee_pose = self.robot.get_init_ee_pose()
        init_joint_pos = self.robot.get_init_joint_pos()

        gripper_width = self.robot.get_gripper_width()

        data = {
            'rgb_data': rgb_data,
            'depth_data':depth_data,
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
        

class StackCubeTask(BaseTask):
    def __init__(self,
                 save_dir = "./mutli_task_results",
                 save_trajectory = False,
                 *args,**kwargs):
        super().__init__(*args,**kwargs)
        self.save_dir = save_dir
        self.save_trajectory = save_trajectory
        self.success = False
        self.success_queue = deque(maxlen=3)  # 存储最近3帧的检测结果
        self.prev_velocities = {}  # 用于速度检测

    def check_success_distance(self, threshold=0.6):
        """检查立方体是否叠放成功（距离小于0.1cm）"""
        cube1_pos = self.scenary.source_cube.get_world_pose()[0]  # 获取前三维坐标 red_cube
        cube2_pos = self.scenary.target_cube.get_world_pose()[0]
        distance = np.linalg.norm(cube1_pos - cube2_pos)
        print(f"Distance between cubes: {distance}")
        print(cube1_pos, cube2_pos)
        return distance < 0.6
    
    def check_success(self, dis_threshold=0.06, z_threshold=0.01, xy_threshold=0.01
                      , dot_threshold=0.7, vel_threshold=0.05):
        """
        triggers for 成功检测
        [Success Check] distance: 0.0525, z_diff: 0.0525, xy: 0.0021, dot: 1.00, vel: 0.01/0.03
        """
        # 获取完整位姿信息（修正之前获取错误target的问题）
        cube1_pos = self.scenary.source_cube.get_world_pose()[0]
        cube2_pos = self.scenary.target_cube.get_world_pose()[0]
        
        # 基础距离检测
        distance_condition = np.linalg.norm(cube1_pos - cube2_pos) <= dis_threshold
        
        # 新增条件1：z轴高度差（红色必须在蓝色上方）
        z_condition = (cube1_pos[2] > cube2_pos[2]) 
        
        # 新增条件2：xy平面距离
        xy_distance = np.linalg.norm(cube1_pos[:2] - cube2_pos[:2])
        xy_condition = (xy_distance < xy_threshold)
        
        # 新增条件3：方向对齐检测（四元数点积）
        cube1_rot = self.scenary.source_cube.get_world_pose()[1]
        cube2_rot = self.scenary.target_cube.get_world_pose()[1]
        dot_product = np.dot(cube1_rot, cube2_rot)
        orientation_condition = (abs(dot_product) > dot_threshold)  # 方向基本一致
        
        # 新增条件4：速度检测（必须静止）
        cube1_vel = self.scenary.source_cube.get_linear_velocity()
        cube2_vel = self.scenary.target_cube.get_linear_velocity()
        velocity_condition = (np.linalg.norm(cube1_vel) < vel_threshold and 
                            np.linalg.norm(cube2_vel) < vel_threshold)
        
        # 组合条件（可根据需要调整逻辑与/或关系）
        current_success = (
            distance_condition and 
            z_condition # and 
            # xy_condition and 
            # orientation_condition and
            # velocity_condition
        )
        
        # 更新连续检测队列
        self.success_queue.append(current_success)
        
        # 调试输出
        # print(f"[Success Check] distance: {distance:.4f}, z_diff: {cube1_pos[2]-cube2_pos[2]:.4f}, "
        #     f"xy: {xy_distance:.4f}, dot: {dot_product:.2f}, "
        #     f"vel: {np.linalg.norm(cube1_vel):.2f}/{np.linalg.norm(cube2_vel):.2f}")
        
        # 连续3帧满足条件才算成功
        # print(self.success_queue, self.success)
        return all(self.success_queue)

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

            # self.success = self.check_success()           
            # self.save_data(
            #     self.save_dir,
            #     cube_pose1=self.scenary.source_cube.get_world_pose()[0],
            #     cube_pose2=self.scenary.source_cube.get_world_pose()[1],
            # )
            # if self.success:
            #     break
            current_success = self.check_success()
            self.save_data()


            if current_success:
                self.success = True
                self.save_data()
                break  # 检测到成功时提前退出

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

    def save_data(self, cube_pose1=None, cube_pose2=None):
        
        # 新增成功状态保存
        meta_dir = os.path.join(self.save_dir, "meta")
        os.makedirs(meta_dir, exist_ok=True)
        np.savetxt(os.path.join(meta_dir, "success.txt"), [int(self.success)], fmt='%d')

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

    def __init__(self,save_dir = "./output_data", save_trajectory = True,*args,**kwargs):

        super().__init__(*args,**kwargs)
        
        self.save_trajectory = save_trajectory
        self.img_count = 0
        self.save_dir = self.get_next_episode_dir(save_dir)
        os.makedirs(self.save_dir,exist_ok=True)

        # 设定随机范围
        x_range = (0.5, 0.6)  # X 轴范围
        x_range_shifted = (x_range[0] + 0.05, x_range[1] + 0.05)
        y_range = (-0.15, 0.15)  # Y 轴范围

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
            np.random.uniform(*x_range_shifted),
            np.random.uniform(*y_range),
            0.12
        ])

        while np.linalg.norm(pos_red - pos_green) < 0.1:
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
            {"t": 40, "xyz": [pos_red[0], pos_red[1], pos_red[2] + 0.20], "quat": init_orientation, "gripper": 1.0},  # 靠近
            {"t": 60, "xyz": [pos_red[0], pos_red[1], pos_red[2] - 0.01], "quat": gripper_quat, "gripper": 1.0},  # 下沉
            {"t": 68, "xyz": [pos_red[0], pos_red[1], pos_red[2] - 0.01], "quat": gripper_quat, "gripper": 0.1},  # 抓取
            {"t": 93, "xyz": [pos_green[0], pos_green[1], pos_green[2] + 0.20], "quat": gripper_quat, "gripper": 0.1},  # 移动
            {"t": 112, "xyz": [pos_green[0], pos_green[1], pos_green[2] + 0.10], "quat": gripper_quat, "gripper": 0.1},  # 下沉
            {"t": 125, "xyz": [pos_green[0], pos_green[1], pos_green[2] + 0.10], "quat": gripper_quat, "gripper": 1.0},  # 释放
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

    def save_depth_map(self,depth_map,colormap=cv2.COLORMAP_JET):
        output_dir = self.save_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        camera_dir = os.path.join(output_dir, "depth_map")
        if not os.path.exists(camera_dir):
            os.makedirs(camera_dir)
        
        # 去除非法值（如 nan/inf）
        depth = np.nan_to_num(depth_map.copy(), nan=10, posinf=10.0, neginf=0.0)
        
        # --- 保存原始深度图（毫米，16-bit PNG） ---
        file_path = os.path.join(camera_dir, f"depth_output_{self.img_count}.png")
        if os.path.exists(file_path):
            print(f"Frame {file_path} already recorded. Skipping save.")
        depth_mm = (depth * 1000).astype(np.uint16)
        Image.fromarray(depth_mm, mode='I;16').save(file_path)

        # --- 生成并保存伪彩色可视化图 ---
        # 按百分位归一化（更稳健）
        d_min, d_max = np.percentile(depth, 2), np.percentile(depth, 98)
        normalized = np.clip((depth - d_min) / (d_max - d_min + 1e-6), 0, 1)
        depth_uint8 = (normalized * 255).astype(np.uint8)

        # 映射颜色
        depth_color = cv2.applyColorMap(depth_uint8, colormap)
        depth_rgb = cv2.cvtColor(depth_color, cv2.COLOR_BGR2RGB)

        file_path = os.path.join(camera_dir, f"depth_output_{self.img_count}_color.png")
        Image.fromarray(depth_rgb).save(f'{file_path}_vis.png')

    def get_raw_data(self):
        rgb_data = self.sensors.get_rgb_data()
        depth_map = self.sensors.get_depth_map()
        self.save_img(rgb_data)
        self.save_depth_map(depth_map)
        
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
                    # 获取动作
                    action = self.controller.forward(data)
                    data['reset'] = reset
                    if self.save_trajectory:
                        # 直接存储专家轨迹作为 action
                        t = self.controller.current_time_step
                        data['action_ee_pose'] = ( self.controller.trajectory[t]['position'],  self.controller.trajectory[t]['orientation'] )
                        data['action_joint_pos'] = np.concatenate( [action[0].joint_positions, np.array([  action[1]])]) 
                        data['action_gripper_width'] =  action[1]
                    data_list.append(data)

                  
                    
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
        action_pos_list = []
        cur_orientation_list = []
        action_orientation_list = []
        gripper_width_list = []
        action_gripper = []
        joint_pos_list = []
        action_joints = []

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

            action_pos_list.append(item["action_ee_pose"][0])
            action_orientation_list.append(item["action_ee_pose"][1])
            action_gripper.append(item["action_gripper_width"])
            action_joints.append(item["action_joint_pos"])
            
        # 处理旋转表示
        cur_orientation_rotvec = [quaternion_to_rotvec(q) for q in cur_orientation_list]
        action_ori_rotevc = [quaternion_to_rotvec(q) for q in action_orientation_list]
        init_orientation_rotvec = quaternion_to_rotvec(init_orientation)
        end_orientation_rotvec = quaternion_to_rotvec(end_orientation)

        #存储机器人初始关节pos数据
        np.savetxt(os.path.join(data_dir, "robot0_init_joint_positions.txt"), init_joint_pos, fmt='%.6f')
        #存储机器人关节pos数据
        np.savetxt(os.path.join(data_dir, "robot0_joint_positions.txt"), joint_pos_list, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "robot0_action_joint_positions.txt"), action_joints, fmt='%.6f')


        # 存储机器人数据
        np.savetxt(os.path.join(data_dir, "robot0_gripper_width.txt"), gripper_width_list, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "robot0_action_gripper_width.txt"), action_gripper, fmt='%.6f')

        
        # 存储轨迹数据
        np.savetxt(os.path.join(data_dir, "robot0_eef_position.txt"), cur_pos_list, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "robot0_action_eef_position.txt"), action_pos_list, fmt='%.6f')

        np.savetxt(os.path.join(data_dir, "robot0_eef_rot_axis_angle.txt"), cur_orientation_rotvec, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "robot0_action_eef_rot_axis_angle.txt"), action_ori_rotevc, fmt='%.6f')

        
        # 存储初始和结束状态
        np.savetxt(os.path.join(data_dir, "robot0_init_position.txt"), init_pos, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "robot0_init_orientation.txt"), init_orientation_rotvec, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "robot0_end_position.txt"), end_pose, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "robot0_end_orientation.txt"), end_orientation_rotvec, fmt='%.6f')

        np.savetxt(os.path.join(data_dir, "cube_red"), cube_pose1, fmt='%.6f')
        np.savetxt(os.path.join(data_dir, "cube_blue"), cube_pose2, fmt='%.6f')
        
        print(f"Data saved successfully in {output_dir}")