from src.task.base import BaseTask
import os
import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from PIL import Image


class BaseDataCollect(BaseTask):
    def __init__(self,save_dir = "./output_data", save_trajectory = True,*args,**kwargs):

        super().__init__(*args,**kwargs)
        
        self.save_trajectory = save_trajectory
        self.img_count = 0
        self.save_dir = self.get_next_episode_dir(save_dir)
        os.makedirs(self.save_dir,exist_ok=True)

        self.collected_data = []

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
    

    def step(self,reset):
        if self.replay_count < self.replay_horizon:
            trajectory_index = self.replay_trajectory_index
            all_action = self.dataset[trajectory_index]['action']
            next_action = all_action[self.replay_count+1]
            self.replay_count += 1
            if self.replay_count == (self.dataset[trajectory_index]['action'].shape[0] - 1):
                self.done_flag = True
        else:
            # 获取传感器数据
            data = self.get_raw_data() 
            data['reset'] = reset
            data['action'] = {}

            # 获取动作
            next_action = self.robot.compute_action(data)

            if self.save_trajectory:
                # 直接存储专家轨迹作为 action
                trajectory = self.robot.get_trajectory()
                data['action']['ee_pose'] = trajectory
                data['action']['joint_pos'] = next_action
          
            self.collected_data.append(data)
            if self.robot.controller.is_done():
                print("done picking and placing")
                self.save_data(
                    self.save_dir,
                    self.collected_data,
                )
                self.done_flag = True
                       
        # 控制机器人
        self.robot.apply_action(next_action)

        # 保存采集的图像
        for name, sensor_data in data['env_sensors'].items():
            self.save_img(sensor_data,name)

        for name,sensor_data in data['robot']['sensors'].items():
            self.save_img(sensor_data,name)
     

        self.img_count += 1
        

    def save_img(self,rgb_data,sensor_name="camera"):
        output_dir = self.save_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        camera_dir = os.path.join(output_dir, "camera")
        if not os.path.exists(camera_dir):
            os.makedirs(camera_dir)
        
        img = Image.fromarray(rgb_data) # 展平

        file_path = os.path.join(camera_dir, f"{sensor_name}_{self.img_count}.png")
        if os.path.exists(file_path):
            print(f"Frame {file_path} already recorded. Skipping save.")
        else:
            img.save(file_path)