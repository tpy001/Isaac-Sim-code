import numpy as np
from isaacsim.core.utils.types import ArticulationAction

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

class StackCube(BaseTask):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        pass