class BaseTask:
    def __init__(self,
                 scenary,
                 robot,
                 controller,
                 sensors,
                 dataset=None,
                 replay_horizon = 0):
        self.scenary = scenary
        self.robot = robot
        self.controller = controller
        self.sensors = sensors

        self.reset_needed = False
        self.dataset = dataset
        self.replay_horizon = replay_horizon if replay_horizon != -1 else self.dataset[0]['action'].shape[0]

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
        self.controller.spawn()

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
                        all_action = self.dataset[0]['action']
                        action = all_action[replay_count+1]
                        replay_count += 1
                        if replay_count == self.dataset[0]['action'].shape[0]:
                            break
                    else:
                        # 获取传感器数据
                        data = self.get_raw_data()
                        data['reset'] = reset

                        # 获取动作
                        action = self.controller.forward(data)

                    # 控制机器人
                    self.robot.apply_action(action)

                i = i +1 

        simulation_app.close()

class StackCube(BaseTask):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        pass