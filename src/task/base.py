from src.utils import draw_single_point

class BaseTask:
    def __init__(self,
                 scenary,
                 robot, # A dictory contains many robots
                 env_sensors  = {},
                 dataset=None,
                 frequencies = None,
                 replay_horizon = 0,
                 replay_trajectory_index = 0,
                 visualize_trajectory = False):
        
        self.scenary = scenary
        self.robot = robot
        self.env_sensors  = env_sensors 

        self.reset_needed = False # set to true if you restart the simulator
        self.dataset = dataset
        self.replay_horizon = replay_horizon if replay_horizon != -1 else self.dataset[0]['action'].shape[0]
        self.replay_trajectory_index = replay_trajectory_index

        self.frequencies = frequencies
        self.done_flag = False
        # build task
        self.build()
        self.replay_count = 0
        self.visualize_trajectory = visualize_trajectory

    def reset(self):
        self.scenary.reset()
        self.robot.reset()
        for _, sensor in self.env_sensors.items():
            sensor.reset()

    def build(self):
        # 1. 加载场景
        self.scenary.load_stage()

        # 2. 创建机器人
        self.robot.spawn(self.scenary.world)

        # 3. 创建传感器
        for _,sensor in self.env_sensors.items():
            sensor.spawn()

        # 5. 重置环境
        self.reset()

    def get_raw_data(self):
        data = {
            'env_sensors': {},
            'robot': {}
        }
        # 获取传感器数据
        for name, sensor in self.env_sensors.items():
            data['env_sensors'][name] = sensor.get_data()
                
        # 获取机器人本体数据
        robot_states = self.robot.get_states()
        robot_sensor_data  = self.robot.get_sensor_data()
          
        data['robot']['states'] = robot_states
        data['robot']['sensors'] = robot_sensor_data

        return data
    
    def step(self,reset):
        # if self.replay_count < self.replay_horizon:
        trajectory_index = self.replay_trajectory_index
        all_action = self.dataset[trajectory_index]['action']
        next_action = all_action[self.replay_count]
        if self.visualize_trajectory:
            ee_pose =  self.dataset[trajectory_index]['ee_pose']
            draw_single_point(ee_pose[self.replay_count][:3],color='expert')
        self.replay_count += 1
        if self.replay_count == (self.dataset[trajectory_index]['action'].shape[0] - 1):
            self.done_flag = True
        # else:
        # 获取传感器数据
        data = self.get_raw_data() 
        data['reset'] = reset

        # 获取动作
        next_action = self.robot.compute_action(data)
                       
        # 控制机器人
        self.robot.apply_action(next_action)
       

    def run(self,simulation_app):
        world = self.scenary.world
        warm_up = 100
        i = 0
        interval = 60 // self.frequencies['control']
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
                    self.replay_count = 0
                    i = 0
                    self.reset_needed = False

                if i % interval == 0:
                    if self.visualize_trajectory:
                        ee_pose = self.robot.get_ee_pose()
                        draw_single_point(ee_pose[:3],color="model")
                        
                    self.step(reset)
                        
                i = i + 1 

                if self.done_flag:
                    break

        simulation_app.close()

