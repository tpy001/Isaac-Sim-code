
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import os


class BaseEnv:
    def __init__(
        self,
        scenery_config,                  # 环境场景，例如房间、城市街景等
        robot_config=None,               # 机器人实例（可选）
        controller_config=None,          # 控制器，控制机器人运动（可选）
        sensor_configs=None,             # 传感器列表（可选）
    ):  
        # 1. scenery 
        self.loaded = False
        self.world = None
        self.stage = None
        self.load_stage(scenery_config)
    
        # 2. Robot
        self.robot = robot
        self.controller = controller
        self.sensors = sensor_list if sensor_list is not None else []

    def reset(self):
        """重置环境"""
        # 可在此清理/重置环境状态
        self.world.reset()
        if self.robot:
            self.robot.reset()
        for sensor in self.sensors:
            sensor.reset()

    

    def spawn_robot(self,robot_cls):


    def step(self, action):
        # """执行一步仿真"""
        # if self.controller:
        #     self.controller.apply(action)
        # if self.physics_engine:
        #     self.physics_engine.step()
        # observations = {s.name: s.read() for s in self.sensors}
        # return observations
        pass
