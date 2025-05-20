from omni.isaac.core.robots import Robot
from isaacsim.core.utils.types import ArticulationAction
import numpy as np
from omni.isaac.core.prims import XFormPrim,RigidPrim

class BaseRobot:
    def __init__(self, robot_prim_path,joints_name= [],ee_prim_path=None,gripper_joint_name=None,sensors={},controller=None):
        self.robot_prim_path = robot_prim_path
        self.robot = None
        self.joints_name = joints_name
        self.joint_indices = []
        self.ee_prim_path = ee_prim_path

        self.init_ee_pose = None     # 初始时刻的 ee 位姿
        self.init_joint_pos = None  # 初始时刻的关节角f度

        self.gripper_joint_name = gripper_joint_name

        self.sensors = sensors
        self.controller = controller
    
    def spawn(self,world,name="franka"):
        self.robot = world.scene.add(
            Robot(prim_path=self.robot_prim_path, name=name)
        )

        for _, sensor in self.sensors.items():
            sensor.spawn()
        
        if self.controller is not None:
            self.controller.spawn(self)
    
    def reset(self):     
        self.gripper_index = self.robot.get_dof_index(self.gripper_joint_name)
        if len(self.joints_name) > 0:
            self.joint_indices = [self.robot.get_dof_index(joint_name) for joint_name in self.joints_name]

        for _, sensor in self.sensors.items():
            sensor.reset()
        
        if self.controller is not None:
            self.controller.reset()

    def apply_action(self,joint_positions,joint_indices = None):
        if joint_indices is None:
            joint_indices = self.joint_indices
        target_joint_action = ArticulationAction(
            joint_positions = joint_positions,
            joint_indices = joint_indices
        )
        articulation_controller = self.robot.get_articulation_controller()
        articulation_controller.apply_action(target_joint_action)

    def apply_gripper_width(self,gripper_width):
        gripper_index = self.gripper_index
        self.apply_action(joint_positions = [gripper_width],joint_indices = [gripper_index])
        
    def get_joint_position(self,joint_indices= None):
        if joint_indices is None:
            joint_indices = self.joint_indices
        joint_pos = [  self.robot.get_joint_positions(joint_idx) for joint_idx in joint_indices]
        return np.column_stack(joint_pos)[0]

    def get_ee_pose(self):
         # 获取 panda_hand 的 Prim
        end_effector_path = self.ee_prim_path
        try:
            hand_prim = XFormPrim(end_effector_path)
            # 获取全局坐标系下的位姿
            state = hand_prim.get_default_state()
            return np.concatenate( [state.position, state.orientation],axis=-1)
        except:
            hand_prim = RigidPrim(end_effector_path)
                # 获取全局坐标系下的位姿
            return  hand_prim.get_world_pose()

    def get_gripper_width(self):
        gripper_width = self.robot.get_joint_positions(self.gripper_index)
        return gripper_width
    

    def get_states(self):
        joint_pos = self.get_joint_position()
        ee_pose = self.get_ee_pose()
        gripper_width = self.get_gripper_width()

        states = {
            'joint_pos': joint_pos,
            'ee_pose': ee_pose,
            'gripper_width': gripper_width,
        }

        return states

    def get_sensor_data(self):
        sensor_data = {}
        for name, sensor in self.sensors.items():
            sensor_data[name] = sensor.get_data()
          
        return sensor_data
        
    def compute_action(self,data):
        # obs = {
        #     'rgb_data': data['robot']['sensors']['camera_rgb_wrist'],
        #     'joint_pos': data['robot']['states']['joint_pos'],
        #     'ee_pose': data['robot']['states']['ee_pose'],
        #     'gripper_width': data['robot']['states']['gripper_width'],
        #     'reset': data['reset']
        # }
        action = self.controller.forward(data)
        return action
    
    def set_trajectory(self,trajectory):
        self.controller.set_trajectory(trajectory)

    def get_trajectory(self):
        return self.controller.get_trajectory()

    def get_sensors(self):
        return self.sensors