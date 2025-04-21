from omni.isaac.core.robots import Robot
from isaacsim.core.utils.types import ArticulationAction
import numpy as np

class BaseRobot:
    def __init__(self, robot_prim_path,joints_name= []):
        self.robot_prim_path = robot_prim_path
        self.robot = None
        self.joints_name = joints_name
        self.joint_indices = []
            
    def spawn(self,world,name="franka"):
        self.robot = world.scene.add(
            Robot(prim_path=self.robot_prim_path, name=name)
        )
    
    def reset(self):
        # 获取 joint name 对应的 index
        if len(self.joints_name) > 0:
            self.joint_indices = [self.robot.get_dof_index(joint_name) for joint_name in self.joints_name]

    def apply_action(self,joint_positions,joint_indices = None):
        if joint_indices is None:
            joint_indices = self.joint_indices
        target_joint_action = ArticulationAction(
            joint_positions = joint_positions,
            joint_indices = joint_indices
        )
        articulation_controller = self.robot.get_articulation_controller()
        articulation_controller.apply_action(target_joint_action)


    def get_joint_position(self,joint_indices= None):
        if joint_indices is None:
            joint_indices = self.joint_indices
        joint_pos = [  self.robot.get_joint_positions(joint_idx) for joint_idx in joint_indices]
        return np.column_stack(joint_pos)[0]


