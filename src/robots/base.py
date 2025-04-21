from omni.isaac.core.robots import Robot
from isaacsim.core.utils.types import ArticulationAction
import numpy as np
from omni.isaac.core.prims import XFormPrim,RigidPrim

class BaseRobot:
    def __init__(self, robot_prim_path,joints_name= [],ee_prim_path=None,gripper_joint_name=None):
        self.robot_prim_path = robot_prim_path
        self.robot = None
        self.joints_name = joints_name
        self.joint_indices = []
        self.ee_prim_path = ee_prim_path

        self.init_ee_pose = None     # 初始时刻的 ee 位姿
        self.init_joint_pos = None  # 初始时刻的关节角度

        self.gripper_joint_name = gripper_joint_name
    
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
    
    def get_init_ee_pose(self):
        if self.init_ee_pose is None:
            self.init_ee_pose = self.get_ee_pose()
        return self.init_ee_pose
    
    def get_init_joint_pos(self):
        if self.init_joint_pos is None:
            self.init_joint_pos = self.get_joint_position()
        return self.init_joint_pos

    def get_ee_pose(self):
         # 获取 panda_hand 的 Prim
        end_effector_path = self.ee_prim_path
        try:
            hand_prim = XFormPrim(end_effector_path)
            # 获取全局坐标系下的位姿
            state = hand_prim.get_default_state()
            return state.position, state.orientation
        except:
            hand_prim = RigidPrim(end_effector_path)
                # 获取全局坐标系下的位姿
            return  hand_prim.get_world_pose()

    def get_gripper_width(self):
        gripper_joint_name = self.gripper_joint_name
        index = self.robot.get_dof_index(gripper_joint_name)  # 假设有这个方法
        joint_positions = self.robot.get_joint_positions()
        gripper_width = joint_positions[index]
        return gripper_width
