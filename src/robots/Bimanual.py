

import numpy as np

class BimanualRobot:
    def __init__(self, left_arm,right_arm,controller=None):
        self.left_arm = left_arm
        self.right_arm = right_arm
        self.controller = controller

    def spawn(self,world,name="franka"):
        self.left_arm.spawn(world,name="left_arm")
        self.right_arm.spawn(world,name="right_arm")
        if self.controller is not None:
            self.controller.spawn(self.left_arm,self.right_arm)

    def reset(self):
        self.left_arm.reset()
        self.right_arm.reset()
        if self.controller is not None:
            self.controller.reset()

    def apply_action(self, joint_positions, joint_indices=None,arm_side="all"):
        if arm_side == 'left':
            self.left_arm.apply_action(joint_positions, joint_indices)
        elif arm_side == 'right':
            self.right_arm.apply_action(joint_positions, joint_indices)
        elif arm_side == 'all':
            # 默认左臂和右臂的 joint_positions 和 joint_indices 合在一个数组里
            joint_num = joint_positions.shape[0]
            left_arm_joint_position = joint_positions[:joint_num //2]
            right_arm_joint_position = joint_positions[(joint_num //2):]
            if joint_indices is not None:
                left_arm_joint_indices = joint_indices[joint_num//2:] 
                right_arm_joint_indices = joint_indices[(joint_num //2):] 
            else:
                left_arm_joint_indices = None
                right_arm_joint_indices = None

            self.left_arm.apply_action(left_arm_joint_position,left_arm_joint_indices)
            self.right_arm.apply_action(right_arm_joint_position,right_arm_joint_indices)
        else:
            raise ValueError(f"Unknown arm_side: {arm_side}")

      
    def apply_gripper_width(self, width,arm_side='all'):
        if arm_side == 'left':
            self.left_arm.apply_gripper_width(width)
        elif arm_side == 'right':
            self.right_arm.apply_gripper_width(width)
        elif arm_side == 'all':
            self.left_arm.apply_gripper_width(width[0])
            self.right_arm.apply_gripper_width(width[1])
        else:
            raise ValueError(f"Unknown arm_side: {arm_side}")

    def get_joint_position(self, joint_indices=None,arm_side='all'):
        if arm_side == 'left':
            return self.left_arm.get_joint_position(joint_indices)
        elif arm_side == 'right':
            return self.right_arm.get_joint_position(joint_indices)
        elif arm_side == 'all':
            joint_num = joint_indices.shape[0]
            return np.concatenate(
                [
                    self.left_arm.get_joint_position(joint_indices[:joint_num]),
                    self.right_arm.get_joint_position(joint_indices[joint_num:])
                ]
            )
        else:
            raise ValueError(f"Unknown arm_side: {arm_side}")

    def get_ee_pose(self, arm_side='all'):
        if arm_side == 'left':
            return self.left_arm.get_ee_pose()
        elif arm_side == 'right':
            return self.right_arm.get_ee_pose()
        elif arm_side == 'all':
            return np.concatenate(
                [
                    self.left_arm.get_ee_pose(),
                    self.right_arm.get_ee_pose()
                ]
            )
        else:
            raise ValueError(f"Invalid arm_side: {arm_side}")
        
    def get_gripper_width(self,arm_side='all'):
        if arm_side == 'left':
            return self.left_arm.get_gripper_width()
        elif arm_side == 'right':
            return self.right_arm.get_gripper_width()
        elif arm_side == 'all':
            return np.array(
                [
                     self.left_arm.get_gripper_width(), self.right_arm.get_gripper_width()
                ]
            )
        else:
            raise ValueError(f"Invalid arm_side: {arm_side}")
        
    def get_states(self,arm_side='all'):
        if arm_side == 'left':
            return self.left_arm.get_states()
        elif arm_side == 'right':
            return self.right_arm.get_states()
        elif arm_side == 'all':
            left_arm_states = self.left_arm.get_states()
            right_arm_states = self.right_arm.get_states()
            states = {
                'joint_pos': np.concatenate( [ left_arm_states['joint_pos'] ,right_arm_states['joint_pos'] ] ),
                'ee_pose': np.concatenate( [ left_arm_states['ee_pose'] ,right_arm_states['ee_pose'] ] ),
                'gripper_width': np.array( [ left_arm_states['gripper_width'],  left_arm_states['gripper_width']])
            }
            return states
        else:
            raise ValueError(f"Invalid arm_side: {arm_side}")
        
    def get_sensor_data(self,arm_side='all'):
        if arm_side == 'left':
            return self.left_arm.get_sensor_data()
        elif arm_side == 'right':
            return self.right_arm.get_sensor_data()
        elif arm_side == 'all':
            sensor_data_left = self.left_arm.get_sensor_data()
            sensor_data_right =  self.right_arm.get_sensor_data()
            merged_data = {}
            for name,data in sensor_data_left.items():
                merged_data[name] = data
            for name,data in sensor_data_right.items():
                merged_data[name] = data
            return merged_data
        else:
            raise ValueError(f"Invalid arm_side: {arm_side}")
        
    def get_trajectory(self):
        return self.controller.get_trajectory()


    def set_trajectory(self,trajectory,arm_side):
        if arm_side == 'left':
            return self.left_arm.set_trajectory(trajectory)
        elif arm_side == 'right':
            return self.right_arm.set_trajectory(trajectory)
        else:
            raise ValueError(f"Invalid arm_side: {arm_side}")

    def compute_action(self,data,arm_side="all"):
        if arm_side == 'left':
            return self.left_arm.compute_action(data)
        elif arm_side == 'right':
            return self.right_arm.compute_action(data)
        elif arm_side == 'all':
            action = self.controller.forward(data)
            return action
        else:
            raise ValueError(f"Invalid arm_side: {arm_side}")
        

    def get_sensors(self, arm_side):
        if arm_side == 'left':
            return self.left_arm.get_sensors()
        elif arm_side == 'right':
            return self.right_arm.get_sensors()
        else:
            raise ValueError(f"Invalid arm_side: {arm_side}")
        
