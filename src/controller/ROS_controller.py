import numpy as np
from std_msgs.msg import Float64MultiArray, UInt8MultiArray,Float64, MultiArrayLayout, MultiArrayDimension
import rospy
from act_dp_service.srv import get_action,get_action_bimanual
from act_dp_service.msg import SingleArmState,DualArmState

from .base import BaseController


class ROSController(BaseController):
    def __init__(self,ros_service_name):
        super().__init__()
        self.ros_service_name = ros_service_name

    def process_rgb_data(self,rgb_data):
        # 处理 rgb data
        rgb_np = rgb_data
        height, width, channels = rgb_np.shape
        layout = MultiArrayLayout(
            dim=[
                MultiArrayDimension(label="height", size=height, stride=width * channels),
                MultiArrayDimension(label="width", size=width, stride=channels),
                MultiArrayDimension(label="channel", size=channels, stride=1),
            ],
            data_offset=0
        )
        rgb_msg = UInt8MultiArray(layout=layout, data=rgb_np.flatten().tolist())
        return rgb_msg

    def process_proprioception(self,observation):
        # 处理机器人的本体数据


        ee_pose = observation['ee_pose']
        ee_pose = Float64MultiArray(
            data = list (
                    ee_pose
            )
        )

        joint_pos = observation['joint_pos']
        joint_pos = Float64MultiArray(
            data = list(joint_pos)
        )


        gripper_width = Float64(data=observation['joint_pos'][-1])
        arm_state = SingleArmState(
            ee_pose,
            joint_pos,
            gripper_width,
        )

        return arm_state
    
    def get_action_from_ros(self,rgb_data,arm_state,reset):
        rospy.wait_for_service(self.ros_service_name)
        get_control_action = rospy.ServiceProxy(self.ros_service_name, get_action)
        target_action = get_control_action(arm_state,rgb_data,reset)
        return target_action

    def forward(self,observation):
        rgb_data = self.process_rgb_data(observation['robot']['sensors']['camera_rgb_wrist'])
        arm_state = self.process_proprioception(observation['robot']['states'])
        reset = Float64(data=observation['reset'])
        target_action = self.get_action_from_ros(rgb_data,arm_state,reset)
        return np.array(target_action.actions.data)
    
class DualArmROSController(ROSController):
    def process_proprioception(self,observation):
        # 处理机器人的本体数据
        ee_length = observation["ee_pose"].shape[0] // 2
        joint_length = observation["joint_pos"].shape[0] // 2
        states = {}
        for arm_side in ["left","right"]:
            if arm_side ==  "left":
                s = slice(None, ee_length)
                s2 = slice(None, joint_length)
            else:
                s = slice(ee_length, None)
                s2 = slice(joint_length, None)

            ee_pose = observation['ee_pose'][s]
            ee_pose = Float64MultiArray(
                data = list (
                      ee_pose
                )
            )

            joint_pos = observation['joint_pos'][s2]
            joint_pos = Float64MultiArray(
                data = list(joint_pos)
            )


            gripper_width = Float64(data=observation['joint_pos'][s2][-1])
            arm_state = SingleArmState(
                ee_pose,
                joint_pos,
                gripper_width,
            )

            states[arm_side] = arm_state
        return states
     
    def get_action_from_ros(self,states,rgb_data,reset):
        rospy.wait_for_service(self.ros_service_name)
        get_control_action = rospy.ServiceProxy(self.ros_service_name, get_action_bimanual)
        target_action = get_control_action(states = states,font_camera = rgb_data,reset = reset)
        return target_action
    
    def forward(self,observation):
        rgb_data = self.process_rgb_data(observation['env_sensors']['camera_rgb_front'])
        arm_states_dict = self.process_proprioception(observation['robot']['states'])
        left_arm_state = arm_states_dict["left"]
        right_arm_states = arm_states_dict["right"]
        arm_states = DualArmState(left_arm_state,right_arm_states)
        reset = Float64(data=observation['reset'])
        target_action = self.get_action_from_ros(arm_states,rgb_data,reset)
        return np.array(target_action.actions.data)
    
  