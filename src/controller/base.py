import numpy as np
from std_msgs.msg import Float64MultiArray, UInt8MultiArray,Float64, MultiArrayLayout, MultiArrayDimension
import rospy
from act_dp_service.srv import get_action
from act_dp_service.msg import RawData

class BaseController:
    def __init__(self):
        pass

    def spawn(self):
        pass

    def reset(self):
        pass

    def forward(self):
        pass

class RandomController(BaseController):
    def __init__(self, dof_num=7, low=-1.0, high=1.0):
        super().__init__()
        self.dof_num = dof_num
        self.low = low
        self.high = high

    def forward(self,observation):
        noise = np.random.uniform(self.low, self.high, size=self.dof_num)
        return observation['joint_pos'] + noise

class ROSServiceController(BaseController):
    def __init__(self,ros_service_name):
        super().__init__()
        self.ros_service_name = ros_service_name

    def forward(self,observation):
        init_ee_pose = observation['init_ee_pose']
        init_ee_pose = Float64MultiArray(
            data = list (
                np.concatenate([init_ee_pose[0],init_ee_pose[1]])
            )
        )

        ee_pose = observation['ee_pose']
        ee_pose = Float64MultiArray(
            data = list (
                np.concatenate([ee_pose[0],ee_pose[1]])
            )
        )

        joint_pos = observation['joint_pos']
        joint_pos = Float64MultiArray(
            data = list(joint_pos)
        )

        init_joint_pos = observation['init_joint_pos']
        init_joint_pos = Float64MultiArray(
            data = list(init_joint_pos)
        )

        gripper_width = Float64(data=observation['gripper_width'])

        reset = Float64(data=observation['reset'])

        rgb_np = observation['rgb_data']
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

        raw_data = RawData(
            ee_pose,
            joint_pos,
            init_ee_pose,
            init_joint_pos,
            gripper_width,
            rgb_msg,
            reset
        )

        rospy.wait_for_service(self.ros_service_name)
        get_control_action = rospy.ServiceProxy(self.ros_service_name, get_action)
        target_action = get_control_action(raw_data)

        return np.array(target_action.actions.data)
