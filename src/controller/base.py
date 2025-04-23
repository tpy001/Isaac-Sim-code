import numpy as np
from std_msgs.msg import Float64MultiArray, UInt8MultiArray,Float64, MultiArrayLayout, MultiArrayDimension
import rospy
from act_dp_service.srv import get_action
from act_dp_service.msg import RawData
import os
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver,LulaKinematicsSolver
from isaacsim.core.prims import SingleArticulation as Articulation



class BaseController:
    def __init__(self):
        pass

    def spawn(self,*args,**kwargs):
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

    def process_obs(self,observation):
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
        return raw_data
    
    def forward(self,observation):
        raw_data = self.process_obs(observation)
        rospy.wait_for_service(self.ros_service_name)
        get_control_action = rospy.ServiceProxy(self.ros_service_name, get_action)
        target_action = get_control_action(raw_data)
        return np.array(target_action.actions.data)
    

class ROSIKController(ROSServiceController):
    def __init__(self,
                 ik_end_effector_name,
                 kinematics_config_dir,
                 robot_description_path,
                 urdf_path,
                 *args,**kwargs):
        super().__init__(*args,**kwargs)

        self.ik_end_effector_name = ik_end_effector_name
        self.kinematics_config_dir = kinematics_config_dir
        self.robot_description_path = robot_description_path
        self.urdf_path = urdf_path

        self.kinematics_solver = None
        self.articulation_kinematics_solver = None
        self.articulation = None
        self.is_initialized = False

    def spawn(self,robot):
        self.articulation = Articulation(robot.robot_prim_path)
        self.kinematics_solver, self.articulation_kinematics_solver = self.init_IK_controller(
            self.articulation ,
            self.ik_end_effector_name,
            self.kinematics_config_dir,
            self.robot_description_path,
            self.urdf_path
        )   

    def forward(self,observation):
        if not self.is_initialized:
             self.articulation.initialize()
             self.is_initialized = True

        raw_data = self.process_obs(observation)
        rospy.wait_for_service(self.ros_service_name)
        get_control_action = rospy.ServiceProxy(self.ros_service_name, get_action)
        next_ee_pose = get_control_action(raw_data)
        target_action = np.array(next_ee_pose.actions.data)

        action, success = self.IK_forward(target_action[:3], target_action[3:7])
        if success:
            return action
        else:
            return None

    def init_IK_controller(
        self,
        articulation,
        end_effector_name,
        kinematics_config_dir,
        robot_description_path,
        urdf_path
    ):
        # 获取 IK Controller
        kinematics_solver = LulaKinematicsSolver(
            robot_description_path= os.path.join(kinematics_config_dir,robot_description_path),
            urdf_path=  os.path.join(kinematics_config_dir,urdf_path),
        )

        articulation_kinematics_solver = ArticulationKinematicsSolver(
                articulation, kinematics_solver, end_effector_name
        )   

        return kinematics_solver,articulation_kinematics_solver
    
        
    def IK_forward(self,target_position, target_orientation): # target origentaton must be quat in wxyz
        # Track any movements of the robot base
        assert self.articulation_kinematics_solver is not None and self.kinematics_solver is not None

        robot_base_translation, robot_base_orientation = self.articulation.get_world_pose()   
        self.kinematics_solver.set_robot_base_pose(robot_base_translation, robot_base_orientation)

        action, success = self.articulation_kinematics_solver.compute_inverse_kinematics(
            target_position, target_orientation
        )
        if success:
            # articulation.apply_action(action)
            pass
        else:
            print("IK did not converge to a solution.  No action is being taken")
            
        return action, success
