import numpy as np
from std_msgs.msg import Float64MultiArray, UInt8MultiArray,Float64, MultiArrayLayout, MultiArrayDimension
import rospy
from act_dp_service.srv import get_action
from act_dp_service.msg import RawData
import os
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver,LulaKinematicsSolver
from isaacsim.core.prims import SingleArticulation as Articulation
from scipy.spatial.transform import Slerp,Rotation


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
        gripper_width = target_action[-1]

        action, success = self.IK_forward(target_action[:3], target_action[3:7])
        if success:
            return action, gripper_width
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

class WayGenController(ROSIKController):
    def __init__(self,waypoints=None,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.waypoints = waypoints
        self.max_time_step = 0
        if waypoints is not None:
            self.trajectory = self.generate_smoothed_trajectory(waypoints)
            self.max_time_step = len(self.trajectory)
        self.current_time_step = 0
        

    def is_done(self):
        if self.current_time_step < (self.max_time_step - 1):
            return False
        else:
            return True
        
    def set_trajectory(self,waypoints):
        self.waypoints = waypoints
        self.trajectory = self.generate_smoothed_trajectory()
        self.max_time_step = len(self.trajectory)
        
    def reset(self):
        self.current_time_step = 0

    def generate_smoothed_trajectory(self):
        if not self.waypoints or 't' not in self.waypoints[0]:
            raise ValueError("Invalid waypoints format: each waypoint must have 't', 'xyz', 'quat', and 'gripper'")

        # Sort waypoints by time to ensure chronological order
        self.waypoints.sort(key=lambda x: x['t'])

        # Initialize the smoothed trajectory
        smoothed_trajectory = []

        # Interpolate between consecutive waypoints
        for i in range(len(self.waypoints) - 1):
            start_wp = self.waypoints[i]
            end_wp = self.waypoints[i + 1]

            start_time = start_wp['t']
            end_time = end_wp['t']
            duration = end_time - start_time

            if duration <= 0:
                continue  # Skip if times are not increasing

            # Number of interpolation points (e.g., 60 points per second for smoothness)
            num_points = duration  # Ensure at least 1 point

            for j in range(num_points):
                t = start_time + j
                alpha = j / num_points

                # Linear interpolation for position (xyz)
                interpolated_pos = (1 - alpha) * np.array(start_wp['xyz']) + alpha * np.array(end_wp['xyz'])

                # Convert quaternions to Quaternion objects for Slerp
                start_quat = np.array(start_wp['quat'])
                end_quat = np.array(end_wp['quat'])

                # 转换为 SciPy 的 [x, y, z, w] 格式
                start_quat = np.roll(start_quat, -1)  # [w, x, y, z] -> [x, y, z, w]
                end_quat = np.roll(end_quat, -1)

                # 创建旋转对象
                quats = np.array([start_quat, end_quat])
                times = [0, 1]  # 时间点：0 对应 start_quat，1 对应 end_quat
                slerp = Slerp(times, Rotation.from_quat(quats))

                # 插值
                alpha = np.clip(alpha, 0, 1)  # 确保 alpha 在 [0, 1]
                interpolated_quat = slerp([alpha]).as_quat()[0]

                # 转换回 [w, x, y, z] 格式（如果需要）
                interpolated_quat = np.roll(interpolated_quat, 1)

                # Linear interpolation for gripper (assuming continuous value 0 to 1)
                interpolated_gripper = (1 - alpha) * start_wp['gripper'] + alpha * end_wp['gripper']

                # Add interpolated point to trajectory
                smoothed_trajectory.append({
                    'time': t,
                    'position': interpolated_pos,  # Convert back to list for consistency
                    'orientation': interpolated_quat,  # Quaternion elements [w, x, y, z]
                    'gripper': interpolated_gripper
                })

        # Add the last waypoint to ensure the trajectory ends correctly
        smoothed_trajectory.append({
            'time': self.waypoints[-1]['t'],
            'position': np.array(self.waypoints[-1]['xyz']),
            'orientation': self.waypoints[-1]['quat'],
            'gripper': self.waypoints[-1]['gripper']
        })

        return smoothed_trajectory  # Return a single trajectory (assuming both arms follow the same path or adjust as needed)

    def forward(self,observation):
        if not self.is_initialized:
             self.articulation.initialize()
             self.is_initialized = True
             
        self.current_time_step += 1
         # Ensure trajectory exists
        if not hasattr(self, 'trajectory') or not self.trajectory:
            raise ValueError("Trajectory not initialized. Call _generate_smoothed_trajectory first.")

        # Assume self.trajectory is a tuple (left_trajectory, right_trajectory)
        trajectory = self.trajectory

        # Find the nearest trajectory points for both arms
        target = trajectory[self.current_time_step]

        # Update robot base pose
        base_pos, base_orient = self.articulation.get_world_pose()
        self.kinematics_solver.set_robot_base_pose(base_pos, base_orient)

        # Compute IK for both arms
        action, success = self.articulation_kinematics_solver.compute_inverse_kinematics(
            target['position'], target['orientation']
        )
        
        # Check for IK convergence and log warnings
        if not success:
            print(f"Warning: IK solution failed to converge - {success}")

        OPEN = 0.04 # Hard code
        CLOSE = 0.02
        interpolate = lambda alpha: alpha * OPEN + (1 - alpha) * CLOSE

        # Return actions and gripper states
        return action,interpolate(target['gripper'])
    

