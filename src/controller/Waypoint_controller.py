import numpy as np
from scipy.spatial.transform import Slerp,Rotation
from .ROS_controller_IK import ROSControllerIK
from .base import BaseController

class WaypointController(ROSControllerIK):
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

    def get_trajectory(self):
        # 获取设定的轨迹
        timestep = self.current_time_step
        target_pos = self.trajectory[timestep]['position']
        target_ori = self.trajectory[timestep]['orientation']
        return np.concatenate( [target_pos, target_ori] )
        
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
        CLOSE = 0.01
        interpolate = lambda alpha: alpha * OPEN + (1 - alpha) * CLOSE

        # Return actions and gripper states
        return  np.append( action.joint_positions,interpolate(target['gripper']))
    
class DualArmWayPointController(BaseController):
    def __init__(self, left_arm_controller=None, right_arm_controller=None):
        self.left_arm_controller = left_arm_controller
        self.right_arm_controller = right_arm_controller
    
    def spawn(self,left_arm,right_arm):
        self.left_arm_controller = left_arm.controller
        self.right_arm_controller = right_arm.controller

    def is_done(self):
        if self.left_arm_controller.is_done() and self.right_arm_controller.is_done():
            return True
        else:
            return False
        
    def set_trajectory(self,left_waypoints,right_way_points):
        self.left_arm_controller.set_trajectory(left_waypoints)
        self.right_arm_controller.set_trajectory(right_way_points)

    def get_trajectory(self):
        left = self.left_arm_controller.get_trajectory()
        right = self.right_arm_controller.get_trajectory()
        return np.concatenate( [left,right])
    
    def reset(self):
        self.left_arm_controller.reset()
        self.right_arm_controller.reset()

    def forward(self,observation):
        left_action = self.left_arm_controller.forward(observation)
        right_action = self.right_arm_controller.forward(observation)
        return np.concatenate( [left_action, right_action]) 