import numpy as np
from std_msgs.msg import Float64
import os
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver,LulaKinematicsSolver
from isaacsim.core.prims import SingleArticulation as Articulation

from .ROS_controller import ROSController


class ROSControllerIK(ROSController):
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

        rgb_data = self.process_rgb_data(observation['robot']['sensors']['camera_rgb_wrist'])
        arm_state = self.process_proprioception(observation['robot']['states'])
        reset = Float64(data=observation['reset'])
        next_ee_pose = self.get_action_from_ros(rgb_data,arm_state,reset)
        target_action = np.array(next_ee_pose.actions.data)
        gripper_width = target_action[-1]

        action, success = self.IK_forward(target_action[:3], target_action[3:7])

        action = np.append( action.joint_positions,gripper_width)
        
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