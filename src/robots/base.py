from omni.isaac.core.robots import Robot

class BaseRobot:
    def __init__(self, robot_prim_path):
        self.robot_prim_path = robot_prim_path

    def spawn(self,name="franka"):
        robot = Robot(prim_path=self.robot_prim_path, name=name)
        return robot
    
    def reset(self):
        pass



