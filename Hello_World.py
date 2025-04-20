# 通过 Python 脚本，修改 IsaacSim 场景中的立方体模型

# 你会看到一个手臂不断振动的机器人
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.utils.rotations import euler_angles_to_quat
from omni.isaac.sensor import Camera
from time import sleep
import threading
from PIL import Image
import os

print("Hello!")

