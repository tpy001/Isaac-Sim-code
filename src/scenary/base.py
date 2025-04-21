from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import os
import omni.usd


class BaseScenary:
    def __init__(self, usd_file_path):
        self.usd_file_path = usd_file_path
        self.world = None
        self.stage = None
        self.loaded = False

    def load_stage(self):
        if self.loaded:
            return
        if not os.path.isfile(self.usd_file_path):
            raise FileNotFoundError(f"USD file not found: {self.usd_file_path}")
        self.world = World(set_defaults=True)
        add_reference_to_stage(self.usd_file_path, "/World")
        self.stage = omni.usd.get_context().get_stage()
        self.loaded = True

    def step(self,render=True):
        self.world.step(render=render)

    def reset(self):
        self.world.reset()