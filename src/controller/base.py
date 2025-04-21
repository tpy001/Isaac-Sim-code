import numpy as np
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
