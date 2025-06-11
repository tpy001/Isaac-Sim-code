from omni.isaac.sensor import Camera as IsaacCamera
import numpy as np

class BaseSensor:
    def __init__(self):
        pass

    def get_data(self):
        pass


class Camera(BaseSensor):
    def __init__(self,
                 camera_prim_path,
                 camera_freq,
                 resolution,
                 *args,**kwargs):
        super().__init__(*args,**kwargs)
        self.camera_prim_path = camera_prim_path
        self.camera_freq = camera_freq
        self.resolution = resolution
        self.camera = None
    
    def reset(self):
        pass


    def spawn(self):
        self.camera = IsaacCamera(
            prim_path=self.camera_prim_path,  # 使用 USD 中的路径，不重新定义
            frequency = self.camera_freq,
            resolution = (self.resolution[0],self.resolution[1]),
        )
        self.camera.initialize()
        self.camera.add_distance_to_camera_to_frame()

    def get_rgb_data(self):
        for i in range(8):  # 最多 get 8 次，还没 get 到 camera 数据则退出
            rgb_data = self.camera.get_rgb()  # 获取 RGB 数据
            if len(rgb_data) == 0:
                continue
            return rgb_data
        return None
    def get_depth_map(self):
        for i in range(8):
            depth_data = self.camera.get_current_frame()["distance_to_camera"]
            if depth_data is None:
                continue
            # depth_data[depth_data == np.inf] = 10
            return depth_data
        return None  # 8次尝试后仍无数据返回None
