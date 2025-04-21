from omni.isaac.sensor import Camera as IsaacCamera

class BaseSensor:
    def __init__(self):
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
    
    def reset(self):
        pass


    def spawn(self):
        camera = IsaacCamera(
            prim_path=self.camera_prim_path,  # 使用 USD 中的路径，不重新定义
            frequency = self.camera_freq,
            resolution = self.resolution,
        )
        return camera