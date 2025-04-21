class BaseTask:
    def __init__(self,
                 scenary,
                 robot,
                 controller,
                 sensors):
        self.scenary = scenary
        self.robot = robot
        self.controller = controller
        self.sensors = sensors

        self.reset_needed = False
        # build task
        self.build()

    def reset(self):
        self.scenary.reset()
        self.robot.reset()
        self.controller.reset()
        self.sensors.reset()

    def build(self):
        # 1. 加载场景
        self.scenary.load_stage()

        # 2. 创建机器人
        self.robot.spawn()

        # 3. 创建传感器
        self.sensors.spawn()

        # 4. 创建控制器    
        self.controller.spawn()

        # 5. 重置环境
        self.reset()


    def run(self,simulation_app):
        world = self.scenary.world
        warm_up = 100
        i = 0
        while simulation_app.is_running():
            # 推进仿真并渲染
            self.scenary.step(render=True)
            if world.is_stopped() and not self.reset_needed:
                self.reset_needed = True
            if i < warm_up:
                i += 1
                continue
            if world.is_playing():
                if self.reset_needed:
                    self.reset()
                    self.reset_needed = False
                i = i +1 

        simulation_app.close()

class StackCube(BaseTask):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        pass