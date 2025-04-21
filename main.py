import hydra
from src.utils import debug

from omegaconf import DictConfig

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

@hydra.main(config_path="./configs", config_name="stack_cube_act",version_base="1.3.2")
def main(cfg: DictConfig):
    # 根据配置创建场景
    scenary_config = cfg.scenary
    scenary = hydra.utils.instantiate(scenary_config)
    print(f"Scenary loaded from {scenary.usd_file_path}")

    # 根据配置创建传感器
    sensor_config = cfg.sensor
    sensors = hydra.utils.instantiate(sensor_config)
    print("Sensor initialized")

    # 根据配置创建机器人
    robot_config = cfg.robot
    robot = hydra.utils.instantiate(robot_config)
    print(f"Robot initialized at {robot.robot_prim_path}")

    # 创建控制器（如果有的话）
    controller_config = cfg.controller
    controller = hydra.utils.instantiate(controller_config)
    print("Controller initialized")
  
    # 创建任务
    task_config = cfg.task
    task = hydra.utils.instantiate(task_config,scenary=scenary,robot=robot,controller=controller,sensors=sensors)

    task.run(simulation_app)


if __name__ == "__main__":
    debug()
    main()