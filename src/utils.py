import numpy as np
from omegaconf.listconfig import ListConfig
from omegaconf.dictconfig import DictConfig
def debug():
    import debugpy

    # 启动调试器，指定调试端口
    debugpy.listen(5678)

    print("Waiting for debugger to attach...")

    # 在这里设置断点
    debugpy.wait_for_client()


def to_numpy_recursive(cfg):
    # 将 OmegaConf 里面的所有 list 转换成 numpy
    if  isinstance(cfg, dict) or isinstance(cfg, DictConfig):
        return {k: to_numpy_recursive(v) for k, v in cfg.items()}
    elif isinstance(cfg, ListConfig):
       return np.array(cfg)
    else:
        return cfg