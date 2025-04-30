import numpy as np
from omegaconf.listconfig import ListConfig
from omegaconf.dictconfig import DictConfig
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, Gf
from omni.isaac.dynamic_control import _dynamic_control
dc = _dynamic_control.acquire_dynamic_control_interface()
from omni.isaac.core.prims import XFormPrim

def debug():
    import debugpy

    # 启动调试器，指定调试端口
    debugpy.listen(5679)

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
    





def quaternion_to_rotvec(quat):
    """
    将四元数 (w, x, y, z) 转换为旋转向量 (Rodrigues 旋转公式).
    
    参数:
        quat (array-like): 长度为 4 的数组，表示四元数 (w, x, y, z).
    
    返回:
        np.ndarray: 旋转向量 (3D).
    """
    # scipy 需要 [x, y, z, w] 格式，因此调整顺序
    rot = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])
    return rot.as_rotvec()



def set_prim_transform(stage, prim_path, translate=None, rotate=None, scale=None):
    """
    修改指定 USD 元素的局部变换。
    
    参数：
        stage (Usd.Stage)：当前打开的 USD Stage。
        prim_path (str)：元素路径，例如 "/World/Cube"。
        translate (tuple or list of 3 floats, optional)：平移量 (x, y, z)。
        rotate (tuple or list of 3 floats, optional)：旋转角度 (X, Y, Z)，单位：度。
        scale (tuple or list of 3 floats, optional)：缩放比例 (X, Y, Z)。
    """
    prim = stage.GetPrimAtPath(prim_path)

    if not prim or not prim.IsValid():
        print(f"未找到 {prim_path}，请检查路径是否正确！")
        return
    
    xprim = XFormPrim(prim_path=prim_path)
    xprim.set_world_pose(translate,rotate)

    

