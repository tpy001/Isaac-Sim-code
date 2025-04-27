import numpy as np
from omegaconf.listconfig import ListConfig
from omegaconf.dictconfig import DictConfig
from scipy.spatial.transform import Rotation

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
    


class Quaternion:
    def __init__(self, elements):
        """Initialize a quaternion from a list or array [w, x, y, z]."""
        self.elements = np.array(elements, dtype=float)
        if len(self.elements) != 4:
            raise ValueError("Quaternion must have 4 components [w, x, y, z]")
        self.normalize()

    def normalize(self):
        """Normalize the quaternion to ensure it has unit length."""
        norm = np.linalg.norm(self.elements)
        if norm == 0:
            raise ValueError("Cannot normalize zero quaternion")
        self.elements /= norm
        return self

    def conjugate(self):
        """Return the conjugate of the quaternion [w, -x, -y, -z]."""
        return Quaternion([self.elements[0], -self.elements[1], -self.elements[2], -self.elements[3]])

    def dot(self, other):
        """Compute the dot product (inner product) with another quaternion."""
        if not isinstance(other, Quaternion):
            other = Quaternion(other)
        return np.sum(self.elements * other.elements)

def slerp_manual(q1, q2, t):
    """
    Perform Spherical Linear Interpolation (Slerp) between two quaternions.
    
    Args:
        q1 (Quaternion): Starting quaternion [w, x, y, z]
        q2 (Quaternion): Ending quaternion [w, x, y, z]
        t (float): Interpolation parameter between 0 and 1
    
    Returns:
        Quaternion: Interpolated quaternion
    """
    if not (0 <= t <= 1):
        raise ValueError("Interpolation parameter t must be between 0 and 1")

    # Ensure q1 and q2 are normalized Quaternions
    q1 = q1.normalize()
    q2 = q2.normalize()

    # Compute the cosine of the angle between q1 and q2
    cos_theta = q1.dot(q2)

    # Handle the case where quaternions are very close or opposite
    if abs(cos_theta) >= 1.0 - 1e-6:  # If quaternions are nearly aligned
        # Use linear interpolation if quaternions are very close
        result = Quaternion(q1.elements + t * (q2.elements - q1.elements))
        return result.normalize()

    # Ensure we take the shortest path by flipping q2 if necessary
    if cos_theta < 0:
        q2 = Quaternion(-q2.elements)  # Flip q2
        cos_theta = -cos_theta

    # Compute the angle and its sine
    theta = np.arccos(cos_theta)
    sin_theta = np.sin(theta)

    if abs(sin_theta) < 1e-6:  # Avoid division by near-zero
        return q1  # Return q1 if no rotation

    # Compute the interpolation factors
    a = np.sin((1 - t) * theta) / sin_theta
    b = np.sin(t * theta) / sin_theta

    # Interpolate
    interpolated = a * q1.elements + b * q2.elements

    return Quaternion(interpolated).normalize()


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
