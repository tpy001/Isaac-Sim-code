import cv2
import torch
import numpy as np
from depth_anything_v2.dpt import DepthAnythingV2

DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
}

encoder = 'vitb'  # or 'vits', 'vitb', 'vitg'

model = DepthAnythingV2(**model_configs[encoder])
model.load_state_dict(torch.load(f'/home/pinkman/projects/Depth-Anything-V2/checkpoints/depth_anything_v2_{encoder}.pth', map_location='cpu'))
model = model.to(DEVICE).eval()

# 读取图像
raw_img = cv2.imread('/home/pinkman/projects/Isaac-Sim-code/data_collect/epo_0/camera/camera_output_56.png')

# 预测深度
depth = model.infer_image(raw_img)  # HxW raw depth map in numpy 480,640,3

def visualize_depth(depth, cmap=cv2.COLORMAP_MAGMA):
    """
    将深度图转换为可视化的彩色图像
    
    参数:
        depth (np.ndarray): 深度图，形状为(H, W)
        cmap (int): OpenCV颜色映射类型
        
    返回:
        np.ndarray: 可视化的深度图像，形状为(H, W, 3)
    """
    # 归一化深度值到0-255范围
    depth_min = depth.min()
    depth_max = depth.max()
    normalized_depth = (depth - depth_min) / (depth_max - depth_min + 1e-8)
    normalized_depth = (normalized_depth * 255).astype(np.uint8)
    
    # 应用颜色映射
    colored_depth = cv2.applyColorMap(normalized_depth, cmap)
    
    return colored_depth

# 可视化深度图
colored_depth = visualize_depth(depth)  # (480, 640, 3)

cv2.imwrite('Depth-Anything-V2/outputs/ori.png', depth)

# 保存原始深度图（作为浮点型PNG）
output_depth_path = 'Depth-Anything-V2/outputs/depth.png'
# 确保深度值在合理范围内，避免保存时出现问题
scaled_depth = (depth * 1000).astype(np.uint16)  # 缩放以保留精度
cv2.imwrite(output_depth_path, scaled_depth)
print(f"原始深度图已保存至: {output_depth_path}")

# 保存可视化的深度图
output_vis_path = 'Depth-Anything-V2/outputs/depth_vis.png'
cv2.imwrite(output_vis_path, colored_depth)
print(f"可视化深度图已保存至: {output_vis_path}")

# 可选: 保存叠加了深度信息的原始图像
overlay = cv2.addWeighted(cv2.cvtColor(raw_img, cv2.COLOR_BGR2RGB), 0.7, colored_depth, 0.3, 0)
output_overlay_path = 'Depth-Anything-V2/outputs/overlay.png'
cv2.imwrite(output_overlay_path, overlay)
print(f"叠加图像已保存至: {output_overlay_path}")
