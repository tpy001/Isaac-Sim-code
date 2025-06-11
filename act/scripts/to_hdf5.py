import os
import re
import numpy as np
from PIL import Image
import h5py

def natural_sort_key(s):
    """
    自然排序的关键函数，用于提取字符串中的数字部分。
    """
    return [int(text) if text.isdigit() else text.lower() for text in re.split('(\d+)', s)]

def process_episode(episode_folder, output_file):
    """
    处理单个 episode 文件夹，将其转换为 HDF5 文件。
    :param episode_folder: episode 文件夹路径
    :param output_file: 输出的 HDF5 文件路径
    """
    # 读取图像数据
    camera_folder = os.path.join(episode_folder, "camera")
    image_files = sorted(os.listdir(camera_folder), key=natural_sort_key)  # 使用自然排序
    images_data = []
    for image_file in image_files:
        image_path = os.path.join(camera_folder, image_file)
        image = Image.open(image_path)
        image_array = np.array(image)  # 将图像转换为 NumPy 数组
        images_data.append(image_array)
    images_data = np.array(images_data)

    # 读取 qpos 数据
    data_folder = os.path.join(episode_folder, "data")
    qpos_file = os.path.join(data_folder, "robot0_joint_positions.txt")
    qpos_data = np.loadtxt(qpos_file)

    # 确保图像数量和 qpos 数据行数一致
    assert len(images_data) == len(qpos_data), "图像数量和 qpos 数据行数不一致"

    # 创建 HDF5 文件
    with h5py.File(output_file, "w") as hdf5_file:
        # 创建 observations 组
        observations_group = hdf5_file.create_group("observations")
        
        # 创建 images 组
        images_group = observations_group.create_group("images")
        
        # 创建 camera 数据集
        camera_dataset = images_group.create_dataset("front", data=images_data, compression="gzip")
        
        # 创建 qpos 数据集
        qpos_dataset = observations_group.create_dataset("qpos", data=qpos_data, compression="gzip")
        
        # 创建 action 数据集（假设 action 是 qpos 的复制）
        action_dataset = hdf5_file.create_dataset("action", data=qpos_data, compression="gzip")

def process_all_episodes(base_folder, output_folder):
    """
    处理所有 episode 文件夹，生成对应的 HDF5 文件。
    :param base_folder: 包含所有 episode 文件夹的根目录
    :param output_folder: 输出 HDF5 文件的目录
    """
    # 确保输出目录存在
    os.makedirs(output_folder, exist_ok=True)

    # 获取所有 episode 文件夹
    episode_folders = sorted([os.path.join(base_folder, d) for d in os.listdir(base_folder) if d.startswith("epo_")])

    for i, episode_folder in enumerate(episode_folders):
        output_file = os.path.join(output_folder, f"episode_{i}.hdf5")
        print(f"Processing {episode_folder} -> {output_file}")
        process_episode(episode_folder, output_file)

# 示例用法
base_folder = "data_collect"  # 替换为你的根目录路径
output_folder = "dataset_test"  # 替换为你希望保存 HDF5 文件的目录
process_all_episodes(base_folder, output_folder)