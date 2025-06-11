# 将数据的格式转为 UMI 需要的 Zarr 格式
import zarr
import numpy as np
import pandas as pd
import os
from PIL import Image
from tqdm import tqdm  # 进度条
from natsort import natsorted

target_img_size = 224
def resize_and_center_crop(image: Image.Image, target_size=224):
    # 获取原始尺寸
    width, height = image.size

    # 计算缩放比例，使短边变为 target_size
    scale = target_size / min(width, height)
    new_width = int(width * scale)
    new_height = int(height * scale)

    # 先等比例缩放
    image = image.resize((new_width, new_height), Image.Resampling.LANCZOS)

    # 计算中心裁剪区域
    left = (new_width - target_size) // 2
    top = (new_height - target_size) // 2
    right = left + target_size
    bottom = top + target_size

    # 进行中心裁剪
    image = image.crop((left, top, right, bottom))

    return image

def debug():
    import debugpy

    # 启动调试器，指定调试端口
    debugpy.listen(5678)

    print("Waiting for debugger to attach...")

    # 在这里设置断点
    debugpy.wait_for_client()

def txt_to_numpy(file_path):
    """ 从 TXT 文件加载数据并转换为 NumPy 数组 """
    return np.loadtxt(file_path)

def csv_to_numpy(file_path):
    """ 从 CSV 文件加载数据并转换为 NumPy 数组 """
    return pd.read_csv(file_path, header=None).values

def images_to_numpy(image_dir,is_depth=False):
    """ 读取图像文件并转换为 NumPy 数组 """
    if is_depth:
        file_suffix = "_render.png"
    else:
        file_suffix = ".png"
    image_files = natsorted([f for f in os.listdir(image_dir) if f.endswith(file_suffix)])
    images = [
        np.array(resize_and_center_crop(Image.open(os.path.join(image_dir, f)),target_size=target_img_size))
        for f in tqdm(image_files, desc="Loading and resizing images")
    ]
    return np.stack(images, axis=0)

def convert_back_to_zarr(input_dir, output_zarr,epoch_name_list):

    # 创建 Zarr ZIP 存储
    store = zarr.ZipStore(output_zarr, mode="w")
    root = zarr.group(store)
    
    camera_rgb ={}
    depth_map = {}
    joint_data = {}
    epoch_len_list = []
    # ✅ 1. 读取 PNG 图像 -> Zarr (存储在 data/camera0_rgb)
    for epoch_name in epoch_name_list:
        image_dir = os.path.join(input_dir, epoch_name, "camera")
        depth_dir = os.path.join(input_dir, epoch_name, "depth_map")
        if os.path.exists(image_dir):
            print("Converting images to Zarr...")
            camera_rgb[epoch_name] = images_to_numpy(image_dir)
            depth_map[epoch_name] = images_to_numpy(depth_dir,is_depth=True)

        # ✅ 2. 读取 CSV -> Zarr（数据存入 data/）
        csv_dir = os.path.join(input_dir, epoch_name, "data")
        csv_files = {
            "robot0_eef_position": "robot0_eef_position.txt",
            "robot0_action_eef_position": "robot0_action_eef_position.txt",
            "robot0_eef_rot_axis_angle": "robot0_eef_rot_axis_angle.txt",
            "robot0_action_eef_rot_axis_angle": "robot0_action_eef_rot_axis_angle.txt",
            "robot0_end_orientation": "robot0_end_orientation.txt",
            "robot0_end_position": "robot0_end_position.txt",
            "robot0_gripper_width": "robot0_gripper_width.txt",
            "robot0_action_gripper_width": "robot0_action_gripper_width.txt",
            "robot0_init_orientation": "robot0_init_orientation.txt",
            "robot0_init_position": "robot0_init_position.txt"
        }

        print("Converting CSV to Zarr (data/)...")
        dict = {}
        for key, filename in csv_files.items():
            file_path = os.path.join(csv_dir, filename)
            if os.path.exists(file_path):
                data = txt_to_numpy(file_path)
                dict[key] = data
        
        # ✅ 3. 合并  robot0_end_orientation  和 robot0_end_position  以及  robot0_init_orientation 和 robot0_init_position
        epoch_len = dict['robot0_eef_position'].shape[0]
        if len(epoch_len_list) == 0:
            epoch_len_list.append(epoch_len)
        else:
            epoch_len_list.append(epoch_len_list[-1] + epoch_len)

        robot0_demo_start_pose  = np.concatenate([
            np.tile(dict['robot0_init_position'], (epoch_len, 1)),
            np.tile(dict['robot0_init_orientation'], (epoch_len, 1))
        ], axis=-1)

        robot0_demo_end_pose = np.concatenate([
            np.tile(dict['robot0_end_position'], (epoch_len, 1)),
            np.tile(dict['robot0_end_orientation'], (epoch_len, 1))
        ], axis=-1)

        dict['robot0_demo_start_pose'] = robot0_demo_start_pose
        dict['robot0_demo_end_pose'] = robot0_demo_end_pose

        del dict['robot0_init_orientation'], dict['robot0_init_position'], dict['robot0_end_position'],dict['robot0_end_orientation']
        
        # 拼接 action
        dict['robot0_action'] = np.concatenate( [ dict['robot0_action_eef_position'], dict['robot0_action_eef_rot_axis_angle'], np.expand_dims(dict['robot0_action_gripper_width'],axis=1) ],axis=-1)
        joint_data[epoch_name] = dict 

    # ✅ 4. 合并所有 epoch
    key_order = epoch_name_list
    camera_rgb_ = np.concatenate([camera_rgb[key] for key in key_order], axis=0)
    depth_map_ = np.concatenate([depth_map[key] for key in key_order], axis=0)
    robot0_demo_start_pose_ = np.concatenate([joint_data[key]['robot0_demo_start_pose'] for key in key_order], axis=0)
    robot0_demo_end_pose_ = np.concatenate([joint_data[key]['robot0_demo_end_pose'] for key in key_order], axis=0)
    robot0_eef_position_ = np.concatenate([joint_data[key]['robot0_eef_position'] for key in key_order], axis=0)

    robot0_eef_rot_axis_angle_ = np.concatenate([joint_data[key]['robot0_eef_rot_axis_angle'] for key in key_order], axis=0)

    robot0_gripper_width_ = np.concatenate([joint_data[key]['robot0_gripper_width'] for key in key_order], axis=0)

    robot0_action = np.concatenate([joint_data[key]['robot0_action'] for key in key_order], axis=0)

    assert camera_rgb_.shape[0] == robot0_eef_position_.shape[0]

    # ✅ 5. 开始写入 Zarr 文件
    root.create_dataset("data/camera0_rgb", data=camera_rgb_, dtype=np.uint8, chunks=(1, 224, 224, 3))
    root.create_dataset("data/depth0_rgb", data=depth_map_, dtype=np.uint8, chunks=(1, 224, 224, 3))
    
    root.create_dataset(f"data/robot0_demo_start_pose", data=robot0_demo_start_pose_, dtype=data.dtype, chunks=True)
    root.create_dataset(f"data/robot0_demo_end_pose", data=robot0_demo_end_pose_, dtype=data.dtype, chunks=True)
    root.create_dataset(f"data/robot0_eef_pos", data=robot0_eef_position_, dtype=data.dtype, chunks=True)
    root.create_dataset(f"data/robot0_eef_rot_axis_angle", data=robot0_eef_rot_axis_angle_, dtype=data.dtype, chunks=True)
    root.create_dataset(f"data/robot0_gripper_width", data=np.expand_dims(robot0_gripper_width_, axis=-1), dtype=data.dtype, chunks=True)

    root.create_dataset(f"data/action", data=robot0_action, dtype=data.dtype, chunks=True)


    print(np.expand_dims(robot0_gripper_width_, axis=-1).shape)
    root.create_dataset(f"meta/episode_ends", data=np.array(epoch_len_list), dtype=np.int32, chunks=False)

    store.close()
    print(f"Zarr reconstruction completed: {output_zarr}")

if __name__ == "__main__":
    input_dir = "./data_collect"  # CSV 和 PNG 文件所在的目录
    output_zarr = "./datasets/train_data_with_depth.zarr.zip"
    # debug()
    epoch_nums = 52
    # excluded_epochs = {7, 15, 24,29,37, 39}  # 需要排除的epoch
    excluded_epochs = {}
    epoch_name = [f"epo_{i}" for i in range(epoch_nums) if i not in excluded_epochs]
    convert_back_to_zarr(input_dir, output_zarr,epoch_name)

