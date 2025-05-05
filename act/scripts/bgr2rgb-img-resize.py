import h5py
import cv2
import numpy as np
import os
from glob import glob

def resize_and_convert_bgr_to_rgb(img, target_size):
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(img_rgb, target_size, interpolation=cv2.INTER_AREA)
    return resized

def process_hdf5_images(input_path, output_path, target_size=(224, 224)):
    with h5py.File(input_path, 'r') as f_in:
        with h5py.File(output_path, 'w') as f_out:
            # 拷贝除 observations/images/front 以外的所有内容
            def recursive_copy(name, obj):
                if isinstance(obj, h5py.Dataset):
                    if name == 'observations/images/front':
                        return  # 跳过图像，后面单独处理
                    f_out.create_dataset(name, data=obj[()], compression="gzip")
                elif isinstance(obj, h5py.Group):
                    f_out.create_group(name)

            f_in.visititems(recursive_copy)

            # 检查图像路径
            if "observations/images/front" not in f_in:
                print(f"Skipped {input_path}: key 'observations/images/front' not found.")
                return

            images = f_in["observations/images/front"][:]
            print(f"Processing {input_path} -> {output_path}, shape: {images.shape}")

            processed_images = [resize_and_convert_bgr_to_rgb(img, target_size) for img in images]
            processed_images = np.stack(processed_images)

            # 确保 group 存在
            if "observations" not in f_out:
                f_out.create_group("observations")
            if "observations/images" not in f_out:
                f_out.create_group("observations/images")

            f_out.create_dataset("observations/images/front", data=processed_images, compression="gzip")

def process_directory(input_dir, output_dir, target_size=(224, 224)):
    os.makedirs(output_dir, exist_ok=True)
    hdf5_files = glob(os.path.join(input_dir, "*.h5")) + glob(os.path.join(input_dir, "*.hdf5"))

    if not hdf5_files:
        print("No HDF5 files found in the directory.")
        return

    for input_path in hdf5_files:
        filename = os.path.basename(input_path)
        output_path = os.path.join(output_dir, f"{filename}")
        process_hdf5_images(input_path, output_path, target_size)

if __name__ == "__main__":
    input_directory = "/data/tangpeiyuan/code/FastUMI_data/grasp_corn_joint_for_real_v2/" # 输入目录，修改为你的路径
    output_directory = "./output_dataset"   # 输出目录
    resolution = (640, 360)
    process_directory(input_directory, output_directory, resolution)
