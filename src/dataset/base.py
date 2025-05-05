import os
import h5py
import numpy as np

class BaseDataset:
    def __init__(self,dataset_dir):
        self.dataset_dir = dataset_dir
        self.data = None

class ACTDataset(BaseDataset):
    def __init__(self,episode_nums = 0,is_degree =False,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.episode_ids = list(range(episode_nums))
        self.is_degree = is_degree # if is_degree is true, the joint position in dataset is in degree
    
    def __len__(self):
        return len(self.episode_ids)
    
    def __getitem__(self, index,start_ts=0):
        episode_id = self.episode_ids[index]
        dataset_path = os.path.join(self.dataset_dir, f'episode_{episode_id}.hdf5')
        with h5py.File(dataset_path, 'r') as root:
            original_action_shape = root['/action'].shape
            episode_len = original_action_shape[0]
            qpos_data = root['/observations/qpos'][start_ts:]
            action_data = root['/action'][start_ts:]

            if self.is_degree:
                # 将角度转换为弧度，同时对 gripper width 去归一化
                for i in range(qpos_data.shape[0]):
                    gripper_width = qpos_data[i][-1]
                    max_width = 0.04
                    qpos_data[i] = np.deg2rad(qpos_data[i])
                    qpos_data[i] = np.concatenate( [qpos_data[i][:-1], [gripper_width * max_width]] )

                    action_data[i] = np.deg2rad(action_data[i])
                    action_data[i] = np.concatenate( [action_data[i][:-1], [gripper_width * max_width]] )
                

            cam_name = 'front'
            image_data = root[f'/observations/images/{cam_name}'][start_ts]
         
       
        # channel last
        # image_data = torch.einsum('k h w c -> k c h w', image_data)

        # normalize image and change dtype to float
        # image_data = image_data / 255.0
        # action_data = (action_data - self.norm_stats["action_mean"]) / self.norm_stats["action_std"]
        # qpos_data = (qpos_data - self.norm_stats["qpos_mean"]) / self.norm_stats["qpos_std"]

        return {
            'image': image_data,
            'qpos': qpos_data,
            'action': action_data
        }