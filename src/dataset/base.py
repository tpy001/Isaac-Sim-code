import os
import h5py
import numpy as np

class BaseDataset:
    def __init__(self,dataset_dir):
        self.dataset_dir = dataset_dir
        self.data = None

class ACTDataset(BaseDataset):
    def __init__(self,episode_nums = 0,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.episode_ids = list(range(episode_nums))
    
    def __len__(self):
        return len(self.episode_ids)
    
    def __getitem__(self, index):
        episode_id = self.episode_ids[index]
        dataset_path = os.path.join(self.dataset_dir, f'episode_{episode_id}.hdf5')
        with h5py.File(dataset_path, 'r') as root:
            original_action_shape = root['/action'].shape
            episode_len = original_action_shape[0]
            start_ts = 0
            qpos_data = root['/observations/qpos'][start_ts]
            cam_name = 'front'
            image_data = root[f'/observations/images/{cam_name}'][start_ts]
         
            action_data = root['/action'][start_ts:]
       
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