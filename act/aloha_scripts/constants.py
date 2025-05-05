### Task parameters
DATA_DIR = './dataset'
# TASK_CONFIGS = {
#     'close_ricecooker_scripted':{
#         'dataset_dir': DATA_DIR + '/close_ricecooker_test_joint',
#         'num_episodes': 20,
#         'episode_len': 120,
#         'camera_names': ['front']
#     },
# }
# isaacsim中stack_cube任务配置
TASK_CONFIGS = {
    'stake_cube_scripted':{
        'dataset_dir': DATA_DIR + '/stack_cube_v3',
        'num_episodes': 51,
        'episode_len': 125,
        'camera_names': ['front']
    },
}