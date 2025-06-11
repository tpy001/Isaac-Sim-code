"""
Usage:
Training:
python train.py --config-name=train_diffusion_lowdim_workspace
"""
import os

# 设置 HTTP 和 HTTPS 代理
os.environ['HTTP_PROXY'] = 'http://127.0.0.1:7890'
os.environ['HTTPS_PROXY'] = 'http://127.0.0.1:7890'

# import debugpy

# # 配置调试服务器
# debugpy.listen(5678)  # 监听端口 5678
# print("Waiting for debugger to attach...")
# debugpy.wait_for_client()  # 等待调试器连接（可选）

import sys
# use line-buffering for both stdout and stderr
sys.stdout = open(sys.stdout.fileno(), mode='w', buffering=1)
sys.stderr = open(sys.stderr.fileno(), mode='w', buffering=1)

import hydra
from omegaconf import OmegaConf
import pathlib
from diffusion_policy.workspace.base_workspace import BaseWorkspace

# allows arbitrary python code execution in configs using the ${eval:''} resolver
OmegaConf.register_new_resolver("eval", eval, replace=True)

@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.joinpath(
        'diffusion_policy','config'))
)
def main(cfg: OmegaConf):
    # resolve immediately so all the ${now:} resolvers
    # will use the same time.

    # import debugpy

    # # 启动调试器，指定调试端口
    # debugpy.listen(5678)

    # print("Waiting for debugger to attach...")

    # # 在这里设置断点
    # debugpy.wait_for_client()

    OmegaConf.resolve(cfg)

    cls = hydra.utils.get_class(cfg._target_)
    workspace: BaseWorkspace = cls(cfg)
    workspace.run()

if __name__ == "__main__":
    main()
