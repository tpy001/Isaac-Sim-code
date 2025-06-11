source ../catkin_ws/devel/setup.bash
python3 scripts_sim/inference_umi_with_depth.py  --simple_render False --ckpt_path /home/pinkman/projects/dp_depth/umi/data/outputs/2025.06.02/21.31.12_train_diffusion_unet_timm_umi/checkpoints/best.ckpt --n_action_steps 4
# python3 scripts_sim/inference_umi.py --ckpt_path ckpt/stack_cube_simv3.ckpt --n_action_steps 4
