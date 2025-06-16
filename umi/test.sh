source ../catkin_ws/devel/setup.bash
python3 scripts_sim/inference_umi_with_depth.py --ckpt_path ckpt/latest.ckpt --n_action_steps 4
