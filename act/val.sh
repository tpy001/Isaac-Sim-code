python3 imitate_episodes_sim.py \
--task_name stake_cube_scripted \
--ckpt_dir grasp_corn_chunk=20_new \
--policy_class ACT --kl_weight 10 --chunk_size 20 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 \
--num_epochs 2000  --lr 1e-5 \
--seed 0 --eval --temporal_agg  \
--open_loop 

