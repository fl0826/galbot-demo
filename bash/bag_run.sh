cd /home/zy/Project/cook-main/openpi


CUDA_VISIBLE_DEVICES=0 XLA_PYTHON_CLIENT_MEM_FRACTION=0.95 \
uv run python scripts/serve_policy.py \
  --port 6687 \
  --base-v \
  --default-prompt "Put a garbage bag in the trash can." \
  --gripper-scale 0.1 \
  policy:checkpoint \
  --policy.config pi05_galbot_fullbody_nolegandheadchassis_stage2 \
  --policy.dir /home/zy/Project/model/black_bag/V4_1/29999

