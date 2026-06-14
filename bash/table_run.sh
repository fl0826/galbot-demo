cd /home/zy/Project/cook-main/openpi


CUDA_VISIBLE_DEVICES=0 XLA_PYTHON_CLIENT_MEM_FRACTION=0.95 \
uv run python scripts/serve_policy.py \
  --port 6686 \
  --base-v \
  --gripper-scale 0.1 \
  policy:checkpoint \
  --policy.config pi05_galbot_leg_and_arms \
  --policy.dir /home/zy/Project/model/table/v1/79999
