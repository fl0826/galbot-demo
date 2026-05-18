cd /home/zy/Project/cook-main/openpi


CUDA_VISIBLE_DEVICES=0 XLA_PYTHON_CLIENT_MEM_FRACTION=0.95 \
uv run python scripts/serve_policy.py \
  --port 6688 \
  --default-prompt "Pick up all the trash on the ground one by one and put it into the trash can until there is no trash left on the ground." \
  --gripper-scale 0.1 \
  policy:checkpoint \
  --policy.config pi05_galbot_fullbody_nolegandhead \
  --policy.dir /home/zy/Project/model/ground/79999
