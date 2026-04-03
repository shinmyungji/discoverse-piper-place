#!/usr/bin/env bash
set -e

source ~/miniconda3/etc/profile.d/conda.sh
conda activate discoverse
cd "$(dirname "$0")/.."

python examples/tasks_airbot_play/piper_place_block_random_eval.py \
  --num_eval_episodes 10 --max_steps 350 \
  --policy_server tcp://127.0.0.1:5555 \
  --demo_like --demo_action_repeat 3 \
  --demo_hold_wait_s 0.6 --demo_grip_close_thresh 0.016 --demo_move_speed 0.75 \
  --print_every 10 \
  --image_transport raw
