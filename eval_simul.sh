#!/usr/bin/env bash
set -euo pipefail

export LEISAAC_DESK_SCENE_USD=/home/rvi/Desktop/leisaac/assets/scenes/desk_with_orange/scene_debug.usd
export LEISAAC_ROBOT_AUTO_BBOX_OFFSET=0
export LEISAAC_ROBOT_ROOT_Z_OFFSET=-0.035
export LEISAAC_ROBOT_ORIENTATION_SOURCE=facing_marker_orientation_yaw

# 터미널 2: Isaac Sim 평가
/home/rvi/anaconda3/envs/leisaac/bin/python scripts/evaluation/policy_inference.py \
  --task=LeIsaac-SO101-DeskPickOrange-v0 \
  --policy_type=lerobot-pi05 \
  --policy_checkpoint_path=seunghoney/orange \
  --policy_host=127.0.0.1 \
  --policy_port=8080 \
  --policy_language_instruction="Pick the orange from the desk and put it into the plate" \
  --policy_action_horizon=10 \
  --step_hz=30 \
  --eval_rounds=10 \
  --episode_length_s=40 \
  --device=cuda \
  --enable_cameras
