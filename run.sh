#!/usr/bin/env bash
set -euo pipefail

LEISAAC_DESK_SCENE_USD=/home/rvi/Desktop/leisaac/assets/scenes/desk_with_orange/scene_debug.usd \
LEISAAC_ROBOT_AUTO_BBOX_OFFSET=0 \
LEISAAC_ROBOT_ROOT_Z_OFFSET=-0.035 \
LEISAAC_ROBOT_ORIENTATION_SOURCE=facing_marker_orientation_yaw \
/home/rvi/anaconda3/envs/leisaac/bin/python scripts/environments/teleoperation/teleop_se3_agent.py \
    --task=LeIsaac-SO101-DeskPickOrange-v0 \
    --teleop_device=so101leader \
    --port=/dev/ttyACM0 \
    --num_envs=1 \
    --device=cuda \
    --enable_cameras \
    --record \
    --use_lerobot_recorder \
    --lerobot_dataset_repo_id=leisaac/orange \
    --lerobot_dataset_fps=30 \
    --num_demos=100 \
    --resume



# LEISAAC_DESK_SCENE_USD=/home/rvi/Desktop/leisaac/assets/scenes/desk_with_orange/scene_debug.usd \
# LEISAAC_ROBOT_AUTO_BBOX_OFFSET=0 \
# LEISAAC_ROBOT_ROOT_Z_OFFSET=-0.035 \
# LEISAAC_ROBOT_ORIENTATION_SOURCE=facing_marker_orientation_yaw \
# python scripts/environments/teleoperation/teleop_se3_agent.py \
#     --task=LeIsaac-SO101-DeskPickOrange-v0 \
#     --teleop_device=keyboard \
#     --port=/dev/ttyACM0 \
#     --num_envs=1 \
#     --device=cuda \
#     --enable_cameras
