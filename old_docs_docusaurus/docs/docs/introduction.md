---
sidebar_position: 1
slug: /
---

# LeIsaac

<video
  controls
  src="https://github.com/user-attachments/assets/763acf27-d9a9-4163-8651-3ba0a6a185d7"
  style={{ width: '100%', maxWidth: '960px', borderRadius: '8px' }}
/>

LeIsaac provides teleoperation and imitation-learning workflows in
[IsaacLab](https://isaac-sim.github.io/IsaacLab/main/index.html) for
[LeRobot](https://github.com/huggingface/lerobot)-style robots. It supports
simulation teleoperation, data collection, dataset conversion, scripted data
generation, policy training, and policy evaluation.

## What You Can Build

- Control SO101 Follower, Bi-Arm SO101 Follower, and LeKiwi robots in IsaacLab.
- Teleoperate with SO101 Leader, Bi-SO101 Leader, keyboard, gamepad, or remote ZMQ streaming.
- Record demonstrations as HDF5 or directly as LeRobot Dataset episodes.
- Generate trajectories programmatically with state-machine scripted policies.
- Evaluate policies from GR00T, LeRobot, and OpenPI in supported LeIsaac tasks.

## Visual Overview

| Custom task simulation | Custom scene asset | Teleoperation target frame |
| :---: | :---: | :---: |
| ![SO101 custom task simulation](/img/tutorials/custom_task_sim.png) | ![Custom Isaac Sim scene](/img/tutorials/custom_scene_usd.png) | ![Target frame for teleoperation](/img/devices/teleop_info/target_frame.jpg) |

| Single-Arm SO101 | Bi-Arm SO101 | LeKiwi |
| :---: | :---: | :---: |
| ![Single-arm SO101 Follower](/img/robots/single_arm_so101.png) | ![Bi-arm SO101 Follower](/img/robots/bi_arm_so101.png) | ![LeKiwi robot](/img/robots/lekiwi.png) |

## Workflow

1. Install LeIsaac, IsaacLab, and the optional policy dependencies you need.
2. Prepare robot and scene assets under `assets/`.
3. Run teleoperation or scripted data generation in IsaacLab.
4. Record data as HDF5 or in LeRobot Dataset format.
5. Train, convert, and evaluate policies in simulation or on real hardware.

## Start Here

- [Installation and setup](/docs/getting_started/installation)
- [Teleoperation](/docs/getting_started/teleoperation)
- [Dataset replay](/docs/getting_started/dataset_replay)
- [Policy training and inference](/docs/getting_started/policy_support)
- [Available robots](/resources/available_robots)
- [Available environments](/resources/available_env)
- [Available devices](/resources/available_devices)
- [Available policies](/resources/available_policy)
