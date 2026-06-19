# SO101 voltage error during scene debugging

This note separates SO101 hardware initialization failures from desk scene alignment work.

## Symptom

Running `teleop_se3_agent.py` with `--teleop_device=so101leader` can fail before teleoperation starts:

```text
RuntimeError: Failed to write 'Acceleration' on id_=6 with '254' after 1 tries.
[RxPacketError] Input voltage error!
```

## Why this is separate from scene alignment

The failure occurs while creating the SO101 leader device, before keyboard control or scene alignment debugging is needed.

Relevant path:

```text
scripts/environments/teleoperation/teleop_se3_agent.py
  -> line 263: only enters SO101 branch when --teleop_device=so101leader
  -> SO101Leader(env, port=...)
  -> source/leisaac/leisaac/devices/lerobot/so101_leader.py line 44: gripper = Motor(6, ...)
  -> SO101Leader.connect()
  -> SO101Leader.configure()
  -> FeetechMotorsBus.configure_motors()
  -> write("Acceleration", motor id 6, 254)
```

This is a hardware/serial initialization issue for motor id 6. It is independent of `assets/scenes/desk_with_orange/scene.usd`, `scene_debug.usd`, `DeskVisual`, and collider transforms.

In `--teleop_device=keyboard` mode, `teleop_se3_agent.py` creates `SO101Keyboard` at line 258 and does not instantiate `SO101Leader`, so `/dev/ttyACM0` and motor id 6 are not used.

## Scene debugging without SO101 hardware

Use keyboard mode so `SO101Leader` is never initialized:

```bash
cd /home/rvi/Desktop/leisaac

LEISAAC_DESK_SCENE_USD=/home/rvi/Desktop/leisaac/assets/scenes/desk_with_orange/scene_debug.usd \
python scripts/environments/teleoperation/teleop_se3_agent.py \
  --task=LeIsaac-SO101-DeskPickOrange-v0 \
  --teleop_device=keyboard \
  --num_envs=1 \
  --device=cuda \
  --enable_cameras
```

Optional robot pose override while aligning the reconstructed table:

```bash
LEISAAC_ROBOT_INIT_POS=1.05,-0.65,0.89
```

## Hardware checklist for later

Use this only after scene alignment debugging is done.

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

If the device is present but permission is denied:

```bash
sudo chmod 666 /dev/ttyACM0
```

Also check:

- Reconnect USB.
- Reconnect or power-cycle the SO101 leader power supply.
- Inspect the gripper motor cable for motor id 6.
- Confirm `/dev/ttyACM0` is the SO101 leader and not another serial device.
- Retry with `--recalibrate` only after the voltage/power issue is resolved.
