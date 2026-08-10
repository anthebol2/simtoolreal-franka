# Franka + right SharPa deployment

This fork replaces the paper's KUKA iiwa14 + left SharPa with a **Franka Panda +
right SharPa** (asset `assets/urdf/franka_right_sharpa_description/`). All
robot-specific deployment constants live in one place — the `RobotProfile`
dataclass in
[`isaacgymenvs/utils/observation_action_utils_sharpa.py`](../isaacgymenvs/utils/observation_action_utils_sharpa.py)
(`FRANKA_RIGHT_SHARPA_PROFILE` / `KUKA_LEFT_SHARPA_PROFILE`). Every deployment
node takes `--robot franka_right_sharpa` (the default) or
`--robot kuka_left_sharpa`.

Key Franka-specific facts (must match training, see `env.py` `use_franka` branches):

| item | value |
|---|---|
| robot base | 0.65 m from the table center-line (world y), `T_W_R = translate(0, 0.65, 0)` |
| table | top at z = 0.53 m (0.475 × 0.4 × 0.3 box, `tableResetZ = 0.38`) |
| arm ROS ns / joints | `/franka/joint_cmd`, `/franka/joint_states`; `panda_joint1..7` |
| hand ROS ns | `/sharpa/*`, positional `joint_0.0 .. joint_21.0` (canonical thumb→pinky order) |
| HOME arm pose | `[-0.6242, -0.4272, -0.7214, -1.1105, -0.1649, 0.9929, 0.1666]` |
| palm body / offset | `panda_panda_link7`, offset `[0.1657, -0.0054, -0.1805]` |
| rate limiter | 0.8 × Panda speed limits `[2.175 x4, 2.61 x3]` rad/s (see caveat below) |

**Velocity caveat:** policies trained before the sim velocity-limit fix can
command rare speed spikes above the real Franka limits. The
`franka_robot_node.py` rate limiter absorbs these (expect occasional limiter
warnings rather than Franka reflex stops), but the clean fix is retraining with
real-spec velocity limits in the URDF.

## Hardware-machine installation

Prerequisites on the machine connected to the robot:
- Ubuntu 20.04 + ROS1 Noetic (native), or any Ubuntu + RoboStack (conda) ROS1.
- For the Franka: a **realtime kernel** (PREEMPT_RT) is required by
  `libfranka`; install `franka_ros` + `libfranka` per the Franka FCI docs.
- The SharPa SDK (set `SHARPA_SDK_PATH` env var to its `python/` directory).
- This repo, installed per [isaacgym_installation.md](isaacgym_installation.md)
  ("Sim2Real Env" section). Isaac Gym itself is only needed for sim2sim, not
  for real deployment.

`franka_ros` controller: the robot node publishes `std_msgs/Float64MultiArray`
position targets to a `position_controllers/JointGroupPositionController`
(default topic `/position_joint_position_controller/command`) and reads
`/franka_state_controller/joint_states`. Add to your `franka_control` config:

```yaml
position_joint_position_controller:
  type: position_controllers/JointGroupPositionController
  joints: [panda_joint1, panda_joint2, panda_joint3, panda_joint4,
           panda_joint5, panda_joint6, panda_joint7]
```

## Node graph (Sim2Real)

Run each in its own terminal (all repo nodes from the repo root, `.venv` or the
ROS env as appropriate):

```bash
# 1. Arm driver (Franka control PC)
roslaunch franka_control franka_control.launch robot_ip:=<ROBOT_IP>
python deployment/franka_robot_node.py                # bridge + rate limiter

# 2. Hand driver
SHARPA_SDK_PATH=/path/to/SharpaWaveSDK/python python deployment/sharpa_node.py

# 3. Perception (FoundationPose fork, separate env — see its README)
python live_tracking_with_ros.py --mesh_path <mesh.obj> --calibration <T_RC.txt>

# 4. Visualization (before running the policy)
python deployment/visualization_node.py --object_name claw_hammer

# 5. Home the robot
python deployment/home_robot.py

# 6. Goal pose node
python deployment/goal_pose_node.py --object_category hammer \
    --object_name claw_hammer --task_name swing_down

# 7. RL policy node
python deployment/rl_policy_node.py --policy_path <policy_dir> \
    --object_name claw_hammer
```

`<policy_dir>` must contain `config.yaml` + `model.pth` (e.g. copy
`runs/<run>/config.yaml` and the best checkpoint from training).

## Bring-up ladder (do these in order)

1. **No-ROS sim2sim** (any machine with Isaac Gym): validates the deployment
   obs/action code against the training env, including an obs parity check:
   ```bash
   PRINT_OBS_DIFF=1 python deployment/isaac/isaac_env_no_ros.py \
       --config_path <run>/config.yaml --checkpoint_path <model.pth> \
       --object_category hammer --object_name claw_hammer \
       --task_name swing_down --headless --device cpu
   ```
   Expect `[OBS_DIFF] ... max=~3e-05`. (Validated 2026-08-10 with the
   overnight Franka checkpoint.)
2. **ROS graph without hardware**: `fake_robot_node.py` + `fake_perception_node.py`
   + viz + goal + policy nodes — validates all topics/message plumbing.
   `franka_robot_node.py --dry_run` can stand in for the fake robot node to also
   exercise the rate limiter.
3. **Open-loop replay on the real arm** (`deployment/replay_trajectory.py`, slow
   factor >= 3) — validates the driver chain with no policy in the loop.
4. **Closed-loop policy**, starting from `home_robot.py`, with the rate limiter
   at a conservative `--vel_limit_fraction 0.5` for the first runs.

## Physical setup checklist

- Franka base bolted 0.65 m from the table center-line, base plate at table-leg
  floor height (z = 0 convention), table top at 0.53 m above the floor plane.
- Right SharPa mounted via the connector + wrist interface exactly as in
  `franka_sharpa/franka_sharpa.urdf` (this is what training simulated).
- Camera rigidly mounted; hand-eye calibration saved as `T_RC` for the
  perception node.
- Franka FCI enabled, external activation device (e-stop) within reach.
