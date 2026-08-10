# HANDOVER: Deploy SimToolReal (Franka + right SharPa) on real hardware

**Audience:** the Claude Code agent (and human) on the hardware machine that is
connected to the real Franka arm and SharPa hand. This file is your starting
point; the full reference is [docs/franka_deployment.md](docs/franka_deployment.md).

## What this is

A SimToolReal fork adapted from KUKA iiwa14 + left SharPa to **Franka Panda +
right SharPa**. A policy was trained from scratch in Isaac Gym (3072 envs,
SAPG, ~18 days on an RTX 3070, reward 58 → 1000+, ~1 success/episode at 7.5 cm
tolerance) and the entire deployment software stack has been ported and
validated on the development machine.

## Validation status (already done — do not redo unless debugging)

| check | result |
|---|---|
| Deployment obs path vs training env (32k steps, physics sim2sim) | max diff **3e-5** |
| Full ROS node graph (fake robot + fake perception + goal + policy nodes) | `/franka/joint_cmd` at 59–60 Hz, correct joint names |
| Policy arm speeds (measured, 276k samples) | p99 = 0.45 rad/s (safe); rare spikes up to sim cap — absorbed by the robot-node rate limiter |
| Isaac Gym DOF ordering, self-collision map, gains, base placement | asserted at env load + smoke-tested |

**Known caveat:** this policy (v1) was trained with unrealistic sim velocity
limits (10 rad/s). Typical motion is far below real Franka limits, but rare
spikes would exceed them; `deployment/franka_robot_node.py` rate-limits
commands to 0.8× the real limits, so expect occasional "Rate limiter active"
warnings instead of Franka reflex stops. A velocity-corrected retrain (v2) is
planned; the deployment stack is unchanged for it.

## What must be transferred from the dev machine

1. **This repository, including uncommitted work.** The Franka port exists as
   local modifications + untracked files (see `git status`): all files under
   `assets/urdf/franka_right_sharpa_description/`, `franka_sharpa/`,
   `deployment/`, `docs/franka_deployment.md`, and the modified
   `isaacgymenvs/` + `rl_games/` files. Either commit & push a branch
   (recommended: `git checkout -b franka-right-sharpa && git add -A && git commit`)
   or rsync the whole directory.
2. **The trained policy: `franka_policy_v1/`** (config.yaml + model.pth,
   ~136 MB — too big for git; scp/rsync/USB it). This is the only artifact
   that cannot be regenerated from the repo.

## Hardware-machine prerequisites (the actual remaining work)

- **Franka control PC**: realtime kernel (PREEMPT_RT), `libfranka` +
  `franka_ros` installed, FCI enabled, e-stop at hand. Add the controller from
  docs/franka_deployment.md ("position_joint_position_controller").
- **ROS1 Noetic**: native on Ubuntu 20.04, or RoboStack (conda) on newer Ubuntu
  (see `docs/isaacgym_installation.md` Sim2Real section; a working RoboStack
  recipe is in the dev machine notes — python 3.11 env with
  `-c robostack-staging ros-noetic-desktop`, then pip: torch-cpu, yourdfpy,
  viser, tyro, pytorch-kinematics, `pip install -e . --no-deps`, `pip install -e ./rl_games`).
- **SharPa SDK** for the right hand; export `SHARPA_SDK_PATH=<sdk>/python`.
- **Camera + perception**: RGB-D/stereo camera rigidly mounted; clone and set up
  the FoundationPose fork (separate env, see its README); hand-eye calibration
  `T_RC`; an `.obj` mesh (in meters) of each real tool.
- **Physical setup** (must match training): Franka base **0.65 m** from the
  table center-line; table top **0.53 m** above the robot-base floor plane;
  right SharPa mounted with the connector + wrist interface from
  `franka_sharpa/franka_sharpa.urdf`.

## Bring-up (in order — do not skip rungs)

Rungs 1–2 already passed on the dev machine; start at rung 2 to verify the
transfer, then proceed.

```bash
# Rung 2 — ROS plumbing, no hardware (repeat here to verify the transfer):
roscore
python deployment/fake/fake_robot_node.py           # or: franka_robot_node.py --dry_run
python deployment/fake/fake_perception_node.py
python deployment/goal_pose_node.py --object_category hammer --object_name claw_hammer --task_name swing_down
python deployment/rl_policy_node.py --policy_path franka_policy_v1 --object_name claw_hammer
# expect: rostopic hz /franka/joint_cmd  ->  ~60 Hz

# Rung 3 — real arm, open loop, no policy:
roslaunch franka_control franka_control.launch robot_ip:=<ROBOT_IP>
python deployment/franka_robot_node.py --vel_limit_fraction 0.5
python deployment/sharpa_node.py
python deployment/visualization_node.py --object_name claw_hammer   # watch at localhost:8080
python deployment/home_robot.py                     # slow 10 s interpolation to HOME
# then optionally: deployment/replay_trajectory.py --SLOW_DOWN_FACTOR 5

# Rung 4 — closed loop (only after rung 3 is smooth):
# start perception (FoundationPose fork) publishing /robot_frame/current_object_pose,
# then goal node + policy node as in rung 2 but against the real drivers.
# Keep --vel_limit_fraction 0.5 for first runs; raise toward 0.8 when confident.
```

All nodes default to `--robot franka_right_sharpa`; every robot-specific
constant lives in `RobotProfile`
(`isaacgymenvs/utils/observation_action_utils_sharpa.py`) — if anything about
the physical setup differs (base offset, topics, joint names), fix it THERE,
not inline in nodes.

## Safety notes for the agent

- Never bypass or widen the rate limiter / restricted joint-limit clamps to
  "fix" sluggish behavior; investigate instead.
- `home_robot.py` moves the arm for 10 s — ensure the workspace is clear and a
  human has the e-stop before every arm-moving command.
- If Franka enters a reflex error: human re-enables via the desk interface;
  check the robot-node log for limiter warnings before retrying.
- Isaac Gym is NOT needed on the hardware machine (only for sim2sim rung 1).

## Who did what (context for questions)

Dev-machine work by Claude Code with the user (anthony): Franka+right-SharPa
asset build (mirror-verified from the left hand), Isaac Gym env integration,
training, deployment port (`RobotProfile`), `franka_robot_node.py`, all
validation above. Open items tracked there: velocity-corrected v2 retrain on
the 8×5090 machine (Isaac Sim backend + multi-GPU), perception-side setup.
