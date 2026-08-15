"""Checkpoint-behavior eval through the DEPLOYMENT pipeline (RlPlayer + deployment obs).

Modes:
  --object claw_hammer  : DexToolBench transfer eval (fixed trajectory goals)
  --object training     : the policy's own training distribution (procedural
                          primitives, env-sampled goals)
Robots: franka_right_sharpa (default) or kuka_left_sharpa (for the authors'
pretrained policy as a harness control).
"""

import argparse

# isort: off
from isaacgymenvs.tasks.simtoolreal.env import SimToolReal  # noqa: F401
import torch
# isort: on

import json
import numpy as np

from deployment.isaac.isaac_env import create_env
from deployment.isaac.isaac_env_no_ros import IsaacEnvNoRos
from deployment.rl_player import RlPlayer
from isaacgymenvs.utils.observation_action_utils_sharpa import (
    create_urdf_object,
    get_robot_profile,
    get_urdf_path,
)
from isaacgymenvs.utils.utils import get_repo_root_dir

p = argparse.ArgumentParser()
p.add_argument("--checkpoint", required=True)
p.add_argument("--label", required=True)
p.add_argument("--config", required=True)
p.add_argument("--robot", default="franka_right_sharpa")
p.add_argument("--object", default="claw_hammer")
p.add_argument("--steps", type=int, default=3600)
p.add_argument("--stochastic", action="store_true")
p.add_argument("--exact_training_env", action="store_true")
p.add_argument("--obs_source", default="deployment", choices=["deployment", "env"])
args = p.parse_args()

overrides = {
    "task.env.numEnvs": 1,
    "task.env.envSpacing": 0.4,
    "task.env.capture_video": False,
    "task.env.useActionDelay": False,
    "task.env.useObsDelay": False,
    "task.env.useObjectStateDelayNoise": False,
    "task.env.objectScaleNoiseMultiplierRange": [1.0, 1.0],
    "task.env.armMovingAverage": 0.1,
    "task.env.forceScale": 0.0,
    "task.env.torqueScale": 0.0,
    "task.env.linVelImpulseScale": 0.0,
    "task.env.angVelImpulseScale": 0.0,
    "task.env.forceProbRange": [0.0001, 0.0001],
    "task.env.torqueProbRange": [0.0001, 0.0001],
    "task.env.linVelImpulseProbRange": [0.0001, 0.0001],
    "task.env.angVelImpulseProbRange": [0.0001, 0.0001],
    "task.env.forceOnlyWhenLifted": True,
    "task.env.torqueOnlyWhenLifted": True,
    "task.env.linVelImpulseOnlyWhenLifted": True,
    "task.env.angVelImpulseOnlyWhenLifted": True,
}

if args.exact_training_env:
    # Match the training/internal-test env exactly: keep yaml defaults for
    # delays, reset noise, force probs; keep DR forces and scale noise on.
    overrides = {
        "task.env.numEnvs": 6,
        "task.env.envSpacing": 0.4,
        "task.env.capture_video": False,
        "task.env.objectScaleNoiseMultiplierRange": [0.9, 1.1],
        "task.env.forceScale": 20.0,
        "task.env.torqueScale": 2.0,
        "task.env.armMovingAverage": 0.1,
    }

if args.robot == "kuka_left_sharpa":
    overrides["task.env.asset.robot"] = (
        "urdf/kuka_sharpa_description/iiwa14_left_sharpa_adjusted_restricted.urdf"
    )

if args.object != "training":
    traj = json.load(
        open(
            get_repo_root_dir()
            / f"dextoolbench/trajectories/hammer/{args.object}/swing_down.json"
        )
    )
    overrides.update(
        {
            "task.env.objectName": args.object,
            "task.env.useFixedGoalStates": True,
            "task.env.fixedGoalStates": traj["goals"],
            "task.env.useFixedInitObjectPose": True,
            "task.env.objectStartPose": traj["start_pose"],
            "task.env.startArmHigher": True,
            "task.env.resetPositionNoiseX": 0.0,
            "task.env.resetPositionNoiseY": 0.0,
            "task.env.resetPositionNoiseZ": 0.0,
            "task.env.randomizeObjectRotation": False,
            "task.env.resetDofPosRandomIntervalFingers": 0.0,
            "task.env.resetDofPosRandomIntervalArm": 0.0,
            "task.env.resetDofVelRandomInterval": 0.0,
            "task.env.tableResetZRange": 0.0,
            "task.env.evalSuccessTolerance": 0.05,
            "task.env.successSteps": 1,
            "task.env.fixedSizeKeypointReward": True,
            "task.env.resetWhenDropped": False,
        }
    )

env = create_env(config_path=args.config, headless=True, device="cuda", overrides=overrides)

ckpt = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
env.set_env_state(ckpt[0]["env_state"])

policy = RlPlayer(
    num_observations=140,
    num_actions=29,
    config_path=args.config,
    checkpoint_path=args.checkpoint,
    device="cuda",
    num_envs=env.num_envs,
)
profile = get_robot_profile(args.robot)
urdf = create_urdf_object(robot_name=profile.urdf_name)
wrapper = IsaacEnvNoRos(env=env, control_dt=1.0 / 60, device="cuda", urdf=urdf, profile=profile)

DETERMINISTIC = not args.stochastic
obs = wrapper.reset()
rest_z = float(env.object_pose[0, 2])
L = args.label
print(f"[{L}] object rest z = {rest_z:.3f}")

ep_max_lift, ep_min_palm_dist, lifts, succs, ep = 0.0, 1e9, [], [], 0
prev_progress = int(env.progress_buf[0])
obs_diff_max = 0.0
for step in range(args.steps):
    action = policy.get_normalized_action(obs, deterministic_actions=DETERMINISTIC)
    obs_dep, _, _, _ = wrapper.step(action)
    obs_env = env.obs_buf[:, :140].clone()
    d = (obs_dep - obs_env).abs()
    non_quat = torch.cat([d[:, :90], d[:, 98:]], dim=1)
    obs_diff_max = max(obs_diff_max, float(non_quat.max()))
    if step % 300 == 299:
        FIELDS = {"joint_pos": (0, 29), "joint_vel": (29, 58), "prev_tgt": (58, 87),
                  "palm_pos": (87, 90), "palm_rot": (90, 94), "obj_rot": (94, 98),
                  "fingertip": (98, 113), "kp_rel_palm": (113, 125),
                  "kp_rel_goal": (125, 137), "scales": (137, 140)}
        parts = " ".join(f"{k}={float(d[:, a:b].max()):.4f}" for k, (a, b) in FIELDS.items())
        print(f"[{L}] step={step} FIELD_DIFFS {parts}")
        obs_diff_max = 0.0
    obs = obs_env if args.obs_source == "env" else obs_dep
    obj_z = float(env.object_pose[0, 2])
    kp_rel_palm = obs[0, 113:125].reshape(4, 3)
    ep_max_lift = max(ep_max_lift, obj_z - rest_z)
    ep_min_palm_dist = min(ep_min_palm_dist, float(kp_rel_palm.norm(dim=-1).mean()))
    progress = int(env.progress_buf[0])
    if progress < prev_progress:
        succ = float(env.successes.float().mean()) if hasattr(env, "successes") else -1
        print(
            f"[{L}] ep{ep}: max_lift={ep_max_lift:.3f} m, "
            f"min_palm_obj_dist={ep_min_palm_dist:.3f} m, successes={succ}"
        )
        lifts.append(ep_max_lift)
        succs.append(succ)
        ep += 1
        ep_max_lift, ep_min_palm_dist = 0.0, 1e9
        policy.reset()
        rest_z = float(env.object_pose[0, 2])
    prev_progress = progress

print(
    f"[{L}] SUMMARY: episodes={ep}, mean_max_lift={np.mean(lifts) if lifts else 0:.3f}, "
    f"lifted>5cm in {sum(1 for x in lifts if x > 0.05)}/{max(len(lifts),1)} eps, "
    f"mean_successes={np.mean(succs) if succs else 0:.2f}"
)
