#!/usr/bin/env python
"""Convert a DexFunGrasp/any2any teleop demo (.npz) into a SimToolReal task json.

SimToolReal's task spec is ONLY an object-pose trajectory (start_pose + goals,
world frame). Their video pipeline (record RGB-D -> FP extract_poses ->
process_poses.py) exists to produce exactly that. Our any2any demos already
contain it: `fp_object_pose_camera_7` tracked live during teleop + the camera
extrinsic. The wrist/hand channels are simply not needed (object-centric policy).
So ONE teleop demo can drive BOTH methods: any2any uses wrist+fingers+object,
SimToolReal uses object-only — ideal for a side-by-side baseline.

Frames: npz poses are CAMERA frame -> base frame via the extrinsic yaml ->
"world" frame by adding the robot-base offset T_W_R (y_offset; 0.65 for the
Franka profile — NOTE their own process_poses.py hardcodes the KUKA +0.8, same
bug family as the goal_pose_node one we fixed; this converter emits the final
task json directly and takes the offset explicitly). If the object mesh needs
re-framing to a canonical dextoolbench frame (e.g. hammer_002_scanned), pass
the same --reframe-json used by the relay.

Example:
  python deployment/convert_any2any_demo_to_task_json.py \
      --npz ~/Desktop/DexFunGrasp/08XX_traj/teleop_hammer_...npz \
      --reframe-json deployment/hammer_002_scanned_reframe.json \
      --object-category hammer --object-name hammer_002_scanned --task-name swing_down_lab
"""

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import tyro
from scipy.spatial.transform import Rotation as R
import yaml


@dataclass
class Args:
    npz: Path
    object_category: str
    object_name: str
    task_name: str
    extrinsic: Path = Path(
        "/home/wmingd/Desktop/DexFunGrasp/deployment/calibration/camera_extrinsic.yaml"
    )
    """camera->base extrinsic ACTIVE WHEN THE DEMO WAS RECORDED (re-captures move it!)"""
    reframe_json: Path | None = None
    """optional T_mesh_canonical json (same file the relay uses)"""
    t_w_r_y: float = 0.65
    """robot-base y offset in world (Franka profile T_W_R; KUKA would be 0.8)"""
    min_lift_m: float = 0.12
    """goals start once the object has risen this far above its rest height
    (equivalent to their min_z=0.65 with a 0.53 table)"""
    goal_spacing_m: float = 0.02
    """resample goals to this arc-length spacing (shipped tasks: ~24 mm median).
    Our demos are 30 Hz, so fixed-factor downsampling gives ~3 mm steps — too dense
    for goal_pose_node's proximity-advance logic. 0 disables (keep every frame)."""
    out_dir: Path = Path(__file__).parent.parent / "dextoolbench/trajectories"


def pose7_to_T(p7: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = p7[:3]
    T[:3, :3] = R.from_quat(p7[3:7]).as_matrix()
    return T


def T_to_pose7(T: np.ndarray) -> list:
    q = R.from_matrix(T[:3, :3]).as_quat()
    return [*np.asarray(T[:3, 3], dtype=float), *q.astype(float)]


def main() -> None:
    args = tyro.cli(Args)
    d = np.load(args.npz, allow_pickle=True)
    op = d["fp_object_pose_camera_7"]
    valid = d["fp_object_pose_valid_t"] > 0
    assert valid.sum() > 30, f"only {valid.sum()} valid FP frames in {args.npz}"

    ext = yaml.safe_load(open(args.extrinsic))["T_world_camera"]
    T_B_C = np.eye(4)
    T_B_C[:3, 3] = ext["xyz"]
    T_B_C[:3, :3] = R.from_quat(ext["quat_xyzw"]).as_matrix()

    T_reframe = np.eye(4)
    if args.reframe_json is not None:
        T_reframe = np.array(json.load(open(args.reframe_json))["T_mesh_canonical"])

    # camera -> base (-> canonical object frame) -> world
    poses_base = np.array(
        [T_to_pose7(T_B_C @ pose7_to_T(p) @ T_reframe) for p in op[valid]]
    )
    poses_world = poses_base.copy()
    poses_world[:, 1] += args.t_w_r_y

    # rest height from the first ~2 s, then lift-off filter (their min_z, made relative)
    rest_z = float(np.median(poses_world[:60, 2]))
    lifted = np.flatnonzero(poses_world[:, 2] >= rest_z + args.min_lift_m)
    assert len(lifted), (
        f"object never rose {args.min_lift_m} m above rest z={rest_z:.3f} "
        f"(max {poses_world[:, 2].max():.3f}) — not a lift/tool-use demo?"
    )
    lifted_poses = poses_world[lifted[0] :]
    if args.goal_spacing_m > 0:
        # arc-length resample: keep a pose once the object moved goal_spacing_m
        keep = [0]
        for i in range(1, len(lifted_poses)):
            if (
                np.linalg.norm(lifted_poses[i, :3] - lifted_poses[keep[-1], :3])
                >= args.goal_spacing_m
            ):
                keep.append(i)
        if keep[-1] != len(lifted_poses) - 1:
            keep.append(len(lifted_poses) - 1)  # always keep the final pose
        goals = lifted_poses[keep]
    else:
        goals = lifted_poses

    out = {
        "start_pose": [float(x) for x in poses_world[0]],
        "goals": [[float(x) for x in g] for g in goals],
    }
    out_path = args.out_dir / args.object_category / args.object_name / f"{args.task_name}.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(out, f, indent=4)
    print(
        f"wrote {out_path}\n  {len(goals)} goals; rest z(world)={rest_z:.3f}; "
        f"lift-off at frame {lifted[0]}/{valid.sum()}; "
        f"goal z range [{goals[:, 2].min():.3f}, {goals[:, 2].max():.3f}]"
    )
    print(
        "REMINDER: goals are only valid for a policy TRAINED with the geometry the demo "
        "was recorded in (current bench -> v2; platform-at-0.53 demo -> v1)."
    )


if __name__ == "__main__":
    main()
