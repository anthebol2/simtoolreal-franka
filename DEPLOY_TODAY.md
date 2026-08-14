# Deploy v1 TODAY — step-by-step (platform-raised bench, right SharPa Franka)

> **2026-08-13 VIRTUAL YAW REMAP — the platform/table CANNOT move, so the software
> rotates the world instead.** The physical workspace sits at base **+x**; training
> had it at **−y**. Rotating the whole task 90° about the base z-axis is an EXACT
> arm symmetry (joint 1 is that axis; FK-verified to 0.0000 mm). Two flags do it:
> `franka_robot_node.py --q1_offset_deg 88.5` (single bridge for all arm state+cmds)
> and `object_pose_relay_node.py --yaw-deg -88.5` (rotates perception into the
> virtual frame). Goal node, policy, home_robot need NO changes — they live in the
> virtual world. See deploy_figs/virtual_rotation_guide.png.
> - Hammer physical placement: real base-frame **(0.57, +0.09, 0.55)** — on the
>   platform, slightly to the robot's LEFT when facing it → maps to the trained
>   start (0.09, −0.57, 0.55).
> - The FP gate criterion is checked on the RELAY OUTPUT (virtual frame):
>   `/robot_frame/current_object_pose` ≈ (0.09, −0.57, 0.55).
> - Real HOME after remap: q1 = +0.95 rad, flange high over the +x platform side.

Goal: run the v1 policy closed-loop today by raising the physical workspace to the
TRAINED geometry, and collect every measurement v2 needs along the way
(-> `HANDOVER_V2_SIM_CHANGES.md` section 4).

Every robot-touching command is run BY YOU (the motion hook blocks the agent — good).
Non-robot verifications are already done (checkpoint loads, 994–1082 Hz CPU inference,
obs pipeline OK, object registered).

**Every terminal:** `source /opt/ros/noetic/setup.bash` (`.zsh` in zsh) ·
`export ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1` · `conda activate simtoolreal` ·
`cd ~/Desktop/DexFunGrasp/simtoolreal-franka`

---

## Phase 0 — before touching the robot (~30 min)

**0.1 Build the platform.** Rigid, stable (the policy may press down), top ≥ 0.5 × 0.4 m.
Top surface at **0.53 m above the Franka base mounting plane** (±2 cm — training only
tolerates ±3 cm). With the current bench ≈0.20 m that's a ~33 cm riser.
Place it in the base's **−y sector** (≈90° from the current workspace, the side the
demos DON'T use), centered ≈0.7 m horizontally from the base axis.
- 📏 **v2 data while you're there:** tape-measure (a) exact bench-top height above the
  base plane (`TABLE_TOP_Z` for `HANDOVER_V2_SIM_CHANGES.md`), (b) usable bench extent,
  (c) the platform top height you actually achieved.

**0.2 Clear the −y sector** (HOME parks the flange at base-frame (−0.09, −0.33, 0.88);
the swing path goes through that side).

**0.3 Stop the any2any stack completely.** No rosmaster, no impedance controller, no
sharpa zombies: `pgrep -fa ros; pgrep -fa sharpa` → kill leftovers (`kill -9` anything
on UDP 54321; never Ctrl-Z the hand node).

**0.4 Rung 2 — full ROS graph, no hardware (5 min):**
```zsh
bash deployment/run_rung2_tmux.sh     # tmux attach -t simtoolreal_rung2
```
PASS = pane 5 shows `/franka/joint_cmd` ≈ 59–60 Hz; policy pane error-free; joint names
panda_joint1..7. Also verify the goal-node fix: `rostopic echo -n1 /robot_frame/goal_object_pose`
→ y ≈ **−0.56** (json 0.09 − 0.65), z ≈ 0.65. Then `tmux kill-session -t simtoolreal_rung2`.

---

## Phase 1 — camera + extrinsic for the NEW workspace (~20 min)

**Camera = the dedicated SimToolReal D415, serial `125322061981`** (verified working
2026-08-12; its own cable is BAD — it uses the D435's cable until a new one arrives; the
original D435 stays untouched at its any2any mount). Launch pinned by serial so there is
never a conflict with the D435:
```zsh
roslaunch realsense2_camera rs_camera.launch serial_no:=125322061981 align_depth:=true publish_tf:=false
```

**1.1** Mount the D415 so it sees the platform AND the wrist L-extension near it
(green band: far side of the platform ~(−0.15, −1.25), height 0.8–1.0 m, ~30° down-tilt,
roughly level roll; exact pose is uncritical — the extrinsic capture measures it).
**1.2** Capture the extrinsic exactly as in the any2any README step 5a/5b (impedance
controller up for `franka_states`, wrist FP on the extension, `set_tool_frame_to_wrist.py --go`,
`capture_extrinsic_from_wrist_fp.py --go`). Park the arm near the platform first.
**1.3** 📏 **v2 data:** run `python deployment/tools/check_wristfp_vs_ote.py` (DexFunGrasp
repo) → fresh extrinsic-bias number (mm/deg) for the v2 noise model.
**1.4** **Shut the impedance controller down** (it and the position controller are
mutually exclusive). Keep roscore + camera running.

## Phase 2 — Rung 3: real arm, open loop (~30 min, e-stop in hand)

```zsh
# T1  arm driver (NO gripper — SharPa is mounted):
roslaunch franka_control franka_control.launch robot_ip:=<IP> load_gripper:=false
#      <IP>: 172.16.0.1 if the right SharPa is on the "LEFT" arm (any2any default), else 172.16.1.1
# T2  spawn the position controller (added to default_controllers.yaml already):
rosrun controller_manager spawner position_joint_position_controller
# T3  bridge + rate limiter (conservative):
python deployment/franka_robot_node.py --vel_limit_fraction 0.5
# T4  hand driver — USE THE PROVEN DexFunGrasp NODE (clamps + holds pose on start):
cd ~/Desktop/DexFunGrasp && conda activate dexserl && python -m deployment.sharpa_node
# T5  viz (watch at localhost:8080):
python deployment/visualization_node.py --object_name hammer_002_scanned
# T6  📏 v2 data — record EVERYTHING from here on:
python deployment/record_robot_state.py
# T7  HOME (10 s interpolation — workspace clear, e-stop):
python deployment/home_robot.py
# then open-loop replay, slow:
python deployment/replay_trajectory.py --SLOW_DOWN_FACTOR 5
```
PASS = smooth motion, no Franka reflex errors, only occasional "Rate limiter active".
📏 The recorded commanded-vs-measured log from T6 during the replay **is the v2 arm-SI
dataset** — keep it.

**2.b 📏 finger step response (v2 hand SI, 2 min):** with the hand clear:
```zsh
python deployment/measure_finger_step_response.py --joint-idx 5 --step-deg 60
python deployment/measure_finger_step_response.py --joint-idx 7 --step-deg 60
python deployment/measure_finger_step_response.py --joint-idx 0 --step-deg 40
```

## Phase 3 — Rung 4: closed loop (the actual deployment)

**3.1 Object = the lab hammer (`hammer_002_scanned`, onboarded today):** re-framed to the
claw_hammer axis convention; the shipped `swing_down` goals apply. Place it lying flat on
the platform, handle pointing ≈ base-frame (+0.09, −0.57) region (±10 cm tolerance).
**3.2 Object FP** (DexFunGrasp stack, world/base frame — NOT camera frame):
```zsh
cd ~/Desktop/DexFunGrasp && bash deployment/tools/respawn_static_tf.sh
conda activate foundationpose
OBJECT_MESH=CoRL/traj_obj/hammer_002_scanned/textured.obj MESH_SCALE=1.0 \
  bash deployment/tools/run_fp_with_sam_cylinder.sh          # SAM-click the hammer
rostopic echo -n1 /object/current_pose | grep frame_id        # MUST be "world"
python deployment/tools/check_fp_jitter.py --duration 8      # PASS <=2mm  (📏 v2 data:
#   then run deployment/tools/analyze_fp_noise.py for the full noise fit — v2 input)
```
**3.3 Relay** (applies the mesh→canonical re-frame):
```zsh
python deployment/object_pose_relay_node.py --reframe-json deployment/hammer_002_scanned_reframe.json --yaw-deg -88.5
rostopic echo -n1 /robot_frame/current_object_pose   # VIRTUAL frame: expect ~(0.10, -0.65, 0.54)
```
**3.4 Sanity in viser** (localhost:8080): the hammer mesh should sit ON the platform where
the real one is, correctly oriented (head direction!). If the mesh floats/flips, stop —
extrinsic or re-frame issue.
**3.5 Goal + policy** (arm at HOME, T6 recorder running, e-stop in hand):
```zsh
python deployment/goal_pose_node.py --object_category hammer --object_name hammer_002_scanned --task_name swing_down
python deployment/rl_policy_node.py --policy_path franka_policy_v1 --object_name hammer_002_scanned
```
Keep `--vel_limit_fraction 0.5` for the first runs (raise toward 0.8 when confident).
Kill the policy node any time — the robot node's watchdog holds position; the hand holds
its last command.

## Expectations & interpretation

- This IS the "does everything work" test: perception → obs → policy → arm+hand, real time.
- Grasp/task success today is a BONUS: the policy is object-centric and size-randomized
  (primitives training), and the hammer's scale obs is set correctly — but this hammer is
  29 cm vs the 19.5 cm training-eval hammer, and the handle-scale numbers are estimates.
  Pipeline runs cleanly + arm approaches/attempts sensibly = today is a success.
- For the PAPER baseline numbers: 3D-print the exact claw_hammer (Cults3D link in
  docs/acquiring_real_world_objects.md) or wait for v2 (real geometry) + re-recorded tasks.

## v2 data checklist produced today (send with HANDOVER_V2_SIM_CHANGES.md)
- [ ] tape-measured TABLE_TOP_Z + bench extent + platform height
- [ ] fresh extrinsic-bias number (1.3)
- [ ] record_robot_state log of the rung-3 replay (arm tracking SI)
- [ ] finger step-response CSVs (2.b)
- [ ] FP noise fit on the new scene (3.2)
- [ ] closed-loop run logs (record_robot_state during 3.5)
