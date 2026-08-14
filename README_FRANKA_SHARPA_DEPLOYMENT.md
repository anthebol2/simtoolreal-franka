# SimToolReal on Franka + right SharPa — Complete Deployment Runbook

Everything needed to reproduce the real-hardware deployment of the SimToolReal
policy on this lab's Franka Panda + right SharPa Wave hand, from bare machine to
a closed-loop policy trial. Written after the full bring-up of 2026-08-12..14;
every step below was actually executed and every failure mode listed was
actually hit. Companion docs: `DEPLOY_TODAY.md` (condensed runbook),
`HANDOVER_V2_SIM_CHANGES.md` (sim-side changes for the v2 retrain).

```
   D415 camera ──► FoundationPose (object, camera frame)
                        │
                        ▼
        object_pose_relay_node  ── reframe json (mesh→canonical)
                        │          ── yaw  (real base → VIRTUAL trained frame)
                        ▼
   /robot_frame/current_object_pose            goal_pose_node (task json goals)
                        │                               │
                        ▼                               ▼
                rl_policy_node  (model.pth, 140 obs → 29 actions @ 60 Hz)
                   │                                   │
        /franka/joint_cmd (VIRTUAL)          /sharpa/joint_cmd (rad)
                   ▼                                   ▼
        franka_robot_node ("bridge")            sharpa_node (DexFunGrasp's)
        · q1 offset (virtual→real)                     │
        · velocity rate limiter                   SharPa SDK → hand
        · reference governor (gap caps)
        · one-point JointTrajectory @50 Hz
                   ▼
    position_joint_trajectory_controller (1 kHz internal quintic interp)
                   ▼
        franka_control (FCI) → arm     [internal joint impedance, SOFTENED]
```

**The virtual-world trick (read this first).** Training put the table at the
robot base's −y; this lab's bench is bolted at +x. Rotating the whole task about
the base z-axis is an *exact* arm symmetry (joint 1 is that axis), so we run the
policy in a "virtual world": the bridge shifts joint 1 by a fixed offset
(`--q1_offset_deg`) and the relay rotates perception by the opposite yaw
(`--yaw-deg`). The yaw is *fitted to wherever the object physically sits* (see
§5), so object placement direction is free. Nothing else (policy, goals, home)
needs to know.

---

## 0. Machine prerequisites (one-time)

Ubuntu 20.04, ROS1 Noetic native, PREEMPT_RT kernel, `libfranka`/`franka_ros`
built in `~/catkin_ws_franka`, SharPa SDK 5.0.4 at `/opt/sharpa-wave-sdk`
(hand firmware 2.0.0 — never flash).

```zsh
# conda env for the simtoolreal stack (py3.10; torch CPU FIRST or pip hangs
# resolving pytorch-kinematics against CUDA wheels):
conda create -y -n simtoolreal python=3.10
pip install "numpy<2"
pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install scipy trimesh==3.23.5 yourdfpy tyro viser pytorch-kinematics \
    termcolor omegaconf "hydra-core>=1.2" gym==0.23.1 pyyaml rospkg catkin-pkg \
    empy defusedxml netifaces tensorboardX tensorboard wandb
pip install -e . --no-deps && pip install -e ./rl_games --no-deps
```

One-time system fixes (all were live bugs on this machine):

1. **CPU governor MUST be `performance`** (powersave causes 3–7% FCI packet
   drops → torque faults, grinding motion):
   `echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`
   (re-apply after reboot, or make persistent).
2. `~/.zshrc` had `export LD_LIBRARY_PATH=~/.mujoco/...` **without appending** —
   it wiped the ROS lib path so every native ROS binary (nodelet, rosout,
   realsense) died with "cannot open shared object file". Must append:
   `export LD_LIBRARY_PATH=~/.mujoco/mujoco210/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}`
3. `~/.zshrc` pins `_CATKIN_SETUP_DIR`, which makes sourcing the franka
   workspace silently load the wrong one. **Every terminal that needs
   `franka_msgs` / `serl_franka_controllers`:**
   `unset _CATKIN_SETUP_DIR; source ~/catkin_ws_franka/devel/setup.zsh`
4. The trajectory controller ships in
   `franka_control/config/default_controllers.yaml`
   (`position_joint_position_controller` was added there too; the trajectory
   controller is the one to use — see §8 "why").
5. `franka_control_node.yaml`: `cutoff_frequency: 30` (was 100) and collision
   thresholds doubled (torques `[60,60,54,54,48,42,36]`, wrenches `[60 ×6]`) —
   the payload + contact-rich task trips factory thresholds.

Every fresh terminal below: `source ~/.zshrc; export ROS_IP=127.0.0.1
ROS_HOSTNAME=127.0.0.1` (this box needs ROS_IP or subscribers silently fail),
plus the conda env and (where noted) the franka workspace lines from item 3.

## 1. Physical setup

- **Object support surface at 0.53 m above the Franka base mounting plane**
  (± 2 cm; training tolerates ±3). This lab: foam-block platform on the bench.
- **Object radius from the base axis 0.50–0.65 m** (trained start is 0.577).
  Direction is FREE (the yaw fit absorbs it). Surface ≥ ~0.4 × 0.4 m, rigid.
- **Camera**: dedicated RealSense **D415, serial 125322061981** (the D435
  belongs to the any2any project — never move it). Mount beyond the platform,
  lens ~0.8–1.0 m above the base plane, ~30° down-tilt, level roll, viewing the
  platform top + the airspace above + wherever the wrist L-extension will park
  for calibration. Exact pose is uncritical (calibration measures it), but
  **after calibration the camera must not move by a millimeter**.
- **Object**: `hammer_002_scanned` (wooden lab hammer, colored dot decals,
  white-capped handle butt). Its handle (0.18 × 0.032 × 0.030 m) is inside the
  trained hammer-handle distribution. Lies flat on the platform.
- E-stop within reach at all times. Arm's HOME parks high over the platform
  side — keep that airspace clear.

## 2. Start core services

```zsh
# T1  roscore                          (plain env)
roscore
# T2  camera (serial-pinned)          (plain env)
roslaunch realsense2_camera rs_camera.launch serial_no:=125322061981 align_depth:=true publish_tf:=false
```
Check: `rostopic hz /camera/color/image_raw /camera/aligned_depth_to_color/image_raw` → ~30 Hz both.

## 3. Camera↔base calibration (the L-extension method)

The policy needs object poses in the robot base frame; FP gives camera frame.
The transform between them is measured ONCE per camera placement using the
known L-extension bolted to the wrist: FP sees the extension in camera frame,
the robot's encoders know it in base frame, one equation gives camera→base.
No checkerboards, ~10 minutes. (All tools live in the DexFunGrasp repo.)

```zsh
# T3  arm driver (needed for robot state; stays up all day; franka-ws lines!)
unset _CATKIN_SETUP_DIR; source ~/catkin_ws_franka/devel/setup.zsh
roslaunch franka_control franka_control.launch robot_ip:=172.16.0.1 load_gripper:=false

# Park the arm (guide mode) so the wrist L-extension clearly faces the camera,
# near the platform (accuracy is best near the capture pose).

# T4  wrist FP                        (conda activate foundationpose)
cd ~/Desktop/DexFunGrasp
OBJECT_MESH=deployment/extension_mesh/extension_palmframe.obj MESH_SCALE=1.0 \
  bash deployment/tools/run_fp_with_sam_wrist.sh
#     SAM-click the L-EXTENSION → ENTER → wait until the overlay is DEAD STILL.

# T5  capture                         (conda activate dexserl + franka-ws lines)
cd ~/Desktop/DexFunGrasp
python deployment/tools/set_tool_frame_to_wrist.py --go     # once per session
python deployment/tools/capture_extrinsic_from_wrist_fp.py --go
```
**Quality gates:** printed `FP spread ≤ 2 mm` (re-run if it warns "FP still
moving" — a bad lock here poisons everything downstream); printed camera
position must match physical reality (z ABOVE the platform if it looks down at
it — a capture claiming z below the tabletop it can see is garbage; recapture).
Then Ctrl-C the wrist FP — it is never needed again (SimToolReal reads the arm
from encoders; only the object is camera-tracked).

## 4. Object perception

```zsh
# T4 (reuse)                          (conda activate foundationpose)
cd ~/Desktop/DexFunGrasp
bash deployment/tools/respawn_static_tf.sh        # republishes the new extrinsic
OBJECT_MESH=CoRL/traj_obj/hammer_002_scanned/textured.obj MESH_SCALE=1.0 \
  bash deployment/tools/run_fp_with_sam_cylinder.sh
#     SAM-click the hammer → lock → CLOSE the debug window (it halves FP rate).
rostopic echo -n1 /object/current_pose | grep frame_id      # MUST be "world"
```

## 5. Fit the virtual yaw + start the relay

The object's measured azimuth defines the rotation. With the object FP live,
compute (this repo, `simtoolreal` env — or ask the agent, it's 5 lines):

```python
# pseudo: p = FP object pose @ canonical frame (apply reframe json first)
# azimuth = atan2(p.y, p.x)  [real base frame]
# yaw = -81.0deg - azimuth        # maps object to the trained start azimuth
# ALSO CHECK: radius = hypot(p.x, p.y) in 0.50..0.65 ; support z = 0.53 ± 0.03
```
2026-08-14 fitted value for this lab's layout: **yaw = −88.5° → relay
`--yaw-deg -88.5`, bridge `--q1_offset_deg 88.5`.** Re-fit whenever the object's
resting spot moves more than a few cm in azimuth.

```zsh
# T6  relay                           (conda activate simtoolreal, this repo)
python deployment/object_pose_relay_node.py \
    --reframe-json deployment/hammer_002_scanned_reframe.json --yaw-deg -88.5
```
**GATE (do not proceed until it passes):** the relay heartbeat must print the
object at ≈ **(0.091, −0.567, 0.545) ± a few cm** — the trained start in the
virtual frame. Wrong sector/height = fix physics, wrong by ~0.7 m = the object
FP is in camera frame (redo §4).

## 6. Arm + hand + recorder

```zsh
# T7  bridge                          (simtoolreal env, this repo)
python deployment/franka_robot_node.py --vel_limit_fraction 0.35 --q1_offset_deg 88.5
#     Wait for the "VIRTUAL YAW REMAP active" banner + "Initialized target to measured q".
#     A fresh bridge printing "Rate limiter active" with nothing running = a stray
#     publisher on /franka/joint_cmd — find and kill it before continuing.

# T8  readiness (ALWAYS this script, NEVER raw spawner commands):
bash deployment/arm_ready.sh
#     Does: error recovery → soften internal impedance [600,600,600,450,250,150,100]
#     → (re)start the trajectory controller (spawner detached — a dying spawner
#     UNLOADS its controller, hence never Ctrl-C it, never wrap it in timeout)
#     → verdict. Want: "impedance set" AND "GO". NO-GO once → run it AGAIN
#     (fault clearing is two-pass). "Can't accept new commands" spam during the
#     script = harmless transient.

# T9  hand                            (conda activate dexserl, DexFunGrasp repo)
python -m deployment.sharpa_node
#     Use the DexFunGrasp node (clamps + holds pose on start; the stock
#     simtoolreal one snaps the hand to zero). Wait for "Holding current pose".
#     Stiff grip for trials:  rostopic pub -1 /sharpa/control_mode std_msgs/String "position"

# T10 recorder                        (simtoolreal env, this repo)
python deployment/record_robot_state.py     # saves .npz on Ctrl-C — the trial data
```

Optional but recommended once per session:
```zsh
python deployment/wiggle_test.py            # 12° single-joint chain test → "ARM MOVED"
python deployment/wiggle_test.py --all --deg 15 --seconds 12   # policy-shaped 7-joint test
```
Smooth + quiet = the chain is right. Grinding/faults = see §9.

## 7. Per-trial protocol (a sim episode reset, done physically)

```zsh
bash deployment/arm_ready.sh                     # GO (×2 if needed)
python deployment/home_robot.py --move_time 30   # arm → trained HOME (verify it ARRIVES)
# reset the object to its calibrated spot (relay heartbeat re-confirms the gate)
bash deployment/arm_ready.sh                     # re-engage after homing
# goal node (leave running across trials):
python deployment/goal_pose_node.py --object_category hammer --object_name hammer_002_scanned --task_name swing_down
# THE TRIAL:
OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 python deployment/rl_policy_node.py \
    --policy_path franka_policy_v1 --object_name hammer_002_scanned
# ... observe ~60 s ... Ctrl-C to end the trial.
```
Optional live 3D view: `python deployment/visualization_node.py --object_name
hammer_002_scanned` → browser `localhost:8080` (solid robot = measured, purple
ghost = commanded target; scene is drawn in the VIRTUAL frame, i.e. rotated by
the yaw vs the room).

**Hard rules:** ONE policy node alive, ever (two publishers on
/franka/joint_cmd = chaos that looks like hardware failure). Ctrl-C to stop;
at a `(Pdb)` prompt type `q` + Enter; **never Ctrl-Z** (leaves a live zombie —
if it happens: `kill -9 %1`). Re-HOME between trials — a non-home start
produces garbage behavior (trained episodes always start at HOME ± 6°).

## 8. Knobs and why they're set this way

| knob | value | why |
|---|---|---|
| controller | `position_joint_trajectory_controller` | streams goals, interpolates internally at 1 kHz; immune to command-stream hiccups. The raw group-position controller turned every dropped packet into a reference jump → `tau_J_range_violation` storms + audible grinding |
| internal impedance | `[600,600,600,450,250,150,100]` (arm_ready sets it) | factory (~2000–3000 Nm/rad) gives the 12 Nm wrist joints a tracking-gap budget of ~0.3° — impossible for streamed references |
| bridge `--vel_limit_fraction` | 0.35 | 0.5 = spec; lower if wrist torque faults recur |
| bridge `GAP_CAP` (in code) | `[0.07,0.07,0.07,0.09,0.024,0.04,0.06]` rad | reference governor: caps target-vs-arm gap so commanded torque ≈ ≤50% of each joint's limit under the soft impedance |
| policy `arm_moving_average` (in `main()`) | 0.08 | ≈ trained 0.1. Values ≪0.1 (e.g. 0.02) make the LSTM's task rhythm outrun the body: it "grasps" on schedule mid-descent and carries a phantom object to the goal height |
| policy sanity gate | 45–120° | absolute-action policies legitimately hold targets 30–60° ahead of a tracking wrist; the gate is a garbage detector, not a motion limiter (bridge limits motion) |

## 9. Troubleshooting (all personally verified)

| symptom | cause | fix |
|---|---|---|
| `robot_mode: 4`, arm deaf | latched reflex | `arm_ready.sh` (twice) |
| `robot_mode: 5` | physical user-stop pressed | release the button (blue light = ready) |
| `tau_J_range_violation` at motion start | stiff impedance / raw position controller / target-vs-arm gap | this runbook's controller+impedance+governor stack; if it still fires, lower `--vel_limit_fraction` |
| `cartesian_reflex` | real contact (>60 N) — or the arm path clipping the platform | clear the path; HOME from a compact-high pose; the collision-checker habit: verify paths, not endpoints |
| arm ignores commands, no error | controller unloaded (a spawner died — incl. `timeout`-wrapped ones) or its stop during arm_ready | `arm_ready.sh`; check `rostopic info /position_joint_trajectory_controller/command` has a subscriber |
| "Rate limiter active" on an idle bridge | stray publisher on /franka/joint_cmd (zombie policy) | `rostopic info /franka/joint_cmd`; kill the zombie |
| `franka_msgs`/pkg not found | wrong workspace ordering | `unset _CATKIN_SETUP_DIR; source ~/catkin_ws_franka/devel/setup.zsh` |
| native ROS binary: "cannot open shared object file" | LD_LIBRARY_PATH clobbered | §0 item 2; or `export LD_LIBRARY_PATH=/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH` |
| FP slow / ~20 Hz | debug image windows open | close them |
| object pose ~0.7 m off | FP publishing camera frame | `respawn_static_tf.sh` + relaunch FP world-frame (§4) |
| policy "grasps the air", hovers, drifts to goal height | ①`arm_moving_average` too low (phantom-grasp rhythm) ②non-home start ③checkpoint itself (see below) | ①=0.08 ②re-HOME ③offline rollout isolates it |
| No Sharpa device | zombie on UDP 54321 or NIC profile stolen | `kill -9` zombies; `nmcli connection up "Sharpa Hand"` |
| motion success<0.99, glitchy | CPU governor powersave | §0 item 1 |

## 10. Current status (2026-08-14) & open item

Deployment stack: **complete and validated end-to-end** (scripted chain tests
move smoothly and quietly; four closed-loop policy trials executed and
recorded). Trial 1 reached the hammer, made contact, pushed it 12 cm (no lift).
Open item: the shipped `franka_policy_v1/model.pth` reproducibly hovers ~23 cm
from the object **even in a pure offline rollout under exact trained-start
conditions** — see the urgent block atop `HANDOVER_V2_SIM_CHANGES.md`
(checksum comparison + Isaac eval request for the dev machine). The moment a
verified-grasping checkpoint lands in `franka_policy_v1/`, the path from file
to first trial is §7 — about fifteen minutes.

Object onboarding recipe (for future tools): usdz scan → mesh; geometric
re-frame to the handle-along-+x convention (handle −x..0, head +x; see
`deployment/hammer_002_scanned_reframe.json` provenance) → register in
`dextoolbench/objects.py` (grasp-box = handle dims ×25) → symlink or record
task trajectories (`deployment/convert_any2any_demo_to_task_json.py` turns
any2any teleop demos into task jsons — no RGB-D video pipeline needed).
