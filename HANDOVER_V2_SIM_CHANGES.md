# HANDOVER: v2 sim-training changes to match the REAL hardware setup

> ## ⚠️ URGENT FIRST (2026-08-14 night): verify the v1 checkpoint itself
> The full deployment now RUNS closed-loop (see DEPLOY_TODAY.md + memory). Across 4 real
> trials AND a pure-offline python rollout (this machine, no robot: exact trained start,
> exact home pose, ideal tracking, deterministic AND stochastic), the shipped
> `franka_policy_v1/model.pth` does the same thing: descends briefly, then HOVERS
> ~23 cm from the object, closing fingers in the air, then rises toward the goal
> height. Object-scale descriptor sweep (2.5/4.5/5.6) changes nothing.
> **Step 1 (5 seconds): md5sum your good-videos checkpoint and compare:**
> our deployed file = `md5 8d1201f8df2b331eb45ce0af8265ac6e` (142,254,602 bytes).
> Different -> wrong snapshot was shipped; send the right one (we deploy same-day).
> **Step 2 (if identical): run eval_isaacgym on THIS EXACT model.pth file** (trained
> start, both deterministic and stochastic) and confirm whether it grasps in Isaac.
> The four real-trial recordings are committed under `recorded_robot_inputs/`
> (q/targets/object/goal streams) for direct comparison.
> Prime suspect: model.pth is a different snapshot than the good-videos checkpoint
> (best-vs-latest mixup). If it grasps in Isaac but hovers in the deployment replica,
> send us the Isaac obs dump for one episode and we will diff obs-by-obs.
> The hardware side is DONE and validated — a verified checkpoint deploys same-day.

**From:** the hardware-machine Claude Code session (real Franka + right SharPa lab).
**To:** the dev-machine Claude Code session that trains SimToolReal.
**Purpose:** the v1 policy (`franka_policy_v1`) was validated end-to-end on the hardware
machine (env, checkpoint, ROS graph, mount geometry — all good), but its trained WORLD
GEOMETRY does not match the real bench. This file lists exactly what v2 must change,
with confidence labels. v2 must be real-matched — the baseline experiments depend on it.

Confidence legend:
- **[SURE]** — code-verified in this repo / datasheet fact / robustly measured. Apply as written.
- **[VERIFY]** — right concept, one cheap check needed before baking in (check listed).
- **[DATA PENDING]** — needs a measurement from the hardware machine; we will supply it.

---

## 0. Measured real-setup geometry (hardware machine, 2026-08-12)

Measured from FoundationPose object rest poses in recorded demos (0809 + 0811),
transformed to the Franka base frame (`panda_link0`) with the current verified
camera extrinsic (captured 2026-08-11, same rig that does mm-level any2any grasps):

| quantity | value | confidence |
|---|---|---|
| table top above the base mounting plane | **≈ 0.20 m** (0.200 / 0.207 in two clean demos) | [VERIFY] estimate ±2–3 cm. **TODO(anthony): replace with tape-measured value: `TABLE_TOP_Z = ____ m`** |
| workspace center (base frame) | ≈ **(x +0.55, y −0.17)**, objects observed x 0.50–0.62, y −0.06…−0.28 | center [VERIFY] ±5 cm |
| workspace direction | along the base's **+x** | **[SURE]** — cannot be an extrinsic artifact |
| trained (v1) geometry for comparison | table top **0.53 m**, objects at (x≈0, **y −0.65…−0.9**) in base frame | **[SURE]** (code) |

So v1 is out-of-distribution on the real bench by ~0.33 m in height and ~90° in base yaw.
(Interim: we can still eval v1 by physically raising a platform to 0.53 m in the −y
sector — that's the stopgap, not the fix.)

---

## 1. REQUIRED geometry changes (the core of v2)

The invariant that must hold end-to-end **[SURE]**:

> (a) sim robot-base pose in world == deployment `FRANKA_T_W_R_np`
> (`isaacgymenvs/utils/observation_action_utils_sharpa.py:300`), and
> (b) sim table pose **in the base frame** == real table pose in the base frame.

Concrete changes (line numbers as of branch `franka-right-sharpa`):

1. **Table height** **[SURE mechanism, VERIFY number]** — `tableResetZ: 0.38 → (TABLE_TOP_Z − 0.15)`
   in the task config (table asset is a 0.475×0.4×0.3 box → top = tableResetZ + 0.15;
   with the current estimate TABLE_TOP_Z=0.20 → `tableResetZ = 0.05`). Object spawning is
   table-relative (`env.py:1173` `_object_start_pose`, `tableObjectZOffset 0.25`) and follows
   automatically **[SURE]**.

2. **Workspace direction + distance** **[SURE that it must change; VERIFY sign/values]** —
   real workspace is at base **+x**, sim has it at **−y**. Since v2 retrains from scratch,
   just make the base-relative geometry match:
   - `env.py:1863`: `robot_base_y 0.65 → ~0.55` (final value = chosen workspace-center
     distance; we measured objects at 0.50–0.62 m).
   - `env.py:1866-1867`: rotate the robot spawn: `robot_pose.r` identity → **Rz(−90°)**
     (quat (0, 0, −0.7071, 0.7071)) so the base's +x faces the table. VERIFY the sign once
     in the viewer: after the change, the table must sit on the base's **+x** side.
   - Shift HOME/default arm q1 by **+90°** (verify sign together with the yaw): env default
     arm dof pos AND `home_arm_qpos` in `FRANKA_RIGHT_SHARPA_PROFILE`
     (`observation_action_utils_sharpa.py:386`). q1 −0.6242 + π/2 ≈ +0.947, inside limits.

3. **Goal-sampling volume is ABSOLUTE and will NOT follow the table** **[SURE]** —
   `env.py:394-396`: `target_volume_origin = [0, 0.05, 0.8]`,
   `target_volume_extent = [[-0.4,0.4],[-0.05,0.3],[-0.12,0.25]]` are world coordinates
   (trained goals z 0.68–1.05 over a 0.53 table). Shift z down by (0.53 − TABLE_TOP_Z)
   and re-express x/y for the new layout — or better, make it table-relative. A config
   override already exists (`targetVolumeMins`/`targetVolumeMaxs`, `env.py:397-425`).

4. **Deployment mirror** **[SURE]** — update `FRANKA_T_W_R_np` to the SAME base pose
   (translation (0, robot_base_y, 0) + the same Rz yaw) and the profile `home_arm_qpos`.
   This is the single sync point; if env.py changes and the profile doesn't, deployment
   breaks silently. Also re-check `deployment/fake/fake_perception_node.py`'s hardcoded
   object pose (robot frame (0,−0.8,0.55)) so rung-2 tests exercise the NEW geometry.

5. **Task trajectories** **[SURE it's an issue; decision needed]** — the shipped
   `dextoolbench/trajectories/*.json` are world-frame poses recorded for the OLD geometry
   (table top 0.53). For v2 either re-record demos on the real bench (the video pipeline;
   poses then convert with the NEW T_W_R and land correctly) or shift the existing jsons
   (z −(0.53−TABLE_TOP_Z), plus the layout re-expression). Re-recording is the honest path
   for the baseline experiments. Also audit the per-task environment assets
   (`assets/urdf/dextoolbench/environments/...` e.g. the nail for hammer tasks) for
   absolute placements.

6. **Table footprint** **[VERIFY]** — the real bench is larger than the 0.475×0.4 sim
   table. Consider matching the sim table footprint to the real usable bench so
   table-collision avoidance is realistic. We will supply the measured bench extent.

7. **Table-height randomization** **[SURE current value]** — `tableResetZRange: 0.01`
   (±1 cm). Recommend widening to ±0.03–0.05 in v2 for robustness to bench tolerance.

---

## 2. Franka parameter / system-ID changes

1. **Arm velocity limits** **[SURE]** — `franka_right_sharpa.urdf` has `velocity="10.0"`
   on all 7 arm joints; real Panda: **2.175 rad/s (J1–4), 2.61 rad/s (J5–7)**. This is the
   already-planned v1→v2 fix; without it the deployment rate limiter (0.8×) silently
   truncates motions training thought were legal.

2. **Arm effort limits** **[SURE numbers; VERIFY gains after]** — URDF has
   (52.2, 200, 52.2, 150, 7.2, 7.2, 7.2) Nm; real Panda: **(87, 87, 87, 87, 12, 12, 12)**.
   These cap the PD force in sim, so J2/J4 are ~2× too strong and J5–7 ~40% too weak.
   Set real values, then re-verify the PD gains / stiffnessScale still track well.

3. **Arm tracking dynamics** **[DATA PENDING]** — real chain is
   `position_controllers/JointGroupPositionController` (libfranka internal, very stiff)
   + our node's 0.8× velocity rate limiter, vs sim PD + `armMovingAverage 0.1`.
   The hardware machine will run the rung-3 open-loop replay and deliver
   commanded-vs-measured joint logs; fit effective bandwidth/latency from those and match
   sim (gains/EMA). Recommend also simulating the rate limiter in training (cheap, exact:
   per-tick target step clamp of vel_limit_fraction × real limits / control_hz).

4. **Perception noise** **[SURE that current model is guessed; measured model exists]** —
   v1 used white noise `objectStateXyzNoiseStd 0.01` / 5° + uniform 0–10-step delay.
   The hardware machine has a MEASURED FoundationPose noise model for this rig
   (per-axis σ, AR(1) time correlation, heavy tails — `deployment/tools/analyze_fp_noise.py`
   + `fp_noise_sim.py` in the DexFunGrasp repo); we will send the fitted parameters.
   Additionally add a **per-episode constant object-pose bias** (a few cm / a few deg,
   resampled each episode): the dominant real error is the camera-extrinsic bias
   (once measured at ~22 mm / 8°), which per-step white noise does not represent.

5. **Hand model** **[VERIFY — measured once on this hand 2026-07-20]** —
   - Real MCP FE hard stop ≈ **84°**; URDF allows 90° (`right_2_index_MCP_FE` upper 1.5708).
     Either clamp URDF FE to the measured stops or accept the 6° saturation.
   - Real AA range collapses when the fingers flex (±17°, ring ±13.5). The sim URDF only
     uses ±2° AA (`right_index_MCP_AA` ±0.0349), so sim ⊂ real — benign, no change needed.
   - URDF finger velocity 16 rad/s is far above the real hand (firmware interpolation +
     `speed_coeff 0.3`). We will measure a finger step response **[DATA PENDING]**;
     set hand joint velocity limits + `handMovingAverage` to match.
   - Real hand runs POSITION or ADMITTANCE control mode; pick one for the baseline and
     keep it fixed (our proven node defaults to admittance; simtoolreal's used position).

6. **No change needed** **[SURE]** — sim dt 1/60 with `controlFrequencyInv 1` already
   equals the 60 Hz deployment loop. Keep `obsDelayMax 3` until we deliver a measured
   end-to-end latency from closed-loop logs.

---

## 3. FYI: deployment-side items already handled on the hardware machine

- **Bug fixed**: `deployment/goal_pose_node.py` hardcoded the KUKA world→robot offset
  (`y − 0.8`); for the Franka profile (T_W_R y=0.65) every goal was published 15 cm off
  in y. Now reads `get_robot_profile(--robot).T_W_R`. Sim2sim never caught it because the
  goal node isn't in that loop. **Same constant also hardcoded in
  `dextoolbench/process_poses.py` (`y + 0.8`)** — fix it the same way before generating
  any Franka task trajectories through that script (our
  `deployment/convert_any2any_demo_to_task_json.py` bypasses it and takes the offset
  explicitly).
- New `deployment/object_pose_relay_node.py`: bridges this lab's existing FoundationPose
  stack (`/object/current_pose`, base frame) to `/robot_frame/current_object_pose` — we
  do NOT need the separate FoundationPose fork for live inference.
- `position_joint_position_controller` added to the local franka_ros config; conda env
  `simtoolreal` built (note: rl_games also needs `tensorboardX`, `tensorboard`, `wandb` —
  missing from the original recipe); checkpoint loads and infers at 1082 Hz CPU;
  mount geometry in the URDF confirmed == the lab's physically calibrated flange→wrist
  transform. Rung 2 script ready (`deployment/run_rung2_tmux.sh`).

## 4. What the hardware machine still owes you (v2 inputs)

1. Tape-measured `TABLE_TOP_Z` (replaces the 0.20 estimate above).
2. Usable bench extent + chosen workspace center in the base frame.
3. Rung-3 commanded-vs-measured arm logs (for item 2.3).
4. Finger step-response measurement (for item 2.5).
5. Fitted FP-noise parameters (for item 2.4).
6. Decision: eval object set (3D-printed claw_hammer vs onboarding lab tools) and the
   corresponding re-recorded task trajectories on the real bench.
