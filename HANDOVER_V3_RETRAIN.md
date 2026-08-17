# HANDOVER V3: geometry-matched retrain — real table, any2any objects, one policy per object

**From:** hardware-machine session, 2026-08-17.
**To:** dev-machine session (8x5090).
**Supersedes:** the geometry items of `HANDOVER_V2_SIM_CHANGES.md` (your reply
`HANDOVER_V2_REPLY.md` acknowledged that list; this doc turns it into the v3 training spec
with final tape-measured numbers). The SI items of V2 remain valid and are restated in §4.

Labels: **[MEASURED]** tape/hardware-verified on this machine · **[SURE]** verified in code
or official Franka datasheet · **[VERIFY]** we believe it, check on your side ·
**[PENDING]** data we still owe you.

---

## 0. Why v3 (context you need)

The v1 policy now works on the real robot — **full task success 2026-08-17**:
`brush_scanned` + `sweep_forward`, all 38 goals, 20 cm lift to the exact trained goal
height, final object-goal distance 0.5 cm, held to trial end (56.5 s). `hammer_002_scanned`
grasped + lifted 5 cm but slipped after 10.5 s (thin 32 mm handle at the edge of the trained
range; the 30 mm-but-grippier brush held).

**But** this only worked because we physically built a 0.53 m platform and virtualized the
azimuth with the yaw remap — recreating v1's trained geometry in the lab. Table height has
proven to matter a lot. v3's goal: **train at the real bench geometry so deployment needs no
platform**, and train **one policy per any2any object** so the baseline compares 1:1 against
the any2any method per object.

---

## 1. Real table geometry — the numbers to bake into sim [MEASURED 2026-08-17]

| quantity | value |
|---|---|
| table top height above the Franka base plane (z=0 at base) | **0.184 m** |
| table footprint | **0.81 m × 1.00 m rectangle** |
| orientation | the **1.00 m edge runs along the Franka-base side** (robot mounted at that edge); the 0.81 m dimension extends away from the robot |
| workspace band (from FP data on this bench, 2026-08-12) | objects at base-frame radius ≈ 0.45–0.70 m, previous estimate "top ≈ 0.20 m" is superseded by the 0.184 tape measure |

Sim knob mapping (mechanisms you confirmed in `HANDOVER_V2_REPLY.md`):

- `tableResetZ`: 0.38 gave top 0.53 (top = tableResetZ + 0.15) → **0.034** for top 0.184.
  Keep the V2 item: widen `tableResetZRange` from 0.01 to **±0.03** (real resets vary).
- `target_volume_origin` (absolute world coords, env.py ~:393): shift z by **−0.346**
  (0.53 → 0.184) and re-check it against the new spawn band.
- Table asset: make the sim tabletop **0.81 × 1.00 m** so edge effects and goal volumes
  match the real bench (v1's table was larger; objects can now legitimately be near an edge).
- Object spawn ranges: keep them table-relative (they follow tableResetZ), but re-check the
  radial band 0.45–0.70 m is covered.

**Layout/azimuth recommendation [SURE]:** keep the trained "table at base −y" convention
UNCHANGED and let our deploy-side yaw remap handle the real azimuth, as it does today
(joint-1 z-symmetry, FK-verified to 0.0000 mm; `--q1_offset_deg` on the bridge +
`--yaw-deg` on the relay). Only heights and sizes change in sim. Re-baking the real
+x layout into training would churn `T_W_R`, home qpos, and every task json for zero benefit.

**Feasibility check to run in sim [VERIFY]:** at top 0.184 the wrist works ~35 cm lower than
v1. Verify grasp poses in the new band don't hit flange-reach (0.855 m) or self-collision /
table-collision limits. Empirically the geometry is fine — the any2any demos were collected
by teleop at exactly this height — but the policy's approach angles may differ.

**Synergy [SURE]:** the entire any2any demo library was recorded at THIS geometry. Once sim
matches it, demos convert directly into task jsons (tool already in repo:
`deployment/convert_any2any_demo_to_task_json.py`, arc-length goal spacing 0.02 m) — no
re-collection, no height adjustment.

---

## 2. Franka ground truth — what to change in the training model

Official Panda/FCI datasheet values, cross-checked against this robot:

| param | sim today | ground truth | action |
|---|---|---|---|
| joint position limits | URDF `±2.8973, ±1.7628, ±2.8973, [−3.0718,−0.0698], ±2.8973, [−0.0175,3.7525], ±2.8973` | same (official) | **keep** [SURE] |
| joint velocity limits | URDF **10.0 rad/s all 7** | **2.1750 (j1–4), 2.6100 (j5–7) rad/s** | **FIX — the #1 item** [SURE] |
| joint torque limits | URDF wrong (52.2/200/52.2/150/7.2×3) but `utils.py:48` overrides with `[87×4, 12×3]` | 87 Nm (j1–4), 12 Nm (j5–7) | keep the code override; optionally fix the URDF too so nothing depends on the override [SURE] |
| joint accel limits | not modeled | 15, 7.5, 10, 12.5, 15, 20, 20 rad/s² | optional: action-rate limit; the vel-limit fix matters far more [SURE] |
| armature | `utils.py` computes damping ASSUMING armature 0.65/0.18 but **never writes armature into dof_props** (comment "Not setting armature") | — | **[VERIFY]** intended? Either set it or recompute damping for armature 0 — right now the damping ratio isn't the 0.3 you think it is |
| stiffness/damping | 400×4,120,80,60 / ζ=0.3 | n/a (sim actuator model) | SI-fit to real tracking (§4) |

Why the velocity fix matters even though "the deployed arm speed seems ok": deployment is ok
because we run `arm_moving_average = 0.08` + the bridge's reference governor — a transport
layer that slows the policy's targets to what the real arm can track. A policy trained with
10 rad/s learns task *timing* the real arm can't deliver, and we then mask it at deploy. Train
with real limits and the timing is learnable honestly (and the LSTM phantom-grasp failure mode
we hit at `arm_moving_average = 0.02` cannot recur).

Real controller reality, for your actuator model [SURE, deployed and logged]:
50 Hz single-point JointTrajectory targets, 80 ms horizon, robot-internal 1 kHz quintic
interpolation, internal joint impedance `[600,600,600,450,250,150,100]` Nm/rad, bridge gap
cap `[0.07,0.07,0.07,0.09,0.024,0.04,0.06]` rad, `vel_limit_fraction 0.35`. Every trial npz
contains both commanded targets and measured joints at ~60 Hz — that is your arm SI dataset
(see §4).

Hand ground truth [MEASURED 2026-07-20, this hand]:
- MCP flexion hard stop ≈ **84°** (sim uses 90°)
- AA ≈ **±17°** when flexed (ring ±13.5°); at FE=0 AA is blocked by neighbor fingers
- never command all-4-fingers flexed + splayed together (mechanical collision)
- SDK `speed_coeff 0.3`, default control mode ADMITTANCE (compliant against contact) —
  finger step-response measurement still [PENDING] on our side
  (`deployment/measure_finger_step_response.py` is ready, not yet run).

---

## 3. Training objects: the any2any roster, one policy per object

**Structure:** single-object policies (not the v1 multi-primitive mix). Baseline comparison
is per-object vs the any2any method, so each simtoolreal policy must own exactly one object.

Three objects are already fully onboarded through the dextoolbench pipeline (canonical mesh
re-framed to a sim-native object's axis convention + `objects.py` registration + trajectory
symlink + relay reframe json — pull this branch to get them):

| object | canonical convention | grasp-box scale (objects.py) | real-trial status |
|---|---|---|---|
| `hammer_002_scanned` | claw_hammer (handle −x→0, head +x) | (0.18, 0.032, 0.030)×25 | grasp+lift 5 cm, slipped 10.5 s |
| `big_hammer` | claw_hammer | (0.13, 0.039, 0.037)×25 | gated, not yet trialed |
| `brush_scanned` | red_brush (handle −x, head +x, bristles +z) | (0.133, 0.030, 0.024)×25 | **full success 38/38 goals** |

Remaining any2any objects with ready meshes (`CoRL/traj_obj/<name>/textured.obj`, all metres,
all with 200-pt pkls): `eggpie, soda_can, yoga_can, flashlight, flashlight_bigfri,
glasses_case, salt_can, triangle_salt_can, water_cup, cup_circle, cup_cube, drill_blue,
hammer_005_hole, half_cylinder_D10_W5(_scanned), seal, screwdriver, hammer_002` (+ the
corl_realworld primitive set). Suggested priority: objects with a clear cylindrical/handle
grasp region inside the trained hand's envelope first (handle 0.02–0.04 m thick per v1's
range) — the brush result says in-range handle thickness is what separates hold from slip.

Per-object deliverables (the recipe used for the three above):
1. canonical mesh: re-frame the scan to a semantically meaningful frame (origin/axes chosen
   like the nearest dextoolbench native object, so existing task structure transfers) —
   `assets/urdf/dextoolbench/<category>/<name>/` + `<name>.urdf`
2. offline collision decomposition (`generate_collision_meshes.py` → `_decomposed.urdf`) —
   needed for sim training, NOT yet done for brush_scanned/big_hammer (we deploy-only so far)
3. `objects.py` entry with the measured grasp-box scale
4. task json(s): convert the object's any2any demo npz with
   `deployment/convert_any2any_demo_to_task_json.py` — at the v3 geometry the demo poses are
   already in-distribution, no height shift needed
5. after training: extract a single SAPG block for deploy (`[64,*] → [1,*]`, slice 3 tensors —
   recipe in the repo memory; block choice = best eval block)

**Demo→task-json conversion: verified end-to-end 2026-08-17, with two hard-won rules
[MEASURED].** We converted a real 0808 hammer_002_scanned teleop demo and validated the frame
chain against the demo's own saved depth frame (tabletop plane projects to z = 0.206 ≈ the
0.184 tape + FP bias — correct).

1. **Each demo MUST be converted with the extrinsic ACTIVE AT RECORD TIME**, not the current
   `camera_extrinsic.yaml`. The camera moved often (70 cm between 08-08 and 08-11!); using
   the wrong snapshot silently shifts every pose by tens of cm. The full timestamped backup
   chain lives in `deployment/calibration/camera_extrinsic.yaml.bak_YYYYMMDD_HHMMSS` (the
   file active at time T = the earliest backup stamped AFTER T). QA gate for every
   conversion: project the demo's `*_first_frame_depth.npy` tabletop plane through the
   chosen extrinsic (intrinsics: `camera_info_d435_317222076100.yaml`) and require
   z ≈ 0.18–0.22.
2. **Some demos start with the object RAISED, not on the table.** The 0808 hammer demo's
   object rests at z ≈ 0.32 — ~12 cm above the tabletop (on a stand/holder for puck-grasp
   access) — and lifts only 11 cm (below the converter's default 0.12 m gate; use
   `--min-lift 0.08`). Decide per object whether v3 sim should spawn matching that raised
   start (faithful to the demo + to how the any2any eval runs) or whether such demos get
   re-recorded on-table. This is a REAL experimental-design decision, not a bug.

Camera note for the eval protocol: FP poses are camera-agnostic once through the extrinsic
(both the low D435 and high D415 output base-frame poses on the same topic). For the paper's
A/B comparison, run BOTH methods' real evals per object with the SAME camera + extrinsic so
tracking-accuracy differences don't confound the method comparison.

---

## 4. SI data status (the five V2 [DATA PENDING] items)

| item | status |
|---|---|
| arm tracking SI (cmd vs measured) | **DONE — in every trial npz** (`recorded_robot_inputs/2026-08-1*/`, 29-dof cmd+meas at ~60 Hz; the 2026-08-17 brush trial is a clean 56.5 s sample incl. contact) |
| finger step response | [PENDING] — script ready, will run and send |
| FP noise model | DONE earlier — `analyze_fp_noise.py` fit (σ, AR1, heavy tail) in repo memory; per-episode extrinsic bias ~2–4 mm class after the L-extension calibration |
| tape measurements | **DONE — §1 of this doc** (0.184 m top, 0.81×1.00 m) |
| extrinsic bias | bounded by the wrist-FP calibration gate (FP spread ≤ 2–4 mm accepted) |

---

## 5. Deployment invariants — do not change these in v3

The deployed stack is validated end-to-end; keep it drop-in compatible:

- observation layout (140-dim) and action space (29 = 7 arm + 22 hand), LSTM seq 16
- `sapg_coef_id = 0.0` default in `RlPlayer` (your fix — exploit block)
- `_align_quat_hemisphere` + `quat_continuity_state` (your fix — keep in any new obs code)
- absolute action targets + EMA blending semantics
- robot profile `franka_right_sharpa`: `T_W_R = (0, 0.65, 0)`, home qpos unchanged
  (home is far from the table; the height change doesn't affect it)
- task-json convention: goals in WORLD frame (robot y = world y − 0.65)

If any of these must move, flag it loudly in your reply — each one has a deploy-side mirror.

---

## 6. Ask

1. Confirm the §1 knob mapping lands the table top at 0.184 m (and the table asset resize).
2. Fix the URDF velocity limits (§2) and answer the armature [VERIFY].
3. Tell us the object order you'll train so we schedule real-object trials to match, and
   whether you want the collision decompositions done on our side.
4. We'll send the finger step-response data next session; don't block on it — it only tunes
   the hand actuator model.
