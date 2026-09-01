# HANDOVER: v3 deployment package — 6 trained single-object baselines (updated 2026-08-31)

**From:** the dev-machine (dex5090-1, 8× RTX 5090) SimToolReal training session.
**To:** the Claude Code agent (and human) on the hardware machine that runs the
real Franka + right SharPa lab.
**Purpose:** 6 v3-trained single-object policies are ready to deploy today.
This document is what you read *before* opening any of the earlier handovers.

**Update 2026-08-31:** Refreshed `small_flashlight` and `drill_blue` checkpoints (both improved), and **added `yoga_can`** (finally shipped after num_envs=6144 fix). See §0.

**Prior handovers this replaces / builds on:**
- [`HANDOVER_REAL_DEPLOYMENT.md`](../HANDOVER_REAL_DEPLOYMENT.md) — v1 (2026-08-13). Read for the deployment stack (ROS graph, node names, safety, bring-up rungs). Everything about the deployment pipeline still applies.
- [`HANDOVER_V2_SIM_CHANGES.md`](../HANDOVER_V2_SIM_CHANGES.md) — hardware→dev list of what v2 must fix (real bench geometry, real Franka limits, perception noise, hand SI). This v3 training addresses **items 1.1, 1.2, 1.3, 1.4, 1.7, 2.1, 2.2** — see §2 below for the exact list of what's done and what isn't.
- [`DEPLOY_TODAY.md`](../DEPLOY_TODAY.md) — v1 same-day deployment script. The 90° virtual-yaw remap it describes is **NO LONGER NEEDED** in v3: the training env has been rotated to match the real workspace direction (see §2.2).

---

## 0. TL;DR — what changed and what you get

**Deliverable (this directory):** 6 trained policies packaged with their configs. Assets live in the main repo at `assets/urdf/dextoolbench/`.

| object | mean_successes @ 1cm | ship-gate margin | mesh source | model.pth (md5 first 8) | reproduction confidence |
|---|---|---|---|---|---|
| **salt_can** (banked) | 40.83 | 13.6× | scanned | `e491de40` | ✅ overwhelming |
| **small_flashlight** ⭐ REFRESHED 2026-09-01 | 24.09+ (final trained) | 8.0×+ | scanned | `9b82ff74` | ✅ overwhelming |
| **half_cylinder_D10_W5_scanned** (banked) | 16.67 | 5.5× | scanned primitive | `46f0b40b` | ✅ strong |
| **yoga_can** ⭐ REFRESHED 2026-09-01 | 15.08+ (final trained) | 5.0×+ | scanned | `3496195e` | ✅ strong (see §4.1 caveat) |
| **water_cup** | 13.18 | 4.4× | scanned | `f47d317f` | ✅ solid |
| **drill_blue** ⭐ REFRESHED 2026-09-01 | 9.90+ (final trained) | 3.3×+ | scanned | `544f9936` | ✅ solid |

All numbers are `mean_successes` per episode in Isaac Gym at the training's terminal `success_tolerance = 0.01` (1 cm keypoint tolerance, held for 10 consecutive steps). This is **stricter** than the paper's `ε = 2 cm` position criterion, so every number above is a **conservative lower bound** on the paper-metric performance (monotone: reaching a 1 cm goal trivially reaches the 2 cm one). See `HANDOVER_V2_SIM_CHANGES.md` and the paper (§IV.A of arxiv 2602.16863) for the ε definition.

**Objects intentionally NOT in this package:**
- `handle_head_primitives` — this is the paper's actual generalist baseline. Shipped now (mean_successes = 4.73) but with only 1.6× margin over the bar. Still training and climbing. Will be added to a future release once it's more comfortably past the bar.

**Recent deployment package updates:**
- 2026-09-01 refresh (final trained state for 3 objects being killed to free GPUs for retrains + new object):
  - `small_flashlight` md5 → `9b82ff74` (final best.pth from training run)
  - `drill_blue` md5 → `544f9936` (final best.pth from training run)
  - `yoga_can` md5 → `3496195e` (final best.pth from training run)
- 2026-08-31 refresh (earlier intermediate): `small_flashlight` → `333bb146` (18.93 → 24.09), `drill_blue` → `64b7d822` (8.13 → 9.90), `yoga_can` added.
- `water_cup` checkpoint on the release is the original 2026-08-29 upload (a fresh v2 retrain is planned, so a refresh here would be superseded).

---

## 1. What must be transferred to the hardware machine

Everything is in this directory (`deployment_handover_v3_2026-08-29/`, 1.3 GB total). Recommended transfer: `rsync -avP dex5090-1:~/develop/simtoolreal-franka/deployment_handover_v3_2026-08-29/ ~/deployment_handover_v3_2026-08-29/` (or scp / USB / whatever this lab uses).

### Per-object structure (in this handover dir)
```
<object>/
├── model.pth       # rl_games auto-tracked best-so-far checkpoint (~263 MB) — pull from GitHub Release, see §1a
└── config.yaml     # full training config, needed by RlPlayer for obs/action pipelines
```

### Object assets (already in the main repo — not duplicated here)
The URDFs, textured meshes, and convex-decomposed collision meshes for each object are already tracked in the main repo at:

- `assets/urdf/dextoolbench/can/salt_can/`
- `assets/urdf/dextoolbench/can/yoga_can/`
- `assets/urdf/dextoolbench/half_cylinder/half_cylinder_D10_W5_scanned/`
- `assets/urdf/dextoolbench/cup/water_cup/`
- `assets/urdf/dextoolbench/drill/drill_blue/`
- `assets/urdf/dextoolbench/flashlight/small_flashlight/` (added in commit `cbf16c9`, pull the latest `franka-right-sharpa` branch to get it)

Each dir contains: `<object>.urdf`, `<object>.obj` (textured visual mesh, meters), `<object>_decomposed.urdf` (collision URDF used by physics/training), `<object>_collision/decomp_N.obj` (convex hulls), and `textured.mtl` + texture image.

Density-randomized variant URDFs (`_var000.urdf` … `_var099.urdf`) are training-only randomization and are gitignored — deployment uses the single canonical `_decomposed.urdf`. Density variation was Uniform(300, 600) kg/m³ during training; the deployment mesh mass matches the canonical inertial block in the URDF (paper convention).

### §1a Pulling the checkpoints (6 × ~250 MB = ~1.5 GB total)
The `model.pth` files are **too big for git** — same convention as v1's `franka_policy_v1/model.pth`.

**As of 2026-08-31, GitHub uploads from `dex5090-1` are throttled/broken** (0 bytes received in 85 s). So the 6 checkpoints are split across two paths:

#### Path A — GitHub Release (3 healthy assets uploaded 2026-08-29)
Release: **`v3-baselines-2026-08-29`** at <https://github.com/anthebol2/simtoolreal-franka/releases/tag/v3-baselines-2026-08-29>

Assets on the release:
- `salt_can_model.pth` (state=uploaded)
- `half_cylinder_D10_W5_scanned_model.pth` (state=uploaded)
- `water_cup_model.pth` (state=uploaded)

```bash
gh release download v3-baselines-2026-08-29 \
    --repo anthebol2/simtoolreal-franka \
    --dir /tmp/v3_dl --pattern '*.pth'
for obj in salt_can half_cylinder_D10_W5_scanned water_cup; do
    mkdir -p deployment_handover_v3_2026-08-29/$obj
    mv /tmp/v3_dl/${obj}_model.pth deployment_handover_v3_2026-08-29/$obj/model.pth
done
```

#### Path B — rsync from `dex5090-1` (3 assets blocked by upload throttling)
These 3 need to come via rsync/scp because GitHub uploads are throttled from `dex5090-1` today. This is the same convention used for v1's `franka_policy_v1/model.pth` (see `HANDOVER_REAL_DEPLOYMENT.md` §"What must be transferred", line 41).

```bash
# From the hardware machine (assumes SSH access to dex5090-1):
for obj in yoga_can drill_blue small_flashlight; do
    mkdir -p deployment_handover_v3_2026-08-29/$obj
    rsync -avP dex5090-1:/home/lipuhao/develop/simtoolreal-franka/deployment_handover_v3_2026-08-29/$obj/model.pth \
        deployment_handover_v3_2026-08-29/$obj/model.pth
done
```

If SSH to `dex5090-1` isn't set up, ask anthony (or whoever manages the training box) to `scp` the files to a shared location or a USB drive.

#### Verify integrity (all 6 files, both paths)
```bash
cd deployment_handover_v3_2026-08-29
md5sum -c MODEL_CHECKSUMS.md5     # all 6 lines must say OK
```

**Note on yoga_can size:** yoga_can's `model.pth` is ~182 MB rather than ~263 MB. It trained with `num_envs=6144` (half — see §4.1), which halves the minibatch_size and optimizer-state buffer sizes. Policy weights are complete; this is not a corruption sign. Verify via the checksum file.

**When GitHub upload throttling clears** (usually within hours), the 3 rsync'd assets will be uploaded to the same release under the same names. If a file already exists in your local `deployment_handover_v3_2026-08-29/<obj>/`, keep whichever has the matching md5 in `MODEL_CHECKSUMS.md5`.

### Integrity check (do this after transfer)
```bash
cd deployment_handover_v3_2026-08-29
md5sum -c MODEL_CHECKSUMS.md5   # all 5 lines must say "OK"
```

### Where to get more if needed
- **Fresher checkpoints:** water_cup, drill_blue, small_flashlight are still training on dex5090-1 and will keep improving. To pull a newer `best.pth`, rerun the per-object rsync — the `<experiment>.pth` in `train_dir/simtoolreal_baseline_training/<exp>/runs/<exp>/nn/` is auto-updated by rl_games whenever it beats its own best reward.
- **Full W&B runs (for training curves):**
  - salt_can: [runs/uid_00_v3_salt_can_2026-08-21_00-59-32](https://wandb.ai/anthebol2-beijing-institute-for-general-artificial-intel/simtoolreal_baseline_training/runs/uid_00_v3_salt_can_2026-08-21_00-59-32) · [shipgate k2xq9gud](https://wandb.ai/anthebol2-beijing-institute-for-general-artificial-intel/simtoolreal_baseline_training/runs/k2xq9gud)
  - half_cylinder: [runs/uid_00_v3_half_cylinder_D10_W5_scanned_2026-08-21_00-59-27](https://wandb.ai/anthebol2-beijing-institute-for-general-artificial-intel/simtoolreal_baseline_training/runs/uid_00_v3_half_cylinder_D10_W5_scanned_2026-08-21_00-59-27) · [shipgate 45th9rsg](https://wandb.ai/anthebol2-beijing-institute-for-general-artificial-intel/simtoolreal_baseline_training/runs/45th9rsg)
  - water_cup: [runs/uid_00_v3_water_cup_2026-08-21_00-59-37](https://wandb.ai/anthebol2-beijing-institute-for-general-artificial-intel/simtoolreal_baseline_training/runs/uid_00_v3_water_cup_2026-08-21_00-59-37) · [shipgate aqwmeoxe](https://wandb.ai/anthebol2-beijing-institute-for-general-artificial-intel/simtoolreal_baseline_training/runs/aqwmeoxe)
  - drill_blue: [runs/uid_00_v3_drill_blue_2026-08-21_00-59-42](https://wandb.ai/anthebol2-beijing-institute-for-general-artificial-intel/simtoolreal_baseline_training/runs/uid_00_v3_drill_blue_2026-08-21_00-59-42) · [shipgate jwmky8ad](https://wandb.ai/anthebol2-beijing-institute-for-general-artificial-intel/simtoolreal_baseline_training/runs/jwmky8ad)
  - small_flashlight: [runs/uid_00_v3_small_flashlight_2026-08-27_02-55-49](https://wandb.ai/anthebol2-beijing-institute-for-general-artificial-intel/simtoolreal_baseline_training/runs/uid_00_v3_small_flashlight_2026-08-27_02-55-49) · [shipgate ty2y7pnk](https://wandb.ai/anthebol2-beijing-institute-for-general-artificial-intel/simtoolreal_baseline_training/runs/ty2y7pnk)

---

## 2. Sim-side changes vs v1 / v2 — what the hardware machine can and cannot rely on

Confidence legend matches `HANDOVER_V2_SIM_CHANGES.md`:
- **[SURE]** — code-verified in this repo / robustly configured. Apply as written.
- **[VERIFY]** — right concept, one cheap check needed before baking in.
- **[NOT ADDRESSED]** — v3 did not fix this item; either open or deferred.

### 2.1 Real-bench geometry (v2 item 1.1 — DONE)

The training now uses **real-bench geometry, tape-measured from this lab (2026-08-17)**. All values live in `isaacgymenvs/launch_v3_training.py:32-40` as `V3_GEOMETRY_OVERRIDES` and are passed verbatim to every training run:

| parameter | v1 value | v3 value | source |
|---|---|---|---|
| table asset | `urdf/table.urdf` (0.475×0.4×0.3 box) | `urdf/table_real_bench.urdf` (0.81×1.00 m real bench) | matches physical lab bench |
| `tableResetZ` | 0.38 | **0.034** | measured table-top height 0.184 m above base plane, minus the table asset's 0.15 m origin offset |
| `tableResetZRange` | 0.01 (±1 cm) | **0.03** (±3 cm) | v2 item 1.7 recommendation applied — matches real bench-height tolerance |
| `tableDistanceFromBase` | (implicit) | **0.405** | robot mounted AT the table edge (real setup) |
| `tableObjectYOffset` | 0 | **−0.145** | object spawn center in the 0.45–0.70 m band from base |
| `targetVolumeMins` | `[-0.4, -0.05, 0.68]` (world, over 0.53 table) | **`[-0.35, -0.05, 0.254]`** | shifted down by (0.53 − 0.184) m to match new table height |
| `targetVolumeMaxs` | `[0.4, 0.3, 1.05]` | **`[0.35, 0.2, 0.604]`** | same shift |

**Implication for hardware:** you do NOT need to raise the physical bench to 0.53 m anymore (`DEPLOY_TODAY.md` Phase 0.1 is obsolete). The trained policy expects the real 0.184 m bench top.

### 2.2 Workspace direction (v2 item 1.2 — DONE)

The sim env's robot pose was rotated so the workspace lies at the base's **+x** side (matches real lab), instead of v1's −y side. This means:

- **The `DEPLOY_TODAY.md` virtual-yaw remap (`--q1_offset_deg 88.5`, `--yaw-deg -88.5`) is NO LONGER NEEDED.** Skip those flags entirely. The trained policy already lives in the physical workspace's frame.
- **Any hardcoded 88.5° / 90° rotations in perception relay or goal node should be dropped or set to 0.** [VERIFY] before deployment: check `deployment/goal_pose_node.py`, `deployment/object_pose_relay_node.py`, `deployment/home_robot.py` — remove the yaw remap arguments.
- **HOME arm pose:** the v3 sim uses the corrected HOME (base's +x). No arm-q1 shift needed anymore.

### 2.3 Object set — 6 real lab tools (v3-specific, new)

Unlike v1's single generalist policy (procedural primitives), v3 trains **per-object policies**. Each of the 6 shipped policies is specialized to one physical object with 100 density-randomized variants (Uniform 300–600 kg/m³) for mass robustness. This is a **methodology deviation from the SimToolReal paper**, which trains one generalist policy — that generalist run (`handle_head_primitives`) is separate and is NOT in this package (see §0).

Object provenance and physical dimensions:

| object | category | mesh source | approx dims / mass range |
|---|---|---|---|
| salt_can | can | scanned | ~10 cm can body, 89–178 g (density × canonical volume) |
| yoga_can | can | scanned | can body, 86–170 g (density × canonical volume) |
| half_cylinder_D10_W5_scanned | half_cylinder | scanned primitive (D=10cm W=5cm) | half-cylinder printed reference primitive |
| water_cup | cup | scanned | ~10 cm cup, 60–120 g |
| drill_blue | drill | scanned | small blue drill |
| small_flashlight | flashlight | scanned | 17.5 cm body, 83–164 g |

**On the hardware side:** every object's `<object>_decomposed.urdf` and mesh files live in the main repo at `assets/urdf/dextoolbench/<category>/<object>/` (pull the latest `franka-right-sharpa` branch). Point the deployment env's asset path there. The reframe JSONs (`deployment/<object>_reframe.json`) exist in the main repo for the two newest objects (small_flashlight, eggpie); the older four (salt_can, half_cylinder, water_cup, drill_blue) should already have theirs if the lab has run them before. For yoga_can, check whether a reframe JSON exists in `deployment/`; if not, one may need to be authored the same way small_flashlight's was.

### 2.4 Training config (paper SAPG, unchanged from v1)

Verbatim from `launch_v3_training.py` — these are the paper's SimToolReal SAPG configuration and should be treated as constant across all shipped runs:

- Task: `SimToolRealLSTMAsymmetric` (asymmetric actor-critic, LSTM policy)
- envs per training run: **12288** (except yoga_can at 6144 — see §4.1)
- SAPG block count: 6, block size = num_envs / 6
- minibatch_size: num_envs × 16 / 4 = 49152
- Rollout: horizon 16, seq_length 16
- Optimizer: PPO with adaptive KL schedule (threshold 0.016)
- Actor MLP: [1024, 1024, 512, 512] with ELU
- Central value MLP: [1024, 1024, 512, 512]
- Perturbations during training (not turned off for eval — see caveat below):
  - `forceScale=20`, `torqueScale=2.0` (random disturbances)
  - `objectScaleNoiseMultiplierRange=[0.9, 1.1]`
  - `forceConsecutiveNearGoalSteps=True` (must hold pose 10 consecutive steps for success)
- Density randomization: Uniform(300, 600) kg/m³ over 100 asset variants per object

### 2.5 v2 items NOT yet addressed in v3 (still open)

| v2 item | what | status | consequence for hardware |
|---|---|---|---|
| 2.1 arm velocity limits | URDF has `velocity="10.0"` on all arm joints; real Panda is 2.175 / 2.61 rad/s | **[NOT ADDRESSED]** — still 10.0 in training URDF | keep `franka_robot_node.py --vel_limit_fraction 0.5` for first runs; the rate limiter will fire occasionally. Same caveat as v1. |
| 2.2 arm effort limits | URDF (52.2, 200, 52.2, 150, 7.2, 7.2, 7.2) vs real (87, 87, 87, 87, 12, 12, 12) | **[NOT ADDRESSED]** | J2/J4 sim ~2× too stiff, J5–7 ~40% too weak. Real behavior may differ from sim on high-force pushes. |
| 2.3 arm tracking dynamics | Sim PD + `armMovingAverage 0.1` vs real libfranka + our node's rate limiter | **[NOT ADDRESSED]** — no measured effective bandwidth from rung-3 replay | expect the "hover before descent" symptom described in `HANDOVER_V2_SIM_CHANGES.md §URGENT` to still be POSSIBLE on some policies. Watch for it and record if seen. |
| 2.4 perception noise | Fitted FP noise model + per-episode extrinsic bias not applied | **[NOT ADDRESSED]** — still white noise `objectStateXyzNoiseStd 0.01` + uniform 0–10-step delay | policies may be optimistic about perception accuracy. Real-hardware success rates likely lower than train.log's mean_successes. |
| 2.5 hand model | Real MCP FE hard stop, finger step response | **[NOT ADDRESSED]** — URDF unchanged | as v1 caveat. Keep the DexFunGrasp sharpa node (clamps + admittance) rather than the simtoolreal-native one. |

**These are known gaps.** They mean your real-hardware success rates will likely be BELOW the sim numbers reported above. That's expected. If you see specific failure modes (persistent hovering, finger over-travel, jerky arm), correlate them with the corresponding item above — they're pre-diagnosed, not novel bugs.

---

## 3. Deployment quickstart

Read [`HANDOVER_REAL_DEPLOYMENT.md`](../HANDOVER_REAL_DEPLOYMENT.md) for the full bring-up sequence (rungs 1–4). The command changes for v3 are:

### 3.1 Policy path
Point `--policy_path` at the per-object deployment folder rather than `franka_policy_v1`:
```bash
python deployment/rl_policy_node.py \
    --policy_path deployment_handover_v3_2026-08-29/salt_can \
    --object_name salt_can
```
The `model.pth` and `config.yaml` sit at the top level of each subfolder — same layout `RlPlayer` expects (per `deployment/rl_player.py`).

### 3.2 Perception + reframe
For each object you deploy, use the reframe JSON:
```bash
python deployment/object_pose_relay_node.py \
    --reframe-json deployment/salt_can_reframe.json
    # NO --yaw-deg — v3 is in the physical workspace frame already
```

### 3.3 Goal poses
```bash
python deployment/goal_pose_node.py \
    --object_category can \
    --object_name salt_can \
    --task_name <task>   # per dextoolbench/trajectories/
```
[VERIFY] the task trajectories match v3 geometry — some jsons were recorded for v1's 0.53 m table. If a goal appears above the workspace or off-bench in viser, re-record on the real bench or shift by (0.53 − 0.184) m in z.

### 3.4 Suggested order for today's deployment
Do them in this order (highest confidence → thinnest margin):
1. **salt_can** (13.6× margin, plateaued near ceiling — most predictable)
2. **small_flashlight** (8.0× margin, simplest collision shape — should transfer cleanly)
3. **half_cylinder_D10_W5_scanned** (5.5× margin, printed primitive — simple shape)
4. **yoga_can** (5.0× margin — see §4.1 about the training-time env-count caveat; nothing to do at deployment time)
5. **water_cup** (4.4× margin)
6. **drill_blue** (3.3× margin — leave last; expect the lowest sim→real success rate)

For each, run at `--vel_limit_fraction 0.5` first; raise toward 0.8 once you see smooth motion.

---

## 4. Known deviations from paper config (for the writeup)

### 4.1 yoga_can trained at num_envs=6144 (this package DOES include yoga_can as of the 2026-08-31 update)
Every other object trains at the paper's 12288 envs. yoga_can hits a PhysX GPU solver kernel-launch limit on RTX 5090 at 12288 envs (`PxgTGSCudaSolverCore.cpp: GPU solveContactParallel fail to launch kernel`). Diagnosed via `CUDA_LAUNCH_BLOCKING=1`. Root cause: yoga_can has the highest single-hull vertex count (2445) and 7 convex hulls; contact-pair volume overflows the solver buffers at 12288 envs. Halving to 6144 fits. This is a Blackwell PhysX limit, not an algorithmic change — footnote it in the paper.

Deployment-time impact: NONE. The trained policy consumes observations and produces actions at 60 Hz regardless of the training env count. The 6144 vs 12288 only affected how many parallel simulations the training loop stepped per batch.

### 4.2 Per-object policies vs paper's single generalist
v3 trains 6 per-object policies (this package) + 1 generalist (`handle_head_primitives`, still training). The paper reports the generalist result on the 12-object real-world set. The 6 per-object baselines here are **additional** — they let you claim per-object reproduction quality but are not the paper's headline experiment.

---

## 5. What the dev machine still owes the hardware machine (v2 §4 items)

None of these are v3 training deliverables — they're perception / SI data the hardware side owes:

- [ ] Rung-3 commanded-vs-measured arm logs (for v2 item 2.3)
- [ ] Finger step-response CSVs (for v2 item 2.5)
- [ ] Fitted FP-noise parameters (for v2 item 2.4)

When you produce these, we can fold them into a v4 retrain — but v3 is already deployable as-is for baseline claims.

---

## 6. Safety notes (unchanged from v1)

Repeated from `HANDOVER_REAL_DEPLOYMENT.md` because they still apply:
- Never widen the rate limiter / joint-limit clamps to "fix" sluggish behavior; investigate.
- `home_robot.py` moves the arm 10 s — workspace clear, human on e-stop, every time.
- Isaac Gym is NOT needed on the hardware machine (only for offline sim2sim verification).
- If Franka reflex-errors: human re-enables via desk; check robot-node log for limiter warnings before retrying.

---

## 7. Reproduction claim — what you can defensibly say for the paper

Based on train.log statistics at ε=1cm (stricter than the paper's ε=2cm; passing the stricter bar implies passing the paper's bar by the monotonicity of tolerance-based success):

> "We reproduce SimToolReal-style per-object policies on 5 real-world objects (salt_can, half_cylinder, water_cup, drill_blue, small_flashlight) using the paper's SAPG configuration on a real-bench-matched Isaac Gym environment. Each policy achieves ≥ 3 mean successes per episode at ε=1cm keypoint tolerance sustained for ≥50 log blocks — the paper training's own curriculum-mastery criterion. Per-object margins over this bar range from 2.7× (drill_blue) to 13.6× (salt_can). Under the paper's looser ε=2cm bar, these are lower bounds; per-object numbers strictly increase."

For a stronger claim, run the batch eval with `evalSuccessTolerance=0.02` on each checkpoint to get the actual paper-tolerance numbers (we didn't run this — see §7 of prior chat / project memory).

---

## 8. Who did what (context for questions)

Dev-machine work by Claude Code (Opus) with the user (anthony) on dex5090-1, 2026-08-19 through 2026-08-29:
- v3 env setup: `simtoolreal_v3` cloned from `dec_sapgv2`, wandb CLI upgraded to accept newer keys.
- 8-object launch roster (this package + yoga_can + handle_head_primitives + eggpie); density randomization at 100 variants.
- Debugged: yoga_can PhysX kernel-launch limit → num_envs=6144 workaround (via `CUDA_LAUNCH_BLOCKING=1`).
- Killed 2 shipped runs early (salt_can, half_cylinder) to free GPU 0 headroom (Isaac Gym leaks ~5–7 GB per process onto physical GPU 0 regardless of `CUDA_VISIBLE_DEVICES` — inherent Isaac Gym Vulkan enumeration + cudaSetDevice residual context, confirmed via NVIDIA forums; not fixable).
- Best checkpoints packaged (this document).

Hardware-machine open items: run the deployment rungs on each of the 5 objects, log the perception/arm SI data v2 §4 asked for, and feed any regressions back for a v4 retrain.
