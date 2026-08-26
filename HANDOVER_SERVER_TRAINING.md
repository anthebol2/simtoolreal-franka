# HANDOVER: v3 baseline training on the 8x5090 server

**Audience:** the Claude Code agent (and human) on the 8x RTX 5090 server
(10.0.11.153). Everything below was built and smoke-tested on the dev machine;
your job is environment setup, launch, and monitoring. **Do not modify training
configs or env code** — every v3 parameter lives in
`isaacgymenvs/launch_v3_training.py` and is deliberate (geometry is
tape-measure-matched to the real bench; the rest reproduces the SimToolReal
paper exactly).

## What is being trained

Eight independent single-GPU runs (GPUs 0-7):

| GPU | object | run type |
|---|---|---|
| 0 | yoga_can | single-object baseline |
| 1 | half_cylinder_D10_W5_scanned | single-object baseline |
| 2 | salt_can | single-object baseline |
| 3 | water_cup | single-object baseline |
| 4 | drill_blue | single-object baseline |
| 5 | handle_head_primitives | original simtoolreal (procedural primitives) |
| 6 | eggpie | single-object baseline |
| 7 | small_flashlight | single-object baseline |

All runs: Franka + right SharPa, real-bench geometry (table top 0.184 m,
0.81x1.00 m, robot at the table edge), paper SAPG config, 12288 envs/GPU.
Single-object runs use 100 density-sampled variants of their object
(Uniform 300-600 kg/m^3 — the paper's printed-object convention).

## Environment setup (order matters)

Isaac Gym does not officially support Blackwell GPUs; this lab's
`dec_sapgv2` conda env reportedly runs it on these 5090s. **Verify before
anything else:**

```bash
# 1. Clone the working env (never install into the teammate's env directly).
#    Source env (lipuhao's, runs Isaac Gym on these 5090s):
#    /home/lipuhao/miniconda3/envs/dec_sapgv2
conda create --clone /home/lipuhao/miniconda3/envs/dec_sapgv2 -n simtoolreal_v3
conda activate simtoolreal_v3

# 2. Repo
git clone https://github.com/anthebol2/simtoolreal-franka.git
cd simtoolreal-franka   # branch franka-right-sharpa is the default

# 3. Install repo packages WITHOUT dependencies (the cloned env's
#    torch/isaacgym are the working versions — never let pip touch them;
#    our pyproject pins would DOWNGRADE/BREAK them if installed with deps)
pip install -e . --no-deps
pip install -e ./rl_games --no-deps   # REQUIRED: our fork (SAPG + crash fixes)

# 4. Install the pure-python deps the training path needs (safe additions;
#    none of these touch torch/cuda). Then resolve any stragglers revealed by
#    the import check — one package at a time, never `pip install -e .` with deps:
pip install tyro yourdfpy scipy trimesh "gym==0.23.1" omegaconf "hydra-core>=1.2" \
  pyyaml wandb tensorboardX tensorboard termcolor pysdf "urdfpy==0.0.22" \
  "imageio[ffmpeg]" matplotlib
python -c "import isaacgymenvs.train" 2>&1 | tail -2

# 5. GPU smoke test — THE gate for the isaacgym-on-5090 claim:
python isaacgymenvs/launch_v3_training.py --object yoga_can --gpu 0 \
  --num_envs 48 --no-wandb_activate --experiment_suffix gpusmoke \
  --extra train.params.config.max_epochs=3 \
    train.params.config.minibatch_size=192 \
    train.params.config.central_value_config.minibatch_size=192 \
    train.params.config.expl_coef_block_size=8 task.env.numAssetsPerType=20
# PASS = it prints per-epoch stats and exits cleanly ("MAX EPOCHS NUM!").
# FAIL (CUDA "no kernel image", PhysX errors, segfault) = STOP. Do not
# debug isaacgym binaries; report back to the dev machine — the fallback
# (Isaac Sim backend) is a dev-machine decision.

# 6. W&B (project: simtoolreal_baseline_training)
wandb login
```

**Notification setup (do once, on wandb.ai, logged in as the account used
above):** Settings -> Notifications -> enable **Scriptable run alerts** for
Email (and Slack if the team has it connected). The ship-gate watcher delivers
its "SHIP READY" notification through `wandb.alert` — without this toggle the
alert only appears in the W&B UI, with it anthony gets an email automatically.
No email addresses live in the code.

## Launch

```bash
bash isaacgymenvs/launch_all_v3.sh
# OOM on any GPU? relaunch that object alone with fewer envs, e.g.:
#   python isaacgymenvs/launch_v3_training.py --object drill_blue --gpu 4 --num_envs 8256
# (num_envs must stay divisible by 6)
```

Expect the first ~5 minutes per run to be silent (asset generation: 100
density variants / procedural primitives, env creation). Startup prints to
check in each `train_dir/simtoolreal_baseline_training/<run>/train.log`:
- `Loading asset urdf/franka_right_sharpa_description/franka_right_sharpa.urdf`
- `objectDensityRandomization: 100 variants of <object>, ... -> mass X-Y g`
  (single-object runs)
- per-epoch blocks with `successes`, `scalars/success_tolerance`, fps

## Monitoring & the ship signal

- **W&B project `simtoolreal_baseline_training`**: curves + periodic rollout
  videos per run. Healthy = rewards climbing, `closest_keypoint_max_dist`
  falling, then `scalars/success_tolerance` stepping down 0.075 -> 0.01 as
  `successes` exceeds 3.
- **Ship gate**: each run has a `ship_gate_watcher.py` attached. When a run
  sustains tolerance 0.01 with successes >= 3 it writes `SHIP_READY` in the
  run dir and fires a W&B alert. **Training keeps going** — nothing to do on
  the server except confirm the alert reached anthony.
- Reference points: the paper's primitive training took ~7 days on an A6000;
  a 5090 is faster and single-object tasks saturate sooner. If a run shows no
  reward progress after 24 h, flag it rather than restarting.

## Harvest (after SHIP_READY)

Ship-gated checkpoints are validated ON THE DEV MACHINE before any robot use
(deployment-pipeline eval + video review — the gate that caught the v1 hover
bugs). To hand a checkpoint over:

```bash
RUN=train_dir/simtoolreal_baseline_training/<run_name>
gh release create v3-<object> \
  "$RUN/runs/<run_name>/nn/<run_name>.pth#model.pth" \
  "$RUN/runs/<run_name>/config.yaml#config.yaml" \
  --repo anthebol2/simtoolreal-franka --title "v3 <object> checkpoint"
```

(nn/<run_name>.pth is the best-so-far model; last_* files are periodic fulls.)

## Rules

1. Never `pip install` with dependency resolution into the cloned env.
2. Never edit launch_v3_training.py values — geometry and SAPG numbers are
   load-bearing; changes go through the dev machine.
3. One Isaac Gym process per GPU (the launcher enforces this via
   CUDA_VISIBLE_DEVICES).
4. Don't stop runs at SHIP_READY; don't restart runs without flagging.
5. New objects (scene2/scene7) arrive as repo updates: `git pull`, then
   `python isaacgymenvs/launch_v3_training.py --object <name> --gpu 6` etc.
