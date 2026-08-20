"""Launcher for the v3 baseline trainings (real-bench geometry, Franka + right SharPa).

One process = one policy = one GPU. Wraps isaacgymenvs.train with EXACTLY the
paper's SAPG configuration (same overrides as launch_training.py) plus the
v3 geometry block (HANDOVER_V3_RETRAIN.md, tape-measured 2026-08-17) and the
object selection:

  --object handle_head_primitives   -> the original simtoolreal training
                                       (procedural primitives, sampled densities)
  --object <name>                   -> single-object baseline training; the object
                                       gets numAssetsPerType density-sampled
                                       variants (Uniform 300-600 kg/m^3, the
                                       primitive printed-object convention)

Examples:
  python isaacgymenvs/launch_v3_training.py --object yoga_can --gpu 0
  python isaacgymenvs/launch_v3_training.py --object handle_head_primitives --gpu 5
"""

import os
import subprocess
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import tyro

# ---------------------------------------------------------------------------
# v3 geometry (validated: reach audit 100% coverage of goal volume + grasp band)
# ---------------------------------------------------------------------------
V3_GEOMETRY_OVERRIDES = [
    'task.env.asset.table="urdf/table_real_bench.urdf"',  # 0.81 x 1.00 m bench
    "task.env.tableResetZ=0.034",  # table top at the measured 0.184 m
    "task.env.tableResetZRange=0.03",  # +-3 cm (real bench tolerance)
    "++task.env.tableDistanceFromBase=0.405",  # robot mounted AT the table edge
    "++task.env.tableObjectYOffset=-0.145",  # spawn center in the 0.45-0.70 m band
    "task.env.targetVolumeMins=[-0.35,-0.05,0.254]",
    "task.env.targetVolumeMaxs=[0.35,0.2,0.604]",
]

KNOWN_OBJECTS = [
    "handle_head_primitives",
    "yoga_can",
    "half_cylinder_D10_W5_scanned",
    "salt_can",
    "water_cup",
    "drill_blue",
    "hammer_002_scanned",
    "big_hammer",
    "brush_scanned",
]


@dataclass
class LaunchV3Args:
    object: str = "handle_head_primitives"
    """Object to train (or handle_head_primitives for the generalist run)."""

    gpu: int = 0
    """GPU index (sets CUDA_VISIBLE_DEVICES for this run)."""

    num_envs: int = 12288
    """Per-GPU env count (12288 fits a 32 GB RTX 5090 with headroom; must be
    divisible by num_blocks)."""

    num_blocks: int = 6
    """SAPG block count (paper value)."""

    seed: int = 0

    checkpoint: Optional[Path] = None
    """Optional checkpoint to finetune from."""

    wandb_project: str = "simtoolreal_baseline_training"

    wandb_entity: Optional[str] = None
    """None = the logged-in account's default entity."""

    wandb_activate: bool = True

    wandb_tags: List[str] = field(default_factory=list)

    experiment_suffix: str = ""
    """Optional extra tag appended to the experiment name."""

    dry_run: bool = False
    """Print the command instead of running it."""

    extra: List[str] = field(default_factory=list)
    """Extra hydra overrides appended verbatim (e.g. smoke tests:
    --extra train.params.config.max_epochs=3 task.env.numEnvs=48)."""

    def __post_init__(self) -> None:
        assert self.num_envs % self.num_blocks == 0
        assert self.object in KNOWN_OBJECTS, (
            f"Unknown object {self.object}; known: {KNOWN_OBJECTS}"
        )

    @property
    def sapg_block_size(self) -> int:
        return self.num_envs // self.num_blocks


def main() -> None:
    args = tyro.cli(LaunchV3Args)

    now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    suffix = f"_{args.experiment_suffix}" if args.experiment_suffix else ""
    # NOTE: the leading integer is parsed as policy_idx by rl_games — keep it.
    experiment_name = f"00_v3_{args.object}{suffix}_{now}"
    hydra_run_dir = f"./train_dir/{args.wandb_project}/{experiment_name}"

    minibatch_size = args.num_envs * 16 // 4  # paper ratio: batch/4 (24576->98304)

    cmd_parts = [
        "python",
        "-m",
        "isaacgymenvs.train",
        # === paper SAPG configuration (identical to launch_training.py) ===
        "++task.env.useSparseReward=False",
        "headless=True",
        f"task.env.numEnvs={args.num_envs}",
        f"train.params.config.minibatch_size={minibatch_size}",
        f"train.params.config.central_value_config.minibatch_size={minibatch_size}",
        "multi_gpu=False",
        "train.params.config.good_reset_boundary=0",
        "task.env.goodResetBoundary=0",
        "train.params.config.use_others_experience=lf",
        "train.params.config.off_policy_ratio=1.0",
        "train.params.config.expl_type=mixed_expl_learn_param",
        "train.params.config.expl_reward_type=entropy",
        f"train.params.config.expl_coef_block_size={args.sapg_block_size}",
        "train.params.config.expl_reward_coef_scale=0.002",
        "train.params.network.space.continuous.fixed_sigma=coef_cond",
        "task=SimToolRealLSTMAsymmetric",
        "task.env.objectScaleNoiseMultiplierRange=[0.9,1.1]",
        "task.env.forceConsecutiveNearGoalSteps=True",
        "task.env.forceScale=20",
        "task.env.torqueScale=2.0",
        "task.env.objectAngVelPenaltyScale=0.0",
        # === v3 geometry ===
        *V3_GEOMETRY_OVERRIDES,
        # === object selection ===
        f"task.env.objectName={args.object}",
        # === bookkeeping ===
        f"seed={args.seed}",
        f"experiment={experiment_name}",
        f"hydra.run.dir={hydra_run_dir}",
        f"wandb_project={args.wandb_project}",
        f"wandb_activate={args.wandb_activate}",
        f"wandb_group={datetime.now().strftime('%Y-%m-%d')}",
        f"wandb_tags=[{','.join(args.wandb_tags)}]",
    ]
    if args.object != "handle_head_primitives":
        # single-object run: primitive-convention density sampling over N variants
        cmd_parts.append("++task.env.objectDensityRandomization=True")
    if args.wandb_entity is not None:
        cmd_parts.append(f"wandb_entity={args.wandb_entity}")
    if args.checkpoint is not None:
        assert args.checkpoint.exists(), args.checkpoint
        cmd_parts.append(f"checkpoint={args.checkpoint}")
    cmd_parts.extend(args.extra)

    log_dir = Path(hydra_run_dir)
    cmd = " ".join(cmd_parts)
    print(f"[launch_v3] GPU {args.gpu} | {args.object} | {experiment_name}")
    print(cmd)
    if args.dry_run:
        return

    log_dir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env["CUDA_VISIBLE_DEVICES"] = str(args.gpu)
    log_path = log_dir / "train.log"
    with open(log_path, "a") as log_f:
        subprocess.run(cmd, shell=True, check=True, env=env,
                       stdout=log_f, stderr=subprocess.STDOUT)


if __name__ == "__main__":
    main()
