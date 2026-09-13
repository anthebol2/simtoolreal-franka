# Manifest — v3 deployment handover (updated 2026-09-11)

## Complete release (10 checkpoints, all on GitHub)

Release: **`v3-baselines-2026-08-29`** at <https://github.com/anthebol2/simtoolreal-franka/releases/tag/v3-baselines-2026-08-29>

| asset | size | notes |
|---|---|---|
| `salt_can_model.pth` | 250 MB | v1 baseline, banked |
| `salt_can_v2_model.pth` | 250 MB | v2 retrain, mean=25 |
| `half_cylinder_D10_W5_scanned_model.pth` | 250 MB | banked |
| `water_cup_model.pth` | 250 MB | v1 baseline |
| `water_cup_v2_model.pth` | 174 MB | v2 retrain, mean=34 (best of batch) |
| `drill_blue_model.pth` | 250 MB | banked |
| `small_flashlight_model.pth` | 250 MB | banked |
| `yoga_can_model.pth` | 174 MB | banked (num_envs=6144) |
| `gum_can_model.pth` | 250 MB | ⭐ new 2026-09-11 — scene2 gum can, mean=22 |
| `handle_head_primitives_model.pth` | 250 MB | paper's generalist baseline, peak=4.7 |

**All v3 training runs are complete.** No pending training on dex5090-1.

## What's in git (docs + configs, small)

```
deployment_handover_v3_2026-08-29/
├── HANDOVER_V3_DEPLOYMENT.md      # READ FIRST
├── MANIFEST.md                    # this file
├── MODEL_CHECKSUMS.md5            # md5s of all 10 model.pth files
├── salt_can/config.yaml
├── salt_can_v2/config.yaml
├── half_cylinder_D10_W5_scanned/config.yaml
├── water_cup/config.yaml
├── water_cup_v2/config.yaml
├── drill_blue/config.yaml
├── small_flashlight/config.yaml
├── yoga_can/config.yaml
├── gum_can/config.yaml
└── handle_head_primitives/config.yaml
```

## One-shot pull command for hardware side

```bash
git pull origin franka-right-sharpa                    # docs + configs
gh release download v3-baselines-2026-08-29 \
    --repo anthebol2/simtoolreal-franka \
    --dir /tmp/v3_dl --pattern '*.pth'
cd deployment_handover_v3_2026-08-29
for obj in salt_can salt_can_v2 half_cylinder_D10_W5_scanned water_cup water_cup_v2 \
           drill_blue small_flashlight yoga_can gum_can handle_head_primitives; do
    mkdir -p $obj
    mv /tmp/v3_dl/${obj}_model.pth $obj/model.pth
done
md5sum -c MODEL_CHECKSUMS.md5     # 10 x OK
```

## Object meshes / URDFs

Already tracked in the main repo at `assets/urdf/dextoolbench/<category>/<object>/` — pull the latest `franka-right-sharpa` branch.

## Per-object summary (from HANDOVER_V3_DEPLOYMENT.md §0)

| object | mean_successes @ 1cm | ship margin | notes |
|---|---|---|---|
| salt_can | 40.83 | 13.6× | v1, plateaued near ceiling |
| water_cup_v2 | 34.45 | 11.5× | best of the batch |
| salt_can_v2 | 25.03 | 8.3× | v2 retrain |
| small_flashlight | 24.09 | 8.0× | killed final |
| gum_can | 22.34 | 7.4× | scene2 gum can, killed final 2026-09-11 |
| yoga_can | 20.94 | 7.0× | killed final; num_envs=6144 (PhysX limit) |
| half_cylinder_D10_W5_scanned | 16.67 | 5.5× | v1, banked |
| water_cup | 13.18 | 4.4× | v1, sealed mesh |
| drill_blue | 11.52 | 3.8× | killed final |
| handle_head_primitives | 4.70 (peak) | 1.6× | paper's generalist; thin margin |

## Not in this package

- **shelf_block** — onboarded 2026-09-01 but never trained (scene2 was later reassigned to gum_can).
- **eggpie** — not trained.
- **`_var000.urdf` … `_var099.urdf`** density-variant URDFs — training-only randomization, deployment uses the single canonical `_decomposed.urdf`.
