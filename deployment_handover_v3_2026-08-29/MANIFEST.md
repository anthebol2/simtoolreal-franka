# Manifest — v3 deployment handover (updated 2026-08-31)

## What's here (committed to git — you already have it after `git pull`)

```
deployment_handover_v3_2026-08-29/
├── HANDOVER_V3_DEPLOYMENT.md      # READ FIRST — sim-side changes, deployment quickstart, caveats
├── MANIFEST.md                    # this file
├── MODEL_CHECKSUMS.md5            # md5s of the 6 model.pth files (run: md5sum -c MODEL_CHECKSUMS.md5)
├── salt_can/config.yaml
├── half_cylinder_D10_W5_scanned/config.yaml
├── water_cup/config.yaml
├── drill_blue/config.yaml
├── small_flashlight/config.yaml
└── yoga_can/config.yaml
```

## Checkpoints (NOT in git — pull from GitHub Release)

Release: **`v3-baselines-2026-08-29`** at <https://github.com/anthebol2/simtoolreal-franka/releases>

Assets (as of 2026-08-31):
- `salt_can_model.pth` (~250 MB) — unchanged from 2026-08-29 upload
- `half_cylinder_D10_W5_scanned_model.pth` (~250 MB) — unchanged from 2026-08-29 upload
- `water_cup_model.pth` (~250 MB) — unchanged from 2026-08-29 upload (v2 retrain planned; this will be superseded)
- `drill_blue_model.pth` (~250 MB) — **refreshed 2026-08-31** (8.13 → 9.90 successes)
- `small_flashlight_model.pth` (~250 MB) — **refreshed 2026-08-31** (18.93 → 24.09 successes)
- `yoga_can_model.pth` (~180 MB) — **NEW 2026-08-31** (15.08 successes; smaller size = num_envs=6144 training, see handover §4.1)

```bash
gh release download v3-baselines-2026-08-29 \
    --repo anthebol2/simtoolreal-franka \
    --dir deployment_handover_v3_2026-08-29 \
    --pattern '*.pth'
# then place each model.pth in the per-object subdir (see HANDOVER_V3_DEPLOYMENT.md §1a)
md5sum -c MODEL_CHECKSUMS.md5     # must show OK for all 6
```

## Object meshes / URDFs

Already tracked in the main repo at `assets/urdf/dextoolbench/<category>/<object>/` — no duplication in this handover dir. Pull the latest `franka-right-sharpa` branch to get them (small_flashlight was added in commit `cbf16c9`).

## Per-object summary (from HANDOVER_V3_DEPLOYMENT.md §0)

| object | mean_successes @ 1cm | ship margin | training status |
|---|---|---|---|
| salt_can | 40.83 | 13.6× | killed 2026-08-26 (plateaued) |
| small_flashlight | 24.09 | 8.0× | killed after 2026-08-31 refresh |
| half_cylinder_D10_W5_scanned | 16.67 | 5.5× | killed 2026-08-26 (well past bar) |
| yoga_can | 15.08 | 5.0× | killed after 2026-08-31 refresh; required num_envs=6144 (see handover §4.1) |
| water_cup | 13.18 | 4.4× | v2 retrain planned |
| drill_blue | 9.90 | 3.3× | killed after 2026-08-31 refresh |

## What is intentionally NOT here

- **handle_head_primitives** — the paper's generalist baseline. Now shipped (4.73 successes, 1.6× margin) but thin — still training and climbing. Will be added to a future release.
- **eggpie** — not trained (user chose to skip).
- **_var000.urdf … _var099.urdf** density-variant URDFs — training-only randomization, deployment uses the single canonical `_decomposed.urdf`.
