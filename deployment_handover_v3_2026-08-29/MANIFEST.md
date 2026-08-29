# Manifest — v3 deployment handover (2026-08-29)

## What's here (committed to git — you already have it after `git pull`)

```
deployment_handover_v3_2026-08-29/
├── HANDOVER_V3_DEPLOYMENT.md      # READ FIRST — sim-side changes, deployment quickstart, caveats
├── MANIFEST.md                    # this file
├── MODEL_CHECKSUMS.md5            # md5s of the 5 model.pth files (run: md5sum -c MODEL_CHECKSUMS.md5)
├── salt_can/config.yaml
├── half_cylinder_D10_W5_scanned/config.yaml
├── water_cup/config.yaml
├── drill_blue/config.yaml
└── small_flashlight/config.yaml
```

## Checkpoints (NOT in git — pull from GitHub Release)

Release: **`v3-baselines-2026-08-29`** at <https://github.com/anthebol2/simtoolreal-franka/releases>

Each `<object>_model.pth` (~263 MB) attaches as a release asset. Total 1.3 GB.

```bash
gh release download v3-baselines-2026-08-29 \
    --repo anthebol2/simtoolreal-franka \
    --dir deployment_handover_v3_2026-08-29 \
    --pattern '*.pth'
# then place each model.pth in the per-object subdir (see HANDOVER_V3_DEPLOYMENT.md §1a)
md5sum -c MODEL_CHECKSUMS.md5     # must show OK for all 5
```

## Object meshes / URDFs

Already tracked in the main repo at `assets/urdf/dextoolbench/<category>/<object>/` — no duplication in this handover dir. Pull the latest `franka-right-sharpa` branch to get them (small_flashlight was added in commit `cbf16c9`).

## Per-object summary (from HANDOVER_V3_DEPLOYMENT.md §0)

| object | mean_successes @ 1cm | ship margin | training status |
|---|---|---|---|
| salt_can | 40.83 | 13.6× | killed 2026-08-26 (plateaued) |
| half_cylinder_D10_W5_scanned | 16.67 | 5.5× | killed 2026-08-26 (well past bar) |
| small_flashlight | 18.93 | 6.3× | still training, climbing fast (was 9.4 → 18.9 in 24h) |
| water_cup | 12.07 | 4.0× | still training, still climbing |
| drill_blue | 8.13 | 2.7× | still training, still climbing |

## What is intentionally NOT here

- **yoga_can** — still training, not yet at final tolerance. Do not deploy.
- **handle_head_primitives** — paper's generalist baseline; oscillating at the ship gate but not sustained. Do not deploy yet.
- **eggpie** — not trained (user chose to skip).
- **_var000.urdf … _var099.urdf** density-variant URDFs — training-only randomization, deployment uses the single canonical `_decomposed.urdf`.
