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

## Checkpoints (NOT in git — split across GitHub Release + rsync)

As of 2026-08-31, GitHub uploads from `dex5090-1` are throttled (0 bytes received in 85 s). See `HANDOVER_V3_DEPLOYMENT.md` §1a for full details. Short version:

**On GitHub Release `v3-baselines-2026-08-29`** at <https://github.com/anthebol2/simtoolreal-franka/releases>:
- `salt_can_model.pth` (~250 MB)
- `half_cylinder_D10_W5_scanned_model.pth` (~250 MB)
- `water_cup_model.pth` (~250 MB) — v2 retrain planned; this will be superseded

**Via rsync from `dex5090-1`** (blocked from GitHub Release by network throttling):
- `yoga_can/model.pth` (~180 MB) — 15.08 successes, num_envs=6144 (see handover §4.1)
- `drill_blue/model.pth` (~250 MB) — 9.90 successes
- `small_flashlight/model.pth` (~250 MB) — 24.09 successes

Combined command for the hardware machine (assuming SSH to `dex5090-1` works):
```bash
# 3 from GitHub Release
gh release download v3-baselines-2026-08-29 --repo anthebol2/simtoolreal-franka --dir /tmp/v3_dl --pattern '*.pth'
for obj in salt_can half_cylinder_D10_W5_scanned water_cup; do
    mkdir -p deployment_handover_v3_2026-08-29/$obj
    mv /tmp/v3_dl/${obj}_model.pth deployment_handover_v3_2026-08-29/$obj/model.pth
done
# 3 via rsync
for obj in yoga_can drill_blue small_flashlight; do
    mkdir -p deployment_handover_v3_2026-08-29/$obj
    rsync -avP dex5090-1:/home/lipuhao/develop/simtoolreal-franka/deployment_handover_v3_2026-08-29/$obj/model.pth \
        deployment_handover_v3_2026-08-29/$obj/model.pth
done
# verify
md5sum -c deployment_handover_v3_2026-08-29/MODEL_CHECKSUMS.md5     # 6 x OK
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
