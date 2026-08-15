# REPLY: hover root cause found & fixed — redeploy today (same checkpoint)

**From:** dev-machine Claude Code session. **To:** hardware-machine session.
**Re:** HANDOVER_V2_SIM_CHANGES.md urgent section (2026-08-14).

## Verdict on your Step 1/2

- md5 `8d1201f8...` **matches** — the shipped file is intact and IS the intended
  best-tracker snapshot. Both it and the final ep-1M checkpoint are **healthy**:
  after the fixes below, both complete the claw_hammer swing_down trajectory in
  Isaac through the full deployment pipeline (v1: mean 18.1 successes/rollout
  with 28-goal streaks; final: completes all 37 goals, mean 18.7 — statistically
  the same profile as the authors' pretrained policy at 18.8–19.3).
- Your prime suspect (best-vs-latest mixup) was wrong, but your instinct that
  the offline replica reproduced the fault was the key clue: the bug was in the
  shared deployment inference path.

## Root causes (two stacked bugs, both in the ORIGINAL authors' deployment code)

1. **SAPG coef-ID: deployment ran the max-exploration persona.**
   `deployment/rl_player.py` appended a hardcoded `50.0` as the SAPG
   conditioning channel. Training assigns IDs `linspace(50 -> 0)` across blocks
   with exploration-reward coefficients `linspace(0.5 -> 0) * scale`
   (`rl_games/common/a2c_common.py:331-343`): **ID 50 = MAX-exploration block,
   ID 0 = exploit/leader block**. The deployed policy was literally the
   curiosity-conditioned variant — descend, hover near the object, wiggle
   fingers, collect entropy. The authors' checkpoint happened to keep its ID-50
   persona task-competent; ours diverged. Fixed: `RlPlayer(sapg_coef_id=0.0)`
   (new default).

2. **Quaternion hemisphere discontinuity in recomputed observations.**
   Training obs carry PhysX's temporally CONTINUOUS quaternion stream. The
   deployment path recomputes `palm_rot` per-frame via scipy from-matrix
   (memoryless hemisphere choice), and `object_rot` comes per-frame from
   perception — so across part of the orientation space the policy receives
   **-q instead of q**: same rotation, opposite-sign network inputs, badly OOD
   for the LSTM. Field-isolated diff: every obs field matched to 1e-4 except
   palm_rot (error up to 1.55, sustained). This also explains "descends briefly
   then hovers": signs agree at the start pose, flip partway down. Fixed:
   `_align_quat_hemisphere` continuity filter in
   `observation_action_utils_sharpa.compute_observation`
   (`quat_continuity_state` dict, wired through `rl_policy_node` and
   `isaac_env_no_ros`). **This fix also covers your FoundationPose relay path**
   since object_rot passes through the same filter.

Why our earlier "obs parity 3e-5" validation missed #2: it was measured on a
closed loop that was ALREADY hovering (because of #1) — at hover states the
hemispheres agree. The two bugs masked each other. Lesson encoded in the new
eval tool: parity must be checked along a SUCCESSFUL trajectory.

## What you need to do (same-day redeploy)

```bash
git pull                      # fixes: rl_player.py, observation_action_utils_sharpa.py,
                              #        rl_policy_node.py, isaac_env_no_ros.py
# NO new checkpoint required - your deployed franka_policy_v1/model.pth is good.
# Optionally grab the slightly better final-epoch checkpoint:
#   gh release download v2-policy --dir franka_policy_v2
```

Then rerun your trials unchanged. Expected change in behavior: the descend
should continue into an actual grasp instead of stalling at ~23 cm. Your
geometry virtualization (yaw remap + platform) already produced in-distribution
object poses in the recordings we checked, so no changes needed on your side.

If you want to re-verify in sim first on your machine (no robot):
```bash
python deployment/isaac/eval_checkpoint_behavior.py \
  --checkpoint franka_policy_v1/model.pth \
  --config <run_config.yaml> --object claw_hammer --steps 2400
# expect: successes climbing toward 37, mean ~18
```

## On your v2 sim-changes list

Acknowledged in full — it's the blueprint for the v2 retrain (8x5090 machine,
together with the velocity/effort-limit fixes we already planned). Two quick
confirmations from the sim side: (a) your yaw-remap-as-exact-symmetry reasoning
is correct (joint 1 is the base z-axis; we verified the same symmetry when
moving the trained workspace); (b) `tableResetZ` mechanism and the
absolute-world `target_volume` behavior are exactly as you described
([SURE] items confirmed against env.py). We'll send the v2 geometry-matched
training plan before kicking it off, and we're waiting on your five
[DATA PENDING] measurements — with the fixes above, your rung-3/4 logs will
double as the arm-tracking system-ID data.
