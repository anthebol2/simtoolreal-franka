#!/bin/bash
# Launch all v3 baseline trainings, one per GPU, each with a ship-gate watcher.
# Run from the repo root with the Isaac Gym environment active.
#
#   bash isaacgymenvs/launch_all_v3.sh              # all 6 runs (GPUs 0-5)
#   NUM_ENVS=8192 bash isaacgymenvs/launch_all_v3.sh   # smaller envs if OOM
#
# GPU assignment (all 8 objects onboarded):
RUNS=(
  "0 yoga_can"
  "1 half_cylinder_D10_W5_scanned"
  "2 salt_can"
  "3 water_cup"
  "4 drill_blue"
  "5 handle_head_primitives"
  "6 eggpie"
  "7 small_flashlight"
)

set -e
NUM_ENVS="${NUM_ENVS:-12288}"
PROJECT="simtoolreal_baseline_training"

mkdir -p train_dir/"$PROJECT"

for run in "${RUNS[@]}"; do
  read -r gpu object <<< "$run"
  echo "=== launching $object on GPU $gpu (num_envs=$NUM_ENVS) ==="
  nohup python isaacgymenvs/launch_v3_training.py \
    --object "$object" --gpu "$gpu" --num_envs "$NUM_ENVS" \
    --wandb_project "$PROJECT" \
    > "train_dir/$PROJECT/launcher_${object}.log" 2>&1 &
  sleep 5
done

# Give the runs time to create their hydra dirs, then attach a watcher to each
sleep 60
for run in "${RUNS[@]}"; do
  read -r gpu object <<< "$run"
  latest=$(ls -td train_dir/"$PROJECT"/00_v3_"$object"_* 2>/dev/null | head -1)
  if [ -n "$latest" ]; then
    nohup python isaacgymenvs/ship_gate_watcher.py \
      --log "$latest/train.log" --wandb_project "$PROJECT" \
      > "$latest/ship_gate.log" 2>&1 &
    echo "watcher attached: $latest"
  else
    echo "WARNING: no run dir found for $object yet - attach watcher manually"
  fi
done

echo "All launched. Monitor: wandb project '$PROJECT' + train_dir/$PROJECT/*/train.log"
