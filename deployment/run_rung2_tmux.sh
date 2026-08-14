#!/bin/bash
# Rung 2 of HANDOVER_REAL_DEPLOYMENT.md — full ROS node graph, NO hardware.
# Launches roscore + fake robot + fake perception + goal node + policy node in
# one tmux session and a /franka/joint_cmd rate check. Expect ~60 Hz.
#
#   bash deployment/run_rung2_tmux.sh          # start
#   tmux attach -t simtoolreal_rung2           # watch
#   tmux kill-session -t simtoolreal_rung2     # stop everything
#
# SAFE only if no real robot drivers are running: the policy node publishes
# /sharpa/joint_cmd and /franka/joint_cmd — the script refuses to start if a
# rosmaster is already up.
# Pane addressing uses tmux pane IDs, so it works with any base-index config.

set -euo pipefail
SESSION=simtoolreal_rung2
REPO=/home/wmingd/Desktop/DexFunGrasp/simtoolreal-franka

ENV_SETUP='source /opt/ros/noetic/setup.zsh; export ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1; source /home/wmingd/Miniconda3/etc/profile.d/conda.sh; conda activate simtoolreal; cd '"$REPO"

if bash -c "source /opt/ros/noetic/setup.bash; timeout 2 rostopic list" >/dev/null 2>&1; then
    echo "ERROR: a rosmaster is already running — refusing to start rung 2 on it."
    echo "Stop all ROS nodes/masters first, then re-run."
    exit 1
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION"

# roscore runs in the PLAIN system ROS env — conda's lib path breaks rosout.
# Panes default to the user's zsh; source the zsh variant of the ROS setup
# (memory/reference_ros_quirks: setup.bash inside zsh half-works, use setup.zsh).
ROS_ONLY='source /opt/ros/noetic/setup.zsh; export ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1'
P0=$(tmux list-panes -t "$SESSION" -F '#{pane_id}' | head -1)
tmux send-keys -t "$P0" "$ROS_ONLY; roscore" C-m
sleep 3

new_pane() { tmux split-window -t "$SESSION" -P -F '#{pane_id}'; tmux select-layout -t "$SESSION" tiled >/dev/null; }

P1=$(new_pane); tmux send-keys -t "$P1" "$ENV_SETUP; sleep 2; python deployment/fake/fake_robot_node.py" C-m
P2=$(new_pane); tmux send-keys -t "$P2" "$ENV_SETUP; sleep 2; python deployment/fake/fake_perception_node.py" C-m
P3=$(new_pane); tmux send-keys -t "$P3" "$ENV_SETUP; sleep 4; python deployment/goal_pose_node.py --object_category hammer --object_name claw_hammer --task_name swing_down" C-m
# thread caps: torch small-model inference is fastest with few threads; the
# default (all cores) thrashes and drops the 60 Hz control loop to ~43 Hz
P4=$(new_pane); tmux send-keys -t "$P4" "$ENV_SETUP; sleep 6; OMP_NUM_THREADS=2 MKL_NUM_THREADS=2 python deployment/rl_policy_node.py --policy_path franka_policy_v1 --object_name claw_hammer" C-m
P5=$(new_pane); tmux send-keys -t "$P5" "$ENV_SETUP; sleep 20; echo '--- expect ~60 Hz: ---'; rostopic hz /franka/joint_cmd" C-m

echo "Started. Attach with:  tmux attach -t $SESSION"
echo "Pass criteria: the last pane shows 'average rate: ~59-60' for /franka/joint_cmd,"
echo "the policy pane has no red errors, joint names are panda_joint1..7."
