#!/bin/bash
# One-shot arm readiness: clear reflexes, re-engage the controller, verify, and
# print GO / NO-GO. Run this EVERY time before home_robot.py or rl_policy_node.py.
#   bash deployment/arm_ready.sh
set -u
source /opt/ros/noetic/setup.bash
unset _CATKIN_SETUP_DIR
source ~/catkin_ws_franka/devel/setup.bash
export ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1

echo "[1/4] clearing any latched reflex (error recovery)..."
rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}" >/dev/null
sleep 3

echo "[1b/4] softening internal joint impedance (factory default ~2000-3000 Nm/rad"
echo "       makes 12 Nm wrist joints fault at ~0.3 deg of tracking gap)..."
# the service needs the motion controller NOT claiming -> stop, set, restart
timeout 8 rosservice call /controller_manager/switch_controller \
    "{start_controllers: [], stop_controllers: ['position_joint_trajectory_controller', 'position_joint_position_controller'], strictness: 1}" >/dev/null 2>&1
sleep 1
if timeout 8 rosservice call /franka_control/set_joint_impedance \
    "joint_stiffness: [600.0, 600.0, 600.0, 450.0, 250.0, 150.0, 100.0]" >/dev/null 2>&1; then
    echo "    impedance set: [600 600 600 450 250 150 100] Nm/rad"
else
    echo "    WARNING: set_joint_impedance FAILED — gains remain stiff; expect tau faults"
fi
timeout 8 rosservice call /controller_manager/switch_controller \
    "{start_controllers: ['position_joint_trajectory_controller'], stop_controllers: [], strictness: 1}" >/dev/null 2>&1

echo "[2/4] checking position controller is loaded..."
SUB=$(rostopic info /position_joint_trajectory_controller/command 2>/dev/null | grep -A1 Subscribers | tail -1)
if ! echo "$SUB" | grep -q franka_control; then
    echo "    controller not loaded — spawning it now (spawner stays alive: a"
    echo "    dying spawner UNLOADS its controller — never wrap it in timeout)..."
    nohup rosrun controller_manager spawner position_joint_trajectory_controller \
        >/tmp/position_controller_spawner.log 2>&1 &
    disown
    sleep 5
fi

echo "[3/4] reading robot state..."
STATE=$(timeout 5 rostopic echo -n1 /franka_state_controller/franka_states 2>/dev/null)
MODE=$(echo "$STATE" | grep -E "^robot_mode" | awk '{print $2}')
RATE=$(echo "$STATE" | grep -E "^control_command_success_rate" | awk '{print $2}')
BRIDGE=$(timeout 3 rostopic echo -n1 /franka/joint_states >/dev/null 2>&1 && echo up || echo DOWN)

echo "[4/4] verdict:"
echo "    robot_mode=$MODE  cmd_success=$RATE  bridge=$BRIDGE"
if [ "$MODE" = "2" ] && [ "$BRIDGE" = "up" ]; then
    echo "    ================ GO — arm engaged and commanded ================"
elif [ "$MODE" = "1" ]; then
    echo "    NO-GO: robot idle — controller did not engage. Re-run this script;"
    echo "           if it persists, restart franka_control and run this again."
elif [ "$MODE" = "4" ]; then
    echo "    NO-GO: still in reflex — re-run this script; if persistent, check"
    echo "           the arm isn't physically touching anything."
else
    echo "    NO-GO: mode=$MODE — franka_control may be down (no state received)."
fi
