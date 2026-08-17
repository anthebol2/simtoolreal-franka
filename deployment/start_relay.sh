#!/bin/bash
# One-command relay launcher.
#   bash deployment/start_relay.sh                       # hammer_002_scanned, yaw -88.5
#   bash deployment/start_relay.sh big_hammer            # other object, yaw -88.5
#   bash deployment/start_relay.sh big_hammer -90.0      # object + yaw override
cd "$(dirname "$0")/.."
OBJ="${1:-hammer_002_scanned}"
YAW="${2:--88.5}"
export ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1
exec /home/wmingd/Miniconda3/envs/simtoolreal/bin/python \
    deployment/object_pose_relay_node.py \
    --reframe-json "deployment/${OBJ}_reframe.json" \
    --yaw-deg "$YAW"
