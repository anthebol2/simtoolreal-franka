#!/usr/bin/env python
"""Minimal chain test: wiggle ONE wrist joint 12 deg out and back, slowly,
through the exact deployment chain (policy topic -> bridge -> controller).
No policy involved. If THIS faults, the plant is broken; if it moves smoothly,
the chain is ready for the policy.

    python deployment/wiggle_test.py            # j7, 12 deg, 4 s each way
    python deployment/wiggle_test.py --joint 5 --deg 8
"""

import argparse

import numpy as np
import rospy

import home_robot as hr


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--joint", type=int, default=6, help="arm joint index 0..6 (default j7)")
    p.add_argument("--deg", type=float, default=12.0)
    p.add_argument("--seconds", type=float, default=4.0)
    p.add_argument("--all", action="store_true",
                   help="move ALL 7 arm joints simultaneously (policy-shaped test)")
    args = p.parse_args()

    hr.PROFILE = hr.get_robot_profile("franka_right_sharpa")
    rospy.init_node("wiggle_test", anonymous=True)
    rospy.Subscriber("/franka/joint_states", hr.JointState, hr.current_joint_pos_iiwa_callback, queue_size=1)
    rospy.Subscriber("/sharpa/joint_states", hr.JointState, hr.current_joint_pos_sharpa_callback, queue_size=1)
    pub_iiwa = rospy.Publisher("/franka/joint_cmd", hr.JointState, queue_size=1)
    pub_sharpa = rospy.Publisher("/sharpa/joint_cmd", hr.JointState, queue_size=1)

    while not rospy.is_shutdown() and (hr.CURRENT_JOINT_POS_IIWA is None or hr.CURRENT_JOINT_POS_SHARPA is None):
        print("waiting for joint states...")
        rospy.sleep(0.2)

    start = np.concatenate([hr.CURRENT_JOINT_POS_IIWA.copy(), hr.CURRENT_JOINT_POS_SHARPA.copy()])
    target = start.copy()
    if args.all:
        # policy-shaped: all 7 joints at once, wrist included, alternating signs
        deltas = np.deg2rad(args.deg) * np.array([1, -1, 1, -1, 1, -1, 1])
        target[:7] += deltas
        args.joint = 6  # report on the wrist joint
        print(f"WIGGLE-ALL: 7 joints ±{args.deg} deg simultaneously over {args.seconds}s ...")
    else:
        target[args.joint] += np.deg2rad(args.deg)
        print(f"WIGGLE: joint {args.joint + 1} +{args.deg} deg over {args.seconds}s ... (watch the arm!)")
    hr.move_to_pose(target, pub_iiwa=pub_iiwa, pub_sharpa=pub_sharpa, move_time=args.seconds)
    rospy.sleep(0.5)
    moved = np.degrees(abs(hr.CURRENT_JOINT_POS_IIWA[args.joint] - start[args.joint]))
    print(f"  measured motion: {moved:.1f} deg  ->  {'ARM MOVED' if moved > args.deg * 0.5 else 'ARM DID NOT FOLLOW'}")

    print("returning...")
    hr.move_to_pose(start, pub_iiwa=pub_iiwa, pub_sharpa=pub_sharpa, move_time=args.seconds)
    print("done.")


if __name__ == "__main__":
    main()
