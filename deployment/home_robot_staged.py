#!/usr/bin/env python
"""Two-stage HOME that clears the platform: LIFT first, then rotate the base.

Why: plain home_robot.py interpolates all joints at once. From a pose parked low
near the platform, the joint-1 base sweep drags the hand laterally THROUGH the
platform zone at riser height (caused a cartesian-reflex collision stop on
2026-08-13). This version:

  stage 1: keep joint 1 fixed, move joints 2..7 to the HOME arm shape
           -> the hand rises to ~0.88 m at its current azimuth (clear of the riser)
  stage 2: rotate joint 1 to the HOME azimuth at altitude -> arcs OVER everything

Uses home_robot.py's own publish/interpolate machinery; same 10 s pacing per
stage, same topics, same virtual-frame convention (the bridge applies q1 offset).
"""

import argparse

import numpy as np
import rospy

import home_robot as hr


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot",
        default="franka_right_sharpa",
        choices=["franka_right_sharpa", "kuka_left_sharpa"],
    )
    parser.add_argument("--stage_time", type=float, default=10.0)
    args = parser.parse_args()
    hr.PROFILE = hr.get_robot_profile(args.robot)
    arm_ns = hr.PROFILE.ros_arm_ns

    rospy.init_node("home_robot_staged", anonymous=True)
    rospy.Subscriber(
        f"/{arm_ns}/joint_states",
        hr.JointState,
        hr.current_joint_pos_iiwa_callback,
        queue_size=1,
    )
    rospy.Subscriber(
        "/sharpa/joint_states",
        hr.JointState,
        hr.current_joint_pos_sharpa_callback,
        queue_size=1,
    )
    pub_iiwa = rospy.Publisher(f"/{arm_ns}/joint_cmd", hr.JointState, queue_size=1)
    pub_sharpa = rospy.Publisher("/sharpa/joint_cmd", hr.JointState, queue_size=1)

    while not rospy.is_shutdown() and (
        hr.CURRENT_JOINT_POS_IIWA is None or hr.CURRENT_JOINT_POS_SHARPA is None
    ):
        print("Waiting for joint states...")
        rospy.sleep(0.2)

    home = hr.home_joint_pos(hr.PROFILE)  # (29,) virtual frame

    # stage 1: home arm SHAPE at the CURRENT base azimuth; hand joints -> home's
    stage1 = home.copy()
    stage1[0] = hr.CURRENT_JOINT_POS_IIWA[0]  # keep current (virtual) joint 1
    print(f"[stage 1/2] LIFT: arm shape -> home, joint1 held at {stage1[0]:.3f} rad")
    hr.move_to_pose(stage1, pub_iiwa=pub_iiwa, pub_sharpa=pub_sharpa,
                    move_time=args.stage_time)
    rospy.sleep(0.5)

    print(f"[stage 2/2] ROTATE at altitude: joint1 -> {home[0]:.3f} rad")
    hr.move_to_pose(home, pub_iiwa=pub_iiwa, pub_sharpa=pub_sharpa,
                    move_time=args.stage_time)
    print("Reached home pose (staged)")


if __name__ == "__main__":
    main()
