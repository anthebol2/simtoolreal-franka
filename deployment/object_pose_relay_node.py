#!/usr/bin/env python
"""Relay the DexFunGrasp FoundationPose object topic to the SimToolReal convention.

The existing perception stack on this machine (DexFunGrasp
deployment/tools/run_fp_with_sam_cylinder.sh + respawn_static_tf.sh) publishes
the object pose as PoseStamped on /object/current_pose in the "world" frame,
where world == panda_link0 (the Franka base). SimToolReal's goal + policy nodes
subscribe to /robot_frame/current_object_pose, whose "robot_frame" is ALSO the
Franka base frame (see fake/fake_perception_node.py). So the relay is an
identity re-publish — but it hard-fails loudly if the source is in a camera
frame, which would silently corrupt the policy obs (~0.7 m offset).

Usage (any env with rospy):
    python deployment/object_pose_relay_node.py
    # optional: --in-topic /object/current_pose --out-topic /robot_frame/current_object_pose
"""

import argparse
import json

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

ALLOWED_FRAMES = ("world", "panda_link0", "robot_frame", "")


class ObjectPoseRelay:
    def __init__(
        self, in_topic: str, out_topic: str, reframe_json: str = "", yaw_deg: float = 0.0
    ):
        # Optional constant right-multiplied transform: the FP tracks the RAW
        # scanned mesh, but the registered dextoolbench object uses a canonical
        # (claw_hammer-like) frame. T_W_canonical = T_W_mesh @ T_mesh_canonical.
        self.T_reframe = None
        if reframe_json:
            with open(reframe_json) as f:
                self.T_reframe = np.array(json.load(f)["T_mesh_canonical"])
            assert self.T_reframe.shape == (4, 4)
        # Optional LEFT-multiplied yaw: virtual-world remap. Pair with
        # franka_robot_node --q1_offset_deg = -yaw_deg (e.g. yaw -90 <-> offset +90):
        # rotates real base-frame poses into the policy's trained virtual frame.
        self.T_yaw = None
        if abs(yaw_deg) > 1e-9:
            c, s = np.cos(np.deg2rad(yaw_deg)), np.sin(np.deg2rad(yaw_deg))
            self.T_yaw = np.array(
                [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
            )
        rospy.init_node("object_pose_relay_node")
        self.pub = rospy.Publisher(out_topic, PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber(in_topic, PoseStamped, self.cb, queue_size=1)
        self._warned_frames = set()
        self._n = 0
        rospy.loginfo(f"[relay] {in_topic} -> {out_topic} (identity, base frame)")

    def cb(self, msg: PoseStamped):
        frame = msg.header.frame_id
        if frame not in ALLOWED_FRAMES:
            if frame not in self._warned_frames:
                self._warned_frames.add(frame)
                rospy.logerr(
                    f"[relay] REFUSING to relay: source frame_id={frame!r} is not a "
                    f"base frame {ALLOWED_FRAMES}. The object FP is probably publishing "
                    "in CAMERA frame — run respawn_static_tf.sh and launch the object FP "
                    "with --world-frame world (see README_AUTOGRASP_REPLAY.md step 8)."
                )
            return
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "robot_frame"
        out.pose = msg.pose
        if self.T_reframe is not None or self.T_yaw is not None:
            p, q = msg.pose.position, msg.pose.orientation
            T = np.eye(4)
            T[:3, 3] = (p.x, p.y, p.z)
            T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            if self.T_reframe is not None:
                T = T @ self.T_reframe
            if self.T_yaw is not None:
                T = self.T_yaw @ T
            out.pose.position.x, out.pose.position.y, out.pose.position.z = T[:3, 3]
            qx, qy, qz, qw = R.from_matrix(T[:3, :3]).as_quat()
            out.pose.orientation.x = qx
            out.pose.orientation.y = qy
            out.pose.orientation.z = qz
            out.pose.orientation.w = qw
        self.pub.publish(out)
        self._n += 1
        if self._n % 300 == 1:
            p = msg.pose.position
            rospy.loginfo(
                f"[relay] {self._n} msgs; latest xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m"
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--in-topic", default="/object/current_pose")
    parser.add_argument("--out-topic", default="/robot_frame/current_object_pose")
    parser.add_argument(
        "--reframe-json",
        default="",
        help="JSON with T_mesh_canonical (4x4); right-multiplied onto every pose "
        "(e.g. deployment/hammer_002_scanned_reframe.json)",
    )
    parser.add_argument(
        "--yaw-deg",
        type=float,
        default=0.0,
        help="Virtual-world yaw remap (left-multiplied Rz). Use -90 with "
        "franka_robot_node --q1_offset_deg 90 when the physical workspace is at "
        "base +x instead of the trained -y.",
    )
    args = parser.parse_args()
    ObjectPoseRelay(args.in_topic, args.out_topic, args.reframe_json, args.yaw_deg)
    rospy.spin()


if __name__ == "__main__":
    main()
