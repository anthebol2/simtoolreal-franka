#!/usr/bin/env python
"""Measure the real SharPa finger step response (v2 sim-matching data).

MOVES THE HAND - a human must run this, e-stop mentality, fingers clear of
objects/table. Requires sharpa_node running (DexFunGrasp's, which clamps).

Sends a step on one joint (default: index MCP FE, 0 -> 60 deg -> 0), records
/sharpa/joint_states, writes a CSV of t, commanded, measured. Repeat for a
couple of joints (index PIP, thumb CMC FE) to characterize the hand.

    python deployment/measure_finger_step_response.py --joint-idx 5 --step-deg 60
    # joint indices: canonical thumb->pinky order, see sharpa_node JOINT_NAMES
    # 0=thumb CMC FE, 5=index MCP FE, 7=index PIP, ...

Output: finger_step_joint<idx>_<stamp>.csv in the repo root.
The v2 fit: rise time / effective joint velocity -> URDF finger velocity limit
+ handMovingAverage. Send the CSVs to the dev machine.
"""

import argparse
import csv
import time

import numpy as np
import rospy
from sensor_msgs.msg import JointState

N_JOINTS = 22


class StepResponse:
    def __init__(self, joint_idx: int, step_deg: float, hold_s: float):
        self.joint_idx = joint_idx
        self.step_rad = np.deg2rad(step_deg)
        self.hold_s = hold_s
        self.measured = []  # (t, q[joint_idx])
        self.commanded = []  # (t, cmd[joint_idx])

        rospy.init_node("finger_step_response")
        self.pub = rospy.Publisher("/sharpa/joint_cmd", JointState, queue_size=1)
        self.sub = rospy.Subscriber(
            "/sharpa/joint_states", JointState, self._cb, queue_size=10
        )

    def _cb(self, msg: JointState):
        self.measured.append((time.time(), float(msg.position[self.joint_idx])))

    def _send(self, value_rad: float):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = [0.0] * N_JOINTS
        msg.position[self.joint_idx] = value_rad
        self.pub.publish(msg)
        self.commanded.append((time.time(), value_rad))

    def run(self):
        rospy.sleep(1.0)
        assert self.measured, "no /sharpa/joint_states - is sharpa_node running?"
        print(f"baseline ok; stepping joint {self.joint_idx} to "
              f"{np.rad2deg(self.step_rad):.0f} deg for {self.hold_s}s")
        t0 = time.time()
        # publish the target at 20 Hz during each phase (sharpa_node latches anyway)
        for target, dur in [(self.step_rad, self.hold_s), (0.0, self.hold_s)]:
            t_phase = time.time()
            while time.time() - t_phase < dur and not rospy.is_shutdown():
                self._send(target)
                rospy.sleep(0.05)
        out = f"finger_step_joint{self.joint_idx}_{int(t0)}.csv"
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "kind", "value_rad"])
            for t, v in self.commanded:
                w.writerow([t - t0, "cmd", v])
            for t, v in self.measured:
                w.writerow([t - t0, "meas", v])
        meas = np.array([(t - t0, v) for t, v in self.measured])
        rise = meas[(meas[:, 1] > 0.1 * self.step_rad) & (meas[:, 0] < self.hold_s)]
        if len(rise) > 2:
            vel = np.gradient(rise[:, 1], rise[:, 0]).max()
            print(f"peak joint velocity during rise: {vel:.2f} rad/s "
                  f"(URDF currently claims 16 rad/s)")
        print(f"wrote {out} - send to the dev machine for v2 hand SI")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--joint-idx", type=int, default=5)
    p.add_argument("--step-deg", type=float, default=60.0)
    p.add_argument("--hold-s", type=float, default=3.0)
    args = p.parse_args()
    StepResponse(args.joint_idx, args.step_deg, args.hold_s).run()


if __name__ == "__main__":
    main()
