#!/usr/bin/env python
"""Franka robot node: bridges the SimToolReal deployment topic convention to a
real Franka arm running franka_ros.

Repo-side interface (matches docs/deployment.md, with arm namespace "franka"):
  subscribes /franka/joint_cmd    (sensor_msgs/JointState, 7 arm joint position targets)
  publishes  /franka/joint_states (sensor_msgs/JointState, 7 arm joint positions+velocities)

Hardware-side interface (franka_ros / ros_control):
  publishes  <output_topic>  (std_msgs/Float64MultiArray, 7 positions) to a
             position_controllers/JointGroupPositionController claiming the
             franka_hw position interface
  subscribes <franka_joint_states_topic> (sensor_msgs/JointState from
             franka_state_controller; the 7 panda_joint* entries are filtered
             out by name)

Safety features:
  * Per-joint velocity rate limiter: the forwarded target moves toward the
    commanded target at most vel_limit_fraction * real joint speed limits.
    This absorbs the rare velocity spikes a sim-trained policy can command,
    which would otherwise trip Franka's reflex stop.
  * Targets clamped to the profile's RESTRICTED joint limits (arm inset 10 deg).
  * Stale-command watchdog: if no command arrives for watchdog_timeout seconds,
    the node holds the last forwarded target.
  * On startup the internal target initializes to the measured joint positions.

Dry-run mode (--dry_run) needs no hardware: the rate-limited target is
integrated as the simulated robot state and republished, so the full ROS node
graph can be tested on any machine.
"""

import argparse
import time

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from termcolor import colored
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from isaacgymenvs.utils.observation_action_utils_sharpa import get_robot_profile

NUM_ARM_JOINTS = 7


def warn(message: str) -> None:
    print(colored(message, "yellow"))


def info(message: str) -> None:
    print(colored(message, "green"))


class FrankaRobotNode:
    def __init__(
        self,
        robot: str = "franka_right_sharpa",
        control_hz: float = 200.0,
        vel_limit_fraction: float = 0.8,
        watchdog_timeout: float = 0.5,
        output_topic: str = "/position_joint_position_controller/command",
        franka_joint_states_topic: str = "/franka_state_controller/joint_states",
        dry_run: bool = False,
        q1_offset_deg: float = 0.0,
    ):
        self.profile = get_robot_profile(robot)
        self.control_hz = control_hz
        self.dt = 1.0 / control_hz
        self.watchdog_timeout = watchdog_timeout
        self.dry_run = dry_run
        # Virtual-world yaw remap: the policy was trained with its workspace at
        # base -y; if the physical bench sits elsewhere, rotating the whole task
        # about the base z-axis is an exact arm symmetry (joint 1 IS that axis).
        # Convention: q1_real = q1_virtual + offset. This node is the single
        # bridge for ALL arm state/commands, so applying the offset here (and
        # rotating perception poses by -offset in the relay) fully virtualizes
        # the rotation; policy, goal node, and home_robot need no changes.
        self.q1_offset = np.deg2rad(q1_offset_deg)
        if abs(self.q1_offset) > 1e-9:
            info(
                f"VIRTUAL YAW REMAP active: q1_real = q1_virtual + {q1_offset_deg:.1f} deg "
                f"(pair with object_pose_relay_node --yaw-deg {-q1_offset_deg:.1f})"
            )

        # Per-joint max target step per control tick [rad]
        self.max_step = self.profile.arm_vel_limits * vel_limit_fraction * self.dt
        self.arm_lower = self.profile.q_lower_restricted[:NUM_ARM_JOINTS]
        self.arm_upper = self.profile.q_upper_restricted[:NUM_ARM_JOINTS]
        self.arm_joint_names = list(self.profile.ros_arm_joint_names)

        rospy.init_node("franka_robot_node")

        # State
        self.latest_cmd: np.ndarray = None  # latest commanded target from policy
        self.latest_cmd_time: float = None
        self.forwarded_target: np.ndarray = None  # rate-limited target sent to robot
        self.measured_q: np.ndarray = None
        self.measured_qd: np.ndarray = None
        self._rate_limited_since_warn = 0

        # Repo-side interface
        arm_ns = self.profile.ros_arm_ns
        self.cmd_sub = rospy.Subscriber(
            f"/{arm_ns}/joint_cmd", JointState, self._cmd_callback, queue_size=1
        )
        self.state_pub = rospy.Publisher(
            f"/{arm_ns}/joint_states", JointState, queue_size=1
        )

        # Hardware-side interface. TRAJECTORY mode (default): stream single-point
        # JointTrajectory goals to position_joint_trajectory_controller, which
        # interpolates internally at 1 kHz (quintic), starts by HOLDING the
        # current pose, and tolerates stream hiccups — unlike the raw
        # JointGroupPositionController, where every dropped 1 kHz packet became
        # a reference jump -> torque impulse -> tau_J_range_violation.
        self.use_traj_controller = "trajectory" in output_topic
        if not self.dry_run:
            if self.use_traj_controller:
                self.hw_cmd_pub = rospy.Publisher(
                    output_topic, JointTrajectory, queue_size=1
                )
            else:
                self.hw_cmd_pub = rospy.Publisher(
                    output_topic, Float64MultiArray, queue_size=1
                )
            self.hw_state_sub = rospy.Subscriber(
                franka_joint_states_topic,
                JointState,
                self._hw_state_callback,
                queue_size=1,
            )
            info(f"HARDWARE mode: commands -> {output_topic}")
        else:
            self.measured_q = np.concatenate(
                [self.profile.home_arm_qpos, np.zeros(0)]
            )[:NUM_ARM_JOINTS].copy()
            self.measured_q[0] += self.q1_offset  # dry-run sim state is REAL-frame
            self.measured_qd = np.zeros(NUM_ARM_JOINTS)
            info("DRY-RUN mode: simulating the arm (no hardware topics)")

    def _cmd_callback(self, msg: JointState) -> None:
        pos = np.array(msg.position)
        if pos.shape != (NUM_ARM_JOINTS,):
            warn(f"Ignoring joint_cmd with {pos.shape[0]} positions (expected 7)")
            return
        pos = pos.copy()
        pos[0] += self.q1_offset  # virtual -> real
        self.latest_cmd = pos
        self.latest_cmd_time = time.time()

    def _hw_state_callback(self, msg: JointState) -> None:
        # franka_state_controller publishes the 7 panda joints (possibly among
        # others depending on setup); filter by name for robustness.
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        missing = [n for n in self.arm_joint_names if n not in name_to_idx]
        if missing:
            warn(f"joint_states missing joints {missing}; got {msg.name}")
            return
        idxs = [name_to_idx[n] for n in self.arm_joint_names]
        self.measured_q = np.array([msg.position[i] for i in idxs])
        self.measured_qd = (
            np.array([msg.velocity[i] for i in idxs])
            if len(msg.velocity) == len(msg.name)
            else np.zeros(NUM_ARM_JOINTS)
        )

    def _publish_state(self) -> None:
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.arm_joint_names
        pos = self.measured_q.copy()
        pos[0] -= self.q1_offset  # real -> virtual
        msg.position = pos.tolist()
        msg.velocity = self.measured_qd.tolist()
        self.state_pub.publish(msg)

    def _rate_limit_and_forward(self) -> None:
        target = self.latest_cmd
        if target is None:
            return

        # Watchdog: hold position on stale commands
        if time.time() - self.latest_cmd_time > self.watchdog_timeout:
            target = self.forwarded_target
            # Re-anchor: if the arm stopped tracking (reflex / controller died)
            # while commands kept coming, forwarded_target walks far from the
            # measured state. Re-publishing that far target when a controller
            # (re)starts causes an instant discontinuity reflex. With stale
            # commands AND a large divergence, snap the held target back to
            # the measured pose so recovery is always jump-free.
            if (
                self.measured_q is not None
                and np.abs(self.forwarded_target - self.measured_q).max() > 0.15
            ):
                warn(
                    "Held target diverged from measured state "
                    f"(max {np.abs(self.forwarded_target - self.measured_q).max():.2f} rad) "
                    "with stale commands — re-anchoring to measured pose."
                )
                self.forwarded_target = self.measured_q.copy()
                self.latest_cmd = self.measured_q.copy()
                target = self.forwarded_target

        target = np.clip(target, self.arm_lower, self.arm_upper)

        step = np.clip(
            target - self.forwarded_target,
            -self.max_step,
            self.max_step,
        )
        if np.any(np.abs(target - self.forwarded_target) > self.max_step * 1.001):
            self._rate_limited_since_warn += 1
            if self._rate_limited_since_warn >= int(self.control_hz):
                warn("Rate limiter active (commanded target faster than vel limit)")
                self._rate_limited_since_warn = 0
        self.forwarded_target = self.forwarded_target + step

        # Reference governor: cap the forwarded target's DISTANCE from the
        # measured arm, per joint. The internal joint-impedance controller turns
        # reference-vs-arm gap into torque (tau = K * gap); the wrist joints
        # (K=[600x4,250,150,50] Nm/rad vs torque limits [87x4,12,12,12] Nm)
        # tolerate only a few degrees of gap before tau_J_range_violation stops
        # the robot — which killed every fast target stream on this rig. This
        # clamp makes the reference wait for the arm: it advances only as fast
        # as the arm actually tracks, keeping commanded torque bounded.
        if not self.dry_run and self.measured_q is not None:
            # Caps sized for the SOFT internal impedance set by arm_ready.sh
            # (K=[600,600,600,450,250,150,100]): tau at cap ~= 50% of each
            # joint's torque limit, leaving headroom for dynamics.
            GAP_CAP = np.array([0.07, 0.07, 0.07, 0.09, 0.024, 0.04, 0.06])
            self.forwarded_target = np.clip(
                self.forwarded_target,
                self.measured_q - GAP_CAP,
                self.measured_q + GAP_CAP,
            )

        if not self.dry_run:
            if self.use_traj_controller:
                # stream a one-point trajectory 80 ms ahead at ~50 Hz — the
                # controller's own interpolator produces the smooth 1 kHz refs
                self._traj_decim = getattr(self, "_traj_decim", 0) + 1
                if self._traj_decim >= 4:
                    self._traj_decim = 0
                    traj = JointTrajectory()
                    traj.joint_names = self.arm_joint_names
                    pt = JointTrajectoryPoint()
                    pt.positions = self.forwarded_target.tolist()
                    pt.time_from_start = rospy.Duration(0.08)
                    traj.points = [pt]
                    self.hw_cmd_pub.publish(traj)
            else:
                out = Float64MultiArray()
                out.data = self.forwarded_target.tolist()
                self.hw_cmd_pub.publish(out)
        else:
            # Simulate: first-order tracking of the forwarded target
            prev_q = self.measured_q.copy()
            ALPHA = 0.3
            self.measured_q = self.measured_q + ALPHA * (
                self.forwarded_target - self.measured_q
            )
            self.measured_qd = (self.measured_q - prev_q) / self.dt

    def run(self) -> None:
        # Wait for the first measured state, then initialize the target to it
        while not rospy.is_shutdown() and self.measured_q is None:
            warn("Waiting for first joint_states from the robot...")
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            return
        self.forwarded_target = self.measured_q.copy()
        info(f"Initialized target to measured q: {np.round(self.measured_q, 3)}")

        rate = rospy.Rate(self.control_hz)
        while not rospy.is_shutdown():
            self._rate_limit_and_forward()
            self._publish_state()
            rate.sleep()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="franka_right_sharpa")
    parser.add_argument("--control_hz", type=float, default=200.0)
    parser.add_argument(
        "--vel_limit_fraction",
        type=float,
        default=0.8,
        help="Fraction of the real joint speed limits used by the rate limiter",
    )
    parser.add_argument("--watchdog_timeout", type=float, default=0.5)
    parser.add_argument(
        "--output_topic",
        default="/position_joint_trajectory_controller/command",
        help="trajectory-controller topic (default, robust) or the raw "
        "/position_joint_position_controller/command",
    )
    parser.add_argument(
        "--franka_joint_states_topic",
        default="/franka_state_controller/joint_states",
    )
    parser.add_argument("--dry_run", action="store_true")
    parser.add_argument(
        "--q1_offset_deg",
        type=float,
        default=0.0,
        help="Virtual yaw remap: q1_real = q1_virtual + this. Use +90 when the "
        "physical workspace is at base +x instead of the trained -y; pair with "
        "object_pose_relay_node --yaw-deg -90.",
    )
    args = parser.parse_args()

    node = FrankaRobotNode(
        robot=args.robot,
        control_hz=args.control_hz,
        vel_limit_fraction=args.vel_limit_fraction,
        watchdog_timeout=args.watchdog_timeout,
        output_topic=args.output_topic,
        franka_joint_states_topic=args.franka_joint_states_topic,
        dry_run=args.dry_run,
        q1_offset_deg=args.q1_offset_deg,
    )
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
