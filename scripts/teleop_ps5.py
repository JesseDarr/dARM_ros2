#!/usr/bin/env python3
import math
import sys
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def _get_urdf_limits(node: Node, names):
    cli = AsyncParameterClient(node, "robot_state_publisher")
    fut = cli.get_parameters(["robot_description"])
    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)

    if not (fut.done() and fut.result()):
        node.get_logger().error("robot_description unavailable")
        return {n: (-math.inf, math.inf) for n in names}

    urdf_xml = fut.result().values[0].string_value or ""
    root = ET.fromstring(urdf_xml) if urdf_xml else ET.Element("robot")

    lims = {}
    for j in root.iter("joint"):
        n = j.attrib.get("name")
        if n in names:
            lim = j.find("limit")
            if lim is not None:
                lims[n] = (
                    float(lim.attrib.get("lower", "-inf")),
                    float(lim.attrib.get("upper", "inf")),
                )
    return {n: lims.get(n, (-math.inf, math.inf)) for n in names}

class PS5Teleop(Node):
    STEP        = 0.4    # rad (or m) per jog tick
    DEADMAN_BTN = 4      # L1 on DualSense
    DEADZONE    = 0.05   # stick dead-zone
    PREC        = 6      # decimals in table

    def __init__(self):
        super().__init__("ps5_teleop")

        self.joint_names = [
            "link_1_joint", "link_2_joint", "link_3_joint", "forearm_joint", 
            "differential_joint", "gripper_joint", "finger_1_joint", "finger_2_joint",
        ]
        self.active_idx = (0, 1)                       # jog only these two

        self.limits  = _get_urdf_limits(self, self.joint_names)
        self.targets = [0.0] * len(self.joint_names)
        self.actual  = [0.0] * len(self.joint_names)

        self._first_table = True
        self._table_lines = len(self.active_idx) + 2   # header + bar + rows

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.cmd_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.get_logger().info("PS5 Tele-op ready")

    def _clamp(self, idx, value):
        lo, hi = self.limits[self.joint_names[idx]]
        return max(min(value, hi), lo)

    def _on_joint_state(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        for i, n in enumerate(self.joint_names):
            if n in pos:
                self.actual[i] = pos[n]

    def _print_table(self):
        # -------- gather all cell strings first --------
        joints = self.joint_names
        prec   = self.PREC

        rows = {
            "Current": [f"{v:.{prec}f}" for v in self.actual],
            "Target" : [f"{v:.{prec}f}" for v in self.targets],
            "Lower"  : [f"{self.limits[n][0]:.{prec}f}" for n in joints],
            "Upper"  : [f"{self.limits[n][1]:.{prec}f}" for n in joints],
        }

        col_w = [
            max(len(joints[i]),
                *(len(r[i]) for r in rows.values()))
            for i in range(len(joints))
        ]

        total_lines = 2 + len(rows)     # always 6 here
        if hasattr(self, "_printed_once"):
            sys.stdout.write(f"\x1b[{total_lines}A")
        else:
            self._printed_once = True

        header = " " * 15
        for i, name in enumerate(joints):
            header += f"| {name:>{col_w[i]}} "
        lines = [header, "-" * len(header)]

        for label, values in rows.items():
            line = f"{label:<15}"
            for i, val in enumerate(values):
                line += f"| {val:>{col_w[i]}} "
            lines.append(line)

        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def _on_joy(self, joy: Joy):
        if not joy.buttons[self.DEADMAN_BTN]:         # dead-man switch
            return

        # apply stick dead-zone
        ax_y = 0.0 if abs(joy.axes[1]) < self.DEADZONE else joy.axes[1]
        ax_x = 0.0 if abs(joy.axes[0]) < self.DEADZONE else joy.axes[0]

        self.targets[0] = self._clamp(0, self.actual[0] + ax_y * self.STEP)
        self.targets[1] = self._clamp(1, self.actual[1] + ax_x * self.STEP)

        # ------------ one-point trajectory ------------
        traj = JointTrajectory()
        traj.joint_names = self.joint_names                 # send all joints

        pt = JointTrajectoryPoint()
        pt.positions = self.targets[:]                      # copy full list
        pt.time_from_start.nanosec = 300_000_000
        traj.points.append(pt)

        self.cmd_pub.publish(traj)
        self._print_table()

def main(args=None):
    try:
        rclpy.init(args=args)
        node = PS5Teleop()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()