#!/usr/bin/env python3
import math
import sys
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray

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
    STEP        = 0.05     # rad or m per jog tick
    DEADMAN_BTN = 4        # L1 button index on DualSense
    DEADZONE    = 0.05     # stick noise filter (∈ [0,1])
    PREC        = 6         # decimal places
    WIDTH       = 14        # column width for numbers

    def __init__(self):
        super().__init__("ps5_teleop")

        self.joint_names = [
            "link_1_joint", "link_2_joint", "link_3_joint", "forearm_joint",
            "differential_joint", "gripper_joint", "finger_1_joint", "finger_2_joint",
        ]
        self.active_idx = (0, 1)          # the two we jog for now

        self.limits  = _get_urdf_limits(self, self.joint_names)
        self.targets = [0.0] * len(self.joint_names)
        self.actual  = [0.0] * len(self.joint_names)

        # table printing bookkeeping
        self._first_table = True
        self._table_lines = len(self.active_idx) + 2   # hdr + bar + rows

        # ROS I/O
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, "/arm_controller/commands", 10)

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
        if not self._first_table:
            sys.stdout.write(f"\x1b[{self._table_lines}A")
        else:
            self._first_table = False

        num = f'{{:>{self.WIDTH}.{self.PREC}f}}'
        hdr = (
            f'{"Joint":<20} | '
            f'{"Current":>{self.WIDTH}} | '
            f'{"Target":>{self.WIDTH}} | '
            f'{"Lower":>{self.WIDTH}} | '
            f'{"Upper":>{self.WIDTH}}'
        )
        bar = "-" * len(hdr)
        lines = [hdr, bar]

        for i in self.active_idx:
            lo, hi = self.limits[self.joint_names[i]]
            lines.append(
                f'{self.joint_names[i]:<20} | '
                f'{num.format(self.actual[i])} | '
                f'{num.format(self.targets[i])} | '
                f'{num.format(lo)} | '
                f'{num.format(hi)}'
            )

        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def _on_joy(self, joy: Joy):
        if not joy.buttons[self.DEADMAN_BTN]:
            return
        
        # apply dead-zone
        ax_y = 0.0 if abs(joy.axes[1]) < self.DEADZONE else joy.axes[1]
        ax_x = 0.0 if abs(joy.axes[0]) < self.DEADZONE else joy.axes[0]

        # left-stick Y/X → joints 0/1
        self.targets[0] = self._clamp(0, self.targets[0] + joy.axes[1] * self.STEP)
        self.targets[1] = self._clamp(1, self.targets[1] + joy.axes[0] * self.STEP)

        self.cmd_pub.publish(Float64MultiArray(data=self.targets))
        self._print_table()

def main(args=None):
    rclpy.init(args=args)
    node = PS5Teleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()