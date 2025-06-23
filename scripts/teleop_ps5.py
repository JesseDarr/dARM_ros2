#!/usr/bin/env python3
import math, rclpy, sys, yaml
import xml.etree.ElementTree as ET
from pathlib import Path
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def _load_poses():
    path = (Path(__file__).resolve().parent / ".." / "config" / "poses.yaml").resolve()
    with path.open("r") as f: poses_raw = yaml.safe_load(f)

    poses = {}                   
    for name, vec in poses_raw.items():
        angles_deg   = vec[:6]             
        fingers_m    = vec[6:]             

        # convert degrees → radians one-by-one, store in new list
        angles_rad = [math.radians(d) for d in angles_deg]

        # populate poses dict
        poses[name] = angles_rad + fingers_m

    return poses

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
    STEP        = 0.8    # rad (or m) per jog tick
    FINGER_STEP = 0.01   # metres per full trigger pull
    DEADZONE    = 0.05   # stick dead-zone
    PREC        = 6      # decimals in table

    SQRE_BTN    = 0 
    CRSS_BTN    = 1
    CRCL_BTN    = 2
    TRGL_BTN    = 3
    LEFT_BMPR   = 4
    RGHT_BMPR   = 5
    LEFT_TRGR   = 6
    RGHT_TRGR   = 7

    def __init__(self):
        super().__init__("ps5_teleop")

        self.joint_names = [
            "link_1_joint", "link_2_joint", "link_3_joint", "forearm_joint", 
            "differential_joint", "gripper_joint", "finger_1_joint", "finger_2_joint",
        ]

        self.pose_dict = _load_poses()
        self.limits    = _get_urdf_limits(self, self.joint_names)
        self.targets   = [0.0] * len(self.joint_names)
        self.actual    = [0.0] * len(self.joint_names)

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

        total_lines = 2 + len(rows)     
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

    def _publish_traj(self, dur_ns: int) -> None:
        # Create trajectory
        traj                           = JointTrajectory()
        traj.joint_names               = self.joint_names

        # Create point
        point                          = JointTrajectoryPoint()
        point.positions                = self.targets[:]
        point.time_from_start.sec      = dur_ns // 1_000_000_000
        point.time_from_start.nanosec  = dur_ns % 1_000_000_000
        traj.points.append(point)   # append point to trajectory

        # Publish trajectory
        self.cmd_pub.publish(traj)

    def _pose_requested(self, joy: Joy) -> str | None:
        if joy.buttons[self.CRSS_BTN]: return "home"
        if joy.buttons[self.TRGL_BTN]: return "vert"
        if joy.buttons[self.SQRE_BTN]: return "left"
        if joy.buttons[self.CRCL_BTN]: return "rght"
        return None

    def _on_joy(self, joy: Joy):
        if not joy.buttons[self.LEFT_TRGR]: # dead-man switch
            return

        # Handle Quick Poses if button is pressed
        pose_pressed = self._pose_requested(joy)
        if pose_pressed:
            self.targets[:] = self.pose_dict[pose_pressed][:]

            # Publish trajectory and print table
            self._publish_traj(500_000_000) 
            self._print_table()
            return           # return to prevent processing stick input
    
        # Get raw joystick input values
        raw_left_y = joy.axes[1]
        raw_rght_x = joy.axes[2]
        raw_dpad_x = joy.axes[6]

        # apply stick dead-zone
        left_y = 0.0 if abs(raw_left_y) < self.DEADZONE else raw_left_y
        rght_x = 0.0 if abs(raw_rght_x) < self.DEADZONE else raw_rght_x

        # Are bumpers pressed?
        left_on = joy.buttons[self.LEFT_BMPR]
        rght_on = joy.buttons[self.RGHT_BMPR]

        # Set Joint targets based on bumper values and stick input
        if left_on == rght_on:
            self.targets[0] = self._clamp(0, self.actual[0] + left_y * self.STEP)   # Joint 1
            self.targets[1] = self._clamp(1, self.actual[1] + rght_x * self.STEP)   # Joint 2
        elif left_on:
            self.targets[2] = self._clamp(2, self.actual[2] + left_y * self.STEP)   # Joint 3
            self.targets[3] = self._clamp(3, self.actual[3] - rght_x * self.STEP)   # Forearm Joint
        elif rght_on:
            self.targets[4] = self._clamp(4, self.actual[4] + rght_x * self.STEP)   # Diff Joint
            self.targets[5] = self._clamp(5, self.actual[5] + left_y * self.STEP)   # Gripper Joint
        
        # Set Finger targets
        self.targets[6] = self._clamp(6, self.actual[6] + raw_dpad_x * self.FINGER_STEP)   # finger_1
        self.targets[7] = self._clamp(7, self.actual[7] + raw_dpad_x * self.FINGER_STEP)   # finger_2 (opposite)

        # Publish trajectory and print table
        self._publish_traj(300_000_000)         
        self._print_table()

def main():
    try:
        rclpy.init(args = None)
        node = PS5Teleop()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()