#!/usr/bin/env python3
"""
mock_odrive.py  – single-file SocketCAN + Gazebo shim
====================================================

* Emulates **eight** ODrive axes on CAN-Simple (heartbeat + encoder at 100 Hz)
* Accepts **Set_Input_Pos** (0x0C) frames from `odrive_ros2_control_plugin`
* Publishes the commanded positions to **ROS topics**
    `/darm/<joint_name>_joint/cmd_pos`  (std_msgs/Float64)
* Spawns an **internal ros_gz_bridge** so those ROS topics drive the
  corresponding Gazebo joints (`gz.msgs.Double`)
* Publishes a minimal `/joint_states` so RViz / tele-op have sane values
  right from startup (avoids NaNs while ros2_control is still warming up)

Only this file is modified – _no changes needed anywhere else_.
"""

import os
import signal
import struct
import subprocess
import sys
import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# --------------------------------------------------------------------------- #
#  Constants & helpers                                                        #
# --------------------------------------------------------------------------- #
NODE_IDS         = range(8)          # we fake 8 axes : 0 … 7
HZ               = 100.0
DT               = 1.0 / HZ

CMD_HEARTBEAT    = 0x01
CMD_ENCODER_EST  = 0x09
CMD_SET_INPUT_POS = 0x0C             # position command

def arb_id(node_id: int, cmd: int) -> int:
    """11-bit arbitration-ID helper (upper 6 bits node, lower 5 bits cmd)."""
    return (node_id << 5) | cmd

# node-id → URDF joint-name  (‼ keep in sync with description.xacro order)
NODE_TO_JOINT = {
    0: "link_1_joint",
    1: "link_2_joint",
    2: "link_3_joint",
    3: "forearm_joint",
    4: "differential_joint",
    5: "gripper_joint",
    6: "finger_1_joint",
    7: "finger_2_joint",
}

# --------------------------------------------------------------------------- #
#  Mock ODrive node                                                           #
# --------------------------------------------------------------------------- #
class MockOdrive(Node):
    def __init__(self):
        super().__init__("mock_odrive")

        # ---------------- CAN socket -------------------------------------- #
        self.bus      = can.interface.Bus(channel="can0", bustype="socketcan")
        self.notifier = can.Notifier(self.bus, [self._on_can_frame])

        # current commanded position per axis
        self.pos = {nid: 0.0 for nid in NODE_IDS}

        # ---------------- ROS pubs ---------------------------------------- #
        self.pub_cmd = {
            nid: self.create_publisher(
                Float64,
                f"/darm/{NODE_TO_JOINT[nid]}/cmd_pos",
                1,
            )
            for nid in NODE_IDS
        }
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)

        # periodic 100 Hz timer
        self.create_timer(DT, self._tick)

        # ---------------- Spawn ros_gz_bridge ----------------------------- #
        # Bridge all cmd_pos topics straight through to Gazebo
        bridge_args = [
            "/darm/link_1_joint/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double",
            "/darm/link_2_joint/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double",
            "/darm/link_3_joint/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double",
            "/darm/forearm_joint/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double",
            "/darm/differential_joint/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double",
            "/darm/gripper_joint/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double",
            "/darm/finger_1_joint/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double",
            "/darm/finger_2_joint/cmd_pos@std_msgs/msg/Float64[gz.msgs.Double",
        ]
        self.get_logger().info("Spawning internal ros_gz_bridge for cmd_pos …")
        self._bridge = subprocess.Popen(
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridge_args],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # clean shutdown on ^C
        signal.signal(signal.SIGINT, self._sigint_cb)

    # ------------------------------------------------------------------- #
    #  CAN RX callback                                                    #
    # ------------------------------------------------------------------- #
    def _on_can_frame(self, msg: can.Message):
        node_id = (msg.arbitration_id >> 5) & 0x3F
        cmd     =  msg.arbitration_id       & 0x1F

        if cmd == CMD_SET_INPUT_POS and node_id in NODE_IDS:
            # payload layout: <f H h B>  (pos, vel_ff, current_ff, ctrl_mode)
            self.pos[node_id] = struct.unpack_from("<f", msg.data)[0]

    # ------------------------------------------------------------------- #
    #  100 Hz tick                                                        #
    # ------------------------------------------------------------------- #
    def _tick(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        for nid in NODE_IDS:
            p = self.pos[nid]
            vel = 0.0

            # 1 — heartbeat
            hb = struct.pack("<IBBBx", 0, 8, 0, 1)   # axis_error, CLOSED_LOOP_CONTROL, …
            self.bus.send(can.Message(arb_id(nid, CMD_HEARTBEAT),
                                      is_extended_id=False, data=hb))

            # 2 — encoder estimate
            enc = struct.pack("<ff", p, vel)
            self.bus.send(can.Message(arb_id(nid, CMD_ENCODER_EST),
                                      is_extended_id=False, data=enc))

            # 3 — publish to Gazebo-bound cmd_pos topic
            self.pub_cmd[nid].publish(Float64(data=p))

            # 4 — fill joint_state for RViz / higher layers
            js.name.append(NODE_TO_JOINT[nid])
            js.position.append(p)

        self.pub_js.publish(js)

    # ------------------------------------------------------------------- #
    #  Signal handler                                                     #
    # ------------------------------------------------------------------- #
    def _sigint_cb(self, signum, frame):
        self.destroy_node()
        if self._bridge and self._bridge.poll() is None:   # running?
            self._bridge.terminate()
            try:
                self._bridge.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._bridge.kill()
        sys.exit(0)

# --------------------------------------------------------------------------- #
#  main                                                                       #
# --------------------------------------------------------------------------- #
def main():
    rclpy.init()
    node = MockOdrive()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
