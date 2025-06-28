#!/usr/bin/env python3
import struct, can, rclpy
from rclpy.node import Node

CAN_IF            = "can0"      # can interface name
NODE_IDS          = range(8)    # eight virtual ODrives, IDs 0-7
HEARTBEAT_ID_BASE = 0x001       # heartbeat base ID
ENCODER_ID_BASE   = 0x009       # encoder-estimate base ID
SEND_PERIOD_S     = 0.01        # 100 Hz

class MockOdrive(Node):
    def __init__(self):
        super().__init__("mock_odrive")
        self.bus = can.interface.Bus(CAN_IF, bustype = "socketcan")
        self.position = {nid: 0.0 for nid in NODE_IDS}
        self.create_timer(SEND_PERIOD_S, self.send_frames)

    def send_frames(self):
        for node_id in NODE_IDS:
            # Heartbeat
            self.bus.send(can.Message(
                arbitration_id = HEARTBEAT_ID_BASE | node_id,
                is_extended_id = False,
                data           = b"\x00\x01\x00\x00"        # axis_error, axis_state, ctrl_error, enc_error
            ))      

            # Encoder Estimate (float32 position, float32 velocity)
            self.bus.send(can.Message(
                arbitration_id = ENCODER_ID_BASE | node_id,
                is_extended_id = False,
                data           = struct.pack("<ff", self.position[node_id], 0.0)
            ))

            # advance fake position
            self.position[node_id] = (self.position[node_id] + 0.01) % 6.28318

def main():
    try:
        rclpy.init()
        rclpy.spin(MockOdrive())
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()