#!/usr/bin/env python3
import struct, can, rclpy
from rclpy.node import Node

NODE_IDS        = range(8)         # node-ids 0-7
HZ              = 100.0
DT              = 1.0 / HZ
CMD_HEARTBEAT   = 0x01
CMD_ENCODER     = 0x09
CMD_SET_INPUT   = 0x0C             # position command sent by controller

def arb_id(node_id: int, cmd: int) -> int:
    """CAN-Simple 11-bit ID: upper 6 bits node-id, lower 5 bits command."""
    return (node_id << 5) | cmd

class MockOdrive(Node):
    def __init__(self):
        super().__init__("mock_odrive")

        self.bus      = can.interface.Bus(channel="can0", bustype="socketcan")
        self.notifier = can.Notifier(self.bus, [self._on_can_frame])
        self.position = {nid: 0.0 for nid in NODE_IDS}
        self.create_timer(DT, self._publish)

    def _on_can_frame(self, msg: can.Message):
        node_id = (msg.arbitration_id >> 5) & 0x3F
        cmd     =  msg.arbitration_id       & 0x1F

        if cmd == CMD_SET_INPUT and node_id in NODE_IDS:
            self.position[node_id] = struct.unpack_from("<f", msg.data)[0]             # payload layout: <f H h B>  (pos, vel_ff, current_ff, ctrl_mode)

    def _publish(self):
        for nid in NODE_IDS:
            pos = self.position[nid]
            vel = 0.0

            # 1) Heartbeat (axis_state = CLOSED_LOOP_CONTROL)
            hb = struct.pack("<IBBBx", 0, 8, 0, 1)
            self.bus.send(can.Message(arbitration_id=arb_id(nid, CMD_HEARTBEAT), is_extended_id=False, data=hb))

            # 2) Encoder estimates
            enc = struct.pack("<ff", pos, vel)
            self.bus.send(can.Message(arbitration_id=arb_id(nid, CMD_ENCODER), is_extended_id=False, data=enc))

def main():
    try:
        rclpy.init()
        node = MockOdrive()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == "__main__":
    main()