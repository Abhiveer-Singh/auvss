import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200


class ArmBridge(Node):

    def __init__(self):
        super().__init__('bridge')

        # Store last sent values (to avoid spam)
        self.last_shoulder = None
        self.last_elbow = None
        self.last_gripper = None

        # ── Open serial port ──────────────────────────────────
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)
            self.ser.flush()

            print(f'\n  [BRIDGE] Connected → {SERIAL_PORT} @ {BAUD_RATE}')
            print('  [BRIDGE] Waiting for ROS2 topics...\n')

        except serial.SerialException as e:
            print(f'[FATAL] Cannot open serial: {e}')
            exit(1)

        # ── ROS2 Subscribers ─────────────────────────────────
        self.create_subscription(Int32, '/arm/shoulder_angle', self.shoulder_cb, 10)
        self.create_subscription(Int32, '/arm/elbow_angle',    self.elbow_cb,    10)
        self.create_subscription(Int32, '/arm/gripper_angle',  self.gripper_cb,  10)

    def clamp(self, angle):
        return max(0, min(180, angle))

    def send(self, cmd):
        try:
            self.ser.write((cmd + '\n').encode())
            self.ser.flush()
            print(f'[TX] {cmd}')
        except serial.SerialException as e:
            print(f'[ERROR] Serial write failed: {e}')

    # ── Callbacks ───────────────────────────────────────────

    def shoulder_cb(self, msg):
        angle = self.clamp(msg.data)

        if angle != self.last_shoulder:
            self.last_shoulder = angle
            self.send(f'S1:{angle}')

    def elbow_cb(self, msg):
        angle = self.clamp(msg.data)

        if angle != self.last_elbow:
            self.last_elbow = angle
            self.send(f'S2:{angle}')

    def gripper_cb(self, msg):
        angle = self.clamp(msg.data)

        if angle != self.last_gripper:
            self.last_gripper = angle
            self.send(f'S3:{angle}')


def main(args=None):
    rclpy.init(args=args)
    node = ArmBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print('\n[BRIDGE] Stopping...')

    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
            print('[BRIDGE] Serial closed.')

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()