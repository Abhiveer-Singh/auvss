# subscriber.py
# Run on JETSON
# source /opt/ros/humble/setup.bash
# python3 subscriber.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ArmSubscriber(Node):
    def __init__(self):
        super().__init__('subscribe')

        self.last_s = None
        self.last_e = None
        self.last_g = None

        self.create_subscription(Int32, '/arm/shoulder_angle', self.shoulder_cb, 10)
        self.create_subscription(Int32, '/arm/elbow_angle',    self.elbow_cb,    10)
        self.create_subscription(Int32, '/arm/gripper_angle',  self.gripper_cb,  10)

        print('ARM SUBSCRIBER READY — waiting for messages...\n')

    def shoulder_cb(self, msg):
        if self.last_s is None:
            self.last_s = msg.data
            print(f'[INIT] Shoulder → {msg.data} deg')
            return
        delta = msg.data - self.last_s
        if delta == 0: return
        print(f'[SHOULDER] {"UP" if delta > 0 else "DOWN"}  {self.last_s} → {msg.data} deg  ({"+" if delta>0 else ""}{delta})')
        self.last_s = msg.data
        self.status()

    def elbow_cb(self, msg):
        if self.last_e is None:
            self.last_e = msg.data
            print(f'[INIT] Elbow    → {msg.data} deg')
            return
        delta = msg.data - self.last_e
        if delta == 0: return
        print(f'[ELBOW]    {"UP" if delta > 0 else "DOWN"}  {self.last_e} → {msg.data} deg  ({"+" if delta>0 else ""}{delta})')
        self.last_e = msg.data
        self.status()

    def gripper_cb(self, msg):
        if self.last_g is None:
            self.last_g = msg.data
            print(f'[INIT] Gripper  → {msg.data} deg')
            return
        delta = msg.data - self.last_g
        if delta == 0: return
        print(f'[GRIPPER]  {"CLOSE" if delta > 0 else "OPEN"}  {self.last_g} → {msg.data} deg  ({"+" if delta>0 else ""}{delta})')
        self.last_g = msg.data
        self.status()

    def status(self):
        print(f'  S:{self.last_s}  E:{self.last_e}  G:{self.last_g}\n')


def main():
    rclpy.init()
    node = ArmSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[EXIT] Subscriber stopped.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
