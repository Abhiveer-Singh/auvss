# SET 1 — NO CRC
# FILE: subscriber.py
# Run in Terminal 2
# Subscribes to topics and prints servo movement info ONLY.
# Does NOT talk to Arduino. That is the bridge's job.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class ArmSubscriber(Node):

    def __init__(self):
        super().__init__('arm_subscriber')

        self.last_s = None
        self.last_e = None
        self.last_g = None

        self.total_s = 0
        self.total_e = 0
        self.total_g = 0

        self.create_subscription(Int32, '/arm/shoulder_angle', self.shoulder_cb, 10)
        self.create_subscription(Int32, '/arm/elbow_angle',    self.elbow_cb,    10)
        self.create_subscription(Int32, '/arm/gripper_angle',  self.gripper_cb,  10)

        print('\n' + '='*46)
        print('  ARM SUBSCRIBER — ROS2 Humble  (No CRC)')
        print('='*46)
        print('  Listening on:')
        print('    /arm/shoulder_angle')
        print('    /arm/elbow_angle')
        print('    /arm/gripper_angle')
        print('='*46)
        print('  Waiting for messages...\n')

    def shoulder_cb(self, msg):
        angle = msg.data
        if self.last_s is None:
            self.last_s = angle
            print(f'  [INIT] Shoulder → {angle} deg')
            return
        delta = angle - self.last_s
        if delta == 0:
            return
        self.total_s += abs(delta)
        self.print_event('SHOULDER (Servo 1)', '/arm/shoulder_angle',
                         'UP — pitch up' if delta > 0 else 'DOWN — pitch down',
                         delta, angle, self.total_s)
        self.last_s = angle
        self.print_live()

    def elbow_cb(self, msg):
        angle = msg.data
        if self.last_e is None:
            self.last_e = angle
            print(f'  [INIT] Elbow    → {angle} deg')
            return
        delta = angle - self.last_e
        if delta == 0:
            return
        self.total_e += abs(delta)
        self.print_event('ELBOW    (Servo 2)', '/arm/elbow_angle',
                         'UP — flex' if delta > 0 else 'DOWN — extend',
                         delta, angle, self.total_e)
        self.last_e = angle
        self.print_live()

    def gripper_cb(self, msg):
        angle = msg.data
        if self.last_g is None:
            self.last_g = angle
            print(f'  [INIT] Gripper  → {angle} deg')
            return
        delta = angle - self.last_g
        if delta == 0:
            return
        self.total_g += abs(delta)
        self.print_event('GRIPPER  (Servo 3)', '/arm/gripper_angle',
                         'CLOSING — gripping' if delta > 0 else 'OPENING — releasing',
                         delta, angle, self.total_g)
        self.last_g = angle
        self.print_live()

    def print_event(self, name, topic, direction, delta, angle, total):
        sign = '+' if delta > 0 else ''
        print()
        print('  ' + '-'*44)
        print(f'   SERVO    : {name}')
        print(f'   Topic    : {topic}')
        print(f'   Direction: {direction}')
        print(f'   Changed  : {sign}{delta} deg')
        print(f'   Now at   : {angle} deg')
        print(f'   Total    : {total} deg')
        print('  ' + '-'*44)

    def print_live(self):
        s = f'{self.last_s} deg' if self.last_s is not None else '--'
        e = f'{self.last_e} deg' if self.last_e is not None else '--'
        g = f'{self.last_g} deg' if self.last_g is not None else '--'
        print(f'   [LIVE]  S:{s}  E:{e}  G:{g}\n')


def main(args=None):
    rclpy.init(args=args)
    node = ArmSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n  [EXIT] Subscriber stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
