# =============================================================
#  FILE 1: arm_teleop_publisher.py
#  ROS2 Humble — Arm Teleop Publisher Node
#  Input: Keyboard (type + Enter) AND/OR Xbox Controller
# =============================================================
#
#  ┌──────────────────────────────────────────────────────┐
#  │   KEYBOARD CONTROLS  (type key + press Enter)        │
#  │   Q / W  →  Shoulder  DOWN / UP     (+/- 5 deg)      │
#  │   A / S  →  Elbow     DOWN / UP     (+/- 5 deg)      │
#  │   D / F  →  Gripper   OPEN / CLOSE  (+/- 5 deg)      │
#  │   H      →  Show help                                 │
#  │   E      →  Quit                                      │
#  ├──────────────────────────────────────────────────────┤
#  │   XBOX CONTROLLER                                     │
#  │   LB  →  Shoulder  DOWN   (-5 deg)  [btn 4]          │
#  │   RB  →  Shoulder  UP     (+5 deg)  [btn 5]          │
#  │   LT  →  Elbow     DOWN   (-5 deg)  [axis 2]         │
#  │   RT  →  Elbow     UP     (+5 deg)  [axis 5]         │
#  │   X   →  Gripper   OPEN   (-5 deg)  [btn 2]          │
#  │   Y   →  Gripper   CLOSE  (+5 deg)  [btn 3]          │
#  └──────────────────────────────────────────────────────┘
#
#  DEPENDENCIES:
#    pip install pygame
#
#  HOW TO RUN:
#    Terminal 1:
#      source /opt/ros/humble/setup.bash
#      python3 arm_teleop_publisher.py
# =============================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pygame
import threading
import time
import sys

# ── Arm angle limits ─────────────────────────────────────────
SHOULDER_MIN, SHOULDER_MAX = 0, 180
ELBOW_MIN,    ELBOW_MAX    = 0, 180
GRIPPER_MIN,  GRIPPER_MAX  = 0, 90

# ── Step size per press ───────────────────────────────────────
STEP = 5

# ── Xbox axis indices ─────────────────────────────────────────
AXIS_LT = 2    # Elbow DOWN
AXIS_RT = 5    # Elbow UP

# ── Xbox button indices ───────────────────────────────────────
BTN_LB = 4     # Shoulder DOWN
BTN_RB = 5     # Shoulder UP
BTN_X  = 2     # Gripper OPEN
BTN_Y  = 3     # Gripper CLOSE

# ── Trigger threshold ─────────────────────────────────────────
TRIGGER_THRESHOLD = 0.1

# ── Debug mode — prints raw trigger values when pressed ───────
DEBUG = False 


class ArmTeleopPublisher(Node):

    def __init__(self):
        super().__init__('arm_teleop_publisher')

        # ── ROS2 Publishers ──────────────────────────────────
        self.pub_shoulder = self.create_publisher(Int32, '/arm/shoulder_angle', 10)
        self.pub_elbow    = self.create_publisher(Int32, '/arm/elbow_angle',    10)
        self.pub_gripper  = self.create_publisher(Int32, '/arm/gripper_angle',  10)

        # ── Current servo positions ──────────────────────────
        self.pos_shoulder = 90
        self.pos_elbow    = 90
        self.pos_gripper  = 0

        # ── Thread lock ───────────────────────────────────────
        self.lock = threading.Lock()

        # ── Trigger hold flags (fire once per press) ─────────
        self.lt_active = False
        self.rt_active = False

        self.get_logger().info('Arm Teleop Publisher started.')

    # ──────────────────────────────────────────────────────────
    def publish_angle(self, publisher, angle):
        msg = Int32()
        msg.data = angle
        publisher.publish(msg)

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

    # ──────────────────────────────────────────────────────────
    #  SHARED MOVE — only this servo publishes, others untouched
    # ──────────────────────────────────────────────────────────
    def move_servo(self, servo, direction, source='KB'):
        with self.lock:
            if servo == 'shoulder':
                self.pos_shoulder = self.clamp(
                    self.pos_shoulder + (STEP if direction == 'up' else -STEP),
                    SHOULDER_MIN, SHOULDER_MAX)
                self.publish_angle(self.pub_shoulder, self.pos_shoulder)
                label = 'UP' if direction == 'up' else 'DOWN'
                print(f'  [{source}] [PUBLISHED] /arm/shoulder_angle  →  {self.pos_shoulder} deg  ({label})')

            elif servo == 'elbow':
                self.pos_elbow = self.clamp(
                    self.pos_elbow + (STEP if direction == 'up' else -STEP),
                    ELBOW_MIN, ELBOW_MAX)
                self.publish_angle(self.pub_elbow, self.pos_elbow)
                label = 'UP' if direction == 'up' else 'DOWN'
                print(f'  [{source}] [PUBLISHED] /arm/elbow_angle     →  {self.pos_elbow} deg  ({label})')

            elif servo == 'gripper':
                self.pos_gripper = self.clamp(
                    self.pos_gripper + (-STEP if direction == 'open' else STEP),
                    GRIPPER_MIN, GRIPPER_MAX)
                self.publish_angle(self.pub_gripper, self.pos_gripper)
                label = 'OPEN' if direction == 'open' else 'CLOSE'
                print(f'  [{source}] [PUBLISHED] /arm/gripper_angle   →  {self.pos_gripper} deg  ({label})')

            self.print_status()

    # ──────────────────────────────────────────────────────────
    #  KEYBOARD LOOP
    # ──────────────────────────────────────────────────────────
    def keyboard_loop(self):
        print_help()
        while rclpy.ok():
            try:
                key = input('  KB > ').strip().lower()
            except (KeyboardInterrupt, EOFError):
                rclpy.shutdown()
                break

            if   key == 'q': self.move_servo('shoulder', 'down',  'KB')
            elif key == 'w': self.move_servo('shoulder', 'up',    'KB')
            elif key == 'a': self.move_servo('elbow',    'down',  'KB')
            elif key == 's': self.move_servo('elbow',    'up',    'KB')
            elif key == 'd': self.move_servo('gripper',  'open',  'KB')
            elif key == 'f': self.move_servo('gripper',  'close', 'KB')
            elif key == 'h': print_help()
            elif key == 'e':
                print('\n  [EXIT] Shutting down...')
                rclpy.shutdown()
                break
            else:
                print(f'  [?] Unknown key: "{key}"')

    # ──────────────────────────────────────────────────────────
    #  XBOX CONTROLLER LOOP
    #  LB/RB → buttons (shoulder)
    #  LT/RT → analog triggers polled via get_axis() (elbow)
    #  X/Y   → buttons (gripper)
    # ──────────────────────────────────────────────────────────
    def controller_loop(self):
        pygame.init()
        pygame.joystick.init()

        while pygame.joystick.get_count() == 0:
            print('  [CTRL] No Xbox controller detected. Waiting...')
            time.sleep(2)
            pygame.joystick.quit()
            pygame.joystick.init()

        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f'\n  [CTRL] Connected: {joy.get_name()}')
        print(f'  [CTRL] Axes: {joy.get_numaxes()}  Buttons: {joy.get_numbuttons()}\n')

        clock = pygame.time.Clock()

        while rclpy.ok():
            # pump() keeps pygame's internal event queue fresh
            # so get_axis() returns up-to-date values
            pygame.event.pump()

            # ── Poll LT (axis 2) → Elbow DOWN ────────────────
            lt_val = joy.get_axis(AXIS_LT)
            if DEBUG and abs(lt_val) > 0.05:
                print(f'  [DEBUG] LT axis {AXIS_LT} = {lt_val:.3f}')

            if lt_val > TRIGGER_THRESHOLD and not self.lt_active:
                self.lt_active = True
                self.move_servo('elbow', 'down', 'CTRL')
            elif lt_val <= TRIGGER_THRESHOLD:
                self.lt_active = False   # Released — ready to fire again

            # ── Poll RT (axis 5) → Elbow UP ──────────────────
            rt_val = joy.get_axis(AXIS_RT)
            if DEBUG and abs(rt_val) > 0.05:
                print(f'  [DEBUG] RT axis {AXIS_RT} = {rt_val:.3f}')

            if rt_val > TRIGGER_THRESHOLD and not self.rt_active:
                self.rt_active = True
                self.move_servo('elbow', 'up', 'CTRL')
            elif rt_val <= TRIGGER_THRESHOLD:
                self.rt_active = False   # Released — ready to fire again

            # ── Button events (LB, RB, X, Y) ─────────────────
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if   event.button == BTN_LB: self.move_servo('shoulder', 'down',  'CTRL')
                    elif event.button == BTN_RB: self.move_servo('shoulder', 'up',    'CTRL')
                    elif event.button == BTN_X:  self.move_servo('gripper',  'open',  'CTRL')
                    elif event.button == BTN_Y:  self.move_servo('gripper',  'close', 'CTRL')

                elif event.type == pygame.JOYDEVICEREMOVED:
                    print('\n  [CTRL] Controller disconnected!')

                elif event.type == pygame.JOYDEVICEADDED:
                    pygame.joystick.quit()
                    pygame.joystick.init()
                    if pygame.joystick.get_count() > 0:
                        joy = pygame.joystick.Joystick(0)
                        joy.init()
                        print(f'\n  [CTRL] Reconnected: {joy.get_name()}\n')

            clock.tick(60)

        pygame.quit()

    # ──────────────────────────────────────────────────────────
    def print_status(self):
        print(f'  [STATUS]  Shoulder: {self.pos_shoulder} deg  |  '
              f'Elbow: {self.pos_elbow} deg  |  '
              f'Gripper: {self.pos_gripper} deg\n')


# ── Help text ──────────────────────────────────────────────────
def print_help():
    print()
    print('  ' + '='*50)
    print('    ARM TELEOP PUBLISHER  |  Controls')
    print('  ' + '='*50)
    print('    -- KEYBOARD (type + Enter) --')
    print('    Q / W  →  Shoulder  DOWN / UP     (+/- 5 deg)')
    print('    A / S  →  Elbow     DOWN / UP     (+/- 5 deg)')
    print('    D / F  →  Gripper   OPEN / CLOSE  (+/- 5 deg)')
    print('    H      →  Show this help')
    print('    E      →  Quit')
    print('  ' + '-'*50)
    print('    -- XBOX CONTROLLER --')
    print('    LB (btn 4)   →  Shoulder  DOWN   (-5 deg)')
    print('    RB (btn 5)   →  Shoulder  UP     (+5 deg)')
    print('    LT (axis 2)  →  Elbow     DOWN   (-5 deg)')
    print('    RT (axis 5)  →  Elbow     UP     (+5 deg)')
    print('    X  (btn 2)   →  Gripper   OPEN   (-5 deg)')
    print('    Y  (btn 3)   →  Gripper   CLOSE  (+5 deg)')
    print('  ' + '='*50)
    print('    Subscriber output in Terminal 2.')
    print('  ' + '='*50)
    print()


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopPublisher()

    ctrl_thread = threading.Thread(target=node.controller_loop, daemon=True)
    ctrl_thread.start()

    kb_thread = threading.Thread(target=node.keyboard_loop, daemon=True)
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n  [EXIT] Ctrl+C detected.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()