# =============================================================
#  SET 1 — NO CRC
#  FILE: publisher.py
#  ROS2 Humble — Arm Teleop Publisher
#  Keyboard + Xbox Controller
# =============================================================
#
#  BEFORE RUNNING:
#    pip install pygame
#    source /opt/ros/humble/setup.bash
#
#  RUN:
#    python3 publisher.py
#
#  KEYBOARD (type key + Enter):
#    Q / W  →  Shoulder  DOWN / UP
#    A / S  →  Elbow     DOWN / UP
#    D / F  →  Gripper   OPEN / CLOSE
#    E      →  Exit
#
#  XBOX CONTROLLER (plug in before running):
#    LB  (btn 4)  →  Shoulder DOWN
#    RB  (btn 5)  →  Shoulder UP
#    LT  (axis 2) →  Elbow    DOWN
#    RT  (axis 5) →  Elbow    UP
#    X   (btn 2)  →  Gripper  OPEN
#    Y   (btn 3)  →  Gripper  CLOSE
#
#  TOPICS PUBLISHED:
#    /arm/shoulder_angle   std_msgs/Int32
#    /arm/elbow_angle      std_msgs/Int32
#    /arm/gripper_angle    std_msgs/Int32
# =============================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pygame
import threading
import time
import sys

# ── Step size per press ───────────────────────────────────────
STEP = 30

# ── Xbox trigger fire threshold ───────────────────────────────
# Linux xpad: triggers report 0.0 (rest) → 1.0 (fully pressed)
TRIGGER_THRESHOLD = 0.1

# ── Xbox axis indices (confirmed on Linux) ────────────────────
AXIS_LT = 2    # Elbow DOWN
AXIS_RT = 5    # Elbow UP

# ── Xbox button indices ───────────────────────────────────────
BTN_LB = 4     # Shoulder DOWN
BTN_RB = 5     # Shoulder UP
BTN_X  = 2     # Gripper OPEN
BTN_Y  = 3     # Gripper CLOSE


class ArmPublisher(Node):

    def __init__(self):
        super().__init__('publisher')

        # ── ROS2 publishers ───────────────────────────────────
        self.pub_shoulder = self.create_publisher(Int32, '/arm/shoulder_angle', 10)
        self.pub_elbow    = self.create_publisher(Int32, '/arm/elbow_angle',    10)
        self.pub_gripper  = self.create_publisher(Int32, '/arm/gripper_angle',  10)

        # ── Current angles ────────────────────────────────────
        self.shoulder = 90
        self.elbow    = 90
        self.gripper  = 0

        # ── Thread lock (keyboard + controller share state) ───
        self.lock = threading.Lock()

        # ── Trigger hold flags — fire once per press ──────────
        self.lt_held = False
        self.rt_held = False

        self.print_controls()

    # ──────────────────────────────────────────────────────────
    #  PUBLISH one angle to one topic
    # ──────────────────────────────────────────────────────────
    def publish(self, publisher, value):
        msg = Int32()
        msg.data = value
        publisher.publish(msg)

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    # ──────────────────────────────────────────────────────────
    #  MOVE — only the named servo publishes, others untouched
    # ──────────────────────────────────────────────────────────
    def move(self, servo, direction, src='KB'):
        with self.lock:

            if servo == 'shoulder':
                self.shoulder = self.clamp(
                    self.shoulder + (STEP if direction == 'up' else -STEP), 0, 180)
                self.publish(self.pub_shoulder, self.shoulder)
                arrow = '▲' if direction == 'up' else '▼'
                print(f'  [{src}] SHOULDER {arrow} → {self.shoulder} deg')

            elif servo == 'elbow':
                self.elbow = self.clamp(
                    self.elbow + (STEP if direction == 'up' else -STEP), 0, 180)
                self.publish(self.pub_elbow, self.elbow)
                arrow = '▲' if direction == 'up' else '▼'
                print(f'  [{src}] ELBOW    {arrow} → {self.elbow} deg')

            elif servo == 'gripper':
                self.gripper = self.clamp(
                    self.gripper + (-STEP if direction == 'open' else STEP), 0, 90)
                self.publish(self.pub_gripper, self.gripper)
                label = 'OPEN' if direction == 'open' else 'CLOSE'
                print(f'  [{src}] GRIPPER  {label} → {self.gripper} deg')

            print(f'  [POS] Shoulder:{self.shoulder}  Elbow:{self.elbow}  Gripper:{self.gripper}\n')

    # ──────────────────────────────────────────────────────────
    #  KEYBOARD LOOP (runs in background thread)
    # ──────────────────────────────────────────────────────────
    def keyboard_loop(self):
        while rclpy.ok():
            try:
                key = input('  KB > ').strip().lower()
            except (KeyboardInterrupt, EOFError):
                rclpy.shutdown()
                break

            if   key == 'q': self.move('shoulder', 'down',  'KB')
            elif key == 'w': self.move('shoulder', 'up',    'KB')
            elif key == 'a': self.move('elbow',    'down',  'KB')
            elif key == 's': self.move('elbow',    'up',    'KB')
            elif key == 'd': self.move('gripper',  'open',  'KB')
            elif key == 'f': self.move('gripper',  'close', 'KB')
            elif key == 'e':
                print('  [EXIT] Shutting down...')
                rclpy.shutdown()
                break
            else:
                print(f'  [?] Unknown key: "{key}"  — E to exit')

    # ──────────────────────────────────────────────────────────
    #  XBOX CONTROLLER LOOP (runs in background thread)
    #
    #  LT/RT are analog triggers — polled via get_axis() every
    #  tick rather than relying on events (more reliable).
    #  They fire ONCE when pressed past threshold, reset on release.
    #
    #  LB/RB/X/Y are buttons — handled via JOYBUTTONDOWN events.
    # ──────────────────────────────────────────────────────────
    def controller_loop(self):
        pygame.init()
        pygame.joystick.init()

        # ── Wait for controller ───────────────────────────────
        print('  [CTRL] Waiting for Xbox controller...')
        while pygame.joystick.get_count() == 0:
            time.sleep(1)
            pygame.joystick.quit()
            pygame.joystick.init()

        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f'  [CTRL] Connected: {joy.get_name()}')
        print(f'  [CTRL] Axes: {joy.get_numaxes()}  Buttons: {joy.get_numbuttons()}\n')

        clock = pygame.time.Clock()

        while rclpy.ok():
            # pump() MUST be called so get_axis() returns fresh values
            pygame.event.pump()

            # ── Poll LT (axis 2) → Elbow DOWN ────────────────
            lt = joy.get_axis(AXIS_LT)
            if lt > TRIGGER_THRESHOLD and not self.lt_held:
                self.lt_held = True
                self.move('elbow', 'down', 'CTRL')
            elif lt <= TRIGGER_THRESHOLD:
                self.lt_held = False

            # ── Poll RT (axis 5) → Elbow UP ──────────────────
            rt = joy.get_axis(AXIS_RT)
            if rt > TRIGGER_THRESHOLD and not self.rt_held:
                self.rt_held = True
                self.move('elbow', 'up', 'CTRL')
            elif rt <= TRIGGER_THRESHOLD:
                self.rt_held = False

            # ── Button events ─────────────────────────────────
            for event in pygame.event.get():

                if event.type == pygame.JOYBUTTONDOWN:
                    if   event.button == BTN_LB:
                        self.move('shoulder', 'down',  'CTRL')
                    elif event.button == BTN_RB:
                        self.move('shoulder', 'up',    'CTRL')
                    elif event.button == BTN_X:
                        self.move('gripper',  'open',  'CTRL')
                    elif event.button == BTN_Y:
                        self.move('gripper',  'close', 'CTRL')

                elif event.type == pygame.JOYDEVICEREMOVED:
                    print('\n  [CTRL] Controller disconnected!')

                elif event.type == pygame.JOYDEVICEADDED:
                    pygame.joystick.quit()
                    pygame.joystick.init()
                    if pygame.joystick.get_count() > 0:
                        joy = pygame.joystick.Joystick(0)
                        joy.init()
                        print(f'\n  [CTRL] Reconnected: {joy.get_name()}\n')

            clock.tick(60)   # 60Hz poll — responsive but light on CPU

        pygame.quit()

    # ──────────────────────────────────────────────────────────
    #  CONTROLS REFERENCE
    # ──────────────────────────────────────────────────────────
    def print_controls(self):
        print('\n' + '='*50)
        print('  ARM PUBLISHER — Keyboard + Xbox Controller')
        print('='*50)
        print('  KEYBOARD (type + Enter):')
        print('    Q / W  →  Shoulder  DOWN / UP')
        print('    A / S  →  Elbow     DOWN / UP')
        print('    D / F  →  Gripper   OPEN / CLOSE')
        print('    E      →  Exit')
        print('  XBOX CONTROLLER:')
        print('    LB / RB  →  Shoulder  DOWN / UP')
        print('    LT / RT  →  Elbow     DOWN / UP')
        print('    X  / Y   →  Gripper   OPEN / CLOSE')
        print('='*50 + '\n')


# ── Entry point ───────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ArmPublisher()

    # Start controller in background thread
    ctrl_thread = threading.Thread(target=node.controller_loop, daemon=True)
    ctrl_thread.start()

    # Start keyboard in background thread
    kb_thread = threading.Thread(target=node.keyboard_loop, daemon=True)
    kb_thread.start()

    # ROS2 spin on main thread
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