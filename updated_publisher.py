from timeit import main

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pygame
import threading
import time
import sys


SHOULDER_MIN, SHOULDER_MAX = 0, 180
ELBOW_MIN,    ELBOW_MAX    = 0, 180
GRIPPER_MIN,  GRIPPER_MAX  = 0, 90

STEP = 5

AXIS_LT = 2
AXIS_RT = 5
BTN_LB  = 4
BTN_RB  = 5
BTN_X   = 2
BTN_Y   = 3

TRIGGER_THRESHOLD = 0.1

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

        # ── Trigger state: tracks if trigger is currently held
        #    True  = held past threshold (don't fire again)
        #    False = released (ready to fire on next press)
        self.lt_active = False
        self.rt_active = False

        self.get_logger().info('Arm Teleop Publisher started.')


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
        print('    LT (axis 2)  →  Shoulder  UP     (+5 deg)')
        print('    RB (btn 5)   →  Elbow     DOWN   (-5 deg)')
        print('    RT (axis 5)  →  Elbow     UP     (+5 deg)')
        print('    X  (btn 2)   →  Gripper   OPEN   (-5 deg)')
        print('    Y  (btn 3)   →  Gripper   CLOSE  (+5 deg)')
        print('  ' + '='*50)
        print('    Subscriber output in Terminal 2.')
        print('  ' + '='*50)
        print()



    def publish_angles(self, publisher, angle):
        msg  = Int32()
        msg.data = angle 
        publisher.publish(msg)

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)
    
    def move_servo(self,servo,direction, source='KB'): 
        with self.lock:
            if servo == 'shoulder':
                self.pos_shoulder = self.clamp(
                        self.pos_shoulder + (STEP if direction == 'up' else -STEP),
                        SHOULDER_MIN, SHOULDER_MAX)
                self.publish_angles(self.pub_shoulder, self.pos_shoulder)

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

    def keyboard_loop(self):
        #print_help()
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
            # elif key == 'h': print_help()
            elif key == 'e':
                print('\n  [EXIT] Shutting down...')
                rclpy.shutdown()
                break
            else:
                print(f'  [?] Unknown key: "{key}"')

        


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
            # Must call event.get() or pump() so pygame updates axis values
            pygame.event.pump()

            # ── Poll LT (axis 2) ──────────────────────────────
            lt_val = joy.get_axis(AXIS_LT)
            # if DEBUG:
            #     if abs(lt_val) > 0.05:
            #         print(f'  [DEBUG] LT raw value = {lt_val:.3f}')

            if lt_val > TRIGGER_THRESHOLD and not self.lt_active:
                self.lt_active = True
                self.move_servo('elbow', 'up', 'CTRL')#changed shoulder to elbow here 
            elif lt_val <= TRIGGER_THRESHOLD:
                self.lt_active = False    # Released — ready for next press

            # ── Poll RT (axis 5) ──────────────────────────────
            rt_val = joy.get_axis(AXIS_RT)
            # if DEBUG:
            #     if abs(rt_val) > 0.05:
            #         print(f'  [DEBUG] RT raw value = {rt_val:.3f}')

            if rt_val > TRIGGER_THRESHOLD and not self.rt_active:
                self.rt_active = True
                self.move_servo('elbow', 'up', 'CTRL')
            elif rt_val <= TRIGGER_THRESHOLD:
                self.rt_active = False

            # ── Poll buttons (LB, RB, X, Y) ──────────────────
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if   event.button == BTN_LB: self.move_servo('shoulder', 'down',  'CTRL')
                    elif event.button == BTN_RB: self.move_servo('elbow',    'down',  'CTRL')
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

