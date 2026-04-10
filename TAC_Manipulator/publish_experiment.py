import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pygame
import threading
import time

STEP = 5

AXIS_LT = 2
AXIS_RT = 5
BTN_LB  = 4
BTN_RB  = 5
BTN_X   = 2
BTN_Y   = 3

THRESHOLD = 0.1


class ArmPublisher(Node):

    def __init__(self):
        super().__init__('publisher')

        self.pub1 = self.create_publisher(Int32, '/arm/shoulder_angle', 10)
        self.pub2 = self.create_publisher(Int32, '/arm/elbow_angle', 10)
        self.pub3 = self.create_publisher(Int32, '/arm/gripper_angle', 10)

        self.s = 90
        self.e = 90
        self.g = 0

        self.lt_active = False
        self.rt_active = False

        print("\nControls:")
        print("Keyboard + Xbox controller enabled\n")

        threading.Thread(target=self.keyboard_loop, daemon=True).start()
        threading.Thread(target=self.controller_loop, daemon=True).start()

    def publish(self):
        self.pub1.publish(Int32(data=self.s))
        self.pub2.publish(Int32(data=self.e))
        self.pub3.publish(Int32(data=self.g))

        print(f"[PUB] S:{self.s} E:{self.e} G:{self.g}")

    def clamp(self, v, mn, mx):
        return max(mn, min(mx, v))

    # ───────── KEYBOARD ─────────
    def keyboard_loop(self):
        while rclpy.ok():
            key = input(">> ").lower()

            if key == 'q': self.s -= STEP
            elif key == 'w': self.s += STEP
            elif key == 'a': self.e -= STEP
            elif key == 's': self.e += STEP
            elif key == 'd': self.g -= STEP
            elif key == 'f': self.g += STEP
            elif key == 'e':
                rclpy.shutdown()
                break
            else:
                continue

            self.apply_limits()
            self.publish()

    # ───────── CONTROLLER ─────────
    def controller_loop(self):
        pygame.init()
        pygame.joystick.init()

        while pygame.joystick.get_count() == 0:
            print("Waiting for controller...")
            time.sleep(2)

        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f"Controller: {joy.get_name()}")

        clock = pygame.time.Clock()

        while rclpy.ok():
            pygame.event.pump()

            lt = joy.get_axis(AXIS_LT)
            rt = joy.get_axis(AXIS_RT)

            if lt > THRESHOLD and not self.lt_active:
                self.s += STEP
                self.lt_active = True
                self.apply_limits()
                self.publish()
            elif lt <= THRESHOLD:
                self.lt_active = False

            if rt > THRESHOLD and not self.rt_active:
                self.e += STEP
                self.rt_active = True
                self.apply_limits()
                self.publish()
            elif rt <= THRESHOLD:
                self.rt_active = False

            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_LB:
                        self.s -= STEP
                    elif event.button == BTN_RB:
                        self.e -= STEP
                    elif event.button == BTN_X:
                        self.g -= STEP
                    elif event.button == BTN_Y:
                        self.g += STEP

                    self.apply_limits()
                    self.publish()

            clock.tick(60)

    def apply_limits(self):
        self.s = self.clamp(self.s, 0, 180)
        self.e = self.clamp(self.e, 0, 180)
        self.g = self.clamp(self.g, 0, 90)


def main():
    rclpy.init()
    node = ArmPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()