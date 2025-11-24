#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import State
from geometry_msgs.msg import Twist

import sys
import termios
import tty


# --------------------------------------------------
# Get 1 character from keyboard (no Enter required)
# --------------------------------------------------
def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        c = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return c


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')

        # Service client for state machine
        self.cli = self.create_client(State, "/state_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /state_service ...")

        # Publisher for teleoperation velocities
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Teleop speed
        self.speed = 0.05

        self.get_logger().info("Keyboard ready!")
        self.get_logger().info("Controls:")
        self.get_logger().info("  A : AUTO mode")
        self.get_logger().info("  I : IK mode → enter X Y Z")
        self.get_logger().info("  F : TELEOP_F")
        self.get_logger().info("  G : TELEOP_G")
        self.get_logger().info("  W/S : Move +X / -X")
        self.get_logger().info("  A/D : Move +Y / -Y")
        self.get_logger().info("  E/Q : Move +Z / -Z")
        self.get_logger().info("  SPACE : Stop")
        self.get_logger().info("  X : exit")

    # --------------------------------------------------
    def send_state(self, state, button="", xyz=None):
        req = State.Request()
        req.state.data = state
        req.button.data = button

        if xyz is not None:
            req.position.x = float(xyz[0])
            req.position.y = float(xyz[1])
            req.position.z = float(xyz[2])

        return self.cli.call_async(req)

    # --------------------------------------------------
    def publish_velocity(self, x, y, z):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        self.vel_pub.publish(msg)

    # --------------------------------------------------
    def loop(self):
        while rclpy.ok():
            key = getch().lower()

            # EXIT
            if key == "x":
                self.get_logger().info("Exit keyboard.")
                break

            # ======================
            # STATE: AUTO
            # ======================
            if key == "a":
                self.get_logger().info("Request AUTO")
                self.send_state("AUTO")

            # ======================
            # STATE: IK + XYZ INPUT
            # ======================
            elif key == "i":
                self.get_logger().info("Enter IK X Y Z:")
                xyz_str = input().split()

                if len(xyz_str) != 3:
                    self.get_logger().error("Invalid input! Use: x y z")
                    continue

                x = float(xyz_str[0])
                y = float(xyz_str[1])
                z = float(xyz_str[2])

                self.get_logger().info(f"Sending IK target → ({x}, {y}, {z})")
                self.send_state("IK", xyz=[x, y, z])

            # ======================
            # STATE: TELEOP MODES
            # ======================
            elif key == "f":
                self.get_logger().info("Switch to TELEOP_F")
                self.send_state("TELEOP", button="F")

            elif key == "g":
                self.get_logger().info("Switch to TELEOP_G")
                self.send_state("TELEOP", button="G")

            # ======================
            # TELEOP MOTION (velocity commands)
            # ======================
            # +X / -X
            elif key == "w":
                self.publish_velocity(self.speed, 0, 0)
            elif key == "s":
                self.publish_velocity(-self.speed, 0, 0)

            # +Y / -Y
            elif key == "a":
                self.publish_velocity(0, self.speed, 0)
            elif key == "d":
                self.publish_velocity(0, -self.speed, 0)

            # +Z / -Z
            elif key == "e":
                self.publish_velocity(0, 0, self.speed)
            elif key == "q":
                self.publish_velocity(0, 0, -self.speed)

            # STOP
            elif key == " ":
                self.get_logger().info("STOP")
                self.publish_velocity(0, 0, 0)

            else:
                self.get_logger().info(f"Unknown key: '{key}'")


# --------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
