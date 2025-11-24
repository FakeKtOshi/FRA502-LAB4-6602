#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_interfaces.srv import RandomEndEffector
import random


class RandomNode(Node):
    def __init__(self):
        super().__init__('random_node')

        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)
        self.server = self.create_service(
            RandomEndEffector, "/random_pose", self.random_callback
        )

        self.r_min = 0.05
        self.r_max = 0.30
        self.l = 0.20

        self.get_logger().info("random_node started")

    def random_callback(self, request, response):

        if request.mode.data != "AUTO":
            response.inprogress = False
            return response

        while True:
            x = random.uniform(-self.r_max, self.r_max)
            y = random.uniform(-self.r_max, self.r_max)
            z = random.uniform(0.05, self.r_max)

            d = (x**2 + y**2 + (z - self.l)**2)

            if self.r_min**2 < d < self.r_max**2:
                break

        # RViz marker
        msg = PoseStamped()
        msg.header.frame_id = "link_0"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.target_pub.publish(msg)

        # service response
        response.position.x = float(x)
        response.position.y = float(y)
        response.position.z = float(z)
        response.inprogress = True

        self.get_logger().info(f"Random target published: {x:.2f}, {y:.2f}, {z:.2f}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RandomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
