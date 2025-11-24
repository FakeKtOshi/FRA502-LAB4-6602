#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.srv import RandomEndEffector, Controller, State
from spatialmath import SE3
import roboticstoolbox as rtb
from math import pi


class StateNode(Node):
    def __init__(self):
        super().__init__("state_node")

        # Frequency
        self.declare_parameter("set_frequency", 100.0)
        self.frequency = self.get_parameter("set_frequency").value
        self.create_timer(1.0 / self.frequency, self.timer_callback)

        # Publisher
        self.robot_state_publisher = self.create_publisher(String, "/robot_state", 10)
        self.robot_state = "IDLE"

        # Clients
        self.random_client = self.create_client(RandomEndEffector, "/random_pose")
        self.controller_client = self.create_client(Controller, "/controller_service")

        # wait
        self.get_logger().info("Waiting for /random_pose ...")
        while not self.random_client.wait_for_service(timeout_sec=1.0):
            pass

        self.get_logger().info("Waiting for /controller_service ...")
        while not self.controller_client.wait_for_service(timeout_sec=1.0):
            pass

        # Service
        self.state_service = self.create_service(
            State, "/state_service", self.state_service_callback
        )

        # Robot Model
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(0, 0, 0.2),
                rtb.RevoluteMDH(pi / 2, 0, 0.02),
                rtb.RevoluteMDH(0, 0.25, 0),
            ],
            tool=SE3.Tx(0.28),
            name="3UR Robot",
        )

        self.get_logger().info("state_node started")

    # ----------------------------------------------------------------------
    def req_random(self):
        req = RandomEndEffector.Request()
        req.mode.data = "AUTO"

        future = self.random_client.call_async(req)
        future.add_done_callback(self.req_random_callback)

    def req_random_callback(self, future):
        res = future.result()

        if not res.inprogress:
            self.get_logger().error("Random pose failed")
            return

        pos = [res.position.x, res.position.y, res.position.z]
        self.get_logger().info(f"Random target = {pos}")

        self.req_controller("AUTO", pos)

    # ----------------------------------------------------------------------
    def run_ik(self, pos):
        x, y, z = pos
        T = SE3(x, y, z)

        sol = self.robot.ikine_LM(
            T, mask=[1, 1, 1, 0, 0, 0], joint_limits=False, q0=[0, 0, 0]
        )

        return sol.q if sol.success else None

    def req_inverse_kinematics(self, request):
        pos = [
            float(request.position.x),
            float(request.position.y),
            float(request.position.z),
        ]

        q_sol = self.run_ik(pos)

        if q_sol is None:
            self.get_logger().error("IK failed, switching to IDLE")
            self.robot_state = "IDLE"
            return False

        self.get_logger().info(f"IK solution: {q_sol}")

        self.req_controller("IK", pos)
        return True

    # ----------------------------------------------------------------------
    def req_controller(self, mode, pos):
        req = Controller.Request()
        req.mode.data = mode
        req.position.x = float(pos[0])
        req.position.y = float(pos[1])
        req.position.z = float(pos[2])

        self.controller_client.call_async(req)

    # ----------------------------------------------------------------------
    def state_service_callback(self, request, response):

        new_state = request.state.data
        self.get_logger().info(f"State change {self.robot_state} → {new_state}")
        self.robot_state = new_state

        if new_state == "AUTO":
            self.req_random()
            response.inprogress = True

        elif new_state == "IK":
            response.inprogress = self.req_inverse_kinematics(request)

        elif new_state == "TELEOP":
            if request.button.data == "F":
                self.req_controller("TELEOP_F", [0,0,0])
            elif request.button.data == "G":
                self.req_controller("TELEOP_G", [0,0,0])
            response.inprogress = True

        response.mode_readback.data = self.robot_state
        return response

    # ----------------------------------------------------------------------
    def timer_callback(self):
        msg = String()
        msg.data = self.robot_state
        self.robot_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
