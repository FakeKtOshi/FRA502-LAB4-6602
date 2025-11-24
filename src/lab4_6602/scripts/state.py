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

        # Robot Model - FIXED: Added joint limits
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0,   a=0.0,  d=0.2,   qlim=[-pi, pi]),
                rtb.RevoluteMDH(alpha=pi/2,  a=0.0,  d=0.02,  qlim=[-pi, pi]),
                rtb.RevoluteMDH(alpha=0.0,   a=0.25, d=0.0,   qlim=[-pi, pi]),
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
    # OPTION 1: Keep IK check in state_node (with proper settings)
    def run_ik(self, pos):
        x, y, z = pos
        T = SE3(x, y, z)

        sol = self.robot.ikine_LM(
            T, 
            mask=[1, 1, 1, 0, 0, 0], 
            joint_limits=True,      # ← FIXED: Enable joint limits
            q0=[0.0, 0.0, 0.0]       # ← FIXED: Better starting pose
        )

        if sol.success:
            self.get_logger().info(f"[State IK] Solution: {sol.q}")
        
        return sol.q if sol.success else None

    def req_inverse_kinematics(self, request):
        pos = [
            float(request.position.x),
            float(request.position.y),
            float(request.position.z),
        ]

        q_sol = self.run_ik(pos)

        if q_sol is None:
            self.get_logger().error("[State IK] Failed, switching to IDLE")
            self.robot_state = "IDLE"
            return False

        self.get_logger().info(f"[State IK] Success: {q_sol}")
        self.req_controller("IK", pos)
        return True

    # ----------------------------------------------------------------------
    # OPTION 2 (ALTERNATIVE): Remove IK check from state_node
    # Uncomment this and comment out the above function if you want
    # controller_node to handle all IK validation
    
    # def req_inverse_kinematics(self, request):
    #     pos = [
    #         float(request.position.x),
    #         float(request.position.y),
    #         float(request.position.z),
    #     ]
    #     
    #     # Let controller_node handle IK validation
    #     self.req_controller("IK", pos)
    #     
    #     # We'll wait for controller response via callback
    #     future = self.controller_client.call_async(
    #         self.create_controller_request("IK", pos)
    #     )
    #     future.add_done_callback(self.controller_ik_callback)
    #     
    #     return True  # Assume success, controller will decide
    # 
    # def controller_ik_callback(self, future):
    #     res = future.result()
    #     if not res.inprogress:
    #         self.get_logger().error("[Controller IK] Failed")
    #         self.robot_state = "IDLE"

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

        if new_state == "AUTO":
            self.robot_state = "AUTO"
            self.req_random()
            response.inprogress = True

        elif new_state == "IK":
            # IK validation happens in req_inverse_kinematics
            # It will set robot_state back to IDLE if IK fails
            self.robot_state = "IK"
            response.inprogress = self.req_inverse_kinematics(request)

        elif new_state == "TELEOP":

            if request.button.data == "F":
                self.robot_state = "TELEOP_F"
                self.req_controller("TELEOP_F", [0, 0, 0])

            elif request.button.data == "G":
                self.robot_state = "TELEOP_G"
                self.req_controller("TELEOP_G", [0, 0, 0])

            else:
                if self.robot_state not in ["TELEOP_F", "TELEOP_G"]:
                    self.robot_state = "TELEOP"

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