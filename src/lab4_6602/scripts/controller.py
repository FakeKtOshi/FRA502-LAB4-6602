#!/usr/bin/python3

from lab4_6602.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from robot_interfaces.srv import Controller
from sensor_msgs.msg import JointState

from math import pi
from spatialmath import SE3
import numpy as np
from scipy.spatial.transform import Rotation as R
import roboticstoolbox as rtb


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # ============================
        # Frequency
        # ============================
        self.declare_parameter("set_frequency", 100.0)
        self.frequency = self.get_parameter("set_frequency").value
        self.dt = 1.0 / self.frequency
        self.create_timer(self.dt, self.timer_callback)

        # ============================
        # Publishers
        # ============================
        self.endeffector_pub = self.create_publisher(PoseStamped, "/end_effector", 10)
        self.target_rviz_pub = self.create_publisher(PoseStamped, "/target_rviz", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        # ============================
        # Subscribers
        # ============================
        self.create_subscription(String, "/robot_state", self.robot_state_callback, 10)
        self.create_subscription(PoseStamped, "/target", self.target_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # ============================
        # Services
        # ============================
        self.controller_service = self.create_service(
            Controller, "/controller_service", self.controller_service_callback
        )

        # ============================
        # Robot model
        # ============================
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteMDH(alpha=0.0,   a=0.0,  d=0.2,   qlim=[-pi, pi]),   # ← ADD
                rtb.RevoluteMDH(alpha=pi/2,  a=0.0,  d=0.02,  qlim=[-pi, pi]),   # ← ADD
                rtb.RevoluteMDH(alpha=0.0,   a=0.25, d=0.0,   qlim=[-pi, pi]),   # ← ADD
            ],
            tool=SE3.Tx(0.28),
            name="3UR Robot"
        )

        # Workspace params (same idea as random node)
        self.r_min = 0.05
        self.r_max = 0.30
        self.base_height = 0.2  # z offset in distance check

        # ============================
        # State variables
        # ============================
        # Start at a non-singular pose (avoid q = [0,0,0])
        self.q = np.array([0.0, 0.0, 0.0], dtype=float)
        self.kp = 1.0

        self.random_target = None    # for AUTO mode (from random_pos / state)
        self.ik_setpoint = [0.0, 0.0, 0.0]   # for IK target (xyz)
        self.robot_state = "IDLE"

        # Teleop velocities
        self.tele_x = 0.0
        self.tele_y = 0.0
        self.tele_z = 0.0
        self.last_cmd_vel_time = self.get_clock().now()

        # TF for EE pose in world frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.source_frame = "link_0"
        self.target_frame = "end_effector"

        self.get_logger().info("controller_node started")

    # ==========================================================
    # SERVICE: Controller
    # ==========================================================
    def controller_service_callback(self, request, response):
        mode = request.mode.data

        if mode == "AUTO":
            # AUTO: just accept target (state node already used random service)
            self.random_target = [
                float(request.position.x),
                float(request.position.y),
                float(request.position.z),
            ]
            self.get_logger().info(f"[AUTO] Target received: {self.random_target}")
            response.inprogress = True

        elif mode == "IK":
            # HYBRID IK:
            # 1) Check workspace + full IK via ikine_LM
            # 2) If success → snap q to IK solution + store ik_setpoint
            x = float(request.position.x)
            y = float(request.position.y)
            z = float(request.position.z)
            target = [x, y, z]

            q_sol = self.compute_inverse_kinematics(target)
            if q_sol is None:
                self.get_logger().error("[IK] IK failed or unreachable")
                response.inprogress = False
                return response

            # Snap joints to IK solution (config-space IK)
            self.q = q_sol.copy()
            self.ik_setpoint = target
            self.get_logger().info(f"[IK] IK accepted, q = {self.q}")
            response.inprogress = True

        elif mode == "TELEOP_F" or mode == "TELEOP_G":
            # Teleop mode is controlled by /robot_state + /cmd_vel
            self.get_logger().info(f"[TELEOP] Mode = {mode}")
            response.inprogress = True

        else:
            self.get_logger().warning(f"Invalid mode request: {mode}")
            response.inprogress = False

        return response

    # ==========================================================
    # FULL IK SOLVER (for IK mode)
    # ==========================================================
    # def compute_inverse_kinematics(self, pos):
    #     x, y, z = pos

    #     # Workspace reachability check
    #     distance_sq = x**2 + y**2 + (z - self.base_height) ** 2
    #     if not (self.r_min**2 <= distance_sq <= self.r_max**2):
    #         self.get_logger().error("[IK] Target out of workspace")
    #         return None

    #     # Build SE3 target
    #     T = SE3(x, y, z)

    #     # Solve IK using Levenberg–Marquardt (position only)
    #     sol = self.robot.ikine_LM(
    #         T,
    #         mask=[1, 1, 1, 0, 0, 0],
    #         joint_limits=True,
    #         q0=self.q,   # start from current configuration
    #     )

    #     if not sol.success:
    #         self.get_logger().error("[IK] ikine_LM failed")
    #         return None

    #     self.get_logger().info(f"[IK] ikine_LM solution: {sol.q}")
    #     return sol.q

    def compute_inverse_kinematics(self, pos):
        x, y, z = pos

        # Workspace reachability check
        distance_sq = x**2 + y**2 + (z - self.base_height) ** 2
        distance = np.sqrt(distance_sq)

        self.get_logger().info(
            f"[IK] Distance check: {distance:.3f}m (valid: {self.r_min:.3f} to {self.r_max:.3f})"
        )

        if not (self.r_min**2 <= distance_sq <= self.r_max**2):
            self.get_logger().error(
                f"[IK] Target out of workspace! Distance: {distance:.3f}m"
            )
            return None

        # Build SE3 target
        T = SE3(x, y, z)

        # Use a safe default starting config instead of current position
        q0 = np.array([0.0, -0.5, 0.5])  # ← Always use this safe config

        self.get_logger().info(f"[IK] Solving IK from q0 = {q0}")

        sol = self.robot.ikine_LM(
            T,
            mask=[1, 1, 1, 0, 0, 0],
            joint_limits=True,
            q0=q0,  # ← Fixed starting point
        )

        if not sol.success:
            self.get_logger().error("[IK] ikine_LM failed to converge")
            return None

        self.get_logger().info(f"[IK] ikine_LM solution: {sol.q}")
        return sol.q

    # ==========================================================
    # Subscribers
    # ==========================================================
    def robot_state_callback(self, msg):
        self.robot_state = msg.data
        self.get_logger().info(f"Robot state: {self.robot_state}")

    def target_callback(self, msg):
        self.random_target = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]
        self.get_logger().info(f"[AUTO] Random target from /target: {self.random_target}")

    def cmd_vel_callback(self, msg):
        self.tele_x = msg.linear.x
        self.tele_y = msg.linear.y
        self.tele_z = msg.linear.z
        self.last_cmd_vel_time = self.get_clock().now()

    # ==========================================================
    # TF helper: get current EE position + orientation
    # ==========================================================
    def get_transform(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame, rclpy.time.Time()
            )

            pos = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ])

            q = t.transform.rotation
            Rm = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

            return pos, Rm

        except Exception as e:
            # Uncomment if you want to see TF warnings
            # self.get_logger().warning(f"TF lookup failed: {e}")
            return None, None

    # ==========================================================
    # Joint state publication
    # ==========================================================
    def publish_joint_angle(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint_1", "joint_2", "joint_3"]
        msg.position = list(q)
        self.joint_pub.publish(msg)

    # ==========================================================
    # Teleoperation (velocity control) with singularity protection
    # ==========================================================
    def control_vel(self, mode):
    # ---- AUTO STOP after 0.1s with no new cmd_vel ----
        now = self.get_clock().now()
        dt = (now - self.last_cmd_vel_time).nanoseconds * 1e-9

        if dt > 0.1:
            # No input → force stop
            self.tele_x = 0.0
            self.tele_y = 0.0
            self.tele_z = 0.0

        # If the command is zero, do nothing (stop)
        if abs(self.tele_x) < 1e-6 and abs(self.tele_y) < 1e-6 and abs(self.tele_z) < 1e-6:
            return

        # ---- Get current transform ----
        pos, Rm = self.get_transform()
        if pos is None:
            return

        # ---- Compute task-space velocity ----
        v = np.array([self.tele_x, self.tele_y, self.tele_z], dtype=float)
        p_dot = v if mode == "TELEOP_G" else Rm @ v

        # ---- Jacobian inverse kinematics ----
        J = self.robot.jacob0(self.q)[0:3, :]

        # Singularity protection
        _, s, _ = np.linalg.svd(J)
        if s[-1] < 1e-5:
            self.get_logger().warning(
                f"[TELEOP] Near singularity σ_min={s[-1]:.4e}. Stop."
            )
            return

        q_dot = np.linalg.pinv(J) @ p_dot
        self.q = self.q + q_dot * self.dt

        self.publish_joint_angle(self.q)


    # ==========================================================
    # Position control (AUTO / IK tracking)
    # ==========================================================
    def control_pos(self, target):
        pos, _ = self.get_transform()
        if pos is None:
            return

        p_err = self.kp * (np.array(target, dtype=float) - pos)

        J = self.robot.jacob0(self.q)[0:3, :]
        # For AUTO / IK you *can* add singularity check if you want:
        # _, s, _ = np.linalg.svd(J)
        # if s[-1] < 1e-4:
        #     self.get_logger().warning(f"[AUTO/IK] Singular Jacobian σ_min={s[-1]:.4f}.")
        #     return

        q_dot = np.linalg.pinv(J) @ p_err
        self.q = self.q + q_dot * self.dt
        self.publish_joint_angle(self.q)

    # ==========================================================
    # RViz markers
    # ==========================================================
    def publish_markers(self, target):
        # Target marker
        t = PoseStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.source_frame
        t.pose.position.x = target[0]
        t.pose.position.y = target[1]
        t.pose.position.z = target[2]
        self.target_rviz_pub.publish(t)

        # Current EE marker
        pos, _ = self.get_transform()
        if pos is not None:
            ee = PoseStamped()
            ee.header.stamp = self.get_clock().now().to_msg()
            ee.header.frame_id = self.source_frame
            ee.pose.position.x = pos[0]
            ee.pose.position.y = pos[1]
            ee.pose.position.z = pos[2]
            self.endeffector_pub.publish(ee)

    # ==========================================================
    # Main timer loop
    # ==========================================================
    def timer_callback(self):
        # Always publish current joints
        self.publish_joint_angle(self.q)

        if self.robot_state == "AUTO" and self.random_target is not None:
            self.control_pos(self.random_target)
            self.publish_markers(self.random_target)

        elif self.robot_state == "IK":
            self.control_pos(self.ik_setpoint)
            self.publish_markers(self.ik_setpoint)

        elif self.robot_state == "TELEOP_G":
            self.control_vel("TELEOP_G")

        elif self.robot_state == "TELEOP_F":
            self.control_vel("TELEOP_F")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
