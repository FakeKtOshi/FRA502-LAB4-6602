from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    pkg = get_package_share_directory("lab4_6602")

    # --- Load RViz file ---
    rviz_path = os.path.join(pkg, "config", "display.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_path],
        output="screen"
    )

    # --- Load xacro robot description ---
    xacro_path = os.path.join(pkg, "robot", "visual", "my-robot.xacro")
    robot_xml = xacro.process_file(xacro_path).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_xml}]
    )

    # --- Controller Node ---
    controller = Node(
        package="lab4_6602",
        executable="controller.py",
        name="controller_node",
        output="screen"
    )

    # --- State Machine ---
    scheduler = Node(
        package="lab4_6602",
        executable="state.py",
        name="state_node",
        output="screen"
    )

    # --- Random Target Generator ---
    random_gen = Node(
        package="lab4_6602",
        executable="random_pos.py",
        name="random_node",
        output="screen"
    )

    # --- Keyboard Teleop (optional) ---
    keyboard = Node(
        package="lab4_6602",
        executable="keyboard.py",
        name="keyboard_node",
        output="screen",
        emulate_tty=True
    )

    return LaunchDescription([
        rviz,
        robot_state_publisher,
        controller,
        scheduler,
        random_gen,
        #keyboard    # comment out if you want manual run
    ])
