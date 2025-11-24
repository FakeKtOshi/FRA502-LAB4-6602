# ROS2-UR_Robot-Visualization
Krit Leetrakul 6602 (Oshi)

# 🚀 Lab4-6602 Robot Controller (ROS2 Humble)
A complete ROS2 control system for a 3-DoF robot arm including AUTO random motion, IK tracking, TELEOP velocity control, and RViz visualization.

# 📂 Project Tree
```
lab4_wspace/
└── src/
    └── lab4_6602/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/launch.py
        ├── config/display.rviz
        ├── meshes/*.stl
        ├── robot/visual/my-robot.xacro
        ├── scripts/
        │   ├── controller.py
        │   ├── state.py
        │   ├── random_pos.py
        │   ├── keyboard.py
        │   ├── initial_jointstate.py
        │   └── dummy_script.py
        └── lab4_6602/dummy_module.py
```

# 🛠 Install
```
cd ~/lab4_wspace/src
git clone https://github.com/<your_repo>/lab4_6602.git
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-tf2-ros ros-humble-rviz2 python3-scipy
cd ~/lab4_wspace
colcon build --symlink-install
source install/setup.bash
```

# ▶️ Run System
# Terminal 1
```
ros2 launch lab4_6602 launch.py
```

# Terminal 2
```
ros2 run lab4_6602 keyboard.py
```

# Keyboard Commands
a = AUTO

i = IK

t = TELEOP

f = TELEOP_F (world frame)

g = TELEOP_G (gripper frame)

