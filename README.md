# 🦾 3-DOF Planar Robotic Arm Control – ROS 2 Challenge

This repository contains a ROS 2-based simulation of a simple 3-DOF planar robotic arm. The main goal is to control the end-effector to track a moving target in real-time using kinematic principles.

---

## 🛠️ Setup Instructions

To get the code up and running:

1. Clone this repository into your ROS 2 workspace (e.g., `~/ros2_ws/src/`).
2. Make sure you have all dependencies installed (Python, NumPy, ROS 2 Foxy/Humble/...).
3. Build the workspace:

   ```bash
   colcon build --packages-select <your_package_name>
   source install/setup.bash
