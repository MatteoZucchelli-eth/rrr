# 🦾 3-DOF Planar Robotic Arm Control – ROS 2 Challenge

This repository contains a ROS 2-based simulation of a simple 3-DOF planar robotic arm. The main goal is to control the end-effector to track a moving target in real-time.

---

## 🛠️ Setup Instructions

To get the code up and running:

1. Clone this repository into your ROS 2 workspace (e.g., `~/ros2_ws/src/`).
2. Make sure you have all dependencies installed (Python, NumPy, ROS 2 Humble).
3. Build the workspace:

   ```bash
   colcon build --packages-select rrr_challenge
   source install/setup.bash

Launch the system with:

bash
Copy code
ros2 launch rrr_challenge display.launch.py
🤖 Robot Modeling
To model the robot:

The simplified robotic arm was designed using Onshape.

The STL files were exported and used to construct a URDF model.

The robot model is visualized using RViz for simulation and debugging.

🧠 Node Architecture
The system consists of three ROS 2 nodes, all written in Python:

1. Master Node
Publishes: Target position (/desired_position)

Description: Periodically generates and publishes a target position that the robot's end-effector must follow.

2. End-Effector Node
Publishes: End-effector position (/current_position)

Description: Computes the current Cartesian coordinates of the end-effector from rviz and publishes a clean, ready-to-use version.

3. Controller Node
Subscribes:

Target position (/desired_position)

Current end-effector position (/current_position)

Publishes: Joint states

Description: Controls the robot by computing the necessary joint velocities that will drive the end-effector to follow the desired trajectory.

🧩 How It Works:
The controller reads both the current and desired end-effector positions.

From robotics kinematics:

Copy code
ẋ = J(θ) · θ̇
where:

ẋ: velocity of the end-effector

J(θ): Jacobian matrix

θ̇: joint velocities

To compute the required joint velocities:

Copy code
θ̇ = J⁺(θ) · ẋ
where J⁺ is the pseudo-inverse of the Jacobian.

Instead of calculating ẋ directly, it is approximated using a proportional controller:

scss
Copy code
ẋ ≈ Kp · (x_desired - x_current)


🧪 Dependencies
Python 3.x

ROS 2 Humble

numpy

RViz

URDF

