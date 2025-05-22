# 🦾 3-DOF Planar Robotic Arm Control – Humanoid Challenge

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
   
   ```
   
4. Launch the system with:

   ```bash

   ros2 launch rrr_challenge display.launch.py

   ```

## 🤖 Robot Modeling
To model the robot:

1. The simplified robotic arm was designed using Onshape.
2. The STL files were exported and used to construct a URDF model.
3. The robot model is visualized using RViz for simulation and debugging.

## 🧠 Node Architecture
The system consists of four ROS 2 nodes, all written in Python:

### 1. **Master Node**
- **Publishes**:
   - `/desired_pose`
   - `/end_effector_path`
- **Description**: Periodically generates and publishes a target position that the robot's end-effector must follow.

---

### 2. **End-Effector Node**
- **Publishes**: `/end_effector_states`
- **Description**: Computes the current Cartesian coordinates of the end-effector from the joint states visualized in RViz and publishes a clean, ready-to-use version.

---

### 3. **Simulation Node**
- **Subscribes**: `/joint_states_vel`
- **Publishes**: `/joint_states`
- **Description**: Simulates the robot by updating joint angles based on commanded velocities using:


    $$\theta_{new} = \theta_{old} +  \dot{\theta} \cdot Δt$$

    where $\theta$ represents joint angles, $\dot{\theta}$ represents joint velocities, and $Δt$ is the simulation time step.

The updated joint states are published and visualized in RViz.
### 4. **Controller Node**
- **Subscribes**:
  - `/desired_position`
  - `/current_position`
- **Publishes**:  `/joint_states_vel`
- **Description**: Controls the robot by computing the joint velocities required to make the end-effector follow the target trajectory.

### 🧩 How It Works:

The controller reads both the current and desired end-effector positions.

From robotics kinematics:

$$\dot{x} = J(\theta) \cdot \dot{\theta}$$

where:

- $\dot{x}$: velocity of the end-effector  
- $J(\theta)$: Jacobian matrix  
- $\dot{\theta}$: joint velocities  

To compute the required joint velocities:

$$\dot{\theta} = J^+(\theta) \cdot \dot{x}$$

where $J^+$ is the **pseudo-inverse** of the Jacobian.

Instead of calculating $\dot{x}$ directly, it is approximated using a proportional controller:

$$\dot{x} \approx K_p \cdot (x_d - x_c)$$

where:
- $x_d$: desired end-effector position  
- $x_c$: current end-effector position  
- $K_p$: proportional gain


pip install catkin_pkg
pip install numpy
🧪 Dependencies
Python 3.x

ROS 2 Humble

numpy

RViz

URDF

