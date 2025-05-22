# 🦾 3-DOF Planar Robotic Arm Control – Humanoid Challenge

This repository contains a ROS 2-based simulation of a simple 3-DOF planar robotic arm. The main goal is to control the end-effector to track a moving target in real-time.

---

## 🛠️ Setup Instructions

To get the code up and running:

1. Clone this repository into your ROS 2 workspace (e.g., `~/ros2_ws/src/`).
2. Activate the virtual environment
   
   ```bash
   
   source rrr/venv/bin/activate
   
   ```
   
4. Build the workspace:

   ```bash
   
   cd ..
   colcon build --packages-select rrr_challenge
   source install/setup.bash
   
   ```
   
5. Launch the system with:

   ```bash

   ros2 launch rrr_challenge display.launch.py

   ```

## ✏️ Task
You will design and implement an end-effector control system for a planar 3-DOF RRR robot to
follow a moving green target. The goal is to demonstrate your ability to implement a motion
control strategy.

### Technical Context
● Target Motion (Theoretical Reference) (not available at runtime):
  ○ X(t) = 2L
  ○ Y(t) = L · sin(2πft) with f = 5 Hz
● Target Position Data: Provided at 30 Hz to the robot controller
● Robot Control Loop: Runs at 1 kHz

### Task A
Task A: Basic 2D Cartesian Tracking
Implement controller for the RRR robot’s end effector to track the green target using only the
target position data (at 30 Hz).

### My solution
https://github.com/user-attachments/assets/9a639036-5139-44ea-b5c3-5dc7dcade0f5

   ```bash

   git checkout main

   ```

### Task B
Change target position frequency to 5Hz and control loop frequency to 50 Hz.
Explain what do you observe?

### My solution
[https://github.com/user-attachments/assets/03719852-d7a6-4d08-bd16-dc84bb37d74d](https://github.com/user-attachments/assets/020e8caa-23c2-4b93-8883-e690f3beda24)

As the frequency of the sinusoid determining how Y is moving and the publishing rate are the same, the master node will publish always the same target position for the end effector. This problem is commonly known as aliasing and occurs when the sampling rate is too low with respect to the function we want to sample. 

To avoid aliasing, the sampling (or publishing) rate must be at least twice the highest frequency component of the signal, according to the Nyquist-Shannon sampling theorem. In this case, increasing the publishing rate above 10 Hz would allow the robot controller to receive a more accurate and varying representation of the sinusoidal motion.

   ```bash

   git checkout slower_new

   ```

### (optional) Obstacle Avoidance + Joint limit constraints
Extend your solution to avoid a fixed circular obstacle:
● Position: X = L, Y = 0.5L
● Diameter: L / 4
Your controller should prevent the end effector from entering the obstacle's area.

Additionally, add and enforce the following constraints in your control algorithm:
● Joint angle position limits

### My solution
https://github.com/user-attachments/assets/9f9592c5-a9cd-445b-8a24-544332be079a
   ```bash

   git checkout new_constrained_case

   ```

## 🤖 Robot Modeling
To model the robot:

1. The simplified robotic arm was designed using Onshape.
2. The STL files were exported and used to construct a URDF model.
3. The robot model is visualized using RViz for simulation and debugging.

## 🧠 Node Architecture
The system consists of four ROS 2 nodes, all written in Python:

### 1. **Master Node**
- **Publishes**: `/desired_pose`
- **Description**: Periodically generates and publishes a target position that the robot's end-effector must follow.

---

### 2. **End-Effector Node**
- **Publishes**: `/end_effector_pose`
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
  - `/desired_pose`
  - `/end_effector_pose`
  - `/joint_states`
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

where $J^+$ is the **damped pseudo-inverse** of the Jacobian.

   ```python

   J_inv = J.T @ np.linalg.inv(J @ J.T + self.lambda_damping**2 * I)

   ```


Instead of calculating the desired end-effector velocity $\dot{x}$ analytically, it is approximated using a **Proportional-Derivative (PD) controller**:

$$
\dot{x} \approx K_p \cdot (x_d - x_c) + K_d \cdot (-\dot{x}_c)
$$

where:
- $x_d$: desired end-effector position  
- $x_c$: current end-effector position  
- $\dot{x}_c$: current end-effector velocity  
- $K_p$: proportional gain  
- $K_d$: derivative gain

### Constrained Solution:

To see the constrained solution, change branch to `constrained_solution`:

   ```
   git checkout constrained_solution
   ```

To successfully solve the constrained problem, I used a Sequential Quadratic Programming (SQP) solver that respects joint limits and avoids entering the red circular area while tracking.

The video shows that the system is able to follow an oscillating trajectory that highlights how the robot reacts, running on a controller operating at f = 50 Hz.
