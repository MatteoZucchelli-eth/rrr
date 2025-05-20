#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

L = 1.0  # Length of the arm

def evaluate_jacobian(theta_input):
    J11 = -L * np.sin(theta_input[0]) - L * np.sin(theta_input[0] + theta_input[1]) - L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
    J12 = -L * np.sin(theta_input[0] + theta_input[1]) - L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
    J13 = -L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
    J21 = L * np.cos(theta_input[0]) + L * np.cos(theta_input[0] + theta_input[1]) + L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
    J22 = L * np.cos(theta_input[0] + theta_input[1]) + L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
    J23 = L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
    jacobian = np.array([[J11, J12, J13], [J21, J22, J23]])
    return jacobian

class MyNode(Node):
    def __init__(self):
        super().__init__('Controller')
        self.get_logger().info('Controller has been started.')
        self.num_joints = 3
        self.pos_dim = 2
        self.timer_period = 0.001
        self.current_theta = np.zeros(self.num_joints) # .T is not necessary for 1D array
        self.current_end_effector_position = np.zeros(self.pos_dim) # .T is not necessary for 1D array
        self.joint_names_ordered = ['joint1', 'joint2', 'joint3'] 
        if len(self.joint_names_ordered) != self.num_joints:
            self.get_logger().error(
                f"Mismatch between number of joints ({self.num_joints}) and defined joint names ({len(self.joint_names_ordered)}). "
                "Please update 'self.joint_names_ordered' in controller.py with the correct joint names from your URDF."
            )
        self.x_dot_desired = np.zeros(self.pos_dim) # .T is not necessary for 1D array
        self.kp = 0.001 # Proportional gain, adjust as needed
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.desired_pose_subscription = self.create_subscription(
            JointState,
            '/desired_pose',
            self.desired_pose_callback,
            10)
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_listener_callback,
            10)
        
        self.end_effector_state_subscription = self.create_subscription(
            JointState,
            '/end_effector_states',
            self.end_effector_states_callback,
            10)
        

        
        self.timer = self.create_timer(self.timer_period, self.controller_loop_callback)
        self.get_logger().info('Controller node initialized with subscriber and timer.')

    def desired_pose_callback(self, msg: JointState):
        self.x_dot_desired = self.kp * (np.array(msg.position).T - self.current_end_effector_position)

    def joint_states_listener_callback(self, msg: JointState):
        """
        Updates the current joint angles from robot feedback.
        """
        # Check if the message contains position data
        if msg.position and len(msg.position) == self.num_joints:
            # Update the current joint angles
            self.current_theta = np.array(msg.position)
            self.get_logger().debug(f"Updated current_theta: {self.current_theta}")
        else:
            self.get_logger().warn(f"Received joint state with unexpected length: {len(msg.position) if msg.position else 0}")

    def end_effector_states_callback(self, msg: JointState):
        """
        Updates the current end effector position from the end_effector_listener node.
        """
        # Check if the message contains position data
        if msg.position and len(msg.position) == self.pos_dim:
            # Update the current end effector position
            self.current_end_effector_position = np.array(msg.position)
            self.get_logger().debug(f"Updated end effector position: {self.current_end_effector_position}")
        else:
            self.get_logger().warn(f"Received end effector state with unexpected length: {len(msg.position) if msg.position else 0}")

    def controller_loop_callback(self):
        theta_current = self.current_theta 
        
        # Ensure current_theta is not empty before Jacobian evaluation
        if theta_current.size == 0:
            self.get_logger().warn("Current theta is empty, skipping control loop iteration.")
            return

        J = evaluate_jacobian(theta_current)

        # Ensure Jacobian is not empty or ill-defined
        if J.size == 0 or J.shape[0] == 0 or J.shape[1] == 0:
            self.get_logger().error("Jacobian is empty or ill-defined, skipping control loop iteration.")
            return
        
        try:
            # Ensure J is compatible with x_dot_desired for matrix multiplication
            if J.shape[0] != len(self.x_dot_desired):
                self.get_logger().error(f"Jacobian rows ({J.shape[0]}) do not match x_dot_desired dimension ({len(self.x_dot_desired)})")
                return

            J_inv = np.linalg.pinv(J)
            
            # Ensure J_inv can be multiplied by x_dot_desired
            if J_inv.shape[1] != len(self.x_dot_desired): 
                 self.get_logger().error(f"Pinv(J) columns ({J_inv.shape[1]}) do not match x_dot_desired dimension ({len(self.x_dot_desired)}) for multiplication.")
                 return

            theta_dot = J_inv @ self.x_dot_desired
        except np.linalg.LinAlgError as e:
            self.get_logger().error(f"Failed to compute pseudo-inverse or matrix multiplication: {e}")
            return
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during kinematic calculations: {e}")
            return
        
        new_theta_command = theta_current + theta_dot * self.timer_period
    
        output_msg = JointState()
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.name = self.joint_names_ordered
        output_msg.position = new_theta_command.tolist()
        
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()