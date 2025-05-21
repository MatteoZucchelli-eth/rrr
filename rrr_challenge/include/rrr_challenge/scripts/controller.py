#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class MyNode(Node):
    def __init__(self):
        super().__init__('Controller')

        self.declare_parameter('link_length_l', 1.0) 
        self.L = self.get_parameter('link_length_l').get_parameter_value().double_value
        self.get_logger().info(f"Controller using L: {self.L}")

        self.get_logger().info('Controller has been started.')
        self.max_vel = 5.0
        self.num_joints = 3
        self.pos_dim = 2
        self.timer_period = 0.001 # f = 1000 Hz
        self.current_theta = np.zeros(self.num_joints) 
        self.current_end_effector_position = np.zeros(self.pos_dim) 
        self.joint_names_ordered = ['joint1', 'joint2', 'joint3'] 
        if len(self.joint_names_ordered) != self.num_joints:
            self.get_logger().error(
                f"Mismatch between number of joints ({self.num_joints}) and defined joint names ({len(self.joint_names_ordered)}). "
                "Please update 'self.joint_names_ordered' in controller.py with the correct joint names from your URDF."
            )
        self.x_dot_desired = np.zeros(self.pos_dim) 
        self.kp = 5.0
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

    def evaluate_jacobian_method(self, theta_input):
        J11 = -self.L * np.sin(theta_input[0]) - self.L * np.sin(theta_input[0] + theta_input[1]) - self.L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
        J12 = -self.L * np.sin(theta_input[0] + theta_input[1]) - self.L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
        J13 = -self.L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
        J21 = self.L * np.cos(theta_input[0]) + self.L * np.cos(theta_input[0] + theta_input[1]) + self.L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
        J22 = self.L * np.cos(theta_input[0] + theta_input[1]) + self.L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
        J23 = self.L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
        jacobian = np.array([[J11, J12, J13], [J21, J22, J23]])
        return jacobian

    def desired_pose_callback(self, msg: JointState):
        self.x_dot_desired = self.kp * (np.array(msg.position) - self.current_end_effector_position)

    def joint_states_listener_callback(self, msg: JointState):
        if msg.position and len(msg.position) == self.num_joints:
            self.current_theta = np.array(msg.position)
            self.get_logger().debug(f"Updated current_theta: {self.current_theta}")
        else:
            self.get_logger().warn(f"Received joint state with unexpected length: {len(msg.position) if msg.position else 0}")

    def end_effector_states_callback(self, msg: JointState):
        if msg.position and len(msg.position) == self.pos_dim:
            self.current_end_effector_position = np.array(msg.position)
            self.get_logger().debug(f"Updated end effector position: {self.current_end_effector_position}")
        else:
            self.get_logger().warn(f"Received end effector state with unexpected length: {len(msg.position) if msg.position else 0}")
            
    def controller_loop_callback(self):
        theta_current = self.current_theta 
        
        if theta_current.size == 0:
            self.get_logger().warn("Current theta is empty, skipping control loop iteration.")
            return

        J = self.evaluate_jacobian_method(theta_current)

        if J.size == 0 or J.shape[0] == 0 or J.shape[1] == 0:
            self.get_logger().error("Jacobian is empty or ill-defined, skipping control loop iteration.")
            return
        
        try:
            if J.shape[0] != len(self.x_dot_desired):
                self.get_logger().error(f"Jacobian rows ({J.shape[0]}) do not match x_dot_desired dimension ({len(self.x_dot_desired)})")
                return

            J_inv = np.linalg.pinv(J)
            
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
        
        if np.any(np.abs(theta_dot) > self.max_vel):
            theta_dot = np.clip(theta_dot, -self.max_vel, self.max_vel)
            self.get_logger().warn(f"Clipping theta_dot to max velocity: {theta_dot}")
        new_theta_command_raw = theta_current + theta_dot * self.timer_period

        new_theta_command_normalized = np.arctan2(np.sin(new_theta_command_raw), np.cos(new_theta_command_raw))
    
        output_msg = JointState()
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.name = self.joint_names_ordered
        output_msg.position = new_theta_command_normalized.tolist()
        
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    controller_node = MyNode()
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()