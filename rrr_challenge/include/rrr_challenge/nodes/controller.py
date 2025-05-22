#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import threading

class MyNode(Node):
    def __init__(self):
        super().__init__('Controller')

        self.declare_parameter('link_length_l', 1.0) 
        self.L = self.get_parameter('link_length_l').get_parameter_value().double_value
        self.get_logger().info(f"Controller has been started. Using L: {self.L}")

        self.max_vel = 100.0
        self.num_joints = 3
        self.pos_dim = 2
        self.f = 1000.0 # f = 1000 Hz
        self.timer_period = 1 / self.f
        self.lambda_damping = 0.01
        self.kp = 47.5
        self.kd = 0.95

        self.end_effector_initialized = False
        self.joint_states_initialized = False
        self.desired_pose_initialized = False
    
        self.joint_names_ordered = ['joint1', 'joint2', 'joint3'] 
        self.current_theta = np.zeros(self.num_joints) 
        self.theta_dot = np.zeros(self.num_joints)
        self.desired_pose = np.zeros(self.pos_dim) 
        self.current_end_effector_position = np.zeros(self.pos_dim) 


        self.theta_lock = threading.Lock()
        self.desired_pose_lock = threading.Lock()
        self.end_effector_lock = threading.Lock()
        self.theta_dot_lock = threading.Lock()

        self.publisher_ = self.create_publisher(JointState, '/joint_states_vel', 10)
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
            '/end_effector_pose',
            self.end_effector_pose_callback,
            10)
        
        self.timer = self.create_timer(self.timer_period, self.controller_loop_callback)
        self.get_logger().info('Controller node initialized. Frequency = {self.f} hz.')

    def get_theta(self):
        with self.theta_lock:
            return self.current_theta.copy()
    
    def get_desired_pose(self):
        with self.desired_pose_lock:
            return self.desired_pose.copy()
        
    def get_end_effector_pose(self):
        with self.end_effector_lock:
            return self.current_end_effector_position.copy()

    def get_theta_dot(self):
        with self.theta_dot_lock:
            return self.theta_dot.copy()

    def set_theta_dot(self, theta_dot):
        with self.theta_dot_lock:
            self.theta_dot = theta_dot.copy()

    def desired_pose_callback(self, msg: JointState):
        with self.desired_pose_lock:
            self.desired_pose = np.array(msg.position)
            if not self.desired_pose_initialized:
                self.get_logger().info(f"Desired pose initialized with position: {self.desired_pose}")
                self.desired_pose_initialized = True


    def joint_states_listener_callback(self, msg: JointState):
        if msg.position and len(msg.position) == self.num_joints:
            with self.theta_lock:
                self.current_theta = np.array(msg.position)
                if not self.joint_states_initialized:
                    self.get_logger().info(f"Joint states initialized with position: {self.current_theta}")
                    self.joint_states_initialized = True
        else:
            self.get_logger().warn(f"Received joint state with unexpected length: {len(msg.position) if msg.position else 0}")

    def end_effector_pose_callback(self, msg: JointState):
        if msg.position and len(msg.position) == self.pos_dim:
            with self.end_effector_lock:
                self.current_end_effector_position = np.array(msg.position)
                if not self.end_effector_initialized:
                    self.get_logger().info(f"End effector initialized with position: {self.current_end_effector_position}")
                    self.end_effector_initialized = True

        else:
            self.get_logger().warn(f"Received end effector state with unexpected length: {len(msg.position) if msg.position else 0}")
    
    def evaluate_jacobian_method(self, theta_input):
        J11 = -self.L * np.sin(theta_input[0]) - self.L * np.sin(theta_input[0] + theta_input[1]) - self.L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
        J12 = -self.L * np.sin(theta_input[0] + theta_input[1]) - self.L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
        J13 = -self.L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
        J21 = self.L * np.cos(theta_input[0]) + self.L * np.cos(theta_input[0] + theta_input[1]) + self.L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
        J22 = self.L * np.cos(theta_input[0] + theta_input[1]) + self.L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
        J23 = self.L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
        jacobian = np.array([[J11, J12, J13], [J21, J22, J23]])
        return jacobian


    def controller_loop_callback(self):

        if not (self.joint_states_initialized and self.end_effector_initialized and self.desired_pose_initialized):
            return

        theta_current = self.get_theta() 
        theta_dot_current = self.get_theta_dot()
        current_end_eff_pose = self.get_end_effector_pose()
        desired_pose = self.get_desired_pose()

        J = self.evaluate_jacobian_method(theta_current)

        current_end_effector_vel = J @ theta_dot_current

        x_dot_desired = self.kp * (desired_pose - current_end_eff_pose) - self.kd * current_end_effector_vel


        try:

            I = np.eye(J.shape[0]) 
            
            J_inv = J.T @ np.linalg.inv(J @ J.T + self.lambda_damping**2 * I)
            

            theta_dot = J_inv @ x_dot_desired
        except np.linalg.LinAlgError as e:
            self.get_logger().error(f"Failed to compute pseudo-inverse or matrix multiplication: {e}")
            return
        
        if np.any(np.abs(theta_dot) > self.max_vel):
            theta_dot = np.clip(theta_dot, -self.max_vel, self.max_vel)
            self.get_logger().warn(f"Clipping theta_dot to max velocity: {theta_dot}")

        self.set_theta_dot(theta_dot)
        output_msg = JointState()
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.name = self.joint_names_ordered
        output_msg.velocity = theta_dot.tolist()
        
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