#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np


class VisualizationNode(Node):
    """
    A ROS2 node that simulates the visualization of a robotic arm's joint states.
    It publishes the joint states at a high frequency and subscribes to joint velocity commands.
    """
    def __init__(self):
        super().__init__('VisualizationNode')
        self.dt_visualization = 0.0005 # f = 2000 Hz 
        first_theta = np.random.uniform(-np.pi, np.pi)           
        second_theta = np.random.uniform(-np.pi/4, np.pi/4)      
        third_theta = np.random.uniform(-np.pi/3, np.pi/3)  
        self.theta_min = np.array([0.0, np.pi/8, np.pi/8]) 
        self.theta_current = np.array([0.0, 0.0, 0.0])
        self.theta_dot = np.zeros(3)
        self.joint_names_ordered = ['joint1', 'joint2', 'joint3']

        self.timer = self.create_timer(self.dt_visualization, self.timer_callback)
        self.js_publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.velocity_subscription = self.create_subscription(
            JointState,
            '/joint_states_vel',
            self.velocity_callback,
            10)

        self.get_logger().info('Visualization Node has been started.')

    def timer_callback(self):
        new_theta_command_raw = self.theta_current + self.theta_dot * self.dt_visualization
        self.theta_current = np.arctan2(np.sin(new_theta_command_raw), np.cos(new_theta_command_raw))
        
        if abs(self.theta_current[1]) < self.theta_min[1]:
            self.theta_current[1] = self.theta_min[1]
        if abs(self.theta_current[2]) < self.theta_min[2]:
            self.theta_current[2] = self.theta_min[2]

        output_msg = JointState()
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.name = self.joint_names_ordered
        output_msg.position = self.theta_current.tolist()
        self.js_publisher_.publish(output_msg)

    def velocity_callback(self, msg : JointState):
        self.theta_dot = np.array(msg.velocity)
          
            

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
