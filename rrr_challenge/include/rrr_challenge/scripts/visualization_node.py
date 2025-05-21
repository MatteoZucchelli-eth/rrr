#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np


class VisualizationNode(Node):

    def __init__(self):
        super().__init__('VisualizationNode')
        self.dt_visualization = 0.0005 # f = 2000 Hz 
        self.theta_current = np.zeros(3)
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
