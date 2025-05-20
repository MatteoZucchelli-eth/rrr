#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')

        # Declare parameters with default values
        self.declare_parameter('link_length_l', 1.0)
        self.declare_parameter('frequency_f', 0.5)

        # Get the parameter values
        self.L = self.get_parameter('link_length_l').get_parameter_value().double_value
        self.f = self.get_parameter('frequency_f').get_parameter_value().double_value
        
        self.get_logger().info(f"Master node using L: {self.L}, f: {self.f}")

        self.publisher_ = self.create_publisher(JointState, '/desired_pose', 10)
        self.task_space_names = ['x_target', 'y_target'] 
        self.timer_period = 0.2  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        self.publish_target()

    def publish_target(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        t = self.get_clock().now()

        # Use self.L and self.f which are read from parameters
        x = 2 * self.L 
        y = self.L * np.sin(t.nanoseconds / 1e9 * 2 * np.pi * self.f) # Corrected t.nanosec to t.nanoseconds
        target_position = np.array([x, y]) 

        msg.name = self.task_space_names 
        msg.position = target_position.tolist()
        
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Publishing desired end-effector pose: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    master_node = MasterNode()
    try:
        rclpy.spin(master_node)
    except KeyboardInterrupt:
        pass
    finally:
        master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()