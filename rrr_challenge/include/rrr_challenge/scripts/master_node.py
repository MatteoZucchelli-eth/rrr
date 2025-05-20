#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        # Publishing to /desired_pose, which the controller expects for task-space targets
        self.publisher_ = self.create_publisher(JointState, '/desired_pose', 10)
        
        # For a 2D task-space target, the names could represent the axes
        self.task_space_names = ['x_target', 'y_target'] 
        
        # Timer to publish the message periodically (or just once if preferred)
        # and remove the timer. For now, it will keep publishing the same target.
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        #self.get_logger().info(f'Master node has been started and will publish desired end-effector pose: {self.target_position.tolist()}')



    def timer_callback(self):
        self.publish_target()

    def publish_target(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # get_time
        t = self.get_clock().now()

        x = 
        y = 

        self.target_position = np.array([x, y]) 

        # Populate message for a 2D task-space target
        msg.name = self.task_space_names 
        msg.position = self.target_position.tolist()
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing desired end-effector pose: {msg.position}')

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