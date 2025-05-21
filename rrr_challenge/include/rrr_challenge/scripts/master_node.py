#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
import numpy as np
from rclpy.time import Time # Import Time
from nav_msgs.msg import Path

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')

        # Declare parameters with default values
        self.declare_parameter('link_length_l', 1.0)
        self.declare_parameter('frequency_f', 0.5)

        # Get the parameter values
        self.L = self.get_parameter('link_length_l').get_parameter_value().double_value
        self.f = self.get_parameter('frequency_f').get_parameter_value().double_value
        
        self.publisher_ = self.create_publisher(JointState, '/desired_pose', 10)
        self.task_space_names = ['x_target', 'y_target'] 
        self.timer_period = 0.02  # f = 50 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.path_publisher_ = self.create_publisher(Path, '/end_effector_path', 10)
        self.path_msg = Path()
        self.path_max_poses = 10

    def timer_callback(self):
        self.publish_target()
        # Request the latest available transform          

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
        

        current_pose = PoseStamped()
        current_pose.header.stamp = msg.header.stamp
        current_pose.header.frame_id = "base_joint_link"
        current_pose.pose.position.x = target_position[1]
        current_pose.pose.position.y = 0.0
        current_pose.pose.position.z = target_position[0]
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0
        current_pose.pose.orientation = q

        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.header.frame_id = "base_joint_link"
        self.path_msg.poses.append(current_pose)

        if len(self.path_msg.poses) > self.path_max_poses:
            self.path_msg.poses.pop(0)
        
        self.publisher_.publish(msg)
        self.path_publisher_.publish(self.path_msg)

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