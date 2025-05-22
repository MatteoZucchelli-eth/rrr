#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
import numpy as np
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')

        self.declare_parameter('link_length_l', 1.0)
        self.declare_parameter('frequency_f', 0.5)

        self.L = self.get_parameter('link_length_l').get_parameter_value().double_value
        self.f = self.get_parameter('frequency_f').get_parameter_value().double_value
        
        self.publisher_ = self.create_publisher(JointState, '/desired_pose', 10)
        self.task_space_names = ['x_target', 'y_target'] 
        self.timer_period = 1 / self.f  # f = 50 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.frequency_sin = 5.0 # Frequency of the sine wave

        # self.path_publisher_ = self.create_publisher(Path, '/end_effector_path', 10)
        # self.path_msg = Path()
        # self.path_max_poses = 10

        self.marker_publisher_ = self.create_publisher(Marker, '/end_effector_marker', 10)
        

        self.get_logger().info(f"Master node has been started. Using L: {self.L}, frequency: {self.f} Hz")


    def timer_callback(self):
        self.publish_target()      

    def publish_target(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        t = self.get_clock().now()

        x = 2 * self.L 
        y = self.L * np.sin(t.nanoseconds / 1e9 * 2 * np.pi * self.frequency_sin)
        target_position = np.array([x, y]) 

        msg.name = self.task_space_names 
        msg.position = target_position.tolist()
        
        self.publisher_.publish(msg)
        
        # current_pose = PoseStamped()
        # current_pose.header.stamp = msg.header.stamp
        # current_pose.header.frame_id = "base_joint_link"
        # current_pose.pose.position.x = target_position[1]
        # current_pose.pose.position.y = 0.1
        # current_pose.pose.position.z = target_position[0]
        # q = Quaternion()
        # q.x = 0.0
        # q.y = 0.0
        # q.z = 0.0
        # q.w = 1.0
        # current_pose.pose.orientation = q

        # self.path_msg.header.stamp = msg.header.stamp
        # self.path_msg.header.frame_id = "base_joint_link"
        # self.path_msg.poses.append(current_pose)

        # if len(self.path_msg.poses) > self.path_max_poses:
        #     self.path_msg.poses.pop(0)
        

        # self.path_publisher_.publish(self.path_msg)

        # Create marker for visualizing the end effector position
        marker = Marker()
        marker.header.frame_id = "base_joint_link"
        marker.header.stamp = msg.header.stamp
        marker.ns = "end_effector"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set the position 
        marker.pose.position.x = target_position[1]  # y in robot frame
        marker.pose.position.y = 0.1
        marker.pose.position.z = target_position[0]  # x in robot frame
        
        # Set the orientation (identity quaternion)
        marker.pose.orientation.w = 1.0
        
        # Set the scale - this controls the size of the sphere
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Set the color - green
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Publish the messages
        self.marker_publisher_.publish(marker)


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