#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time # Import Time
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState


class EndEffectorListener(Node):
    """
    A ROS2 node that listens to the end effector pose and publishes it as a JointState message.
    This node uses the tf2 library to transform the end effector pose from the 'base_joint_link' frame
    to the 'end_effector_tip' frame.
    It publishes the end effector pose at a high frequency (2000 Hz) for visualization purposes.
    """

    def __init__(self):
        super().__init__('end_effector_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.0005, self.timer_callback) # f = 2000 Hz for the visualization
        self.ee_publisher_ = self.create_publisher(JointState, '/end_effector_pose', 10)

        self.get_logger().info('End Effector Listener has been started.')

    def timer_callback(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'base_joint_link',  
                'end_effector_tip', 
                Time(),             
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
           
            ee_state_msg = JointState()
       
            ee_state_msg.header.stamp = trans.header.stamp 
            ee_state_msg.header.frame_id = trans.header.frame_id 
           
            ee_state_msg.position = [trans.transform.translation.z, trans.transform.translation.x]
            self.ee_publisher_.publish(ee_state_msg)

            
        except Exception as e:
            self.get_logger().warn(f"Could not transform or publish: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
