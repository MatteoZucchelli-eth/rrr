#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time # Import Time
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

class EndEffectorListener(Node):

    def __init__(self):
        super().__init__('end_effector_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.publisher_ = self.create_publisher(JointState, '/end_effector_states', 10)
        self.get_logger().info('End Effector Listener has been started.')

    def timer_callback(self):
        try:
            # Request the latest available transform
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'base_joint_link',  # Target frame (frame to transform INTO)
                'end_effector_tip', # Source frame (frame to transform FROM)
                Time(),             # Get the latest available transform
                timeout=rclpy.duration.Duration(seconds=0.5) # Reduced timeout slightly
            )
           
            ee_state_msg = JointState()
       
            ee_state_msg.header.stamp = trans.header.stamp 
            ee_state_msg.header.frame_id = trans.header.frame_id 
           
            ee_state_msg.position = [trans.transform.translation.z, trans.transform.translation.x]
            self.publisher_.publish(ee_state_msg)

            # self.get_logger().info(
            #     f"Published end-effector position: x={ee_state_msg.position[0]}, "
            #     f"y={ee_state_msg.position[1]} relative to {trans.header.frame_id} at time {trans.header.stamp.sec}.{trans.header.stamp.nanosec}"
            # )
            
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
