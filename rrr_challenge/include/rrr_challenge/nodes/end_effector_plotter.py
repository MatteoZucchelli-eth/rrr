#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import time

class EndEffectorPlotter(Node):

    def __init__(self):
        super().__init__('end_effector_plotter')
        self.subscription = self.create_subscription(
            JointState,
            '/end_effector_pose',
            self.listener_callback,
            10)
        self.subscription 
        self.x_values = []
        self.time_values = []
        self.start_time = time.time()
        self.get_logger().info('End Effector Plotter has been started.')

    def listener_callback(self, msg: JointState):
        if len(msg.position) >= 2:
            # Assuming x is the second element as per end_effector_listener.py
            # ee_state_msg.position = [trans.transform.translation.z, trans.transform.translation.x]
            current_x = msg.position[1]
            current_time = time.time() - self.start_time # Time relative to node start
            
            self.x_values.append(current_x)
            self.time_values.append(current_time)
            # self.get_logger().info(f'Received x: {current_x} at time: {current_time:.2f}s')
        else:
            self.get_logger().warn('Received JointState message with insufficient position data.')

    def plot_and_save(self):
        if not self.x_values or not self.time_values:
            self.get_logger().info('No data collected to plot.')
            return

        plt.figure(figsize=(10, 6))
        plt.plot(self.time_values, self.x_values, label='End Effector X Position')
        plt.xlabel('Time (s)')
        plt.ylabel('X Position (m)')
        plt.title('End Effector X Position vs. Time')
        plt.legend()
        plt.grid(True)
        
        plot_filename = 'end_effector_x_position.png'
        try:
            plt.savefig(plot_filename)
            self.get_logger().info(f'Plot saved to {plot_filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to save plot: {e}')
        plt.show() # Optionally display the plot

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, generating plot...')
    finally:
        node.plot_and_save()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()