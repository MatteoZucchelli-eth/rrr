import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String # Make sure String is imported for the publisher
import numpy as np

# Placeholder for the Jacobian evaluation function
def evaluate_jacobian(theta_input):
    pass

class MyNode(Node):
    def __init__(self):
        super().__init__('Controller')
        self.get_logger().info('Controller has been started.')

        self.num_joints = 3
        self.current_theta = np.zeros(self.num_joints)
        self.joint_names_ordered = []
        self.dt = 0.1
        self.x_dot_desired = np.zeros(self.num_joints)

        # Create publisher for calculated commands
        self.publisher_ = self.create_publisher(String, 'calculated_theta_commands', 10)

        # Create subscription to /joint_states
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_listener_callback,
            10)
        
        # Create a timer for the control loop
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.controller_loop_callback)

        self.get_logger().info('Controller node initialized with subscriber and timer.')

    def joint_states_listener_callback(self, msg):
        # This callback updates the current joint positions
        # It's important to map msg.name to a consistent order for self.current_theta
        # For simplicity, let's assume the incoming message has joints in the order we need
        # or that we are interested in all of them in the received order.
        # A more robust way is to map by name.
        
        # Example: If you need a specific order or subset of joints:
        # if not self.joint_names_ordered: # Initialize on first message or from parameter
        #    self.joint_names_ordered = ["joint1", "joint2"] # Define your order
        # temp_theta = np.zeros(self.num_joints)
        # for i, name in enumerate(self.joint_names_ordered):
        #    try:
        #        idx = msg.name.index(name)
        #        temp_theta[i] = msg.position[idx]
        #    except ValueError:
        #        self.get_logger().warn(f"Joint {name} not found in received JointState message.")
        # self.current_theta = temp_theta

        # Simpler approach: if the message order is usable directly and matches num_joints
        if len(msg.position) >= self.num_joints:
            self.current_theta = np.array(msg.position[:self.num_joints])
            # self.get_logger().info(f"Updated current_theta: {self.current_theta}", throttle_duration_sec=1.0)
        else:
            self.get_logger().warn(f"Received JointState with {len(msg.position)} joints, expected at least {self.num_joints}.")


    def controller_loop_callback(self):
        # This callback is executed periodically by the timer
        # It uses the latest self.current_theta (updated by the subscriber)
        
        theta_current = self.current_theta 
        
        # Ensure current_theta is not empty before Jacobian evaluation
        if theta_current.size == 0:
            self.get_logger().warn("Current theta is empty, skipping control loop iteration.")
            return

        J = evaluate_jacobian(theta_current)

        # Ensure Jacobian is not empty or ill-defined
        if J.size == 0 or J.shape[0] == 0 or J.shape[1] == 0:
            self.get_logger().error("Jacobian is empty or ill-defined, skipping control loop iteration.")
            return
        
        try:
            # Ensure J is compatible with x_dot_desired for matrix multiplication
            if J.shape[0] != len(self.x_dot_desired):
                self.get_logger().error(f"Jacobian rows ({J.shape[0]}) do not match x_dot_desired dimension ({len(self.x_dot_desired)})")
                return

            J_inv = np.linalg.pinv(J)
            
            # Ensure J_inv can be multiplied by x_dot_desired
            if J_inv.shape[1] != len(self.x_dot_desired): # This check might be redundant if the previous one passed for square/fat J
                 self.get_logger().error(f"Pinv(J) columns ({J_inv.shape[1]}) do not match x_dot_desired dimension ({len(self.x_dot_desired)})")
                 return

            theta_dot = J_inv @ self.x_dot_desired
        except np.linalg.LinAlgError as e:
            self.get_logger().error(f"Failed to compute pseudo-inverse or matrix multiplication: {e}")
            return
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during kinematic calculations: {e}")
            return
        
        new_theta_command = theta_current + theta_dot * self.dt
        # Here, new_theta_command represents the desired next joint positions.
        # Depending on your robot, you might publish joint velocities (theta_dot)
        # or target positions (new_theta_command).

        # --- Publishing the result ---
        output_msg = String()
        # For actual joint commands, you'd use sensor_msgs/JointState or trajectory_msgs/JointTrajectory
        # and populate it with `new_theta_command` or `theta_dot`.
        output_msg.data = f"Calculated Command (e.g., New Theta): {new_theta_command.tolist()}"
        self.publisher_.publish(output_msg)
        # self.get_logger().info(f'Publishing: "{output_msg.data}"', throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()