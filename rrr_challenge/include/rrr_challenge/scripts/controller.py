#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from scipy.optimize import minimize, LinearConstraint
from scipy import sparse
import numpy as np
class MyNode(Node):
    def __init__(self):
        super().__init__('Controller')

        self.declare_parameter('link_length_l', 1.0) 
        self.L = self.get_parameter('link_length_l').get_parameter_value().double_value
        self.get_logger().info(f"Controller using L: {self.L}")

        # Constraints
        self.center = np.array([self.L, 0.5*self.L])
        self.ray = self.L / 4.0
        self.activate_joint_limit = True
        

        self.get_logger().info('Controller has been started.')
        self.num_joints = 3
        self.pos_dim = 2
        self.timer_period = 0.05
        self.current_theta = np.zeros(self.num_joints) 
        self.current_end_effector_position = np.zeros(self.pos_dim) 
        self.joint_names_ordered = ['joint1', 'joint2', 'joint3'] 
        if len(self.joint_names_ordered) != self.num_joints:
            self.get_logger().error(
                f"Mismatch between number of joints ({self.num_joints}) and defined joint names ({len(self.joint_names_ordered)}). "
                "Please update 'self.joint_names_ordered' in controller.py with the correct joint names from your URDF."
            )
        self.x_dot_desired = np.zeros(self.pos_dim) 
        self.kp = 5
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.desired_pose_subscription = self.create_subscription(
            JointState,
            '/desired_pose',
            self.desired_pose_callback,
            10)
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_listener_callback,
            10)
        
        self.end_effector_state_subscription = self.create_subscription(
            JointState,
            '/end_effector_states',
            self.end_effector_states_callback,
            10)
        
        self.timer = self.create_timer(self.timer_period, self.controller_loop_callback)
        self.get_logger().info('Controller node initialized with subscriber and timer.')

    def evaluate_jacobian(self, theta_input):
        J11 = -self.L * np.sin(theta_input[0]) - self.L * np.sin(theta_input[0] + theta_input[1]) - self.L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
        J12 = -self.L * np.sin(theta_input[0] + theta_input[1]) - self.L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
        J13 = -self.L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
        J21 = self.L * np.cos(theta_input[0]) + self.L * np.cos(theta_input[0] + theta_input[1]) + self.L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
        J22 = self.L * np.cos(theta_input[0] + theta_input[1]) + self.L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
        J23 = self.L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
        jacobian = np.array([[J11, J12, J13], [J21, J22, J23]])
        return jacobian
    
    def forward_kinematics(self, theta_input):
        x = self.L * np.cos(theta_input[0]) + self.L * np.cos(theta_input[0] + theta_input[1]) + self.L * np.cos(theta_input[0] + theta_input[1] + theta_input[2])
        y = self.L * np.sin(theta_input[0]) + self.L * np.sin(theta_input[0] + theta_input[1]) + self.L * np.sin(theta_input[0] + theta_input[1] + theta_input[2])
        return np.array([x, y])

    def desired_pose_callback(self, msg: JointState):
        self.x_dot_desired = self.kp * (np.array(msg.position) - self.current_end_effector_position)

    def joint_states_listener_callback(self, msg: JointState):
        if msg.position and len(msg.position) == self.num_joints:
            self.current_theta = np.array(msg.position)
            self.get_logger().debug(f"Updated current_theta: {self.current_theta}")
        else:
            self.get_logger().warn(f"Received joint state with unexpected length: {len(msg.position) if msg.position else 0}")

    def end_effector_states_callback(self, msg: JointState):
        if msg.position and len(msg.position) == self.pos_dim:
            self.current_end_effector_position = np.array(msg.position)
            self.get_logger().debug(f"Updated end effector position: {self.current_end_effector_position}")
        else:
            self.get_logger().warn(f"Received end effector state with unexpected length: {len(msg.position) if msg.position else 0}")
            
    def controller_loop_callback(self):
        theta_current = self.current_theta 
        
        if theta_current.size == 0:
            self.get_logger().warn("Current theta is empty, skipping control loop iteration.")
            return

        J = self.evaluate_jacobian(theta_current)
        p = self.forward_kinematics(theta_current)


        theta_dot = np.zeros(self.num_joints)

        try:
            if J.shape[0] != len(self.x_dot_desired):
                self.get_logger().error(f"Jacobian rows ({J.shape[0]}) do not match x_dot_desired dimension ({len(self.x_dot_desired)})")
                return

            J_pinv = np.linalg.pinv(J)
            theta_dot_unconstrained = J_pinv @ self.x_dot_desired
            
            p_next = p + J @ theta_dot_unconstrained * self.timer_period
            distance_to_center = np.linalg.norm(p_next - self.center)
            
            if distance_to_center >= self.ray:
                # No constraint violation, use the simple solution
                theta_dot = theta_dot_unconstrained
            else:
                # Constraint would be violated, try the optimization approach
                self.get_logger().info("Using constrained optimization")
                
                def objective(theta_dot_var):
                    return np.linalg.norm(J @ theta_dot_var - self.x_dot_desired)**2
                
                def constraint_func(theta_dot_var):
                    p_next = p + J @ theta_dot_var * self.timer_period
                    return np.linalg.norm(p_next - self.center) - self.ray
                
              
                constraint = {'type': 'ineq', 'fun': constraint_func}
                
                result = minimize(
                    objective, 
                    theta_dot_unconstrained,  # Start from unconstrained solution
                    method='SLSQP', 
                    constraints=[constraint],
                    options={'disp': False, 'ftol': 1e-6}
                )
                
                if result.success:
                    theta_dot = result.x
                else:
                    self.get_logger().error(f"Optimization failed: {result.message}")
                    # Fall back to unconstrained solution
                    theta_dot = theta_dot_unconstrained
                    
        except ValueError as e:
            self.get_logger().error(f"ValueError during optimization: {e}")
            try:
                J_pinv = np.linalg.pinv(J)
                theta_dot = J_pinv @ self.x_dot_desired
                self.get_logger().warn("Falling back to unconstrained pseudoinverse solution")
            except Exception as e2:
                self.get_logger().error(f"Fallback also failed: {e2}")
                return
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
            return

        new_theta_command_raw = theta_current + theta_dot * self.timer_period

        new_theta_command_normalized = np.arctan2(np.sin(new_theta_command_raw), np.cos(new_theta_command_raw))
    
        output_msg = JointState()
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.name = self.joint_names_ordered
        output_msg.position = new_theta_command_normalized.tolist()
        
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    controller_node = MyNode()
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()