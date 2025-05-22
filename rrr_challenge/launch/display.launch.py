import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('rrr_challenge'),
        'urdf',
        'rrr.urdf')
    
    # Read the URDF file content
    with open(urdf_file_path, 'r') as file:
        robot_description = file.read()

    # Start the robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Add a joint_state_publisher_gui
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # Optional: Launch RViz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('rrr_challenge'), 'config', 'default.rviz')]
    )

    # Add controller node
    controller_node = Node(
        package='rrr_challenge',
        executable='controller.py', # This should match the script name
        name='controller_node',
        output='screen',
        parameters=[
            {'link_length_l': 1.0}  # Set the L parameter for the controller
        ]
    )

    # Add end_effector_listener node
    end_effector_listener_node = Node(
        package='rrr_challenge',
        executable='end_effector_listener.py', # This should match the script name
        name='end_effector_listener_node',
        output='screen'
    )

    # Add master node
    master_node = Node(
        package='rrr_challenge',
        executable='master_node.py', # Ensure this matches your script name
        name='master_node',
        output='screen',
        parameters=[
            {'link_length_l': 1.0},  # Set L for the master node
            {'frequency_f': 50.0}     # Set f for the master node
        ]
    )

    # Add the visualization node
    visualization_node = Node(
        package='rrr_challenge',
        executable='visualization_node.py', # Ensure this matches your script name
        name='visualization_node',
        output='screen'
    )

    # Add the end_effector_plotter node
    end_effector_plotter_node = Node(
        package='rrr_challenge',
        executable='end_effector_plotter.py', # Ensure this matches your script name
        name='end_effector_plotter_node',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        #joint_state_publisher,
        rviz2,
        controller_node,
        end_effector_listener_node,
        master_node,
        visualization_node,
        #end_effector_plotter_node
    ])