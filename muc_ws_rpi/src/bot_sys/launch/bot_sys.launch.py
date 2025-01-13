from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control',       # Replace with your package name
            executable='motor_control_node',     # Replace with your node executable name
            name='motor_control',               # Optional name for the node
            output='log'             # Print output to the terminal
        ),
        Node(
            package='encoder_reader',
            executable='encoder_reader_node',
            name='encoder_reader',
            output = 'log'
        ),
	Node(
	   package = 'publish_imu',
	   executable = 'imu_publisher',
	   name = 'imu_publisher',
	  output = 'log'
	)
    ])
