from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
	Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='log',
        ),
        # Start the teleop_twist_joy node     
	Node(
            package='odom_publisher',  # Replace with your package name
            executable='odom_publisher_node',  # Name of the first node executable
            name='odom_publisher',        # Node name (optional)
            output='log'
        ),
        Node(
            package='publish_pwm_values',  # Replace with your package name
            executable='publish_pwm_values',  # Name of the second node executable
            name='publish_pwm_values',       # Node name (optional)
            output='log'
        ),
        Node(
            package='subscribe_encoder_values',  # Replace with your package name
            executable='subscribe_encoder_values',  # Name of the third node executable
            name='subscribe_encoder_values',       # Node name (optional)
            output='log'
        ),
	
    ])
