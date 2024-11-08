from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument to allow setting the publish frequency
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='500',
        description='Frequency (in ms) at which the publisher publishes messages.'
    )

    # Configure the Talker (Publisher) node with the publish frequency parameter
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='publisher_service_node',
        parameters=[{
            'publish_frequency': LaunchConfiguration('publish_frequency')
        }],
        output='screen',
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )

    # Configure the Listener (Subscriber) node
    listener_node = Node(
        package='beginner_tutorials',
        executable='listener',
        name='minimal_subscriber',
        output='screen'
    )

    # Return the LaunchDescription with the nodes and argument
    return LaunchDescription([
        publish_frequency_arg,
        talker_node,
        listener_node
    ])
