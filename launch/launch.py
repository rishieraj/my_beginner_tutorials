from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# import os

# def generate_bag_recording(context, *args, **kwargs):
#     enable_recording = LaunchConfiguration("enable_recording").perform(context)
#     if enable_recording.lower() == "true":
#         bag_directory = os.path.expanduser("~/ros2_bags")
#         return [
#             Node(
#                 package="rosbag2",
#                 executable="record",
#                 name="rosbag_record",
#                 arguments=["-a", "-o", bag_directory],
#                 output="screen",
#             )
#         ]
#     return []

def generate_launch_description():
    # Declare a launch argument to allow setting the publish frequency
    freq_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='2.0',  # Default frequency if none is provided
        description='Frequency for the publisher node in Hz'
    )

    # Launch argument for enabling/disabling bag recording
    record_bag_arg = DeclareLaunchArgument(
        'enable_recording',
        default_value='false',  # Default to not recording if not specified
        description='Enable or disable ros bag recording'
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
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # Configure the Listener (Subscriber) node
    listener_node = Node(
        package='beginner_tutorials',
        executable='listener',
        name='minimal_subscriber',
        output='screen'
    )

    # Create handle for conditional recording of ros bag
    def conditional_rosbag_record(context):
        if LaunchConfiguration('enable_recording').perform(context) == 'true':
            return [
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '--all'],
                    output='screen',
                    cwd='src/beginner_tutorials/results'  # Set the working directory
                )
            ]
        return []

    # Return the LaunchDescription with the nodes and argument
    return LaunchDescription([
        freq_arg,
        record_bag_arg,
        talker_node,
        listener_node,
        OpaqueFunction(function=conditional_rosbag_record)
    ])
