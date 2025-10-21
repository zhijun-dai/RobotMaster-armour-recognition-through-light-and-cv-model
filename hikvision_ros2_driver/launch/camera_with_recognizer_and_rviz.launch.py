# Launch both Hikvision driver, Python armor recognizer, and RViz2
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Name of the Hikvision camera'
    )
    camera_name = LaunchConfiguration('camera_name')

    # Hikvision driver node
    camera_node = Node(
        package='hikvision_ros2_driver',
        executable='hikvision_ros2_driver_node',
        name='hikvision_ros2_driver_node',
        namespace=['/driver/hikvision/', camera_name],
        parameters=[{'camera_name': camera_name}],
        output='screen',
    )

    # Python recognizer node (runs in same namespace or global?)
    # Topics we use are absolute (e.g. /armor_crop), keep it global for simplicity.
    recognizer_node = Node(
        package='hikvision_ros2_driver',
        executable='armor_recognizer_node.py',
        name='armor_recognizer_node',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        camera_name_arg,
        camera_node,
        recognizer_node,
        rviz_node,
    ])
