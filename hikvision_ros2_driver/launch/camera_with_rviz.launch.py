# hikvision_ros2_driver/launch/camera_with_rviz.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 声明参数
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera', # 设置默认相机名称
        description='Name of the Hikvision camera'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to PyTorch model .pth; leave empty to use default inside script'
    )

    # 获取参数
    camera_name = LaunchConfiguration('camera_name')
    model_path = LaunchConfiguration('model_path')

    # 启动相机节点 (standalone 模式)
    camera_node = Node(
        package='hikvision_ros2_driver',
        executable='hikvision_ros2_driver_node', # 注意：可执行文件名可能根据 CMakeLists.txt 中的设置而变化
        name='hikvision_ros2_driver_node',
        namespace=[ '/driver/hikvision/', camera_name ], # 保持与 YAML 文件相同的命名空间
        parameters=[
            {'camera_name': camera_name},
            # 可以在这里添加其他参数，如曝光、增益等
            # {'exposure_time': 5000.0},
            # {'gain': 0.0},
            # {'frame_rate': 30.0},
            # {'pixel_format': 'PixelFormat_BayerRG8'},
        ],
        output='screen', # 输出到终端
    )

    # 启动 Python 识别节点（仅推理）
    recognizer_params = []
    # 仅当用户提供了 model_path 时再传递该参数（空字符串将被脚本默认值替代）
    recognizer_params.append({'model_path': model_path})

    recognizer_node = Node(
        package='hikvision_ros2_driver',
        executable='armor_recognizer_node.py',
        name='armor_recognizer_node',
        output='screen',
        parameters=recognizer_params,
    )

    # 启动 rviz2 节点（预置一个包含三路 Image 显示的配置）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('hikvision_ros2_driver'), 'config', 'three_streams.rviz'])]
    )

    return LaunchDescription([
        camera_name_arg,
        model_path_arg,
        camera_node,
        recognizer_node,
        rviz_node,
    ])
