from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os # 用于路径处理

def generate_launch_description():
    # 声明可视化参数（保持不变）
    enable_viz_arg = DeclareLaunchArgument(
        'enable_gazebo_viz',
        default_value='true',
        description='Enable Gazebo marker visualization (simulation only)'
    )

    # 关键：直接写死源码目录中 params.yaml 的绝对路径
    # 注意：请核对你的实际路径是否与下面一致！
    # 路径格式：/home/用户名/工作空间/src/包所在目录/cfg/params.yaml
    params_path = "/home/liuyao/px4_ros2_examples_ws/src/tracktor-beam/src/precision_land/cfg/params.yaml"

    return LaunchDescription([
        enable_viz_arg,

        # 1. 桥接Gazebo的/camera到ROS2的/camera（核心修改）
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_image_bridge',  # 重命名节点便于区分
            arguments=[
                # 格式：Gazebo话题@ROS2消息类型@gz消息类型
                '/camera@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            output='screen',
        ),

        # 2. 桥接Gazebo的/camera_info到ROS2的/camera_info（核心修改）
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',  # 重命名节点便于区分
            arguments=[
                # 格式：Gazebo话题@ROS2消息类型@gz消息类型
                '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            output='screen',
        ),

        # 3. 桥接处理后的图像（如需要，保持与之前一致）
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='image_proc_bridge',
            arguments=[
                '/image_proc@sensor_msgs/msg/Image@gz.msgs.Image'  # 修正格式为@
            ],
            parameters=[{
                'qos_overrides./image_proc.subscription.reliability': 'best_effort',
                'qos_overrides./image_proc.publisher.reliability': 'best_effort'
            }],
            output='screen',
        ),

        # 4. Aruco跟踪节点（默认订阅ROS2的/camera和/camera_info，无需修改）
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params.yaml'])
            ]
        ),

        # 5. 精准降落节点（保持不变）
        Node(
            package='precision_land',
            executable='precision_land',
            name='precision_land',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('precision_land'), 'cfg', 'params.yaml'])
            ]
        ),

        # 6. 可视化节点（保持不变）
        Node(
            package='precision_land_viz',
            executable='tag_pose_visualizer',
            name='tag_pose_visualizer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_gazebo_viz'))
        ),
    ])
