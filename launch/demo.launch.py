import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
    get_package_share_directory('patchworkpp'),
    'config',
    'params_ros2.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('cloud_topic', default_value="/kitti/point_cloud"),
        
        Node(
            package='patchworkpp',
            executable='demo',
            name='ground_segmentation',
            output='screen',
            parameters=[config],
            arguments=[LaunchConfiguration('cloud_topic')],
        ),
    ])
 


