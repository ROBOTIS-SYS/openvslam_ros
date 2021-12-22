from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, declare_launch_argument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition

config_parameters = [
    ## openvslam parameter

    ]

def generate_launch_description() :

    return LaunchDescription([

        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name='camera',
            namespace='camera',
            parameters=[
                {'align_depth'       :       True},
                {'enable_infra1'     :       False},
                {'enable_infra2'     :       False},
                {'enable_pointcloud' :       False},
                {'color_qos'         :       'DEFAULT'},
                {'depth_qos'         :       'DEFAULT'},
                {'base_frame_id'     :       'rs_camera_link'},
                {'color_height'      :       720},
                {'color_width'       :       1280},
                {'color_fps'         :       30.0},
            ],
            output='screen',
        ),
    ])
