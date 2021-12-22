#!/usr/bin/env python3

import os

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
1
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

        Node(
            package="openvslam_ros",
            executable="run_localization_ros2io",
            name='run_slam',
            parameters=[
                {'cfg_file_path'    :   './config/realsense_RGBD.yaml'},
                {'vocab_file_path'  :   './orb_vocab.fbow'},
                {'map_db_path'      :   './map-db/6th_floor_office.msg'},
                {'odom_frame'       :   'cam_odom'},
                {'map_frame'        :   'cam_map'},
            ],
            output='screen',
            remappings=[
                ('camera/depth/image_raw', '/camera/aligned_depth_to_color/image_raw'),
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments = ['0','0','0','0','0','0', 'cam_odom', 'rs_camera_link'],
        ),
    ])
