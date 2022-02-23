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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']
            ),

            launch_arguments= {
                'align_depth'       :       'true',
                'enable_pointcloud' :       'false',
                'color_height'      :       '720',
                'color_width'       :       '1280',
                'color_fps'         :       '30.0',
                'clip_distance'     :       '3.0',
            }.items(),
        ),

        Node(
            package="openvslam_ros",
            executable="run_slam_ros2io_rgbd",
            name='run_slam',
            parameters=[
                {'cfg_file_path'    :   './config/realsense_RGBD.yaml'},
                {'vocab_file_path'  :   './orb_vocab.fbow'},
                {'map_db_path'      :   './output_map_db.msg'}],
            output='screen',
            remappings=[
                ('camera/depth/image_raw', '/camera/aligned_depth_to_color/image_raw'),
            ]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments = ['0','0','0','0','0','0', 'cam_odom', 'camera_link']
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments = ['-6.1366', '32.7447', '0.304093', '0.00719071', '0.0515719', '-0.787266', '0.614422', 'map_current', 'cam_map']
        )
    ])
