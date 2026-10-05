# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    limo_cartographer_prefix = get_package_share_directory('limo_bringup')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  limo_cartographer_prefix, 'config_files'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='limo_lds_2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.03')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    open_rviz = LaunchConfiguration('open_rviz', default='true')
    rviz_config = LaunchConfiguration('rviz_config')
    use_scan_filter = LaunchConfiguration('use_scan_filter', default='true')
    raw_scan_topic = LaunchConfiguration('raw_scan_topic', default='/scan')
    filtered_scan_topic = LaunchConfiguration('filtered_scan_topic', default='/scan_filtered')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan_filtered')

    rviz_config_dir = os.path.join(
        limo_cartographer_prefix,
        'config_files',
        'demo_2d.rviz'
    )
    laser_filters_yaml = os.path.join(
        limo_cartographer_prefix,
        'config_files',
        'laser_filters.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'open_rviz',
            default_value='true',
            description='Open RViz with the Cartographer map view'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_dir,
            description='Full path to the RViz config file'),
        DeclareLaunchArgument(
            'use_scan_filter',
            default_value='true',
            description='Start a moderate LaserScan filter from raw_scan_topic to filtered_scan_topic'),
        DeclareLaunchArgument(
            'raw_scan_topic',
            default_value='/scan',
            description='Raw LaserScan topic from the LiDAR driver'),
        DeclareLaunchArgument(
            'filtered_scan_topic',
            default_value='/scan_filtered',
            description='Filtered LaserScan topic published for SLAM'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan_filtered',
            description='LaserScan topic consumed by Cartographer'),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='cartographer_scan_filter_chain',
            output='screen',
            condition=IfCondition(use_scan_filter),
            parameters=[laser_filters_yaml],
            remappings=[
                ('scan', raw_scan_topic),
                ('scan_filtered', filtered_scan_topic),
            ]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('scan', scan_topic)
            ]
        ),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(open_rviz),
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

    ])
