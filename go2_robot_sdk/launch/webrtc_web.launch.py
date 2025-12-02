# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


class WebRTCLaunchConfig:
    """Configuration container for WebRTC launch parameters"""
    def __init__(self):
        self.robot_ip = os.getenv('ROBOT_IP', os.getenv('GO2_IP', ''))
        self.urdf_file_name = os.getenv('URDF_FILE_NAME', 'go2.urdf')
        self.enable_video = os.getenv('ENABLE_VIDEO', 'true')
        self.send_buffer_limit = os.getenv('SEND_BUFFER_LIMIT', '100000000')
        self.on_exit = os.getenv('ON_EXIT', 'shutdown')
        self.elevenlabs_api_key = os.getenv('ELEVENLABS_API_KEY', '')
        self.voice_name = os.getenv('VOICE_NAME', 'default')
        self.obstacle_avoidance = os.getenv('OBSTACLE_AVOIDANCE', 'false')
        self.enable_foxglove_bridge = os.getenv('ENABLE_FOXGLOVE_BRIDGE', 'true')
        self.pkg_dir = get_package_share_directory('go2_robot_sdk')
        self.robot_ip_list = os.getenv('ROBOT_IP_LIST', '').split(',')
        self.map_name = os.getenv('MAP_NAME', 'go2_map')
        self.save_map = os.getenv('SAVE_MAP', 'false').lower() == 'true'

        self.package_dir = get_package_share_directory('go2_robot_sdk')
        self.config_paths = self._get_config_paths()


    def _get_config_paths(self):
        """Get all configuration file paths"""
        return {
            'slam': os.path.join(self.package_dir, 'config', 'mapper_params_online_async.yaml'),
            'nav2': os.path.join(self.package_dir, 'config', 'nav2_wo_lidar.yaml'),
            'urdf': os.path.join(self.package_dir, 'urdf', self.urdf_file_name),
        }



class WebRTCNodeFactory:
    """Factory for creating WebRTC nodes"""

    def __init__(self, config: WebRTCLaunchConfig):
        self.config = config

    def create_launch_arguments(self):
        return [
            DeclareLaunchArgument('robot_ip', default_value=self.config.robot_ip, description='IP address of the robot'),
            DeclareLaunchArgument('enable_video', default_value=self.config.enable_video, description='Enable video streaming'),
            DeclareLaunchArgument('urdf_file_name', default_value=self.config.urdf_file_name, description='Name of the URDF file'),
            DeclareLaunchArgument('send_buffer_limit', default_value=self.config.send_buffer_limit, description='Foxglove Bridge send buffer limit'),
            DeclareLaunchArgument('on_exit', default_value=self.config.on_exit, description='Behavior when a node exits'),
            DeclareLaunchArgument('elevenlabs_api_key', default_value=self.config.elevenlabs_api_key, description='API key for ElevenLabs TTS service'),
            DeclareLaunchArgument('voice_name', default_value=self.config.voice_name, description='Voice name for TTS'),
            DeclareLaunchArgument('obstacle_avoidance', default_value=self.config.obstacle_avoidance, description='Enable obstacle avoidance'),
            DeclareLaunchArgument('enable_foxglove_bridge', default_value=self.config.enable_foxglove_bridge, description='Enable Foxglove Bridge'),
        ]

    def create_robot_state_publisher_node(self):
        return Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='webrtc_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['cat ', self.config.config_paths['urdf']]), value_type=str)
            }],
            on_exit=LaunchConfiguration('on_exit'),
        )

    def create_pointcloud_to_laserscan_node(self, namespace: str = None) -> Node:
        """Create pointcloud to laserscan conversion node"""
        return Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='go2_pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', 'point_cloud2'),
                ('scan', 'scan'),
            ],
            parameters=[{
                'target_frame': 'base_link',
                'max_height': 0.5
            }],
            output='screen',
        )

    def create_core_nodes(self):
        robot_ip = LaunchConfiguration('robot_ip')
        enable_video = LaunchConfiguration('enable_video')
        on_exit = LaunchConfiguration('on_exit')
        send_buffer_limit = LaunchConfiguration('send_buffer_limit')
        obstacle_avoidance = LaunchConfiguration('obstacle_avoidance')
        return [
            Node(
                package='go2_robot_sdk',
                executable='go2_driver_node',
                parameters=[{
                    'robot_ip': robot_ip,
                    'token': '',
                    'conn_type': 'webrtc',
                    'enable_video': enable_video,
                    'decode_lidar': False,
                    'publish_raw_voxel': True,
                    'obstacle_avoidance': obstacle_avoidance,
                },
                {
                    "qos_overrides": {
                        "/camera/image_raw": {
                            "publisher": {
                                "reliability": "reliable",
                                "history": "keep_last",
                                "depth": 1,
                            }
                        }
                    }
                }],
                remappings=[('cmd_vel_out', 'cmd_vel')],
                on_exit=on_exit,
            ),
            Node(
                package='image_transport',
                executable='republish',
                name='image_republisher',
                arguments=['raw', 'compressed'],
                remappings=[('in', 'camera/image_raw'), ('out/compressed', 'camera/compressed')],
                on_exit=on_exit,
            ),
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                parameters=[{'send_buffer_limit': send_buffer_limit}],
                condition=IfCondition(LaunchConfiguration('enable_foxglove_bridge')),
                on_exit=on_exit,
            ),
            Node(
                package='speech_processor',
                executable='tts_node',
                name='tts_node',
                parameters=[{
                    'api_key': LaunchConfiguration('elevenlabs_api_key'),
                    'provider': 'elevenlabs',
                    'voice_name': LaunchConfiguration('voice_name'),
                    'local_playback': False,
                    'use_cache': True,
                    'audio_quality': 'standard'
                }],
            ),
        ]

    def create_navigation_nodes(self) -> List[IncludeLaunchDescription]:
        """Create included launch descriptions"""
        use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        with_foxglove = LaunchConfiguration('foxglove', default='true')
        with_slam = LaunchConfiguration('slam', default='true')
        with_nav2 = LaunchConfiguration('nav2', default='true')
        
        foxglove_launch = os.path.join(
            get_package_share_directory('foxglove_bridge'),
            'launch', 'foxglove_bridge_launch.xml'
        )
        
        return [
            # SLAM Toolbox
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('slam_toolbox'),
                                'launch', 'online_async_launch.py')
                ]),
                condition=IfCondition(with_slam),
                launch_arguments={
                    'slam_params_file': self.config.config_paths['slam'],
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
            # Nav2
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('nav2_bringup'),
                                'launch', 'navigation_launch.py')
                ]),
                condition=IfCondition(with_nav2),
                launch_arguments={
                    'params_file': self.config.config_paths['nav2'],
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
        ]


def generate_launch_description():
    config = WebRTCLaunchConfig()
    factory = WebRTCNodeFactory(config)

    launch_args = factory.create_launch_arguments()
    robot_state_publisher_node = factory.create_robot_state_publisher_node()
    core_nodes = factory.create_core_nodes()
    navigation_nodes = factory.create_navigation_nodes()
    pointcloud_to_laserscan_node = factory.create_pointcloud_to_laserscan_node()

    launch_entities = launch_args + [robot_state_publisher_node] + core_nodes + navigation_nodes + [pointcloud_to_laserscan_node]

    return LaunchDescription(launch_entities)
