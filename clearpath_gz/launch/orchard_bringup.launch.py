# Copyright 2023 Clearpath Robotics, Inc.
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
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. SETUP - Point to your generated file
    # Ensure this matches the actual filename you generated
    world_file = 'generated_orchard.sdf'
    
    # This must match the <world name="..."> inside your SDF file
    # The generator script uses 'procedural_orchard' by default
    world_name_arg = 'generated_orchard'

    # 2. START GAZEBO DIRECTLY
    # We bypass the package search and point straight to the file
    gazebo_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen'
    )

    # 3. SPAWN ROBOT USING CLEARPATH SPAWNER
    # Now that we created robot.yaml, this script will work!
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    robot_spawn_launch = PathJoinSubstitution([
        pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'
    ])

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments={
            'world': world_name_arg,
            'x': '0.0',
            'y': '0.0',
            'z': '0.3',
            'yaw': '0.0',
            'setup_path': '/MRTP/MRTP/my_robot_config/'
        }.items()
    )

    # 4. BRIDGE (Manual Override)
    # Clearpath sets up bridges automatically, but sometimes misses custom world topics.
    # We add a manual bridge here to be safe.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # General
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            
            # Sensors (Adjusted for the robot.yaml names above)
            '/a200_0000/sensors/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/a200_0000/sensors/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            
            # Drive
            '/a200_0000/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/a200_0000/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_sim,
        robot_spawn,
        bridge
    ])
