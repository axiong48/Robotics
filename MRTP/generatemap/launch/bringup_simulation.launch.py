import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # Configurations
    pkg_my_map = 'generatemap'
    pkg_world_source = 'clearpath_gz'
    pkg_robot_desc = 'amiga_ros2_description'
    
    world_file_name = 'generated_orchard.sdf'
    xacro_file_name = 'amiga_descr.urdf.xacro'

    # resolve path
    xacro_path = os.path.join(
        get_package_share_directory(pkg_robot_desc),
        'urdf',
        xacro_file_name
    )

    try:
        share_dir = get_package_share_directory(pkg_world_source)
        world_path = os.path.join(share_dir, 'worlds', world_file_name)
        models_path = os.path.join(share_dir, 'models')
        meshes_path = os.path.join(share_dir, 'meshes')
    except Exception:
        print("[WARN] Using absolute path fallback.")
        base_path = '/MRTP/MRTP/src/clearpath_simulator/clearpath_gz'
        world_path = os.path.join(base_path, 'worlds', world_file_name)
        models_path = os.path.join(base_path, 'models')
        meshes_path = os.path.join(base_path, 'meshes')

    # model path
    robot_share_parent = os.path.dirname(get_package_share_directory(pkg_robot_desc))

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + 
            models_path + ':' + 
            meshes_path + ':' + 
            robot_share_parent 
        ]
    )

    # proccess xacro 
    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toxml()

    # Our Nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'amiga_robot',
            '-x', '5.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock 
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            
            # Velocity Commands 
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            
            # Odometry
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            
            # Transforms
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Lidar
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )
    # nav2
    nav2_params_path = os.path.join(
        get_package_share_directory('generatemap'), 'config', 'nav2_params.yaml'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'slam': 'True', 
            'params_file': nav2_params_path
        }.items(),
    )
    # rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path, 
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        nav2,
        rviz,
    ])

    