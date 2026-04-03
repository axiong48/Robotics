from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bin_file = LaunchConfiguration('bin_file')

    tcp_demux = Node(
        package='generatemap',
        executable='tcp_demux_node',
        name='tcp_demux_node',
        output='screen',
        parameters=[{
            'output_path': '/MRTP/MRTP/clearpath_simulator/clearpath_gz/worlds/generated_orchard.sdf'
        }]
    )

    send_bin = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'cd /MRTP/MRTP/generatemap && nc localhost 12346 < "$1"',
            'bash',
            bin_file
        ],
        output='screen'
    )

    rebuild_and_bringup = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'cd /MRTP/MRTP && colcon build && source install/setup.bash && ros2 launch generatemap bringup_simulation.launch.py'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('bin_file', default_value='castle.bin'),
        tcp_demux,
        TimerAction(period=2.0, actions=[send_bin]),
        TimerAction(period=6.0, actions=[rebuild_and_bringup]),
    ])