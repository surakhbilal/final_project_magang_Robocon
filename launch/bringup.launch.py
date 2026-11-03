import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Cari package webots (karena harus di-include)
    webots_pkg_dir = get_package_share_directory('robocon_2027_webots')

    # Path langsung ke folder src (karena description ada di dalam itsr_latihan)
    desc_pkg_dir = '/home/ezco/main/latihan2/src/itsr_latihan2/robocon_2027_description'

    # ============================================================
    # INCLUDE SIMULASI WEBOTS
    # ============================================================
    webots_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(webots_pkg_dir, 'launch', 'simulation.launch.py')
        )
    )

    # ============================================================
    # KONFIGURASI TR ROBOT
    # ============================================================
    tr_xacro_file = os.path.join(desc_pkg_dir, 'urdf', 'tr.urdf.xacro')
    tr_rviz_config_file = os.path.join(desc_pkg_dir, 'rviz', 'tr.rviz')

    tr_robot_description = ParameterValue(
        Command(['xacro ', tr_xacro_file]),
        value_type=str
    )

    tr_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='tr_robot_state_publisher',
        namespace='tr',
        output='screen',
        parameters=[{
            'robot_description': tr_robot_description,
            'frame_prefix': 'tr/'
        }]
    )

    tr_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='tr_rviz',
        output='screen',
        arguments=['-d', tr_rviz_config_file]
    )

    # ============================================================
    # KONFIGURASI BR ROBOT
    # ============================================================
    br_xacro_file = os.path.join(desc_pkg_dir, 'urdf', 'br.urdf.xacro')
    br_rviz_config_file = os.path.join(desc_pkg_dir, 'rviz', 'br.rviz')

    br_robot_description = ParameterValue(
        Command(['xacro ', br_xacro_file]),
        value_type=str
    )

    br_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='br_robot_state_publisher',
        namespace='br',
        output='screen',
        parameters=[{
            'robot_description': br_robot_description,
            'frame_prefix': 'br/'
        }]
    )

    br_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='br_rviz',
        output='screen',
        arguments=['-d', br_rviz_config_file]
    )

    # ============================================================
    # FOXGLOVE BRIDGE
    # ============================================================
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': ['.*'],
            'send_buffer_limit': 10000000,
        }]
    )

    return LaunchDescription([
        webots_simulation,
        tr_robot_state_publisher,
        tr_rviz,
        br_robot_state_publisher,
        br_rviz,
        foxglove_bridge,
    ])