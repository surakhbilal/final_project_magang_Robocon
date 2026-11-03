import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_directory = get_package_share_directory(
        'robocon_2027_webots'
    )

    world_path = os.path.join(
        package_directory,
        'worlds',
        'simple_arena.wbt'
    )

    # ============================================================
    # BACA URDF TR
    # ============================================================
    tr_robot_description_path = os.path.join(
        package_directory,
        'urdf',
        'tr_webots.urdf'
    )

    with open(tr_robot_description_path, 'r', encoding='utf-8') as urdf_file:
        tr_robot_description = urdf_file.read()

    # ============================================================
    # BACA URDF BR
    # ============================================================
    br_robot_description_path = os.path.join(
        package_directory,
        'urdf',
        'br_webots.urdf'
    )

    with open(br_robot_description_path, 'r', encoding='utf-8') as urdf_file:
        br_robot_description = urdf_file.read()

    # ============================================================
    # WEBOTS LAUNCHER
    # ============================================================
    webots = WebotsLauncher(
        world=world_path
    )

    # ============================================================
    # TR WEBOTS CONTROLLER
    # ============================================================
    tr_webots_controller = WebotsController(
        robot_name='tr_robot',
        parameters=[
            {
                'robot_description': tr_robot_description
            }
        ],
        respawn=True,
    )

    # ============================================================
    # BR WEBOTS CONTROLLER
    # ============================================================
    br_webots_controller = WebotsController(
        robot_name='br_robot',
        parameters=[
            {
                'robot_description': br_robot_description
            }
        ],
        respawn=True,
    )

    # ============================================================
    # SHUTDOWN HANDLER
    # ============================================================
    shutdown_handler = (
        launch.actions.RegisterEventHandler(
            event_handler=(
                launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[
                        launch.actions.EmitEvent(
                            event=launch.events.Shutdown()
                        )
                    ]
                )
            )
        )
    )

    # ============================================================
    # RETURN LAUNCH DESCRIPTION
    # ============================================================
    return LaunchDescription([
        webots,
        tr_webots_controller,
        br_webots_controller,
        shutdown_handler,
    ])