import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to the controller configuration file
    controller_config = os.path.join(
        get_package_share_directory("hsr_velocity_controller"),
        "config",
        "my_controller_realtime_test.yaml"
    )

    # Unspawner node (disabling trajectory controllers)
    controller_unspawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "head_trajectory_controller"],
        namespace="/hsrb",
        output="screen"
    )

    # Spawner for the realtime controller
    realtime_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["realtime_body_controller_real", "--controller-manager", "/hsrb/controller_manager"],
        namespace="/hsrb",
        output="screen"
    )

    return LaunchDescription([
        controller_unspawner,
        realtime_controller_spawner
    ])
