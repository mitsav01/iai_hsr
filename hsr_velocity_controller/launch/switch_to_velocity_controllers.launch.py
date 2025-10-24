import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    controller_yaml = os.path.join(
        get_package_share_directory("hsr_velocity_controller"),
        "config",
        "my_controller_realtime_test.yaml"
    )

    # ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': os.path.join(
                get_package_share_directory('hsrb_description'),
                'robots', 'hsrb4s.urdf.xacro')
        }, controller_yaml],
        output='screen',
    )

    # Spawner for velocity controller
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['realtime_body_controller_real', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        ros2_control_node,
        velocity_controller_spawner
    ])
