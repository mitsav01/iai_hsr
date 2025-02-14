import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare launch arguments
    return LaunchDescription([
        DeclareLaunchArgument('velocity_controller', default_value='False', description='Activate velocity controller'),
        DeclareLaunchArgument('apartment_map', default_value='True', description='Load apartment map'),

        # Include upload_hsrb.launch from hsr_description
        IncludeLaunchDescription(
            launch_description_source=[os.path.join(
                get_package_share_directory('hsr_description'), 'launch', 'upload_hsrb.launch')],
            launch_arguments={}.items()
        ),
        
        # Robot state publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_pub',
            respawn=True,
            output='screen',
            namespace='hsrb/robot_state'
        ),

        # Conditionally include the apartment map launch file
        IncludeLaunchDescription(
            launch_description_source=[os.path.join(
                get_package_share_directory('hsr_navigation'), 'launch', 'hsr_map.launch')],
            condition=IfCondition(LaunchConfiguration('apartment_map'))
        ),

        # Conditionally include the velocity controller launch file
        IncludeLaunchDescription(
            launch_description_source=[os.path.join(
                get_package_share_directory('hsr_velocity_controller'), 'launch', 'switch_to_velocity_controllers.launch')],
            condition=IfCondition(LaunchConfiguration('velocity_controller'))
        ),
    ])
