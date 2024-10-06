from launch import LaunchDescription
from launch_ros.actions import Node;
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess ,IncludeLaunchDescription

def generate_launch_description():
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("engineer_moveit_config"),
                                                    'launch'),'/demo.launch.py'])

    )

    package_directory = get_package_share_directory("engineer_control")
    initial_dof_yaml = os.path.join(package_directory, 'config','initial_dof.yaml')
    return LaunchDescription([
        Node(
            package = "engineer_control",
            executable = "engineer_control",
            name = "engineer_control",
            output = "screen",
        ),
        Node(
            package = "engineer_control",
            executable = "engineer_frame",
            name = "engineer_frame",
        ),
        Node(
            package = "engineer_control",
            executable = "key_control",
            name = "key_control",
        ),
        Node(
            package = "engineer_control",
            executable = "control_scheduling",
            name = "control_scheduling",
            parameters = [initial_dof_yaml],
            output = "screen",
        ),
        
    ])

