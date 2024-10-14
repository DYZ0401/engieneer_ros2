from launch import LaunchDescription
from launch_ros.actions import Node;
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess ,IncludeLaunchDescription

def generate_launch_description():
    package_directory = get_package_share_directory("engineer_control")
    initial_dof_yaml = os.path.join(package_directory, 'config','initial_dof.yaml')
    return LaunchDescription([
        Node(
            package = "engineer_control",
            executable = "engineer_control",
            name = "engineer_control",
            output = "screen",
            parameters = [{"use_sim_time": True},],

        ),
        Node(
            package = "engineer_control",
            executable = "engineer_frame",
            name = "engineer_frame",
            parameters = [{"use_sim_time": True},],

        ),
        Node(
            package = "engineer_control",
            executable = "control_scheduling",
            name = "control_scheduling",
            parameters = [initial_dof_yaml,{"use_sim_time": True},],
        ),
        
    ])

