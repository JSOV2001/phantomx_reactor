# Libraries for file handling
import os
from ament_index_python import get_package_share_directory
import xacro
# Libraries for node launching
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


pkg_file_path = get_package_share_directory("phantomx_reactor")

# URDF's filepath
xacro_filepath = os.path.join(pkg_file_path, "description", "phantomx_reactor.urdf.xacro")
robot_description_file = xacro.process_file(xacro_filepath)

visualization_filepath = os.path.join(pkg_file_path, "config", "visualization.rviz")

# ROS2 arguments for launch configutation
use_sim_time = LaunchConfiguration("use_sim_time")

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", 
        default_value= "false", 
        description= "Use sim time if true"
    )
    
    rsp_node = Node(
        package= "robot_state_publisher",
        executable= "robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_file.toxml(),
                "use_sim_time": use_sim_time
            }
        ]
    )

    rviz_cmd = ExecuteProcess(
        cmd= ["rviz2", "-d", visualization_filepath]
    )

    jsp_gui_node = Node(
        package= "joint_state_publisher_gui",
        executable= "joint_state_publisher_gui",
    )

    nodes_to_run = [
        use_sim_time_arg,
        rsp_node, 
        rviz_cmd, 
        jsp_gui_node
    ]
    return LaunchDescription(nodes_to_run)
       
