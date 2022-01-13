from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from launch.actions.execute_process import ExecuteProcess

param_path = os.path.join(
    get_package_share_directory("costmap_converter"), 
    'params', 
    'params.yaml')

def generate_launch_description():

    ns="sim"

    param_substitutions = {}

    remappings = [('/tf', '/sim/tf'),
                  ('/tf_static', '/sim/tf_static'),
                  ('/sim/map', '/sim/slope_filtered')]

    configured_params = RewrittenYaml(  
        source_file=param_path,
        root_key=ns,
        param_rewrites=param_substitutions,
        convert_types=True)


    return LaunchDescription([ 
        
        Node(
            package='costmap_converter',
            namespace=ns,
            executable='standalone_converter',
            name='standalone_converter',
            output={
                    "stdout": "screen",
                    "stderr": "screen",
            },
            parameters=[configured_params],
            remappings=remappings,
        ),          

    ])
