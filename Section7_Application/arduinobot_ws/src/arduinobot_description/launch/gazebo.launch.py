import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arduinobot_description = get_package_share_directory('arduinobot_description')

    model_arg = DeclareLaunchArgument(name='model', default_value=os.path.join(
                                        arduinobot_description, 'urdf', 'arduinobot.urdf.xacro'
                                        ),
                                      description='Absolute path to robot urdf file'
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            str(Path(arduinobot_description).parent.resolve())
            ]
        )
    
    ros_distro = os.environ['ROS_DISTRO']
    is_ignition = 'True' if ros_distro == 'humble' else 'False'

    robot_description = ParameterValue(Command([
            'xacro ',
            LaunchConfiguration('model'),
            ' is_ignition:=',
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [' -v 4', ' -r']
                    )
                ]
             )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'arduinobot'],
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
    ])
