import launch
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    teleop_node = launch_ros.actions.Node(
        package='nesfr_teleop',
        executable='nesfr_teleop_node',
        output='screen',
        name='nesfr_teleop_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    bridge_node = launch_ros.actions.Node(
        package='nesfr_bridge',
        executable='nesfr_ros_bridge',
        name='nesfr_ros_bridge',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                             description='Flag to enable use_sim_time'),
        joy_node,
        teleop_node,
        bridge_node
    ])
