from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='panda_teleop').find('panda_teleop')
    panda_ros2_gazebo_pkg_share = launch_ros.substitutions.FindPackageShare(package='panda_ros2_gazebo').find('panda_ros2_gazebo')
    panda_ros2_gazebo_parameter_file_path = os.path.join(panda_ros2_gazebo_pkg_share,
        "config",
        "params.yaml"
    )
    default_model_path = os.path.join(pkg_share,
        "description",
        "models",
        "panda",
        "panda.urdf",
    )

    panda_teleop_node = launch_ros.actions.Node(
        package='panda_teleop',
        # namespace=...
        executable='panda_teleop_control',
        name='panda_teleop_control',
        parameters=[
            panda_ros2_gazebo_parameter_file_path
        ],
        output='screen'
    )

    # TODO: Do I need to do any remappings somewhere since I don't want my robot description loaded to the standard robot_description topic?
    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description_visual", "-entity", "panda"],
        output="screen",
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                             description='Absolute path to robot urdf file'),
        panda_teleop_node,
        # spawn_entity
    ])