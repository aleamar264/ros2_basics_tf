import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Launch my robot node"""
    # Get the path to the URDF file
    urdf_path = os.path.join(get_package_share_directory('my_robot_description'),'urdf/my_robot.urdf.xacro')
    robot_urdf = xacro.process_file(urdf_path)

    rviz_path = os.path.join(get_package_share_directory('my_robot_description'),'rviz/urdf_config.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_pusblisher',
        parameters=[
            {'robot_description': robot_urdf.toxml()},
        ]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d' , rviz_path]
    )

    gazebo_ros = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot'
        ]
    )

    gazebo_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    return LaunchDescription([robot_state_publisher, rviz2,
                              gazebo_ros,
                              gazebo_launch_description])