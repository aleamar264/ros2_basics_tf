import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Launch my robot node"""
    # Get the path to the URDF file
    urdf_path = os.path.join(get_package_share_directory('my_robot_description'),'urdf/my_robot.urdf')
    robot_urdf = xacro.process_file(urdf_path)

    rviz_path = os.path.join(get_package_share_directory('my_robot_description'),'rviz/urdf_config.rviz')
    rviz_config = xacro.process_file(rviz_path)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_pusblisher',
        parameters=[
            {'robot_description': robot_urdf.toxml()},
        ]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d' ,rviz_config.toxml()]
    )

    return LaunchDescription([robot_state_publisher, joint_state_publisher_gui, rviz2])