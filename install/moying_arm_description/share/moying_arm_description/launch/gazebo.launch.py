'''
Author: JakeFishcode 
Mail: wobudajianpan@qq.com
Description: 
'''
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,GroupAction,DeclareLaunchArgument,ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import launch_ros.substitutions
import launch


def generate_launch_description():
    # Initialize Arguments
    # mode = LaunchConfiguration("mode")
    # controller = LaunchConfiguration("controller")

    # Declare arguments
    mode= "gazebo"
    declared_arguments = [DeclareLaunchArgument('mode', default_value='gazebo')]
    # Get URDF via xacro
    gazebo_controller_settings = PathJoinSubstitution(
        [
            FindPackageShare("moying_mcr_description"),
            "config",
            "test_controllers.yaml"
        ]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("moying_mcr_description"),
                    "urdf",
                    "moying_mcr.urdf.xacro",
                ]
            ),
            ' settings:=', gazebo_controller_settings,
            " mode:=", mode,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    spawn_entity = GroupAction(
        actions = [
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-topic", "robot_description", "-entity", "moying_mor", "-z", "1.5","-Y","0.0"],
                output="screen",
            ),
        ]
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "false"}.items(),
    )
    load_test_right_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["test_right_arm_controller", "--controller-manager", "/controller_manager"],
    )
    load_test_left_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["test_left_arm_controller", "--controller-manager", "/controller_manager"],
    )

    
    load_mecanum_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_controller", "--controller-manager", "/controller_manager"],
        output="log",
    )

    nodes = [
        spawn_entity,
        gazebo,
        robot_state_publisher_node,
        load_joint_state_controller,
        load_test_left_arm_controller,
        load_test_right_arm_controller,
        load_mecanum_controller,
        # load_controller,
    ]

    return LaunchDescription(declared_arguments + nodes)
