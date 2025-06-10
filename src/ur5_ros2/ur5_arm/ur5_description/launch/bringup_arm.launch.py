import os
from launch import LaunchDescription
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix,get_package_share_path


def generate_launch_description():

    # ARGUMENTS ---------------------------------------------------------------
    mode = LaunchConfiguration("mode")
    declared_arguments = [DeclareLaunchArgument('mode', default_value='gazebo')]
    rviz_arg = DeclareLaunchArgument("rviz_gui", default_value="false",
                                                 description="Flag to enable rviz")

    # current package path
    pkg_share_path = get_package_share_directory("moying_arm_description")

    # Rviz config path
    rviz_config_path = PathJoinSubstitution(
        [pkg_share_path, "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_share_path, "urdf", "moying_arm.urdf.xacro"]
            ),
            " ",
            # "name:=ur5",
            # ' mode:=',mode
        ]
    )

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=None)}

    # NODES -----------------------------------------------------------------

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz_gui"))
    )

    # GAZEBO -----------------------------------------------------------------

    description_share = os.path.join(get_package_prefix("moying_arm_description"), "share")
    gazebo_env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", description_share)

    gazebo_server = ExecuteProcess(
        cmd=[
              "gzserver",
              "--verbose",
              "-s", "libgazebo_ros_factory.so",
              os.path.join(pkg_share_path, "worlds", "empty.world"),
            ],
        output="screen"
    )
    
    gazebo_client = ExecuteProcess(cmd=["gzclient"], output="screen")

    spawn_robot = Node(
        name="spawn_robot",
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "ur5_robot", 
                   "-topic", "/robot_description",
                   "-x", "0.0",
                   "-y", "0.0",
                   "-z", "0.0"]
    )

    # ROS 2 CONTROL ----------------------------------------------------------

    ros2_controllers_path = os.path.join(
        get_package_share_directory("moying_arm_bringup"),
        "config",
        "moying_arm_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=["/robot_description", ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        }
    )

    joint_state_broadcaster = Node(
        name="joint_state_broadcaster",
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"       
        ]
    )

    # position_controller = Node(
    #     name="position_controller",
    #     package="controller_manager",
    #     executable="spawner",
    #     output="screen",
    #     arguments=[
    #         "position_controller",
    #         "--controller-manager",
    #         "/controller_manager",
    #         "--params-file",  # 关键参数
    #         "ros2_controllers_path"  # 替换为实际路径       
    #     ]
    # )

    # joint_trajectory_controller = Node(
    #     name="joint_trajectory_controller",
    #     package="controller_manager",
    #     executable="spawner",
    #     output="screen",
    #     arguments=[
    #         "joint_trajectory_controller",
    #         "--controller-manager",
    #         "/controller_manager",
    #         "--params-file",  # 关键参数
    #         "ros2_controllers_path"  # 替换为实际路径       
    #     ]
    # )

    mpc_controller = Node(
        name="mpc_controller",
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "mpc_controller",
            "--controller-manager",
            "/controller_manager"       
        ]
    )


    # Return Launch Function -------------------------------------------------

    return LaunchDescription(
        [
            *declared_arguments,
            rviz_arg,
            rviz_node,
            robot_state_publisher_node,
            
            gazebo_env_var,
            gazebo_server,
            gazebo_client,
            spawn_robot,
            
            ros2_control_node,
            joint_state_broadcaster,
            # position_controller,
            # joint_trajectory_controller,
            # mpc_controller
        ]
    )