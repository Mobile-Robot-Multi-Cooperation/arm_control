import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import GroupAction,DeclareLaunchArgument,ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

def generate_launch_description():
    # Initialize Arguments
    mode = LaunchConfiguration("mode")
    log_level = LaunchConfiguration("log_level")

    # Declare arguments
    declared_arguments = [DeclareLaunchArgument('mode', default_value='gazebo'),
                          DeclareLaunchArgument(name='log_level', default_value='info')]
    # Get URDF via xacro 这两个文件都没有
    elfin_drivers_yaml = os.path.join(get_package_share_directory("moying_arm_bringup"),
        "config","arm_drivers.yaml")
    pkg_share_path = get_package_share_directory("moying_arm_description")
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_share_path, "urdf", "moying_arm.urdf.xacro"]
            ),
            " ",
        ]
    )
    # robot_description_content = Command(
    #   [
    #     PathJoinSubstitution([FindExecutable(name="xacro")]),
    #     " ",
    #     PathJoinSubstitution(
    #       [
    #         FindPackageShare("moying_arm_description"),
    #         "urdf",
    #         "moying_arm.urdf.xacro",
    #       ]
    #     ),
    #     ' mode:=',mode,
    #   ]
    # )

    robot_description = {"robot_description": ParameterValue(robot_description_content)}

    # real_controller_settings = PathJoinSubstitution(
    #   [
    #     FindPackageShare("moying_arm_bringup"),
    #     "config",
    #     "moying_arm_controllers.yaml"
    #   ]
    # )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("moying_arm_bringup"),
        "config",
        "moying_arm_controllers.yaml",
    )

    spawn_entity = Node(
          package="controller_manager",
          executable="ros2_control_node",
          parameters=["/robot_description",ros2_controllers_path],
          output={
              "stdout":"screen",
              "stderr":"screen",
          },
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"frame_prefix": ""}],
    )

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )

    load_arm_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager",
                   '--ros-args', '--log-level', log_level],
        output="log",
    )
    load_mpc_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mpc_controller", "--controller-manager", "controller_manager",
                   '--ros-args', '--log-level', log_level],
        output="log",
    )
    joy_entity = Node(
          package="joy",
          executable="joy_node",
          namespace="/mor/robotiq_controller",
          output={
              "stdout":"screen",
              "stderr":"screen",
          },
    )
    load_arm_velcoity_interface_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_velocity_interface_controller", "--controller-manager", "controller_manager",
                   '--ros-args', '--log-level', log_level],
        output="log",
    )

    nodes = [
        spawn_entity,
        robot_state_publisher_node,
        load_joint_state_controller,
        # joy_entity,
        # load_velcoity_interface_controller,
        # load_mpc_controller,
        # load_arm_trajectory_controller,

    ]

    return LaunchDescription(declared_arguments + nodes)