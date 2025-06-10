import os
from ament_index_python.packages import get_package_share_directory

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
    declared_arguments = [DeclareLaunchArgument('mode', default_value='real'),
                          DeclareLaunchArgument(name='log_level', default_value='info')]
    # Get URDF via xacro
    simulation_controller_settings = PathJoinSubstitution(
      [
        FindPackageShare("moying_mor_bringup"),
        "config",
        "simulation_controller.yaml"
      ]
    )
    
    robot_description_content = Command(  
      [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
          [
            FindPackageShare("ur_gripper_description"),
            "urdf",
            "ur.urdf.xacro",
          ]
        ),
        ' mode:=',mode,
        ' settings:=', simulation_controller_settings,
      ]
    )

    robot_description = {"robot_description": robot_description_content}

    real_controller_settings = PathJoinSubstitution(
      [
        FindPackageShare("ur5_arm_bringup"),
        "config",
        "ur5_arm_controller.yaml"
      ]
    )

    spawn_entity = Node(
          package="controller_manager",
          executable="ros2_control_node",
          namespace="/ur5",
          parameters=[robot_description,real_controller_settings],
          output={
              "stdout":"screen",
              "stderr":"screen",
          },
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace="/ur5",
        parameters=[robot_description, {"frame_prefix": "/ur5"}],
    )

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace="/ur5",
        arguments=["joint_state_broadcaster", "--controller-manager", "/ur5/controller_manager"],
    )

    load_arm_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace="/ur5",
        arguments=["arm_trajectory_controller", "--controller-manager", "/ur5/controller_manager",
                   '--ros-args', '--log-level', log_level],
        output="log",
    )


    nodes = [
        spawn_entity,
        robot_state_publisher_node,
        load_joint_state_controller,
        # joy_entity,
        # load_velcoity_interface_controller,
        load_arm_trajectory_controller,

    ]

    return LaunchDescription(declared_arguments + nodes)