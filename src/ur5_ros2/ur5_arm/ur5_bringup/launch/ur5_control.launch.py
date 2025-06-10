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


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    
def generate_launch_description():
    # rviz
# ARGUMENTS ---------------------------------------------------------------

    rviz_arg = DeclareLaunchArgument("rviz_gui", default_value="true",
                                                 description="Flag to enable rviz")

    # current package path
    pkg_share_path = get_package_share_directory("ur5_description")

    # Rviz config path
    rviz_config_path = PathJoinSubstitution(
        [pkg_share_path, "rviz", "view_robot.rviz"]
    )

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

    description_share = os.path.join(get_package_prefix("ur5_description"), "share")
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
        get_package_share_directory("ur5_description"),
        "config",
        "robot_controllers.yaml",
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

    ur5_controller = Node(
        name="ur5_controller",
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "ur5_controller",
            "--controller-manager",
            "/controller_manager"       
        ]
    )

    # Return Launch Function -------------------------------------------------

    return LaunchDescription(
        [
            rviz_arg,
            rviz_node,
            robot_state_publisher_node,
            
            gazebo_env_var,
            gazebo_server,
            gazebo_client,
            spawn_robot,
            
            ros2_control_node,
            joint_state_broadcaster,
            # ur5_controller
        ]
    )

    



































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

    # ARGUMENTS ---------------------------------------------------------------

    rviz_arg = DeclareLaunchArgument("rviz_gui", default_value="true",
                                                 description="Flag to enable rviz")

    # current package path
    pkg_share_path = get_package_share_directory("ur5_description")

    # Rviz config path
    rviz_config_path = PathJoinSubstitution(
        [pkg_share_path, "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_share_path, "urdf", "ur5_urdf.xacro"]
            ),
            " ",
            "name:=ur5"
        ]
    )
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