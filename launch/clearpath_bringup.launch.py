from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable


def generate_launch_description():
    description_pkg = FindPackageShare("clearpath_docking")
    urdf_file = PathJoinSubstitution(
        [description_pkg, "urdf", "clearpath_motor.urdf.xacro"]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            urdf_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controllers_file = PathJoinSubstitution(
        [description_pkg, "config", "clearpath_controllers.yaml"]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="screen",
    )

   
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    vel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["clearpath_velocity_controller"],
        output="screen",
    )

    return LaunchDescription([
        ros2_control_node,
        jsb_spawner,
        vel_spawner,
    ])