import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ur5_robot_description = get_package_share_directory("ur5_robot_description")

    # Declare argument for robot model file
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            ur5_robot_description, "urdf", "ur5_gripper_main.xacro"
        ),
        description="Absolute path to robot URDF file"
    )

    # Determine if Gazebo Classic is being used based on ROS 2 distro
    ros_distro = os.environ.get("ROS_DISTRO", "iron")  # Default to "humble" if not set
    is_classic = "true" if ros_distro == "iron" else "false"  # Always pass as a string

    # Set Gazebo environment variables
    gazebo_resource_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            str(Path(ur5_robot_description).parent.resolve())
        ]
    )

    gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value="/opt/ros/" + ros_distro + "/lib/"
    )

    # Process robot description using xacro
    robot_description = ParameterValue(
        Command([
            "xacro ", LaunchConfiguration("model"),
            " is_classic:=", is_classic
        ]),
        value_type=str
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # Include Gazebo Classic launch file
    gazebo_classic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ]),
        launch_arguments={"verbose": "true"}.items(),
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "ur5_robot",
            "-topic", "/robot_description"
        ],
        output="screen"
    )

    # Return launch description
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        gz_plugin_path,
        robot_state_publisher_node,
        gazebo_classic,
        spawn_robot
    ])
