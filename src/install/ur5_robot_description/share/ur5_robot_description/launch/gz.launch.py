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

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            ur5_robot_description, "urdf", "ur5_gripper_main.xacro"
        ),
        description="Absolute path to robot URDF file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            str(Path(ur5_robot_description).parent.resolve())
        ]
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # Launch Gazebo Classic
    gazebo_classic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ]),
        launch_arguments={"verbose": "true"}.items(),
    )

    # Spawn the robot in Gazebo Classic
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "ur5_robot",
            "-topic", "robot_description"
        ],
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo_classic,
        spawn_robot
    ])
