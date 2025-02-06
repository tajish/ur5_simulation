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

     

    # Set Gazebo environment variables
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(ur5_robot_description).parent.resolve())
        ]
    )

    ros_distro = os.environ["ROS_DISTRO"] 
    is_classic = "True" if ros_distro == "humble" else "False" 

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


    gazebo_classic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4 -r empty.sdf"]
                    )
                ]
             )
    

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "ur5_robot"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            "/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output='screen',
    )
    
    
    
    tf2_ros_bridge = Node(
            package='tf2_ros',
            namespace = 'base_to_wrist_3',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "base_link", "ur5_robot/wrist_3_link/camera"]
        )
    
    """
    tf2_ros2_bridge = Node(
            package='tf2_ros',
            namespace = 'wrist3_to_camera',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "ur5_robot/wrist_3_link/camera", "camera_frame_link"]
        )"""
    
    

    # Return launch description
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo_classic,
        gz_spawn_entity,
        gz_ros2_bridge,
        tf2_ros_bridge,
        #tf2_ros2_bridge,
    ])
