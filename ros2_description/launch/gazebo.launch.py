import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ros2_description = get_package_share_directory("ros2_description")

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                ros2_description, "urdf", "ros2.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution([
            ros2_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(ros2_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("ros2_description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "False" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True
                    }]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    # "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                    "gz_args": PythonExpression(["'", world_path, " -v 0 -r --headless-rendering'"])
                }.items()
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "sarsora_robot"],
        parameters=[{"use_sim_time": True}],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/sarsora_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",

            "/model/sarsora_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/sarsora_robot/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            # IMU
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # LiDAR
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            # Remap model-namespaced TF to the global /tf topic
            ("/model/sarsora_robot/tf", "/tf"),
            # Remap IMU to the topic the imu_republisher expects
            ("/imu", "/imu/out"),
            ("/model/sarsora_robot/odom", "/diff_drive_controller/odom"),
        ],
        parameters=[{"use_sim_time": True}],
    )

    cmd_vel_relay = Node(
    package="topic_tools",
    executable="relay",
    name="cmd_vel_relay",
    parameters=[{"use_sim_time": True}],
    arguments=[
        "diff_drive_controller/cmd_vel_unstamped",
        "/model/sarsora_robot/cmd_vel"
    ],
    )


    
    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        cmd_vel_relay,
    ])