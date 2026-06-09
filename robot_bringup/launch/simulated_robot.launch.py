# simulated_robot_launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false",
    )
    use_slam = LaunchConfiguration("use_slam")

    # ── Gazebo simulation ────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ros2_description"),
            "launch", "gazebo.launch.py"
        ),
    )

    # ── ros2_control spawners ────────────────────────────────────────────────
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ros2_controller"),
            "launch", "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python":            "False",
            "use_sim_time":          "True",
        }.items(),
    )

    # ── Joystick teleop + twist_mux ──────────────────────────────────────────
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ros2_controller"),
            "launch", "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    # ── Global localisation (AMCL + map_server) - 
    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_localization"),
            "launch", "global_localization.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True",
            "map_name": "small_house",
        }.items(),
        condition=UnlessCondition(use_slam),
    )

    # ── SLAM toolbox ─────────────────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ros2_mapping"),
            "launch", "slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True",
        }.items(),
        condition=IfCondition(use_slam),
    )

    navigation = IncludeLaunchDescription(
         os.path.join(
            get_package_share_directory("robot_navigation"),
            "launch", "navigation.launch.py"
        ),
    )
    # ── Safety stop (LiDAR-based obstacle reaction) ──────────────────────────
    safety_stop = Node(
        package="ros2_utils",
        executable="safety_stop",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz= Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("nav2_bringup"),
            "rviz", "nav2_default_view.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )


    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        # safety_stop,
        localization,
        slam,
        navigation,
        rviz,
    ])