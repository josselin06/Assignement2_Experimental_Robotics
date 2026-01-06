import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_my = get_package_share_directory("my_opencv")
    pkg_plansys = get_package_share_directory("plansys2_bringup")

    domain = os.path.join(pkg_my, "pddl", "domain.pddl")
    problem = os.path.join(pkg_my, "pddl", "problem.pddl")

    use_sim_time = LaunchConfiguration("use_sim_time")

    plansys_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_plansys, "launch", "plansys2_bringup_launch_distributed.py")
        ),
        launch_arguments={
            "model_file": domain,
            "problem_file": problem,
            "use_sim_time": use_sim_time,
        }.items()
    )

    explore_action = Node(
        package="my_opencv",
        executable="explore_action",
        output="screen",
        parameters=[{
            "action_name": "explore", 
            "use_sim_time": True,
            "image_topic": "/camera/image",
            "nav_action_name": "/navigate_to_pose",
            "nav_goal_frame": "map",
            "scan_duration_s": 2.5,
            "wait_nav_server_timeout_s": 20.0,
            "overall_timeout_s": 900.0,
            "reset_ids_file_on_start": True,
        }],
    )

    take_picture_next_action = Node(
        package="my_opencv",
        executable="take_picture_next_action",
        output="screen",
        parameters=[{
            "action_name": "take_picture_next",
            "use_sim_time": True,
            "image_topic": "/camera/image",
            "cmd_vel_topic": "/cmd_vel",
            "center_tolerance_px": 6.0,
            "search_ang_vel": 0.5,
            "kp": -0.5,
            "overall_timeout_s": 300.0,
            "reset_done_file_on_start": True,
        }],
    )

    return LaunchDescription([
        plansys_bringup,
        explore_action,
        take_picture_next_action,
    ])
