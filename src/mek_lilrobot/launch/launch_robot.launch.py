import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "mek_lilrobot"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "false", "use_ros2_control": "true"}.items(),
    )

    robot_description = Command(
        ["ros2 param get --hide-type /robot_state_publisher robot_description"]
    )

    controller_params = os.path.join(
        get_package_share_directory(
            "mek_lilrobot"
        ),
        "config",
        "controllers.yaml",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_params],
    )

    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    range_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["range_broad"],
    )

    gripper_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=diff_drive_spawner,
            on_start=[joint_broad_spawner],
        )
    )
    delayed_rb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[range_broad_spawner],
        )
    )
    delayed_gc = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[gripper_cont_spawner],
        )
    )

    # Launch the nav2 stack
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "sim_nav.launch.py",
                )
            ]
        )
    )

    delayed_nav2 = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=joint_broad_spawner, on_exit=[nav2])
    )


    # Launch the camera driver
    v4l2_camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
    )

    # Launch the object tracker
    object_tracker = Node(
        package="object_tracker",
        executable="object_tracker",
        remappings=[
            ("/image_in", "/image_raw"),
        ],
    )

    # Launch robot behaviour action server
    robot_server = Node(
        package="mek_lilrobot",
        executable="robot",
    )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            delayed_rb,
            delayed_gc,
            delayed_nav2,
            v4l2_camera,
            object_tracker,
            robot_server
        ]
    )
