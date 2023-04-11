import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit

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
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo_params_path = os.path.join(
        get_package_share_directory(package_name), "config", "gazebo_params.yaml"
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            #'world':os.path.join(get_package_share_directory(package_name),'worlds','arena.sdf'),
            "extra_gazebo_args": "--ros-args --params-file "
            + gazebo_params_path
        }.items(),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_bot"],
        output="screen",
    )

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

    delayed_spawn_dc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity, on_exit=[diff_drive_spawner]
        )
    )
    delayed_spawn_jb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner, on_exit=[joint_broad_spawner]
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

    # Launch the object tracker
    object_tracker = Node(
        package="object_tracker",
        executable="object_tracker",
        remappings=[
            ("/image_in", "/camera/image_raw"),
        ],
    )

    delayed_object_tracker = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner, on_exit=[object_tracker]
        )
    )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
            delayed_spawn_jb,
            delayed_spawn_dc,
            delayed_nav2,
            delayed_object_tracker,
        ]
    )
