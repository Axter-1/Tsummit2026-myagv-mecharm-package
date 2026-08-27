#!/usr/bin/env python3

import os

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch_ros.actions import Node

from mobile_manipulator_sim.model_builder import (
    build_robot_description
)


def generate_launch_description():
    # ...
    # Gazebo
    # robot_state_publisher
    # spawn_robot
    # ...
    # ========================================================
    # GPU
    # ========================================================

    use_nvidia = SetEnvironmentVariable(
        name="MESA_D3D12_DEFAULT_ADAPTER_NAME",
        value="NVIDIA"
    )

    # ========================================================
    # Robot description
    # ========================================================

    robot_description = build_robot_description()

    # ========================================================
    # Paths
    # ========================================================

    sim_share = get_package_share_directory(
        "mobile_manipulator_sim"
    )

    gazebo_ros_share = get_package_share_directory(
        "gazebo_ros"
    )

    # Empty World
    world = os.path.join(
        sim_share,
        "worlds",
        "empty.world"
    )

    # ========================================================
    # Gazebo Classic Server
    # ========================================================

    gzserver = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(
                gazebo_ros_share,
                "launch",
                "gzserver.launch.py"
            )
        ),

        launch_arguments={

            "world": world,

            "verbose": "true",

            "pause": "false",

            "physics": "ode",

        }.items()
    )

    # ========================================================
    # Gazebo Classic Client
    # ========================================================

    gzclient = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(
                gazebo_ros_share,
                "launch",
                "gzclient.launch.py"
            )
        )
    )

    # ========================================================
    # Robot State Publisher
    # ========================================================

    robot_state_publisher = Node(

        package="robot_state_publisher",

        executable="robot_state_publisher",

        name="robot_state_publisher",

        output="screen",

        parameters=[

            {
                "robot_description":
                    robot_description,

                "use_sim_time":
                    True,
            }

        ]
    )

    # ========================================================
    # Spawn del robot
    # ========================================================

    spawn_robot = Node(

        package="gazebo_ros",

        executable="spawn_entity.py",

        name="spawn_mobile_manipulator",

        output="screen",

        arguments=[

            "-entity",
            "myagv_mecharm270",

            "-topic",
            "robot_description",

            "-x",
            "0.0",

            "-y",
            "0.0",

            "-z",
            "0.02",

        ]
    )

    # ========================================================
    # Joint State Broadcaster
    # ========================================================

    joint_state_broadcaster_spawner = Node(

        package="controller_manager",

        executable="spawner",

        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],

        output="screen",
    )

    # ========================================================
    # Mecanum controller
    # ========================================================

    mecanum_controller_spawner = Node(

        package="controller_manager",

        executable="spawner",

        arguments=[
            "mecanum_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],

        output="screen",
    )

    # ========================================================
    # MechArm trajectory controller
    # ========================================================

    mecharm_controller_spawner = Node(

        package="controller_manager",

        executable="spawner",

        arguments=[
            "mecharm_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],

        output="screen",
    )

    # ========================================================
    # Retrasar activación de controladores
    # ========================================================
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            mecanum_controller_spawner,
            mecharm_controller_spawner,
        ]
    )

    # Esperamos a que gzserver cargue factory plugin.
    delayed_spawn = TimerAction(

        period=3.0,

        actions=[
            spawn_robot
        ]
    )

    # ========================================================
    # Launch
    # ========================================================

    return LaunchDescription([

        use_nvidia,

        gzserver,

        gzclient,

        robot_state_publisher,

        delayed_spawn,

	    delayed_controllers,
    ])
