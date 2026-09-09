from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
)

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from typing import List

from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.substitutions import (
    FindPackageShare,
)


def generate_launch_description():

    default_mission_file = (
        PathJoinSubstitution([
            FindPackageShare(
                'home_service_mission'
            ),
            'config',
            'test_mission.yaml',
        ])
    )

    mission_file_arg = (
        DeclareLaunchArgument(
            'mission_file',
            default_value=(
                default_mission_file
            ),
            description=(
                'YAML mission definition'
            )
        )
    )

    use_sim_time_arg = (
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        )
    )

    # Sobrescritura de las variables de la mision, en pares
    # nombre=valor. Permite que un mismo YAML de reto sirva sin saber
    # todavia que pieza fisica hay en cada ArUco:
    #
    #   mission_vars:="['pieza_verde=poste','pieza_azul=rueda']"
    mission_vars_arg = (
        DeclareLaunchArgument(
            'mission_vars',
            default_value='[]',
            description=(
                'Pares nombre=valor que sobrescriben la seccion '
                'vars de la mision'
            )
        )
    )

    # Mapa sobre el que correr, y con el las posiciones guardadas.
    # Vacio = el que declare la mision.
    map_name_arg = (
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description=(
                'Nombre del mapa; usa <mapa>.poses.yaml de maps/'
            )
        )
    )

    mission_manager = Node(
        package=(
            'home_service_mission'
        ),
        executable=(
            'mission_manager'
        ),
        name=(
            'home_service_mission_manager'
        ),
        output='screen',
        parameters=[
            {
                'mission_file':
                    LaunchConfiguration(
                        'mission_file'
                    ),

                'use_sim_time':
                    LaunchConfiguration(
                        'use_sim_time'
                    ),

                'mission_vars':
                    ParameterValue(
                        LaunchConfiguration('mission_vars'),
                        value_type=List[str],
                    ),

                'map_name':
                    LaunchConfiguration('map_name'),
            }
        ]
    )

    return LaunchDescription([
        mission_file_arg,
        use_sim_time_arg,
        mission_vars_arg,
        map_name_arg,
        mission_manager,
    ])
