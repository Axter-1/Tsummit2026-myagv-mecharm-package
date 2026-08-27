# myAGV Home Service

Proyecto ROS 2 Humble para un robot móvil Elephant Robotics myAGV con brazo MechArm 270 M5.

## Estado actual

La versión actual incluye:

- Simulación en Gazebo Classic 11
- Movimiento Mecanum
- Control del MechArm mediante `JointTrajectoryController`
- Cámara frontal simulada
- LiDAR simulado
- Detección de ArUco
- Aproximación geométrica usando la normal del ArUco
- Medición de distancia final mediante LiDAR
- SLAM y mapas guardados
- Nav2 + AMCL
- Arbitraje de velocidades mediante `twist_mux`
- Mission Manager configurable mediante YAML
- Repetición finita o infinita de la misión

## Entorno soportado

Entorno principal de desarrollo:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11

El proyecto está preparado para evitar rutas dependientes de un usuario concreto.

## Instalación en una máquina nueva

Primero clonar el repositorio:

    git clone <PRIVATE_REPOSITORY_URL> myagv_home_service_ws

Entrar al workspace:

    cd myagv_home_service_ws

Ejecutar la instalación de dependencias y compilación:

    ./scripts/bootstrap.sh

Cargar el entorno:

    source scripts/env.sh

## Compilar nuevamente

    ./scripts/build.sh

## Cargar el entorno manualmente

Desde la raíz del workspace:

    source scripts/env.sh

Esto configura automáticamente:

- ROS 2 Humble
- El workspace actual
- `GAZEBO_MODEL_PATH`
- `HOME_SERVICE_WS`

## Ejecutar Nav2 en simulación

    ./scripts/run_nav2_sim.sh

## Ejecutar la misión en simulación

    ./scripts/run_mission_sim.sh

## Configuración de la misión

Las misiones se definen mediante archivos YAML.

Ejemplo:

    mission:

      name: nav_aruco_nav_test

      repeat: 3

      stop_on_failure: true

      steps:

        - type: navigate
          name: go_near_aruco
          x: 0.50
          y: 0.20
          yaw_deg: 0.0
          frame_id: map
          retries: 1

        - type: aruco
          name: approach_aruco
          id: 3
          stop_distance: 0.12
          timeout_sec: 0.0
          retries: 1

        - type: navigate
          name: go_final
          x: 0.0
          y: 0.0
          yaw_deg: 0.0
          frame_id: map
          retries: 1

## Repetición de misión

Ejecutar una vez:

    repeat: 1

Ejecutar cinco veces:

    repeat: 5

Ejecutar indefinidamente hasta detener el nodo:

    repeat: 0

Cuando `repeat` vale `0`, el Mission Manager ejecuta la secuencia continuamente hasta recibir `Ctrl+C` o hasta que ROS se cierre.

## Arquitectura de control

Flujo general:

    Mission Manager
          |
          +--------------------+
          |                    |
          v                    v
    NavigateToPose      ArucoLidarApproach
          |                    |
          v                    v
       /cmd_vel        /cmd_vel_aruco
          |                    |
          +---------+----------+
                    |
                    v
                twist_mux
                    |
                    v
    /mecanum_drive_controller/reference_unstamped
                    |
                    v
                  myAGV

La entrada de ArUco tiene mayor prioridad en `twist_mux` que la navegación de Nav2.

## Aproximación ArUco

La rutina actual utiliza:

    SEARCHING
        |
        v
    LOCK_TARGET
        |
        v
    ALIGN_HEADING_TO_ARUCO
        |
        v
    ALIGNING_LATERAL
        |
        v
    APPROACHING
        |
        v
    REACHED

La orientación y geometría se obtienen mediante el ArUco.

La distancia final se obtiene mediante LiDAR.

## Mapas

Los mapas utilizados por el proyecto están almacenados en:

    maps/

El mapa principal actual de Home Service Challenge es:

    maps/home_service_challenge_myagv.yaml

con su imagen asociada:

    maps/home_service_challenge_myagv.pgm

## Modelos Gazebo propios

Los modelos Gazebo que pertenecen al proyecto se almacenan en:

    gazebo_models/

Esto evita depender exclusivamente de:

    ~/.gazebo/models

## Dependencias externas

Los repositorios ROS externos no se almacenan dentro de este repositorio.

Sus versiones exactas se encuentran en:

    dependencies.repos

Durante:

    ./scripts/bootstrap.sh

se restauran mediante `vcs import`.

Esto permite usar las mismas revisiones que fueron validadas durante el desarrollo.

## Robot real

La lógica de misión está diseñada para reutilizarse posteriormente en el robot físico.

En hardware real deberá utilizarse:

    use_sim_time:=false

La versión real necesitará además los drivers físicos correspondientes para:

- myAGV
- MechArm 270 M5
- LiDAR
- Cámara

La simulación y el bringup del hardware se mantendrán separados de la lógica de misión.

## Versión

La versión actual del proyecto se encuentra en:

    VERSION

Baseline inicial:

    1.0.0-sim
