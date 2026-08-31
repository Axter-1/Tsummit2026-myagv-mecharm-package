#!/bin/bash

# Función para abrir una nueva terminal y ejecutar un comando
lanzar_terminal() {
    local titulo="$1"
    local comando="$2"
		xterm \
      -T "$titulo" \
			-e bash -lc "
         source /opt/ros/galactic/setup.bash
         source ~/myagv_home_service_ws/install/setup.bash
         $comando
         exec bash
			" &
}

# Comandos base almacenados en variables para no repetir código
CMD_GAZEBO="ros2 launch mobile_manipulator_sim gazebo_hsc.launch.py"
CMD_NAV2="ros2 launch nav2_bringup bringup_launch.py map:=\$HOME/myagv_home_service_ws/maps/home_service_challenge_myagv.yaml use_sim_time:=true params_file:=\$HOME/myagv_home_service_ws/src/mobile_manipulator_sim/config/nav2_sim.yaml autostart:=true use_composition:=False"
CMD_RVIZ="ros2 launch nav2_bringup rviz_launch.py"
CMD_TWIST_MUX="ros2 run twist_mux twist_mux --ros-args --params-file \$HOME/myagv_home_service_ws/src/mobile_manipulator_sim/config/twist_mux.yaml -r cmd_vel_out:=/mecanum_drive_controller/reference_unstamped"
CMD_ARUCO_DETECTOR="ros2 launch home_service_perception aruco_detector.launch.py"
CMD_ARUCO_LIDAR="ros2 launch home_service_behaviors aruco_lidar_approach.launch.py"
CMD_ARUCO_APPROACH="ros2 launch home_service_behaviors aruco_approach.launch.py"
CMD_SEND_GOAL="ros2 action send_goal /aruco_lidar_approach home_service_interfaces/action/ArucoApproach \"{target_id: 3, stop_distance: 0.12, timeout_sec: 0.0}\" --feedback"
CMD_MISSION="ros2 launch home_service_mission mission.launch.py"

clear
echo "=========================================================="
echo "          MENÚ DE LANZAMIENTO - HOME SERVICE ROBOT        "
echo "=========================================================="
echo "1) Detección y movimiento hacia un Aruco"
echo "2) Movimiento autónomo Lidar"
echo "3) Misión completa (Lidar + Aruco)"
echo "4) Salir"
echo "=========================================================="
read -p "Selecciona una opción [1-4]: " OPCION

case $OPCION in
    1)
        echo "Iniciando Detección y movimiento hacia un Aruco..."
        lanzar_terminal "Gazebo" "$CMD_GAZEBO"
        sleep 5
        lanzar_terminal "Aruco Detector" "$CMD_ARUCO_DETECTOR"
        lanzar_terminal "Twist Mux" "$CMD_TWIST_MUX"
        lanzar_terminal "Aruco Lidar Approach" "$CMD_ARUCO_LIDAR"
        sleep 2
        lanzar_terminal "Nav2 Bringup" "$CMD_NAV2"
        sleep 4
        lanzar_terminal "Aruco Approach" "$CMD_ARUCO_APPROACH"
        sleep 2
        lanzar_terminal "Send Goal (Manual)" "$CMD_SEND_GOAL"

				echo
				echo "Todos los nodos fueron Lanzados"
				echo "Presiona ENTER para cerrar todas las terminales..."
				read

				pkill -f xterm
        ;;
    2)
        echo "Iniciando Movimiento autónomo Lidar..."
        lanzar_terminal "Gazebo" "$CMD_GAZEBO"
        sleep 5
        lanzar_terminal "Nav2 Bringup" "$CMD_NAV2"
        sleep 4
        lanzar_terminal "RViz" "$CMD_RVIZ"
        lanzar_terminal "Twist Mux" "$CMD_TWIST_MUX"

				echo
				echo "Todos los nodos fueron Lanzados"
				echo "Presiona ENTER para cerrar todas las terminales..."
				read

				pkill -f xterm
        ;;
    3)
        echo "Iniciando Misión Completa..."
        lanzar_terminal "Gazebo" "$CMD_GAZEBO"
        sleep 15
        
        # Nodos de Navegación
        lanzar_terminal "Nav2 Bringup" "$CMD_NAV2"
        sleep 5
        lanzar_terminal "RViz" "$CMD_RVIZ"
        
        # Nodos de Percepción y Aruco (Se excluye el send_goal manual)
        lanzar_terminal "Aruco Detector" "$CMD_ARUCO_DETECTOR"
        sleep 5
				lanzar_terminal "Aruco Lidar Approach" "$CMD_ARUCO_LIDAR"
        sleep 5
				lanzar_terminal "Aruco Approach" "$CMD_ARUCO_APPROACH"
        sleep 5
        
        # Twist Mux ejecutado una sola vez
        lanzar_terminal "Twist Mux" "$CMD_TWIST_MUX"
        sleep 5
        
        # Lanzamiento final de la Misión
        lanzar_terminal "Mission Node" "$CMD_MISSION"
        echo "¡Todos los nodos para la misión han sido lanzados!"

        echo
        echo "Todos los nodos fueron Lanzados"
        echo "Presiona ENTER para cerrar todas las terminales..."
        read
        ;;
    4)
        echo "Saliendo..."
        exit 0
        ;;
    *)
        echo "Opción no válida. Por favor ejecuta el script de nuevo y selecciona 1, 2, 3 o 4."
        ;;
esac
