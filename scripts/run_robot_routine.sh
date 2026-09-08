#!/usr/bin/env bash
# Controla los stacks del robot real dentro de myagv-robot.
# Los subcomandos de prueba no mueven el robot; los que pueden moverlo exigen
# una confirmacion explicita para que el operador mantenga el area despejada.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${CONTAINER:-myagv-robot}"
MAP="${MAP:-}"
MAP_DIR="${MAP_DIR:-${ROOT}/maps}"
PARAMS="${PARAMS:-/workspace/install/home_service_bringup/share/home_service_bringup/config/nav2_real.yaml}"
TWIST_MUX_PARAMS="${TWIST_MUX_PARAMS:-/workspace/install/home_service_bringup/share/home_service_bringup/config/twist_mux_real.yaml}"
# Perfil "mapa estable" para medir (no el del laberinto). El WS entero
# esta montado en /workspace, asi que se lee de src sin recompilar.
SLAM_PARAMS="${SLAM_PARAMS:-/workspace/src/home_service_bringup/config/slam_toolbox_mapping.yaml}"
SLAM_RVIZ="${SLAM_RVIZ:-/workspace/src/home_service_bringup/rviz/slam_real.rviz}"
# SLAM_REPORT_BLIND=1 -> el scan_sanitizer informa que sectores nunca
# devuelven eco (para calibrar el cono ciego del robot).
SLAM_REPORT_BLIND="${SLAM_REPORT_BLIND:-0}"
LOG_DIR="${LOG_DIR:-/workspace/log/robot_routine}"
# Visualizacion: pantalla fisica de la Jetson para rviz/rqt; puerto del
# puente Foxglove (se ve desde un portatil, sin GPU en la Jetson).
VIZ_DISPLAY="${VIZ_DISPLAY:-:0}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
# Reparto de nucleos (se calcula en cpu_budget). Vacio = automatico.
VIZ_CPUS="${VIZ_CPUS:-}"
SLAM_CPUS="${SLAM_CPUS:-}"
# Hilos del rasterizador software de Mesa. Por defecto llvmpipe abre un
# hilo por nucleo (4) y se salta el taskset del proceso padre en la
# practica: limitarlo es lo que evita que RViz invada el nucleo serie.
VIZ_GL_THREADS="${VIZ_GL_THREADS:-1}"
RVIZ_CONFIG="${RVIZ_CONFIG:-}"
RQT_PKGS="${RQT_PKGS:-ros-humble-rqt ros-humble-rqt-graph ros-humble-rqt-topic ros-humble-rqt-console ros-humble-rqt-reconfigure}"
DDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

# --- Modo distribuido (procesamiento en un portatil) ------------------
# DISTRIBUTED=1 + ROBOT_IP + LAPTOP_IP cambia el DDS de "solo loopback"
# a la interfaz real con PEERS UNICAST (el multicast sobre WiFi, y mas
# en un hotspot de movil, no es fiable), y hace que 'aruco' NO arranque
# el detector ni la aproximacion: esos corren en el portatil con
# scripts/tsummit_offboard.sh. La camara pasa a publicar solo JPEG
# (246.8 -> 6.1 Mbit/s medidos).
#
# La interfaz se fija por DIRECCION (la propia, ROBOT_IP), no con
# autodetermine: con tailscale0 y docker0 tambien presentes,
# autodetermine elegia otra interfaz y el portatil no veia NI UN topic
# del robot, sin ningun error. Verificado en la Jetson.
DISTRIBUTED="${DISTRIBUTED:-0}"
DDS_MULTICAST="${DDS_MULTICAST:-true}"
ROBOT_IP="${ROBOT_IP:-}"
LAPTOP_IP="${LAPTOP_IP:-}"

# IPv4 propia por la que sale el trafico (interfaz con ruta por defecto),
# o la de WIFI_IFACE si se indica.
own_ip() {
    if [ -n "${WIFI_IFACE:-}" ]; then
        ip -4 -o addr show "${WIFI_IFACE}" 2>/dev/null \
            | awk '{print $4}' | cut -d/ -f1 | head -1
    else
        ip -4 route get 1.1.1.1 2>/dev/null \
            | awk '{for (i = 1; i <= NF; i++) if ($i == "src") print $(i + 1)}' \
            | head -1
    fi
}

ip_for_peer() {
    local peer="$1"
    ip -4 route get "${peer}" 2>/dev/null \
        | awk '{for (i = 1; i <= NF; i++) if ($i == "src") print $(i + 1)}' \
        | head -1
}

has_local_ip() {
    ip -4 -o addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1 \
        | grep -qx "$1"
}

if [ "${DISTRIBUTED}" = "1" ]; then
    # ROBOT_IP se detecta solo si no se pasa. Aprendido a base de golpes:
    # el DHCP del hotspot cambia la IP entre arranques y, si CYCLONEDDS
    # apunta a una IP que ya no existe en ninguna interfaz, los nodos NO
    # arrancan y el error es cripticO ("rcl node's rmw handle is
    # invalid"), sin mencionar la red por ningun lado.
    if [ -z "${ROBOT_IP}" ]; then
        if [ -n "${LAPTOP_IP}" ]; then
            ROBOT_IP="$(ip_for_peer "${LAPTOP_IP}")"
        else
            ROBOT_IP="$(own_ip)"
        fi
        [ -n "${ROBOT_IP}" ] \
            && printf 'ROBOT_IP detectada: %s\n' "${ROBOT_IP}"
    fi
    if [ -z "${ROBOT_IP}" ] || [ -z "${LAPTOP_IP}" ]; then
        printf 'ERROR: DISTRIBUTED=1 exige LAPTOP_IP (y ROBOT_IP si no se detecta).\n' >&2
        exit 2
    fi
    if ! has_local_ip "${ROBOT_IP}"; then
        printf 'ERROR: ROBOT_IP=%s no esta en ninguna interfaz de esta maquina.\n' \
            "${ROBOT_IP}" >&2
        printf '       IPs actuales: %s\n' \
            "$(ip -4 -o addr show | awk '{print $4}' | cut -d/ -f1 | grep -v '^127' | tr '\n' ' ')" >&2
        printf '       (el DHCP del hotspot la habra cambiado; reserva una IP fija)\n' >&2
        exit 2
    fi
    # DDS_MULTICAST=true (por defecto): camino estandar de ROS 2. Poner
    # 'false' desactiva el multicast y deja solo los peers unicast; era
    # el default anterior, por la suposicion de que el multicast en WiFi
    # no es fiable. Esa suposicion resulto ser el principal sospechoso
    # de que el descubrimiento no funcionara.
    DDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"${ROBOT_IP}\"/></Interfaces><AllowMulticast>${DDS_MULTICAST}</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>32</MaxAutoParticipantIndex><Peers><Peer address=\"${ROBOT_IP}\"/><Peer address=\"${LAPTOP_IP}\"/></Peers></Discovery></Domain></CycloneDDS>"
fi

DOCKER=(docker)
if ! docker info >/dev/null 2>&1; then
    DOCKER=(sudo docker)
fi

source_env='source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash'

require_container() {
    if ! "${DOCKER[@]}" inspect -f '{{.State.Running}}' "${CONTAINER}" \
        2>/dev/null | grep -qx true; then
        printf 'ERROR: inicia el contenedor con ./docker/run_jetson_robot.sh sleep infinity\n' >&2
        exit 1
    fi
}

# rm de guardia: borra el log ANTES de arrancar el nodo que lo escribe.
#
# Por que existe. El 2026-09-06 la placa de la base entro en su bucle de
# "if you want restore run, pls input 1, then press enter" y
# myagv_odometry_node lo escupio 63 millones de veces: 1.97 GB en media
# hora, con la tarjeta al 94% de partida. Ese mensaje no esta limitado en
# ritmo, asi que escribe tan rapido como el disco aguante. Llenar la SD en
# mitad de un reto tumba la Jetson entera.
#
# Avisa del tamaño al borrar, porque un log gigante del arranque anterior
# es la unica pista de que la placa se desbocó: el nodo dice "myAGV
# initialized successful" igual, y reiniciarlo lo arregla, asi que sin
# este aviso el incidente pasa desapercibido.
#
# OJO: esto protege entre arranques, NO durante uno. Un desbocamiento
# dentro de la misma sesion crece sin freno igual.
guard_log() {
    local log="$1"

    local bytes
    bytes="$("${DOCKER[@]}" exec "${CONTAINER}" \
        stat -c %s "${log}" 2>/dev/null || echo 0)"

    if [ "${bytes:-0}" -gt 104857600 ]; then
        printf 'AVISO: %s ocupaba %s MB del arranque anterior.\n' \
            "${log}" "$(( bytes / 1048576 ))" >&2
        printf '       Un log asi es sintoma de un nodo desbocado (la placa\n' >&2
        printf '       de la base lo hace al perder el puerto serie). Se borra.\n' >&2
    fi

    "${DOCKER[@]}" exec "${CONTAINER}" rm -f "${log}" 2>/dev/null || true
}

run_bg() {
    local name="$1"
    local command="$2"

    guard_log "${LOG_DIR}/${name}.log"

    "${DOCKER[@]}" exec -d -e "CYCLONEDDS_URI=${DDS_URI}" "${CONTAINER}" bash -lc \
        "mkdir -p '${LOG_DIR}'; ${source_env}; ${command} >'${LOG_DIR}/${name}.log' 2>&1"
    printf '%s iniciado. Log: %s/%s.log\n' "${name}" "${LOG_DIR}" "${name}"
}

is_running() {
    "${DOCKER[@]}" exec "${CONTAINER}" pgrep -f "$1" >/dev/null 2>&1
}

node_visible() {
    local node_name="$1"
    "${DOCKER[@]}" exec -e "CYCLONEDDS_URI=${DDS_URI}" \
        "${CONTAINER}" bash -lc \
        "${source_env}; ros2 node list --no-daemon 2>/dev/null | grep -qx '${node_name}'"
}

node_running() {
    local process_pattern="$1" node_name="$2"
    is_running "${process_pattern}" && node_visible "${node_name}"
}

restart_stale() {
    local process_pattern="$1" node_name="$2"
    if is_running "${process_pattern}" && ! node_visible "${node_name}"; then
        printf 'Proceso de %s invisible en el DDS actual: reiniciando.\n' \
            "${node_name}"
        "${DOCKER[@]}" exec "${CONTAINER}" bash -lc \
            "pkill -KILL -f '${process_pattern}' || true"
        sleep 1
    fi
}

action_visible() {
    local action_name="$1"
    "${DOCKER[@]}" exec -e "CYCLONEDDS_URI=${DDS_URI}" \
        "${CONTAINER}" bash -lc \
        "${source_env}; ros2 action list 2>/dev/null | grep -qx '${action_name}'"
}

cpu_budget() {
    # Reparte los nucleos de la Nano en tres zonas que NO se solapan.
    # run_all_robot_nodes.sh ya reserva el ULTIMO nucleo para el laser y
    # la odometria con taskset + chrt -r. El fallo historico era que
    # run_gui lanzaba RViz SIN taskset: sus hilos (incluidos los de
    # llvmpipe, uno por nucleo) caian sobre ese nucleo reservado y
    # preemptaban el hilo serie del X2 -> "Checksum error" -> scans
    # corruptos -> paredes dobles. De ahi venia "RViz arruina el mapeo".
    #
    #   nucleo 3    laser + odometria   (tiempo real, lo fija el bringup)
    #   nucleo 1-2  slam_toolbox + scan_sanitizer
    #   nucleo 0    RViz / RQt
    #
    # Con esto RViz ya no compite con el mapeo: como mucho se ve a menos
    # fps, que es exactamente lo que queremos que sufra.
    if [ -n "${VIZ_CPUS}" ] && [ -n "${SLAM_CPUS}" ]; then
        return
    fi
    local ncpu
    ncpu="$("${DOCKER[@]}" exec "${CONTAINER}" nproc 2>/dev/null || echo 4)"
    if [ "${ncpu}" -ge 4 ]; then
        VIZ_CPUS="${VIZ_CPUS:-0}"
        SLAM_CPUS="${SLAM_CPUS:-1-$(( ncpu - 2 ))}"
    elif [ "${ncpu}" -eq 3 ]; then
        VIZ_CPUS="${VIZ_CPUS:-0}"
        SLAM_CPUS="${SLAM_CPUS:-0-1}"
    else
        VIZ_CPUS="${VIZ_CPUS:-0}"
        SLAM_CPUS="${SLAM_CPUS:-0}"
    fi
}

select_map() {
    local -a maps
    local choice

    shopt -s nullglob
    maps=("${MAP_DIR}"/*.yaml)
    shopt -u nullglob

    if [ "${#maps[@]}" -eq 0 ]; then
        printf 'ERROR: no hay mapas .yaml en %s\n' "${MAP_DIR}" >&2
        exit 1
    fi

    if [ -n "${MAP}" ]; then
        return
    fi

    if [ ! -t 0 ]; then
        printf 'ERROR: selecciona un mapa con MAP=/workspace/maps/<archivo>.yaml\n' >&2
        exit 2
    fi

    printf 'Mapas disponibles:\n'
    for choice in "${!maps[@]}"; do
        printf '  %d) %s\n' "$((choice + 1))" "$(basename "${maps[choice]}")"
    done

    while true; do
        read -r -p 'Selecciona un mapa: ' choice
        if [[ "${choice}" =~ ^[0-9]+$ ]] \
            && [ "${choice}" -ge 1 ] \
            && [ "${choice}" -le "${#maps[@]}" ]; then
            MAP="/workspace/maps/$(basename "${maps[choice - 1]}")"
            printf 'Mapa seleccionado: %s\n' "${MAP}"
            return
        fi
        printf 'Seleccion no valida.\n' >&2
    done
}

list_maps() {
    local map_file

    shopt -s nullglob
    for map_file in "${MAP_DIR}"/*.yaml; do
        printf '%s\n' "$(basename "${map_file}")"
    done
    shopt -u nullglob
}

stop() {
    "${DOCKER[@]}" exec "${CONTAINER}" bash -lc \
        "pkill -INT -f '[r]os2 launch' || true; \
         pkill -TERM -f '[r]un_all_robot_nodes.sh' || true; \
         sleep 2; \
         pkill -KILL -f '[r]os2 launch' || true; \
         pkill -KILL -f '[r]un_all_robot_nodes.sh' || true; \
         pkill -KILL -f '[m]yagv_odometry_node' || true; \
         pkill -KILL -f '[y]dlidar_ros2_driver_node' || true; \
         pkill -KILL -f '[s]tatic_transform_publisher' || true; \
         pkill -KILL -f '[b]luetooth_gamepad_teleop' || true; \
         pkill -KILL -f '[c]si_camera_node' || true; \
         pkill -KILL -f '[a]ruco_detector_node' || true; \
         pkill -KILL -f '[a]ruco_lidar_approach_server' || true; \
         pkill -KILL -f '[s]can_sanitizer_node' || true; \
         pkill -KILL -f '[t]wist_mux' || true; \
         pkill -KILL -f '[m]ap_server' || true; pkill -KILL -f '[a]mcl' || true; \
         pkill -KILL -f '[c]ontroller_server' || true; pkill -KILL -f '[p]lanner_server' || true; \
         pkill -KILL -f '[b]t_navigator' || true; pkill -KILL -f '[l]ifecycle_manager' || true; \
         pkill -KILL -f '[f]oxglove_bridge' || true; pkill -KILL -f '[r]viz2' || true; \
         pkill -KILL -f '[r]qt' || true; pkill -KILL -f '[s]lam_toolbox' || true; \
          pkill -KILL -f '[r]obot_state_publisher' || true"
    # Una parada explicita invalida la marca que permite a grasp saltarse la
    # preparacion; la siguiente mision debe ejecutar `tsummit.sh prepare`.
    "${DOCKER[@]}" exec "${CONTAINER}" rm -f \
        /workspace/log/robot_routine/tsummit-grasp-ready \
        /workspace/log/robot_routine/tsummit-approach-ready \
        >/dev/null 2>&1 || true
    printf 'Stacks detenidos.\n'
}

status() {
    "${DOCKER[@]}" exec -e "CYCLONEDDS_URI=${DDS_URI}" "${CONTAINER}" bash -lc \
        "${source_env}; printf '%s\\n' '--- nodes DDS actual ---'; ros2 node list --no-daemon; \
         printf '%s\\n' '--- actions ---'; ros2 action list; \
          printf '%s\\n' '--- velocity publishers ---'; ros2 topic info /cmd_vel -v || true"
}

mux() {
    # twist_mux arbitra /cmd_vel_joy (200) /cmd_vel_aruco (100) /cmd_vel_nav (50)
    # hacia /cmd_vel, que es lo unico que consume myagv_odometry. Sin el,
    # el mando, ArUco y Nav2 publican al vacio y la base no se mueve.
    # OJO: el patron de is_running debe ser mas especifico que solo
    # "twist_mux". aruco() lanza robot.launch.py con el argumento
    # 'start_twist_mux:=false', y esa cadena TAMBIEN hace match con
    # pgrep -f '[t]wist_mux' -> falso positivo -> mux() se cree que ya
    # esta arrancado y nunca lo lanza -> /cmd_vel no existe nunca.
    # 'ros2 run twist_mux twist_mux' es la invocacion real y no aparece
    # en ningun argumento de otro launch.
    restart_stale '[r]os2 run twist_mux twist_mux' '/twist_mux'
    if node_running '[r]os2 run twist_mux twist_mux' '/twist_mux'; then
        # Un mux lanzado con otra CYCLONEDDS_URI puede seguir vivo pero ser
        # invisible para esta invocacion. En ese caso el teleop publica al
        # vacio: comprueba que el nodo pertenece al grafo DDS actual antes
        # de reutilizar el proceso.
        if "${DOCKER[@]}" exec -e "CYCLONEDDS_URI=${DDS_URI}" \
            "${CONTAINER}" bash -lc \
            "${source_env}; ros2 node list 2>/dev/null | grep -qx '/twist_mux'";
        then
            printf 'twist_mux ya iniciado.\n'
            return
        fi
        printf 'twist_mux antiguo o invisible en el DDS actual: reiniciando.\n'
        "${DOCKER[@]}" exec "${CONTAINER}" bash -lc \
            "pkill -KILL -f '[t]wist_mux' || true"
        sleep 1
    fi
    run_bg twist_mux \
        "ros2 run twist_mux twist_mux --ros-args --params-file '${TWIST_MUX_PARAMS}' -r cmd_vel_out:=/cmd_vel"
}

model() {
    # robot_state_publisher con el URDF del myAGV (raiz base_footprint,
    # encaja con la TF odom->base_footprint). Aporta /robot_description
    # y base_footprint->base_link, para ver el modelo en RViz/Foxglove.
    restart_stale '[r]obot_state_publisher' '/robot_state_publisher'
    if node_running '[r]obot_state_publisher' '/robot_state_publisher'; then
        return
    fi
    run_bg model \
        "ros2 run robot_state_publisher robot_state_publisher \
         /workspace/install/myagv_description/share/myagv_description/urdf/myAGV.urdf"
}

base() {
    restart_stale '[m]yagv_odometry_node' '/myagv_odometry_node'
    if node_running '[m]yagv_odometry_node' '/myagv_odometry_node'; then
        printf 'Base ya iniciada.\n'
    else
        run_bg base 'START_BRINGUP=0 bash /workspace/docker/run_all_robot_nodes.sh'
    fi
    model
    mux
}

teleop() {
    # El mando publica a /cmd_vel_joy, pero la base es quien consume la
    # salida final /cmd_vel. Hacer teleop autosuficiente evita que el mando
    # funcione en ROS y el robot permanezca inmovil por falta de suscriptor.
    base
    restart_stale '[b]luetooth_gamepad_teleop' '/bluetooth_gamepad_teleop'
    if node_running '[b]luetooth_gamepad_teleop' '/bluetooth_gamepad_teleop'; then
        printf 'Teleop ya iniciado.\n'
        return
    fi
    run_bg teleop 'ros2 launch myagv_teleop_joy bluetooth_gamepad_teleop.launch.py cmd_vel_topic:=/cmd_vel_joy'
}

aruco() {
    # En distribuido el detector vive en el portatil y AQUI no corre
    # nunca, asi que vigilar 'aruco_detector_node' daria siempre falso
    # y cada llamada lanzaria otra camara encima de la anterior.
    local guard='[a]ruco_detector_node'
    local guard_node='/aruco_detector_node'
    if [ "${DISTRIBUTED}" = "1" ]; then
        guard='[c]si_camera_node'
        guard_node='/csi_camera_node'
    fi
    restart_stale "${guard}" "${guard_node}"
    if node_running "${guard}" "${guard_node}"; then
        printf 'Pila ArUco ya iniciada.\n'
        return
    fi
    # La camara y el detector fuera del nucleo serie (el ultimo), que
    # 'base' reserva con chrt -r para el laser y la odometria. Sin
    # taskset, aruco_detector (>150% CPU) aterrizaba en ese nucleo y
    # ahogaba la lectura del laser -> Checksum error + deteccion a
    # 0.1 Hz -> la aproximacion gira sin ver el marcador.
    cpu_budget
    # start_twist_mux:=false -> el mux lo gestiona este script (mux()), asi
    # nunca hay dos twist_mux peleando por /cmd_vel.
    local extra=""
    if [ "${DISTRIBUTED}" = "1" ]; then
        # El detector y la aproximacion viven en el portatil.
        # La camara deja de publicar la imagen cruda: por WiFi solo va
        # el JPEG, y publicar ambas seria gastar CPU para nada.
        extra="start_aruco_detector:=false start_aruco_approach:=false \
               camera_publish_raw:=false camera_publish_compressed:=true \
               camera_framerate:=${CAMERA_FRAMERATE:-15} \
               camera_exposure_time_us:=${CAMERA_EXPOSURE_TIME_US:-0} \
               camera_gain:=${CAMERA_GAIN:-0.0}"
        printf 'MODO DISTRIBUIDO: detector y aproximacion NO se lanzan aqui.\n'
        printf '  Arrancalos en el portatil con:\n'
        printf '    ROBOT_IP=%s LAPTOP_IP=%s ./scripts/tsummit_offboard.sh run\n' \
            "${ROBOT_IP}" "${LAPTOP_IP}"
    fi
    # Montaje de la camara. Se pasa por entorno porque robot.launch.py
    # solo existe DENTRO del contenedor (el host de la Jetson tiene
    # Galactic y otro workspace): lanzarlo a mano desde el host falla
    # con "package 'home_service_bringup' not found".
    local cam_tf=""
    cam_tf="camera_x:=${CAMERA_X:-0.16} camera_y:=${CAMERA_Y:-0.0} \
            camera_z:=${CAMERA_Z:-0.07} camera_roll:=${CAMERA_ROLL:-0.0} \
            camera_pitch:=${CAMERA_PITCH:-0.0} camera_yaw:=${CAMERA_YAW:-0.0}"
    printf 'Camara en base_link: x=%s y=%s z=%s  rpy=%s/%s/%s\n' \
        "${CAMERA_X:-0.16}" "${CAMERA_Y:-0.0}" "${CAMERA_Z:-0.07}" \
        "${CAMERA_ROLL:-0.0}" "${CAMERA_PITCH:-0.0}" "${CAMERA_YAW:-0.0}"

    run_bg aruco "taskset -c ${SLAM_CPUS} ros2 launch home_service_bringup \
        robot.launch.py use_sim_time:=false start_arm:=false start_twist_mux:=false \
        ${cam_tf} ${extra}"
    mux
}

nav2() {
    mux
    restart_stale '[c]ontroller_server' '/controller_server'
    if node_running '[c]ontroller_server' '/controller_server'; then
        printf 'Nav2 ya iniciado.\n'
        return
    fi
    select_map
    run_bg nav2 "ros2 launch home_service_bringup nav2.launch.py map:='${MAP}' params_file:='${PARAMS}' autostart:=true"
}

slam() {
    # Mapeo SLAM en vivo, SIN Nav2 ni la mision del laberinto: se conduce
    # a mano con 'teleop'. scan_sanitizer limpia /scan -> /scan_filtered
    # (paredes fantasma) y slam_toolbox construye el mapa y publica
    # map->odom. Mismos nodos y parametros que maze.launch.py con
    # run_maze_runner:=false, pero sin arrancar el stack de navegacion.
    mux
    # slam_toolbox y el saneador FUERA del nucleo serie (el ultimo), que
    # 'base' reserva para el laser y la odometria, y fuera tambien del
    # nucleo de RViz. Asi Ceres nunca interrumpe la lectura del puerto
    # -> no hay scans corruptos.
    cpu_budget
    local other_cpus="${SLAM_CPUS}"
    restart_stale '[s]can_sanitizer_node' '/scan_sanitizer_node'
    if ! node_running '[s]can_sanitizer_node' '/scan_sanitizer_node'; then
        local san="taskset -c ${other_cpus} ros2 run home_service_navigation scan_sanitizer_node"
        if [ "${SLAM_REPORT_BLIND}" = "1" ]; then
            san="${san} --ros-args -p report_blind_sectors:=true"
        fi
        run_bg scan_sanitizer "${san}"
        sleep 2
    fi
    restart_stale '[a]sync_slam_toolbox_node' '/slam_toolbox'
    if node_running '[a]sync_slam_toolbox_node' '/slam_toolbox'; then
        printf 'SLAM ya iniciado.\n'
        return
    fi
    # slam_toolbox (Ceres) es CPU-bound: nice +10 y fuera del nucleo serie.
    run_bg slam \
        "taskset -c ${other_cpus} nice -n 10 ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file '${SLAM_PARAMS}'"
    printf 'Perfil: %s (mapeo en nucleo(s) %s)\n' \
        "$(basename "${SLAM_PARAMS}")" "${SLAM_CPUS}"
    if node_running '[r]qt' '/rqt_gui'; then
        printf 'AVISO: RQt no esta confinado como RViz y compite con el mapeo.\n'
        printf '       Cierralo mientras mapeas (run_robot_routine.sh stop no lo salva).\n'
    fi
    printf 'Ver el mapa en vivo:           ./scripts/run_robot_routine.sh rviz\n'
    printf 'Conduce DESPACIO y en BUCLES:  ./scripts/run_robot_routine.sh teleop\n'
    printf 'Guarda el mapa:                ./scripts/run_robot_routine.sh save-map <nombre>\n'
    printf 'Para MEDIR: guarda y trabaja sobre el .pgm; el mapa vivo no esta congelado.\n'
}

mapping() {
    # Secuencia completa y ordenada de mapeo. El orden importa:
    # la base publica odom->base_footprint ANTES de que slam_toolbox
    # arranque (si no, slam descarta scans por falta de TF), y RViz se
    # abre al final para que su primer frame ya tenga mapa que dibujar.
    base
    printf 'Esperando a la base (odom + laser)...\n'
    sleep 8
    slam
    sleep 4
    rviz
    sleep 2
    teleop
    printf '\n--- Listo para mapear ---\n'
    printf 'Conduce DESPACIO y cierra BUCLES: volver a pasar por un sitio ya\n'
    printf 'visitado es lo unico que elimina las paredes dobles.\n'
    printf 'Al terminar: ./scripts/run_robot_routine.sh save-map <nombre>\n'
}

save_map() {
    local name="${1:?uso: save-map <nombre> (se guarda en /workspace/maps)}"
    if ! node_running '[a]sync_slam_toolbox_node' '/slam_toolbox'; then
        printf 'ERROR: no hay SLAM en marcha; nadie publica /map.\n' >&2
        exit 1
    fi
    "${DOCKER[@]}" exec -e "CYCLONEDDS_URI=${DDS_URI}" "${CONTAINER}" bash -lc \
        "${source_env}; mkdir -p /workspace/maps; cd /workspace/maps; \
         ros2 run nav2_map_server map_saver_cli -f '${name}' \
             --ros-args -p save_map_timeout:=10.0"
    printf 'Mapa guardado: /workspace/maps/%s.{yaml,pgm}\n' "${name}"
    printf 'Uso con Nav2:  MAP=/workspace/maps/%s.yaml ./scripts/run_robot_routine.sh nav2\n' "${name}"
}

all() {
    base
    sleep 3
    aruco
    sleep 3
    teleop
    sleep 3
    nav2
}

# --- Visualizacion en vivo ------------------------------------------

ensure_pkg() {
    # Instala paquetes apt dentro del contenedor si el binario/prueba falla.
    # La imagen NO se reconstruye (disco de la Jetson casi lleno); para que
    # persista tras recrear el contenedor: docker commit myagv-robot <imagen>.
    local test_cmd="$1"; shift
    if "${DOCKER[@]}" exec "${CONTAINER}" bash -lc \
        "source /opt/ros/humble/setup.bash 2>/dev/null; ${test_cmd}" >/dev/null 2>&1; then
        return
    fi
    printf 'Instalando en el contenedor (una vez): %s\n' "$*" >&2
    if ! "${DOCKER[@]}" exec -e DEBIAN_FRONTEND=noninteractive "${CONTAINER}" bash -lc \
        "apt-get update -qq && apt-get install -y --no-install-recommends $*" >&2; then
        printf 'ERROR: fallo la instalacion (revisa el espacio en disco: df -h /).\n' >&2
        exit 1
    fi
}

run_gui() {
    # Lanza una GUI del contenedor contra la pantalla fisica de la Jetson.
    # GL por software: la Jetson Nano no expone la GPU dentro del contenedor.
    #
    # Confinamiento (ver cpu_budget): la GUI va fijada a VIZ_CPUS y
    # llvmpipe limitado a VIZ_GL_THREADS hilos, para que jamas toque el
    # nucleo de tiempo real del laser. Sin esto RViz metia scans
    # corruptos en el mapa.
    local name="$1" command="$2"
    cpu_budget
    guard_log "${LOG_DIR}/${name}.log"
    xhost "+SI:localuser:root" >/dev/null 2>&1 || true
    "${DOCKER[@]}" exec -d \
        -e "DISPLAY=${VIZ_DISPLAY}" \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e GALLIUM_DRIVER=llvmpipe \
        -e "LP_NUM_THREADS=${VIZ_GL_THREADS}" \
        -e "OGRE_NUM_THREADS=${VIZ_GL_THREADS}" \
        -e QT_X11_NO_MITSHM=1 \
        -e "CYCLONEDDS_URI=${DDS_URI}" \
        "${CONTAINER}" bash -lc \
        "mkdir -p '${LOG_DIR}'; ${source_env}; \
         taskset -c '${VIZ_CPUS}' nice -n 15 ${command} \
             >'${LOG_DIR}/${name}.log' 2>&1"
    printf '%s abierto en la pantalla de la Jetson (DISPLAY=%s). Log: %s/%s.log\n' \
        "${name}" "${VIZ_DISPLAY}" "${LOG_DIR}" "${name}"
    printf 'CPU: %s en nucleo(s) %s (%s hilo(s) GL); mapeo en %s; laser aislado.\n' \
        "${name}" "${VIZ_CPUS}" "${VIZ_GL_THREADS}" "${SLAM_CPUS}"
}

foxglove() {
    ensure_pkg 'ros2 pkg prefix foxglove_bridge' ros-humble-foxglove-bridge
    restart_stale '[f]oxglove_bridge' '/foxglove_bridge'
    if node_running '[f]oxglove_bridge' '/foxglove_bridge'; then
        printf 'foxglove_bridge ya iniciado (puerto %s).\n' "${FOXGLOVE_PORT}"
    else
        run_bg foxglove \
            "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=${FOXGLOVE_PORT}"
    fi
    # OJO: NO usar 'hostname -I | awk {print $1}'. Esta maquina tiene
    # eth0, wlan0, tailscale0 y docker0; la primera que devuelve es la
    # del CABLE, y desde el portatil por WiFi esa direccion no responde.
    # El sintoma es "Check that the WebSocket server at ws://... is
    # reachable", que suena a que el bridge no arranco cuando en
    # realidad esta anunciando la interfaz equivocada.
    local ip
    ip="${ROBOT_IP:-$(own_ip)}"
    printf 'Abre Foxglove (app de escritorio o https://app.foxglove.dev) y conecta a:\n'
    printf '  ws://%s:%s\n' "${ip:-<ip-de-la-jetson>}" "${FOXGLOVE_PORT}"
    if [ -n "${ip}" ]; then
        printf 'Otras direcciones de esta maquina (por si conectas desde otra red):\n'
        ip -4 -o addr show scope global 2>/dev/null \
            | awk -v cur="${ip}" '{split($4,a,"/"); \
                 printf "  %-16s %s%s\n", a[1], $2, (a[1]==cur ? "   <- la anunciada" : "")}'
    fi
    printf 'Paneles utiles: 3D (mapa, /scan, TF, costmaps), Image (/camera), Raw Messages.\n'
}

rviz() {
    ensure_pkg 'command -v rviz2' ros-humble-rviz2
    local cfg="${RVIZ_CONFIG}"
    if [ -z "${cfg}" ] \
        && "${DOCKER[@]}" exec "${CONTAINER}" test -f "${SLAM_RVIZ}"; then
        cfg="${SLAM_RVIZ}"   # config de SLAM (scan Best Effort, frame map)
    fi
    local cmd='rviz2'
    [ -n "${cfg}" ] && cmd="rviz2 -d '${cfg}'"
    restart_stale '[r]viz2' '/rviz2'
    if node_running '[r]viz2' '/rviz2'; then
        printf 'RViz ya iniciado.\n'
        return
    fi
    run_gui rviz "${cmd}"
}

rqt() {
    ensure_pkg 'command -v rqt' ${RQT_PKGS}
    restart_stale '[r]qt' '/rqt_gui'
    if node_running '[r]qt' '/rqt_gui'; then
        printf 'RQt ya iniciado.\n'
        return
    fi
    run_gui rqt 'rqt'
}

check() {
    # OJO con dos trampas, las dos vividas el 2026-09-06:
    #
    # 1. Sin CYCLONEDDS_URI, CycloneDDS se ata a lo (que no admite
    #    multicast) y NO VE NADA. Esta funcion no lo pasaba mientras que
    #    run_bg y aruco_goal si, asi que en modo distribuido informaba de
    #    una lista de nodos vacia con la pila entera funcionando. Dos
    #    sesiones distintas dieron la pila por caida estando sana.
    #
    # 2. El daemon de ros2 cachea el grafo. Ha llegado a devolver 2
    #    topicos habiendo 26. --no-daemon lo evita, pero solo vale para
    #    `topic list` y `node list`: en `action list` no existe.
    #
    # Ademas de los nodos se comprueba la TF, porque los dos fallos de la
    # prueba de aproximacion de ese dia fueron precondiciones, no control:
    # sin `odom` (placa de la base caida) el servidor navega contra una
    # estimacion congelada, y sin `laser_frame` la llegada no se declara
    # nunca.
    "${DOCKER[@]}" exec -e "CYCLONEDDS_URI=${DDS_URI}" "${CONTAINER}" bash -lc \
        "${source_env}; \
         ros2 pkg prefix myagv_teleop_joy >/dev/null; \
         ros2 pkg prefix home_service_behaviors >/dev/null; \
         ros2 pkg prefix home_service_bringup >/dev/null; \
         echo '=== acciones ==='; ros2 action list; \
         echo '=== nodos DDS actual ==='; ros2 node list --no-daemon; \
         echo '=== TF imprescindibles ==='; \
         python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

rclpy.init()
n = Node('check_tf')
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, n)

t = time.time()
while time.time() - t < 5.0:
    rclpy.spin_once(n, timeout_sec=0.1)

falta = []
for padre, hijo, pista in (
    ('odom', 'base_link', 'placa de la base caida: reinicia SOLO myagv_odometry_node'),
    ('base_link', 'laser_frame', 'sin LiDAR: la llegada no se declara nunca'),
):
    try:
        buf.lookup_transform(padre, hijo, Time())
        print('  OK    %s -> %s' % (padre, hijo))
    except Exception:
        print('  FALTA %s -> %s   (%s)' % (padre, hijo, pista))
        falta.append(hijo)

if falta:
    print('  NO LANCES la aproximacion hasta que esas TF existan.')

rclpy.shutdown()
PY
         echo '=== /cmd_vel ==='; \
         timeout 3 ros2 topic echo --once /cmd_vel || true"
}

confirm_motion() {
    if [ "${ALLOW_MOTION:-0}" != "1" ]; then
        printf 'ERROR: esta accion puede mover el robot. Repite con ALLOW_MOTION=1 y area despejada.\n' >&2
        exit 2
    fi
}

aruco_goal() {
    confirm_motion
    local marker_id="${1:?uso: aruco-goal <id> [stop_distance_m] [timeout_s]}"
    local stop_dist="${2:-${STOP_DISTANCE:-0.20}}"
    # 30 s no daban ni para una vuelta de busqueda paso-y-mira: cada
    # paso son ~1.15 s y hacen falta bastantes para barrer 360 grados.
    local timeout_s="${3:-${APPROACH_TIMEOUT:-180.0}}"
    action_visible '/aruco_lidar_approach' \
        || { printf 'ERROR: /aruco_lidar_approach no es visible en el DDS actual.\n' >&2; exit 1; }
    "${DOCKER[@]}" exec -i -e "CYCLONEDDS_URI=${DDS_URI}" "${CONTAINER}" bash -lc \
        "${source_env}; ros2 action send_goal /aruco_lidar_approach \
         home_service_interfaces/action/ArucoApproach \
         '{target_id: ${marker_id}, stop_distance: ${stop_dist}, timeout_sec: ${timeout_s}}' --feedback"
}

nav2_goal() {
    confirm_motion
    local x="${1:?uso: nav2-goal <x> <y> [yaw_deg]}"
    local y="${2:?uso: nav2-goal <x> <y> [yaw_deg]}"
    local yaw="${3:-0.0}"
    action_visible '/navigate_to_pose' \
        || { printf 'ERROR: /navigate_to_pose no es visible en el DDS actual.\n' >&2; exit 1; }
    "${DOCKER[@]}" exec -i -e "CYCLONEDDS_URI=${DDS_URI}" "${CONTAINER}" bash -lc "${source_env}; python3 - <<PY
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
rclpy.init()
node = Node('routine_nav2_goal')
client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
if not client.wait_for_server(timeout_sec=15.0):
    raise SystemExit('Nav2 no disponible')
goal = NavigateToPose.Goal()
goal.pose = PoseStamped()
goal.pose.header.frame_id = 'map'
goal.pose.header.stamp = node.get_clock().now().to_msg()
goal.pose.pose.position.x = float(${x})
goal.pose.pose.position.y = float(${y})
yaw = math.radians(${yaw})
goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future)
if future.result() is None or not future.result().accepted:
    raise SystemExit('Objetivo rechazado')
result = future.result().get_result_async()
rclpy.spin_until_future_complete(node, result)
print('Nav2 status:', result.result().status)
PY"
}

initial_pose() {
    local x="${1:?uso: initial-pose <x> <y> [yaw_deg]}"
    local y="${2:?uso: initial-pose <x> <y> [yaw_deg]}"
    local yaw="${3:-0.0}"
    "${DOCKER[@]}" exec -e "CYCLONEDDS_URI=${DDS_URI}" "${CONTAINER}" \
        bash -lc "${source_env}; python3 - <<PY
import math
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init()
node = rclpy.create_node('routine_initial_pose')
pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
message = PoseWithCovarianceStamped()
message.header.frame_id = 'map'
message.header.stamp = node.get_clock().now().to_msg()
message.pose.pose.position.x = float(${x})
message.pose.pose.position.y = float(${y})
yaw = math.radians(${yaw})
message.pose.pose.orientation.z = math.sin(yaw / 2.0)
message.pose.pose.orientation.w = math.cos(yaw / 2.0)
message.pose.covariance[0] = 0.25
message.pose.covariance[7] = 0.25
message.pose.covariance[35] = 0.0685
for _ in range(3):
    pub.publish(message)
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
PY"
    printf 'Pose inicial publicada: x=%s y=%s yaw=%s grados.\n' "$x" "$y" "$yaw"
}

require_container
case "${1:-help}" in
    base) base ;;
    mux) mux ;;
    model) model ;;
    teleop) teleop ;;
    aruco) aruco ;;
    nav2) nav2 ;;
    slam) slam ;;
    mapping|mapear) mapping ;;
    save-map) shift; save_map "$@" ;;
    foxglove|viz) foxglove ;;
    rviz) rviz ;;
    rqt) rqt ;;
    maps) list_maps ;;
    all) all ;;
    check) check ;;
    status) status ;;
    stop) stop ;;
    initial-pose) shift; initial_pose "$@" ;;
    aruco-goal) shift; aruco_goal "$@" ;;
    nav2-goal) shift; nav2_goal "$@" ;;
    help|-h|--help)
        printf '%s\n' \
            'Uso: ./scripts/run_robot_routine.sh <comando>' '' \
            '  base                 base, LiDAR, modelo (robot_state_publisher) y twist_mux' \
            '  mux                  solo twist_mux: /cmd_vel_joy|aruco|nav -> /cmd_vel' \
            '  model                solo robot_state_publisher (/robot_description + TF del URDF)' \
            '  teleop               base + mando Bluetooth -> /cmd_vel_joy' \
            '  aruco                camara, detector y aproximacion (sin brazo)' \
            '  nav2                 AMCL + Nav2 con el mapa configurado' \
            '  slam                 mapeo SLAM en vivo (scan_sanitizer + slam_toolbox), sin Nav2' \
            '  mapping | mapear     TODO el mapeo en orden: base -> slam -> rviz -> teleop' \
            '  save-map <nombre>    guarda el mapa de SLAM en /workspace/maps/<nombre>.{yaml,pgm}' \
            '  foxglove | viz       puente Foxglove (ws://<jetson>:8765); se ve desde un portatil' \
            '  rviz                 RViz2 en la pantalla de la Jetson (RVIZ_CONFIG=<ruta.rviz>)' \
            '  rqt                  RQt en la pantalla de la Jetson (instala ~100 MB la 1a vez)' \
            '  maps                 lista los mapas seleccionables' \
            '  all                  inicia base, ArUco, teleop y Nav2' \
            '  check                valida nodos, acciones y /cmd_vel sin mover' \
            '  status               muestra nodos, acciones y publicadores' \
            '  initial-pose <x> <y> [yaw_deg]  inicializa AMCL sin mover' \
            '  aruco-goal <id>      aproxima al marcador; exige ALLOW_MOTION=1' \
            '  nav2-goal <x> <y> [yaw_deg]  navega; exige ALLOW_MOTION=1' \
            '  stop                 parada de todos los stacks' \
            '' \
            'Nav2: ./scripts/run_robot_routine.sh nav2 abre un menu.' \
            '      MAP=/workspace/maps/archivo.yaml ./scripts/run_robot_routine.sh nav2 evita el menu.' \
            '' \
            'MAPEAR (via oficial):  ./scripts/run_robot_routine.sh mapping' \
            '  Abre RViz en la pantalla de la Jetson con rviz/slam_real.rviz.' \
            '  Reparto de nucleos (4 en la Nano), sin solapes:' \
            '    nucleo 3    laser + odometria  (chrt -r, lo fija el bringup)' \
            '    nucleo 1-2  slam_toolbox + scan_sanitizer' \
            '    nucleo 0    RViz  (taskset + LP_NUM_THREADS=1 + 10 fps)' \
            '  Antes RViz iba SIN taskset y pisaba el nucleo del laser: de ahi los' \
            '  "Checksum error" y las paredes dobles. Ya no compite con el mapeo.' \
            '  Ajustes: VIZ_CPUS=, SLAM_CPUS=, VIZ_GL_THREADS= si cambias de placa.' \
            '  Conduce DESPACIO y en BUCLES; el cierre de bucle es lo que quita paredes dobles.' \
            '  save-map <n> cuando cuadre; para MEDIR trabaja sobre el .pgm, no el mapa vivo.' \
            '  NO abras RQt mientras mapeas (ese si va suelto). Perfil de SLAM:' \
            '  slam_toolbox_mapping.yaml.' \
            'La Jetson tiene ROS Galactic en el host: incompatible con el stack Humble del' \
            'contenedor, por eso rviz/rqt del host no ven nada. Usa estos subcomandos.'
        ;;
    *) printf 'Comando desconocido: %s\n' "$1" >&2; exit 2 ;;
esac
