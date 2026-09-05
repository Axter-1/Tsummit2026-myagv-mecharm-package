#!/usr/bin/env bash
# =====================================================================
#  Reto 4 — LABERINTO. Lanzador de pruebas.
# ---------------------------------------------------------------------
#  Todo el stack ROS 2 vive dentro del contenedor Docker
#  "myagv-robot" (imagen myagv-home-service:jetson-robot, Humble).
#  Este script se ejecuta en el sistema NATIVO de la Jetson y habla con
#  el contenedor por 'docker exec', que es la via fiable: el host tiene
#  ROS 2 Galactic y el contenedor Humble, y no conviene depender de la
#  interoperabilidad DDS entre distros para operar el robot.
#
#  Para inspeccionar topics desde una terminal NATIVA (RViz, ros2 topic
#  echo, ...) usa:   ./scripts/run_maze.sh env
#
#  ORDEN DE ARRANQUE RECOMENDADO
#  =============================
#    1)  START_BRINGUP=0 ./docker/run_jetson_robot.sh
#        (base + LiDAR; sin camara/ArUco/brazo para dejar CPU libre)
#
#    2)  ./scripts/run_maze.sh run
#
#  SUBCOMANDOS
#  ===========
#    run      Lanza el laberinto completo (SLAM + Nav2 + maze_runner)
#    manual   Igual pero sin enviar el objetivo (auto_start:=false)
#    diag     Diagnostico de sectores ciegos del LiDAR (no mueve nada)
#    start    Dispara el recorrido si se lanzo con 'manual'
#    goal     Envia un objetivo suelto:  goal <x> <y> [yaw_deg]
#    status   Muestra /maze/status en vivo
#    scan     Compara /scan y /scan_filtered (Hz)
#    savemap  Guarda el mapa construido por SLAM
#    rviz     Abre RViz2 dentro del contenedor (requiere DISPLAY)
#    stop     Detiene el stack del laberinto
#    env      Imprime los exports para hablar con el contenedor desde
#             una terminal nativa
#    shell    Abre una shell con el entorno ROS ya cargado
# =====================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONTAINER="${CONTAINER:-myagv-robot}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
RMW="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# Objetivo FINISH por defecto, relativo a START (origen de "map" con SLAM).
GOAL_X="${GOAL_X:-3.7}"
GOAL_Y="${GOAL_Y:--2.2}"
GOAL_YAW="${GOAL_YAW:-0.0}"

# SLAM en vivo (por defecto) o AMCL contra un mapa ya guardado. Con un
# mapa guardado del MISMO punto de partida, START sigue siendo (0,0) en
# el frame "map" (asi se genero al mapear), asi que GOAL_X/Y no cambian.
SLAM="${SLAM:-true}"
MAP="${MAP:-}"

# Sectores del LiDAR ocluidos por el propio robot (grados, por pares).
BLIND_SECTORS="${BLIND_SECTORS:-[-50.0, 50.0]}"

DOCKER=(docker)
if ! docker info >/dev/null 2>&1; then
    DOCKER=(sudo docker)
fi

SOURCE_ENV='source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash'
LOG_DIR="${LOG_DIR:-/workspace/log/robot_routine}"
# DETACH=1: usa 'docker exec -d' en vez de adjuntarse a la terminal.
# Lo necesita quien orquesta esto desde OTRO script (tsummit.sh reto4):
# 'run' se queda pegado a la vida del ros2 launch (es lo correcto para
# uso manual, ves el log en vivo con Ctrl-C para parar), pero eso
# bloquea para siempre a quien lo llama y le impide seguir con los
# pasos de despues (publicar la pose inicial, abrir RViz...).
DETACH="${DETACH:-0}"

exec_flags() {
    if [ -t 0 ] && [ -t 1 ]; then printf -- '-it'; else printf -- '-i'; fi
}

require_container() {
    if ! "${DOCKER[@]}" inspect -f '{{.State.Running}}' "${CONTAINER}" \
         2>/dev/null | grep -q true; then
        cat >&2 <<EOF
ERROR: el contenedor '${CONTAINER}' no esta corriendo.

Arrancalo primero (modo ligero, sin camara/ArUco/brazo):

    START_BRINGUP=0 ./docker/run_jetson_robot.sh
EOF
        exit 1
    fi
}

in_container() {
    "${DOCKER[@]}" exec "$(exec_flags)" "${CONTAINER}" bash -lc "$1"
}

in_container_quiet() {
    "${DOCKER[@]}" exec -i "${CONTAINER}" bash -lc "$1" 2>/dev/null
}

# ¿Hay ya un twist_mux corriendo (porque se lanzo robot.launch.py)?
twist_mux_running() {
    in_container_quiet "${SOURCE_ENV}; ros2 node list" \
        | grep -qx '/twist_mux'
}

launch_maze() {
    local auto_start="$1"
    local run_runner="$2"
    local report_blind="$3"

    require_container

    if [ "${SLAM}" = "false" ] && [ -z "${MAP}" ]; then
        echo "ERROR: SLAM=false exige MAP=/workspace/maps/<archivo>.yaml" >&2
        exit 2
    fi

    local start_mux='true'
    if twist_mux_running; then
        start_mux='false'
        echo "[i] twist_mux ya esta activo: no se relanza."
    else
        echo "[i] twist_mux no detectado: lo lanza maze.launch.py."
    fi

    echo "[i] FINISH = (${GOAL_X}, ${GOAL_Y}) yaw=${GOAL_YAW} deg"
    echo "[i] sectores ciegos del LiDAR = ${BLIND_SECTORS}"
    if [ "${SLAM}" = "false" ]; then
        echo "[i] localizacion: AMCL contra mapa guardado (${MAP})"
    else
        echo "[i] localizacion: SLAM en vivo (map->odom se construye sobre la marcha)"
    fi
    echo

    # OJO: 'map:=' con valor vacio es un argumento MALFORMADO para
    # ros2 launch (exige <name>:=<value>) y aborta el lanzamiento entero
    # antes de arrancar un solo nodo. En SLAM en vivo (MAP="") no se
    # pasa el argumento en absoluto y el launch usa su default ('').
    local map_arg=""
    if [ -n "${MAP}" ]; then
        map_arg="map:='${MAP}'"
    fi

    local launch_cmd="${SOURCE_ENV}; \
        ros2 launch home_service_bringup maze.launch.py \
            use_sim_time:=false \
            slam:=${SLAM} \
            ${map_arg} \
            auto_start:=${auto_start} \
            run_maze_runner:=${run_runner} \
            report_blind_sectors:=${report_blind} \
            start_twist_mux:=${start_mux} \
            goal_x:=${GOAL_X} \
            goal_y:=${GOAL_Y} \
            goal_yaw_deg:=${GOAL_YAW} \
            blind_sectors_deg:='${BLIND_SECTORS}'"

    if [ "${DETACH}" = "1" ]; then
        "${DOCKER[@]}" exec -d "${CONTAINER}" bash -lc \
            "mkdir -p '${LOG_DIR}'; ${launch_cmd} >'${LOG_DIR}/maze.log' 2>&1"
        echo "[i] maze.launch.py en marcha (detached). Log: ${LOG_DIR}/maze.log"
    else
        in_container "${launch_cmd}"
    fi
}

case "${1:-run}" in

    run)
        # AUTO_START lo puede forzar a 'false' quien nos llama (p.ej.
        # tsummit.sh reto4) cuando ha comprobado de antemano que
        # GOAL_X/GOAL_Y cae sobre una pared del mapa: mejor arrancar en
        # manual y dejar que el operador marque una meta valida desde
        # RViz que quedarse reintentando un objetivo imposible.
        launch_maze "${AUTO_START:-true}" true false
        ;;

    manual)
        echo "[i] El robot NO arrancara solo."
        echo "[i] Dispara con: ./scripts/run_maze.sh start"
        launch_maze false true false
        ;;

    diag)
        echo "[i] Diagnostico del LiDAR. El robot no se movera."
        echo "[i] Gira el robot A MANO despacio durante ~30 s y observa"
        echo "    que sectores siguen sin devolver eco: esos son las"
        echo "    oclusiones del propio robot -> BLIND_SECTORS."
        require_container
        in_container "${SOURCE_ENV}; \
            ros2 run home_service_navigation scan_sanitizer_node \
                --ros-args \
                -p report_blind_sectors:=true \
                -p report_period_sec:=5.0 \
                -p blind_sectors_deg:='[]'"
        ;;

    start)
        require_container
        in_container "${SOURCE_ENV}; \
            ros2 service call /maze/start std_srvs/srv/Trigger '{}'"
        ;;

    goal)
        require_container
        gx="${2:-${GOAL_X}}"
        gy="${3:-${GOAL_Y}}"
        gyaw="${4:-0.0}"
        in_container "${SOURCE_ENV}; python3 - <<'PY'
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

rclpy.init()
node = Node('manual_goal')
client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
if not client.wait_for_server(timeout_sec=15.0):
    node.get_logger().error('Nav2 no disponible')
    raise SystemExit(1)

yaw = math.radians(${gyaw})
goal = NavigateToPose.Goal()
goal.pose = PoseStamped()
goal.pose.header.frame_id = 'map'
goal.pose.header.stamp = node.get_clock().now().to_msg()
goal.pose.pose.position.x = ${gx}
goal.pose.pose.position.y = ${gy}
goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future)
handle = future.result()
if handle is None or not handle.accepted:
    node.get_logger().error('Objetivo rechazado')
    raise SystemExit(1)
node.get_logger().info('Objetivo aceptado; esperando resultado...')
result_future = handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future)
node.get_logger().info(f'status={result_future.result().status}')
PY"
        ;;

    status)
        require_container
        in_container "${SOURCE_ENV}; ros2 topic echo /maze/status"
        ;;

    scan)
        require_container
        echo "--- /scan (crudo) ---"
        in_container "${SOURCE_ENV}; timeout 6 ros2 topic hz /scan" || true
        echo "--- /scan_filtered (saneado) ---"
        in_container "${SOURCE_ENV}; timeout 6 ros2 topic hz /scan_filtered" || true
        ;;

    savemap)
        require_container
        name="${2:-/workspace/maps/maze_$(date +%Y%m%d_%H%M%S)}"
        in_container "${SOURCE_ENV}; \
            ros2 run nav2_map_server map_saver_cli -f '${name}'"
        echo "[i] Mapa guardado en ${name}.{yaml,pgm}"
        ;;

    rviz)
        require_container
        if [ -z "${DISPLAY:-}" ]; then
            echo "ERROR: DISPLAY no esta definido." >&2
            exit 1
        fi
        xhost +local:docker >/dev/null 2>&1 || true
        in_container "${SOURCE_ENV}; rviz2"
        ;;

    stop)
        require_container
        # Gracia (INT) y despues SIGKILL siempre. smoother_server,
        # behavior_server y velocity_smoother dependen de un "bond" con
        # el lifecycle_manager: si este muere primero, se quedan
        # colgados esperando un heartbeat que ya no llega y un SIGTERM
        # simple NO los mata (se ha visto en pista: quedan vivos
        # indefinidamente y el siguiente 'run' arranca con procesos
        # duplicados). SIGKILL no se puede ignorar, así que es la unica
        # garantia real de que esto termina.
        # [x]xxx en cada termino: sin el corchete, el propio texto del
        # patron (que viaja dentro del argv de este mismo 'bash -lc')
        # hace self-match y pkill se mata a si mismo a mitad de script,
        # antes de llegar al 'sleep 2; pkill -KILL' de mas abajo. Con
        # '[c]ontroller_server' el regex exige que la 'c' vaya SEGUIDA
        # de "ontroller_server"; en el propio argv, tras la 'c' viene un
        # ']', asi que no hace self-match mientras que SI cuadra contra
        # un proceso real cuyo cmdline es ".../controller_server".
        maze_proc_names='[m]aze.launch.py|[a]sync_slam_toolbox_node|[s]can_sanitizer_node|[m]aze_runner_node|[n]av2_core.launch.py|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[v]elocity_smoother|[s]moother_server|[l]ifecycle_manager'
        in_container "pkill -INT -f '${maze_proc_names}' || true; \
                      sleep 2; \
                      pkill -KILL -f '${maze_proc_names}' || true" || true
        echo "[i] Stack del laberinto detenido."
        ;;

    env)
        cat <<EOF
# ---------------------------------------------------------------------
# Exports para hablar con el contenedor desde una terminal NATIVA.
#
# Funciona porque el contenedor corre con --network host: comparte la
# pila de red de la Jetson, incluido el loopback al que CycloneDDS esta
# restringido.
#
#   source <(./scripts/run_maze.sh env)
# ---------------------------------------------------------------------
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export RMW_IMPLEMENTATION=${RMW}
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'

# AVISO: el host tiene ROS 2 Galactic y el contenedor Humble. La
# interoperabilidad DDS funciona para los mensajes estandar
# (sensor_msgs, nav_msgs, geometry_msgs, tf2_msgs), que es lo que
# necesitan RViz y 'ros2 topic echo'. Las interfaces propias
# (home_service_interfaces) NO se veran salvo que las compiles tambien
# para Galactic. Para OPERAR el robot usa siempre 'docker exec'
# (es decir, este script).
EOF
        ;;

    shell)
        require_container
        in_container "${SOURCE_ENV}; exec bash"
        ;;

    help|-h|--help)
        sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
        ;;

    *)
        echo "Subcomando desconocido: $1" >&2
        "${BASH_SOURCE[0]}" --help >&2
        exit 2
        ;;
esac
