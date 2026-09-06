#!/usr/bin/env bash
# =====================================================================
#  T-SUMMIT Challenge — consola unica del robot real
# ---------------------------------------------------------------------
#  Un solo punto de entrada para todo lo que se hace en pista:
#  mapeo, cada reto, y las pruebas de subsistema.
#
#  Este script NO reimplementa nada: compone run_robot_routine.sh (que
#  sigue siendo el que sabe de contenedor, nucleos y logs) y anade la
#  capa de "fases del concurso".
#
#      ./scripts/tsummit.sh help
#
#  REGLA DE SEGURIDAD
#  ==================
#  Todo lo que puede mover el robot exige ALLOW_MOTION=1 explicito.
#  No se hereda, no se recuerda: se escribe en cada invocacion.
# =====================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROUTINE="${ROOT}/scripts/run_robot_routine.sh"
MAZE="${ROOT}/scripts/run_maze.sh"
CONTAINER="${CONTAINER:-myagv-robot}"
LOG_DIR="${LOG_DIR:-/workspace/log/robot_routine}"

# Config DDS con la que este script habla con los nodos.
#
# Por defecto solo loopback: todo corre dentro del contenedor y asi se
# evita que el trafico salga a la red.
#
# CUIDADO con DISTRIBUTED=1: el detector y el servidor de aproximacion
# viven en el PORTATIL. Con la config de loopback, un 'action send_goal'
# desde aqui no los alcanza y se queda esperando sin error legible: el
# sintoma es "mando el goal y el robot no se mueve".
DISTRIBUTED="${DISTRIBUTED:-0}"

if [ "${DISTRIBUTED}" = "1" ]; then
    if [ -n "${WIFI_IFACE:-}" ]; then
        ROBOT_IP="${ROBOT_IP:-$(ip -4 -o addr show "${WIFI_IFACE}" 2>/dev/null \
            | awk '{print $4}' | cut -d/ -f1 | head -1)}"
    else
        ROBOT_IP="${ROBOT_IP:-$(ip -4 route get 1.1.1.1 2>/dev/null \
            | awk '{for (i=1;i<=NF;i++) if ($i=="src") print $(i+1)}' | head -1)}"
    fi
    [ -n "${ROBOT_IP:-}" ] || { printf 'ERROR: DISTRIBUTED=1 exige ROBOT_IP.\n' >&2; exit 1; }
    [ -n "${LAPTOP_IP:-}" ] || { printf 'ERROR: DISTRIBUTED=1 exige LAPTOP_IP.\n' >&2; exit 1; }
    DDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"${ROBOT_IP}\"/></Interfaces><AllowMulticast>${DDS_MULTICAST:-true}</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>32</MaxAutoParticipantIndex><Peers><Peer address=\"${ROBOT_IP}\"/><Peer address=\"${LAPTOP_IP}\"/></Peers></Discovery></Domain></CycloneDDS>"
else
    DDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
fi

DOCKER=(docker)
if ! docker info >/dev/null 2>&1; then
    DOCKER=(sudo docker)
fi

source_env='source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash'

say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

routine() { "${ROUTINE}" "$@"; }

in_container() {
    "${DOCKER[@]}" exec -e "CYCLONEDDS_URI=${DDS_URI}" "${CONTAINER}" \
        bash -lc "${source_env}; $*"
}

run_bg() {
    local name="$1" command="$2"
    "${DOCKER[@]}" exec -d -e "CYCLONEDDS_URI=${DDS_URI}" "${CONTAINER}" \
        bash -lc "mkdir -p '${LOG_DIR}'; ${source_env}; ${command} \
                  >'${LOG_DIR}/${name}.log' 2>&1"
    printf '%s iniciado. Log: %s/%s.log\n' "${name}" "${LOG_DIR}" "${name}"
}

is_running() {
    "${DOCKER[@]}" exec "${CONTAINER}" pgrep -f "$1" >/dev/null 2>&1
}

ensure_container() {
    if "${DOCKER[@]}" inspect -f '{{.State.Running}}' "${CONTAINER}" \
        2>/dev/null | grep -qx true; then
        return
    fi
    say "El contenedor ${CONTAINER} no esta en marcha: levantandolo"
    "${ROOT}/docker/run_jetson_robot.sh" sleep infinity
    sleep 3
    "${DOCKER[@]}" inspect -f '{{.State.Running}}' "${CONTAINER}" \
        2>/dev/null | grep -qx true \
        || die "no se pudo levantar ${CONTAINER}"
}

confirm_motion() {
    [ "${ALLOW_MOTION:-0}" = "1" ] \
        || die "esta accion mueve el robot. Repite con ALLOW_MOTION=1 y el area despejada."
}

# VIZ=rviz|foxglove|none. RViz en la Jetson (aunque este confinado a un
# solo nucleo, ver run_gui() en run_robot_routine.sh) sigue gastando CPU
# real en rasterizar por software; foxglove_bridge solo serializa y
# manda por WebSocket, el render lo hace el portatil. En una Nano
# saturada (Nav2 completo + AMCL + maze_runner) ese margen se nota.
open_viz() {
    case "${VIZ:-rviz}" in
        rviz)
            routine rviz
            ;;
        foxglove|fox)
            routine foxglove
            ;;
        none|off|"")
            printf 'Visualizacion desactivada (VIZ=none): mas CPU libre para Nav2.\n'
            ;;
        *)
            die "VIZ desconocido: '${VIZ}' (usa rviz|foxglove|none)"
            ;;
    esac
}

# Comprueba si (x, y) cae en una celda LIBRE del mapa .yaml/.pgm dado.
# 0 = libre, se puede confiar en el goal automatico.
# 1 = ocupado/desconocido/fuera de rango, o no se pudo leer el mapa:
#     mas vale arrancar en manual que dejar que Nav2 se estrelle
#     contra una pared reintentando un objetivo imposible.
# 'map_path' se acepta en ruta de CONTENEDOR (/workspace/...) o de host.
goal_is_free() {
    local map_path="$1" x="$2" y="$3"
    local host_path="${map_path/#\/workspace/${ROOT}}"
    [ -f "${host_path}" ] || return 1
    python3 - "${host_path}" "${x}" "${y}" <<'PY' 2>/dev/null
import sys
import os
import yaml

map_yaml, x, y = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])

with open(map_yaml, 'r', encoding='utf-8') as handle:
    meta = yaml.safe_load(handle)

res = float(meta['resolution'])
ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
negate = int(meta.get('negate', 0))
image_path = os.path.join(os.path.dirname(map_yaml), meta['image'])

with open(image_path, 'rb') as f:
    magic = f.readline()
    if not magic.startswith(b'P5'):
        sys.exit(1)  # solo PGM binario, que es lo que escribe map_saver_cli
    line = f.readline()
    while line.startswith(b'#'):
        line = f.readline()
    w, h = (int(v) for v in line.split())
    f.readline()  # maxval
    data = f.read(w * h)

mx = int(round((x - ox) / res))
my = int(round((y - oy) / res))
if not (0 <= mx < w and 0 <= my < h):
    sys.exit(1)  # fuera del mapa

row = h - 1 - my
value = data[row * w + mx]
if negate:
    value = 255 - value

# Convenio map_server: >200 ~ libre (blanco), <50 ~ ocupado (negro),
# el resto ~ desconocido. Solo "libre" confirmado cuenta como seguro.
sys.exit(0 if value > 200 else 1)
PY
}

# =====================================================================
#  Compilacion
# ---------------------------------------------------------------------
#  El repositorio esta montado en /workspace dentro del contenedor
#  (-v ROOT:/workspace:rw en run_jetson_robot.sh), asi que los ficheros
#  YA estan dentro: editar en el host es editar en el contenedor. Solo
#  hace falta compilar cuando se anaden nodos o entry points nuevos.
#  Los .yaml y .rviz se leen en caliente y no necesitan build.
# =====================================================================

build() {
    ensure_container
    local pkgs="${*:-home_service_behaviors}"
    say "colcon build --packages-select ${pkgs}"
    "${DOCKER[@]}" exec "${CONTAINER}" bash -lc \
        "source /opt/ros/humble/setup.bash; cd /workspace; \
         colcon build --symlink-install --packages-select ${pkgs} \
             --event-handlers console_direct+"
}

# =====================================================================
#  Fase 0 — Mapeo
# =====================================================================

mapping() {
    ensure_container
    say "Mapeo SLAM (base -> slam -> rviz -> teleop)"
    routine mapping
}

save_map() {
    routine save-map "$@"
}

# =====================================================================
#  Subsistemas
# =====================================================================

arm() {
    ensure_container
    if is_running '[m]echarm_driver_node'; then
        printf 'Driver del MechArm ya iniciado.\n'
        return
    fi
    run_bg mecharm 'ros2 launch myagv_mecharm_service mecharm_driver.launch.py'
    sleep 3
}

perception() {
    ensure_container
    local guard='[a]ruco_detector_node'
    [ "${DISTRIBUTED}" = "1" ] && guard='[c]si_camera_node'
    if is_running "${guard}"; then
        printf 'Camara + detector ArUco ya iniciados.\n'
        return
    fi
    routine aruco
    sleep 3
}

# =====================================================================
#  APROXIMACION A UN ARUCO  (base + camara -> aruco_lidar_approach)
# ---------------------------------------------------------------------
#  El servidor de aproximacion necesita TRES cosas que 'perception'
#  solo no da:
#    /aruco/detections  <- camara + detector      (perception)
#    /scan_filtered      <- LiDAR + scan_sanitizer (base)   << faltaba
#    /odom               <- odometria de la base   (base)   << faltaba
#  y saca velocidad por /cmd_vel_aruco -> twist_mux -> /cmd_vel, que
#  tambien exige base + twist_mux.
#
#      ./scripts/tsummit.sh approach-check          arranca la pila, NO mueve
#      ALLOW_MOTION=1 ./scripts/tsummit.sh approach <id> [stop_m]
# =====================================================================

approach_stack() {
    ensure_container
    routine base
    sleep 3
    perception
    sleep 2
    # sanity: que las 3 entradas del servidor de aproximacion tengan
    # vida. 'ros2 topic hz' por CLI es poco fiable en esta Nano
    # (rcl context invalid); un one-shot con echo --once es robusto.
    say "Comprobando entradas del servidor de aproximacion"
    in_container "for t in /scan_filtered /aruco/detections /odom; do \
        if timeout 6 ros2 topic echo --once \"\$t\" >/dev/null 2>&1; then \
            echo \"  OK    \$t\"; \
        else \
            echo \"  ? \$t  (sin respuesta; la CLI de ros2 falla a ratos \
en la Nano, reintenta 'tsummit.sh status')\"; \
        fi; done"
}

approach_check() {
    approach_stack
    printf '\nPila lista. Detecciones en vivo:\n'
    printf '  docker exec %s bash -lc "source /opt/ros/humble/setup.bash; \\\n' "${CONTAINER}"
    printf '    source /workspace/install/setup.bash; ros2 topic echo /aruco/detections"\n'
    printf 'Cuando quieras mover:\n'
    printf '  ALLOW_MOTION=1 ./scripts/tsummit.sh approach <id> [stop_m]\n'
}

approach() {
    confirm_motion
    local marker_id="${1:?uso: approach <id_aruco> [stop_distance_m]}"
    local stop_dist="${2:-0.20}"
    local timeout_s="${3:-${APPROACH_TIMEOUT:-180.0}}"
    approach_stack
    say "Enviando goal /aruco_lidar_approach  (id=${marker_id}, stop=${stop_dist} m, timeout=${timeout_s} s)"
    in_container "ros2 action send_goal /aruco_lidar_approach \
        home_service_interfaces/action/ArucoApproach \
        '{target_id: ${marker_id}, stop_distance: ${stop_dist}, timeout_sec: ${timeout_s}}' \
        --feedback"
}

# =====================================================================
#  PRUEBA DE TOMA DE PIEZA  (retos 1 y 2)
# ---------------------------------------------------------------------
#  Cadena completa: ver ArUco -> aproximar la base -> tomar la pieza.
#  Los tres casos (engranaje / poste / rueda) salen del catalogo
#  src/home_service_behaviors/config/grasp_catalog.yaml.
#
#      ./scripts/tsummit.sh grasp-dry            ensayo, no mueve nada
#      ALLOW_MOTION=1 ./scripts/tsummit.sh grasp auto
#      ALLOW_MOTION=1 ./scripts/tsummit.sh grasp engranaje
# =====================================================================

grasp_stack() {
    local enable_arm="$1" enable_approach="$2"
    ensure_container
    perception
    if [ "${enable_arm}" = "true" ]; then
        arm
    fi
    if is_running '[o]bject_grasp_server'; then
        printf 'object_grasp_server ya iniciado. Reinicia con: tsummit.sh stop\n'
        return
    fi
    run_bg grasp \
        "ros2 launch home_service_behaviors object_grasp.launch.py \
         enable_arm:=${enable_arm} enable_approach:=${enable_approach}"
    sleep 5
}

grasp_send() {
    local piece="${1:-auto}"
    say "Enviando goal /grasp_object  (pieza: ${piece})"
    in_container "ros2 action send_goal /grasp_object \
        home_service_interfaces/action/PickPlace \
        '{operation: pick, target_pose_name: ${piece}}' --feedback"
}

grasp() {
    confirm_motion
    grasp_stack true true
    grasp_send "${1:-auto}"
}

grasp_dry() {
    # Ensayo: identifica la pieza y calcula el agarre, pero no manda
    # nada al brazo ni a las ruedas. Es la forma de validar el catalogo
    # y el mapa ArUco->pieza sin riesgo.
    say "ENSAYO en seco: sin brazo y sin mover la base"
    grasp_stack false false
    grasp_send "${1:-auto}"
}

grasp_catalog() {
    # Vuelca el catalogo tal y como lo interpreta el nodo (conversion a
    # valores 0..100 de pinza incluida). Util para revisar de un vistazo
    # que ninguna pieza se sale del recorrido de la pinza.
    ensure_container
    in_container "ros2 run home_service_behaviors object_grasp_server \
        --ros-args -p enable_arm:=false -p enable_approach:=false" \
        2>&1 | sed -n '1,40p' &
    local pid=$!
    sleep 6
    kill "${pid}" 2>/dev/null || true
    in_container "pkill -f object_grasp_server" >/dev/null 2>&1 || true
}

# =====================================================================
#  Retos
# =====================================================================

reto1() {
    confirm_motion
    say "Reto 1 — Clasificacion"
    ensure_container
    routine base
    perception
    arm
    grasp_stack true true
    printf 'Pila lista. Lanza la toma con:\n'
    printf '  ALLOW_MOTION=1 ./scripts/tsummit.sh grasp <engranaje|poste|rueda|auto>\n'
}

reto2() {
    confirm_motion
    say "Reto 2 — Kitting"
    ensure_container
    routine base
    perception
    arm
    grasp_stack true true
    printf 'Pila lista (misma que reto 1; la secuencia de kitting la\n'
    printf 'orquesta home_service_mission/mission_manager).\n'
}

reto4() {
    # OJO: esto NO es "base + nav2 generico". Eso es lo que habia antes
    # y por lo que el robot no se movia: nadie enviaba un goal ni
    # arrancaba maze_runner. El ejecutor autonomo del laberinto vive en
    # home_service_bringup/maze.launch.py (via scripts/run_maze.sh), que
    # ademas de Nav2 lanza maze_runner_node (envia el goal solo tras
    # start_delay_sec) y, opcionalmente, AMCL contra un mapa guardado.
    confirm_motion
    say "Reto 4 — Laberinto"
    ensure_container
    routine base
    sleep 3

    local goal_x="${GOAL_X:-3.7}" goal_y="${GOAL_Y:--2.2}"
    local auto_start="true"

    # DETACH=1: 'run_maze.sh run' por defecto se queda pegado al ros2
    # launch (correcto para uso manual). Sin esto, reto4() se quedaba
    # colgado ahi para siempre y JAMAS llegaba a publicar la pose
    # inicial ni a abrir RViz.
    if [ -n "${MAP:-}" ]; then
        printf 'Localizacion: AMCL contra mapa guardado (%s)\n' "${MAP}"

        # Comprobado en pista: goal_x/goal_y es una estimacion de
        # diseno y puede caer literalmente sobre una pared del mapa
        # real. Mandar ese goal igualmente encadena recuperaciones
        # (backup/wait) que solo empeoran las cosas: si las ruedas
        # patinan (robot sujeto o atascado), la odometria se desvia y
        # AMCL puede acabar "fuera del mapa". Se comprueba ANTES de
        # lanzar: si esta libre, se manda solo (auto_start=true) como
        # siempre; si no, se arranca en manual y el operador marca la
        # meta el mismo con la herramienta "Nav2 Goal" de RViz (ya
        # incluida en slam_real.rviz) sin tener que reiniciar nada.
        if goal_is_free "${MAP}" "${goal_x}" "${goal_y}"; then
            printf 'FINISH (%s, %s): libre segun el mapa. Se manda automaticamente.\n' \
                "${goal_x}" "${goal_y}"
        else
            auto_start="false"
            printf 'AVISO: FINISH (%s, %s) cae sobre un obstaculo/zona desconocida del mapa.\n' \
                "${goal_x}" "${goal_y}"
            printf '       Arranca en modo MANUAL: no se envia ningun goal solo.\n'
            printf '       Marca tu la meta en RViz con "Nav2 Goal" (panel Navigation 2),\n'
            printf '       o corrige GOAL_X/GOAL_Y y repite.\n'
        fi

        DETACH=1 SLAM=false MAP="${MAP}" AUTO_START="${auto_start}" \
            GOAL_X="${goal_x}" GOAL_Y="${goal_y}" "${MAZE}" run
    else
        printf 'Localizacion: SLAM en vivo (por defecto).\n'
        printf 'Para navegar sobre un mapa ya guardado:\n'
        printf '  MAP=/workspace/maps/<archivo>.yaml ALLOW_MOTION=1 tsummit.sh reto4\n'
        printf 'AVISO: con SLAM en vivo el mapa arranca vacio; no se puede comprobar\n'
        printf '       de antemano si FINISH cae libre. Si Nav2 lo rechaza, marca la\n'
        printf '       meta a mano en RViz con "Nav2 Goal".\n'
        DETACH=1 GOAL_X="${goal_x}" GOAL_Y="${goal_y}" "${MAZE}" run
    fi

    if [ -n "${MAP:-}" ]; then
        # Sin esto AMCL se queda para siempre sin publicar map->odom
        # ("AMCL cannot publish a pose... Please set the initial
        # pose") y Nav2 rechaza cualquier objetivo: no hay frame "map".
        # (0,0,0) es el START tal cual quedo grabado al mapear (ver
        # comentario de maze.launch.py); si el robot NO esta ahi ahora,
        # sobreescribe con INITIAL_X/INITIAL_Y/INITIAL_YAW, o corrigelo
        # luego en RViz con "2D Pose Estimate".
        sleep 5
        say "Pose inicial para AMCL"
        printf 'Asumiendo robot en START = (%s, %s, %s deg). Si no es asi:\n' \
            "${INITIAL_X:-0}" "${INITIAL_Y:-0}" "${INITIAL_YAW:-0}"
        printf '  INITIAL_X=<x> INITIAL_Y=<y> INITIAL_YAW=<deg> ALLOW_MOTION=1 tsummit.sh reto4\n'
        if [ "${VIZ:-rviz}" = "rviz" ]; then
            printf '  (o corrigela en RViz con la herramienta "2D Pose Estimate")\n'
        fi
        routine initial-pose "${INITIAL_X:-0}" "${INITIAL_Y:-0}" "${INITIAL_YAW:-0}"
    fi

    sleep 2
    say "Visualizacion (VIZ=${VIZ:-rviz}: rviz|foxglove|none)"
    open_viz
    printf '\n'
    if [ "${auto_start}" = "true" ]; then
        printf 'maze_runner enviara el goal solo tras el arranque (start_delay_sec).\n'
    else
        case "${VIZ:-rviz}" in
            rviz)
                printf 'MODO MANUAL: nadie enviara ningun goal solo. Marca la meta en RViz\n'
                printf '("Nav2 Goal", panel Navigation 2) o dispara la de siempre con:\n'
                printf '  ./scripts/run_maze.sh start\n'
                ;;
            *)
                printf 'MODO MANUAL: nadie enviara ningun goal solo. Sin RViz no hay clic\n'
                printf 'en el mapa: usa\n'
                printf '  ./scripts/run_maze.sh goal <x> <y> [yaw_deg]\n'
                printf 'o dispara el FINISH de siempre con:\n'
                printf '  ./scripts/run_maze.sh start\n'
                ;;
        esac
    fi
    printf 'Log en vivo:     tail -f log/robot_routine/maze.log  (o docker exec %s tail -f %s/maze.log)\n' \
        "${CONTAINER}" "${LOG_DIR}"
    printf 'Estado en vivo:  ./scripts/run_maze.sh status\n'
    printf 'Goal manual:     ./scripts/run_maze.sh goal <x> <y> [yaw_deg]\n'
    printf 'Parar el reto:   ./scripts/run_maze.sh stop\n'
}

# =====================================================================
#  Estado / parada
# =====================================================================

status() {
    ensure_container
    say "Nodos"
    in_container 'ros2 node list' || true
    say "Acciones"
    in_container 'ros2 action list' || true
    say "Procesos clave"
    for p in myagv_odometry_node ydlidar_ros2_driver_node slam_toolbox \
             rviz2 aruco_detector_node mecharm_driver_node \
             object_grasp_server twist_mux; do
        if is_running "[${p:0:1}]${p:1}"; then
            printf '  [ON ] %s\n' "${p}"
        else
            printf '  [off] %s\n' "${p}"
        fi
    done
}

logs() {
    local name="${1:?uso: logs <base|slam|rviz|grasp|mecharm|aruco|nav2|teleop>}"
    "${DOCKER[@]}" exec "${CONTAINER}" tail -n "${2:-60}" -f \
        "${LOG_DIR}/${name}.log"
}

stop() {
    "${DOCKER[@]}" exec "${CONTAINER}" bash -lc \
        "pkill -KILL -f '[o]bject_grasp_server' || true; \
         pkill -KILL -f '[m]echarm_driver_node' || true" 2>/dev/null || true
    "${MAZE}" stop 2>/dev/null || true
    routine stop
}

# =====================================================================

case "${1:-help}" in
    build)        shift; build "$@" ;;
    mapping|mapear) mapping ;;
    save-map)     shift; save_map "$@" ;;

    arm|brazo)    arm ;;
    perception|vision) perception ;;

    approach-check|aprox-check) approach_check ;;
    approach|aproximar) shift; approach "$@" ;;

    grasp|tomar)  shift; grasp "${1:-auto}" ;;
    grasp-dry|ensayo) shift; grasp_dry "${1:-auto}" ;;
    grasp-catalog|catalogo) grasp_catalog ;;

    reto1)        reto1 ;;
    reto2)        reto2 ;;
    reto4)        reto4 ;;

    rviz)         routine rviz ;;
    # El argumento manda, pero si no lo hay se respeta VIZ del entorno:
    # con "${1:-rviz}" a secas, 'VIZ=foxglove ... viz' abria RViz.
    viz)          shift; VIZ="${1:-${VIZ:-rviz}}" open_viz ;;
    teleop)       routine teleop ;;
    status)       status ;;
    logs)         shift; logs "$@" ;;
    stop)         stop ;;

    help|-h|--help)
        cat <<'EOF'
T-SUMMIT Challenge — consola unica

  MAPEO
    mapping                 base -> slam -> rviz -> teleop (todo en orden)
    save-map <nombre>       guarda /workspace/maps/<nombre>.{yaml,pgm}

  APROXIMACION A UN ARUCO
    approach-check         arranca base + camara y comprueba entradas, NO mueve
    approach <id> [stop_m] aproxima la BASE al marcador <id> (exige ALLOW_MOTION=1)
                           stop_m = distancia final, por defecto 0.20 m

  TOMA DE PIEZA  (retos 1 y 2)
    grasp-dry [pieza]       ENSAYO: identifica y calcula, no mueve nada
    grasp [pieza]           cadena completa (exige ALLOW_MOTION=1)
    grasp-catalog           vuelca el catalogo como lo lee el nodo
      pieza = auto | engranaje | poste | rueda
      auto  -> deduce la pieza del ArUco que este viendo

  RETOS
    reto1                   Clasificacion   (ALLOW_MOTION=1)
    reto2                   Kitting         (ALLOW_MOTION=1)
    reto4                   Laberinto: SLAM en vivo + auto-navega (ALLOW_MOTION=1)
                            MAP=<archivo.yaml> -> AMCL sobre mapa guardado en vez de SLAM
                            VIZ=rviz|foxglove|none -> que visualizacion abre (o ninguna)

  SUBSISTEMAS
    arm                     driver del MechArm 270
    perception              camara CSI + detector ArUco
    rviz / teleop           visualizacion y mando
    viz [rviz|foxglove|none]  abre la visualizacion elegida (o ninguna)

  UTIL
    build [paquetes]        colcon build dentro del contenedor
    status                  nodos, acciones y procesos clave
    logs <nombre> [n]       tail -f de un log
    stop                    para todos los stacks

  Todo lo que mueve el robot exige ALLOW_MOTION=1 en cada llamada.
  El repo esta montado en /workspace: editar aqui = editar dentro del
  contenedor. Solo hay que 'build' al anadir nodos o entry points.
EOF
        ;;
    *) die "comando desconocido: $1  (prueba: help)" ;;
esac
