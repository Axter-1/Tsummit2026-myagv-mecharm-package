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
# Tiempo maximo para encontrar y aproximarse al ArUco durante un grasp.
# Se puede ampliar por invocacion sin editar codigo.
GRASP_APPROACH_TIMEOUT="${GRASP_APPROACH_TIMEOUT:-120.0}"
# Distancia LiDAR (m) a la que se detiene la base antes de agarrar.
# Vacia: prepare grasp elige la parada calibrada de cada pieza.
GRASP_STOP_DISTANCE="${GRASP_STOP_DISTANCE:-}"
# Marca de tiempo monotona de la invocacion. Solo se informa al enviar el
# goal: no mide busqueda, aproximacion ni movimiento del brazo.
GRASP_COMMAND_STARTED_NS="$(date +%s%N)"

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

ip_for_peer() {
    local peer="$1"
    ip -4 route get "${peer}" 2>/dev/null \
        | awk '{for (i = 1; i <= NF; i++) if ($i == "src") print $(i + 1)}' \
        | head -1
}

if [ "${DISTRIBUTED}" = "1" ]; then
    if [ -n "${WIFI_IFACE:-}" ]; then
        ROBOT_IP="${ROBOT_IP:-$(ip -4 -o addr show "${WIFI_IFACE}" 2>/dev/null \
            | awk '{print $4}' | cut -d/ -f1 | head -1)}"
    elif [ -n "${LAPTOP_IP:-}" ]; then
        ROBOT_IP="${ROBOT_IP:-$(ip_for_peer "${LAPTOP_IP}")}"
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

# El daemon de ros2 conserva el primer entorno DDS que vio y puede ocultar
# nodos del mismo contenedor tras reinicios. Las operaciones del robot usan
# descubrimiento directo para consultar siempre el grafo actual.
source_env='export ROS2_DISABLE_DAEMON=1; source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash'

# Requisitos de los perfiles de operacion. Es shell intencionadamente: esta
# consola se ejecuta antes de entrar al contenedor y no depende de PyYAML.
# shellcheck disable=SC1091
source "${ROOT}/scripts/command_requirements.sh"

say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

routine() { "${ROUTINE}" "$@"; }

in_container() {
    "${DOCKER[@]}" exec -e "CYCLONEDDS_URI=${DDS_URI}" "${CONTAINER}" \
        bash -lc "${source_env}; $*"
}

in_container_interactive() {
    "${DOCKER[@]}" exec -it -e "CYCLONEDDS_URI=${DDS_URI}" "${CONTAINER}" \
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

require_container_running() {
    if ! "${DOCKER[@]}" inspect -f '{{.State.Running}}' "${CONTAINER}" \
        2>/dev/null | grep -qx true; then
        die "el contenedor ${CONTAINER} no esta en marcha; ejecuta la preparacion del robot"
    fi
}

clear_prepare_state() {
    "${DOCKER[@]}" exec "${CONTAINER}" bash -lc \
        "rm -f '${PREPARE_GRASP_READY_FILE}' '${PREPARE_APPROACH_READY_FILE}'" \
        >/dev/null 2>&1 || true
}

prepared_state_matches() {
    local file="$1" profile="$2" piece="$3" table_mm="$4"
    in_container "test -r '${file}' \
        && grep -Fxq 'profile=${profile}' '${file}' \
        && grep -Fxq 'piece=${piece}' '${file}' \
        && grep -Fxq 'table_height_mm=${table_mm}' '${file}' \
        && grep -Fxq 'distributed=1' '${file}' \
        && grep -Fxq 'robot_ip=${ROBOT_IP}' '${file}' \
        && grep -Fxq 'laptop_ip=${LAPTOP_IP}' '${file}'"
}

write_prepared_state() {
    local file="$1" profile="$2" piece="$3" table_mm="$4"
    in_container "mkdir -p \"\$(dirname '${file}')\"; printf '%s\\n' \
        'profile=${profile}' 'piece=${piece}' \
        'table_height_mm=${table_mm}' 'distributed=1' \
        'robot_ip=${ROBOT_IP}' 'laptop_ip=${LAPTOP_IP}' > '${file}'"
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

doctor() {
    bash "${ROOT}/scripts/check_robot_integrity.sh"
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

driver_uses_current_dds() {
    local needle
    if [ "${DISTRIBUTED}" = "1" ]; then
        needle="NetworkInterface address=\"${ROBOT_IP}\""
    else
        needle='NetworkInterface name="lo"'
    fi
    "${DOCKER[@]}" exec "${CONTAINER}" bash -lc \
        "pid=\$(pgrep -f '[m]echarm_driver_node' | head -n 1); \
         [ -n \"\${pid}\" ] && tr '\\0' '\\n' < /proc/\${pid}/environ | \
         grep -Fq '${needle}'"
}

arm() {
    ensure_container
    if is_running '[m]echarm_driver_node' && ! driver_uses_current_dds; then
        printf 'Driver del MechArm iniciado con otro DDS: reiniciando.\n'
        stop arm
    fi
    if ! is_running '[m]echarm_driver_node'; then
        run_bg mecharm 'ros2 launch myagv_mecharm_service mecharm_driver.launch.py'
    fi

    in_container 'python3 /workspace/scripts/check_move_arm_action.py --timeout 15' \
        || die "mecharm_driver_node no publica /mecharm/move_arm; revisa ${LOG_DIR}/mecharm.log"
}

calibrate_grasp() {
    if [ "${1:-}" = "--menu" ] || [ "${1:-}" = "menu" ]; then
        ensure_container
        in_container_interactive 'python3 /workspace/scripts/calibrate_grasp.py --menu'
        return
    fi
    confirm_motion
    ensure_container
    local object_name="${1:?uso: calibrate-grasp <pieza> <altura_mm> --action pick|place}"
    local table_mm="${2:?uso: calibrate-grasp <pieza> <altura_mm> --action pick|place}"
    case "${object_name}" in
        engranaje|poste|rueda|estrella) ;;
        *) die "pieza desconocida: '${object_name}' (usa engranaje|poste|rueda|estrella)" ;;
    esac
    case "${table_mm}" in
        ''|*[!0-9]*) die "altura invalida: '${table_mm}' (mm, entero)" ;;
    esac
    shift 2 || true
    if is_running '[m]echarm_driver_node'; then
        die "deten el driver primero: ./scripts/tsummit.sh stop. La calibracion abre /dev/ttyACM0 directamente."
    fi
    say "Calibracion manual de ${object_name} sobre plataforma de ${table_mm} mm"
    in_container_interactive "python3 /workspace/scripts/calibrate_grasp.py '${object_name}' '${table_mm}' $*"
}

test_grasp_calibration() {
    confirm_motion
    ensure_container
    local object_name="${1:?uso: test-grasp <pieza> <altura_mm>}"
    local table_mm="${2:?uso: test-grasp <pieza> <altura_mm>}"
    case "${object_name}" in
        engranaje|poste|rueda|estrella) ;;
        *) die "pieza desconocida: '${object_name}'" ;;
    esac
    case "${table_mm}" in
        ''|*[!0-9]*) die "altura invalida: '${table_mm}' (mm, entero)" ;;
    esac
    arm
    say "Ensayo de poses: ${object_name} a ${table_mm} mm"
    in_container "python3 /workspace/scripts/test_grasp_calibration.py '${object_name}' '${table_mm}'"
}

test_pick_lift() {
    confirm_motion
    ensure_container
    local object_name="${1:?uso: test-pick-lift <pieza> <altura_mm>}"
    local table_mm="${2:?uso: test-pick-lift <pieza> <altura_mm>}"
    case "${object_name}" in
        engranaje|poste|rueda|estrella) ;;
        *) die "pieza desconocida: '${object_name}'" ;;
    esac
    case "${table_mm}" in
        ''|*[!0-9]*) die "altura invalida: '${table_mm}' (mm, entero)" ;;
    esac
    shift 2 || true
    arm
    say "Prueba de agarre y elevacion: ${object_name} a ${table_mm} mm"
    in_container "python3 /workspace/scripts/test_pick_lift.py '${object_name}' '${table_mm}' $*"
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
#      DISTRIBUTED=1 LAPTOP_IP=<ip> ./scripts/tsummit.sh approach-check
#      ALLOW_MOTION=1 DISTRIBUTED=1 LAPTOP_IP=<ip> \
#          ./scripts/tsummit.sh approach <id> [stop_m]
# =====================================================================

require_distributed_aruco() {
    # La CSI solo puede tener un CaptureSession. El detector y la
    # aproximacion corren en el portatil, por lo que arrancar una pila
    # ArUco local en paralelo abre una segunda camara y deja el grasp en
    # SEARCHING antes de que llegue a mandar nada al brazo.
    [ "${DISTRIBUTED}" = "1" ] || die \
        "ArUco requiere DISTRIBUTED=1: define tambien LAPTOP_IP=<ip actual del portatil>."
}

approach_stack() {
    ensure_container
    require_distributed_aruco
    routine base
    sleep 3
    perception
    sleep 2
    # Una suscripcion ROS nativa evita el descubrimiento de tipo inestable
    # de `ros2 topic echo` cuando CycloneDDS opera entre Nano y portatil.
    say "Comprobando entradas del servidor de aproximacion"
    in_container 'python3 /workspace/scripts/check_approach_inputs.py --timeout 30'
    write_prepared_state "${PREPARE_APPROACH_READY_FILE}" approach any 0
}

require_prepared_approach() {
    require_container_running
    require_distributed_aruco

    # El marcador es una cache de `approach_stack`, no la prueba de que los
    # nodos sigan vivos. `tsummit_offboard.sh prepare` puede haber levantado
    # correctamente el servidor desde el portatil sin crear este archivo en
    # el contenedor. En ese caso consulta el grafo ROS actual y permite usar
    # `approach` directamente si todas las entradas estan disponibles.
    if ! prepared_state_matches "${PREPARE_APPROACH_READY_FILE}" approach any 0; then
        printf 'Aviso: no hay un marcador de approach valido; comprobando la pila en vivo...\n'
        if ! in_container 'python3 /workspace/scripts/check_aruco_approach_action.py --timeout 3' \
            || ! in_container 'python3 /workspace/scripts/check_approach_inputs.py --timeout 3'; then
            printf 'ERROR: La infraestructura requerida no esta preparada.\n' >&2
            printf 'Ejecuta primero:\n  ROBOT_IP=<jetson> LAPTOP_IP=<laptop> ./scripts/tsummit_offboard.sh prepare\n' >&2
            printf 'Si los nodos ya estan levantados, comprueba que publican /scan_filtered, /aruco/detections y /odom.\n' >&2
            exit 1
        fi
        write_prepared_state "${PREPARE_APPROACH_READY_FILE}" approach any 0
    fi
    in_container 'python3 /workspace/scripts/check_aruco_approach_action.py --timeout 1' \
        || die "La infraestructura requerida no esta disponible. Repite la preparacion del portatil y de la Jetson."
}

approach_check() {
    approach_stack
    printf '\nPila lista. Detecciones en vivo:\n'
    printf '  docker exec %s bash -lc "source /opt/ros/humble/setup.bash; \\\n' "${CONTAINER}"
    printf '    source /workspace/install/setup.bash; ros2 topic echo /aruco/detections"\n'
    printf 'Cuando quieras mover:\n'
    printf '%s\n' '  ALLOW_MOTION=1 DISTRIBUTED=1 LAPTOP_IP=<ip> \' \
        '  ./scripts/tsummit.sh approach <id> [stop_m]'
}

approach() {
    confirm_motion
    local marker_id="${1:?uso: approach <id_aruco> [stop_distance_m]}"
    local stop_dist="${2:-0.20}"
    local timeout_s="${3:-${APPROACH_TIMEOUT:-180.0}}"
    require_prepared_approach
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
#      DISTRIBUTED=1 LAPTOP_IP=<ip> ./scripts/tsummit.sh grasp-dry
#      ALLOW_MOTION=1 DISTRIBUTED=1 LAPTOP_IP=<ip> \
#          ./scripts/tsummit.sh grasp rueda --action pick --table-height 100
# =====================================================================

# Compatibilidad para `grasp-dry`: esta variante no forma parte de la ruta
# normal de pick/place. Las misiones reales usan exclusivamente `prepare`.
grasp_stack() {
    local enable_arm="$1" enable_approach="$2" piece="${3:-auto}" table_mm="${4:-100}"
    local stop_distance="${GRASP_STOP_DISTANCE}"
    if [ -z "${stop_distance}" ]; then
        case "${piece}" in
            # Ensenada con la rueda a 0.09 m de la lectura LiDAR final.
            # Con el LiDAR ~0.123 m por detras del borde del chasis, 0.09 m
            # invadia la pared. 0.20 m deja ~0.077 m de despeje teorico.
            rueda) stop_distance="0.20" ;;
            *)     stop_distance="0.20" ;;
        esac
    fi
    ensure_container
    require_distributed_aruco

    # Tras `prepare grasp` no repitas sleeps ni comprobaciones DDS costosas.
    # Los pgrep son deliberadamente baratos; si un proceso desaparecio se
    # cae al camino normal, que vuelve a levantar y verificar la pila.
    if grasp_stack_ready "${enable_arm}" "${enable_approach}" "${table_mm}"; then
        printf 'Pila de grasp ya preparada (tabla=%s mm); reutilizando nodos.\n' \
            "${table_mm}"
        return
    fi

    if [ "${enable_approach}" = "true" ]; then
        # La toma comparte las precondiciones de la accion publica de
        # aproximacion: base, LiDAR, odometria y detector remoto.
        approach_stack
    else
        perception
    fi
    if [ "${enable_arm}" = "true" ]; then
        arm
    fi
    if is_running '[o]bject_grasp_server'; then
        if grasp_server_table_matches "${table_mm}"; then
            printf 'object_grasp_server ya iniciado (tabla=%s mm); reutilizando.\n' \
                "${table_mm}"
            return
        fi
        printf 'object_grasp_server usa otra altura; reiniciando.\n'
        stop grasp
    fi
    start_grasp_server "${enable_arm}" "${enable_approach}" \
        "${table_mm}" "${stop_distance}"
}

start_grasp_server() {
    local enable_arm="$1" enable_approach="$2" table_mm="$3" stop_distance="$4"
    run_bg grasp \
        "ros2 launch home_service_behaviors object_grasp.launch.py \
          enable_arm:=${enable_arm} enable_approach:=${enable_approach} \
          approach_stop_distance:=${stop_distance} \
          table_height_mm:=${table_mm} \
          approach_timeout_sec:=${GRASP_APPROACH_TIMEOUT}"
    sleep 5
}

require_prepared_grasp() {
    local piece="$1" table_mm="$2"
    require_container_running
    require_distributed_aruco
    if ! prepared_state_matches "${PREPARE_GRASP_READY_FILE}" grasp "${piece}" "${table_mm}"; then
        printf 'ERROR: La infraestructura requerida no esta preparada para %s a %s mm.\n' \
            "${piece}" "${table_mm}" >&2
        printf 'Ejecuta primero:\n  ROBOT_IP=<jetson> LAPTOP_IP=<laptop> ./scripts/tsummit_offboard.sh prepare\n  DISTRIBUTED=1 LAPTOP_IP=<laptop> ./scripts/tsummit.sh prepare grasp %s --table-height %s\n' \
            "${piece}" "${table_mm}" >&2
        exit 1
    fi
    if ! in_container 'python3 /workspace/scripts/check_grasp_ready.py --timeout 0.5'; then
        die "La infraestructura requerida no esta disponible. Repite la preparacion del portatil y de la Jetson."
    fi
}

grasp_server_table_matches() {
    local table_mm="$1"
    in_container "ros2 param get /object_grasp_server table_height_mm 2>/dev/null \
        | grep -Eq 'Integer value: ${table_mm}$'"
}

grasp_stack_ready() {
    local enable_arm="$1" enable_approach="$2" table_mm="$3"
    is_running '[o]bject_grasp_server' \
        && grasp_server_table_matches "${table_mm}" \
        || return 1

    if [ "${enable_arm}" = "true" ]; then
        is_running '[m]echarm_driver_node' || return 1
    fi
    if [ "${enable_approach}" = "true" ]; then
        is_running '[m]yagv_odometry_node' || return 1
        is_running '[r]os2 run twist_mux twist_mux' || return 1
        is_running '[c]si_camera_node' || return 1
    fi
}

prepare() {
    ensure_container
    require_distributed_aruco

    local profile="${1:-grasp}" piece="auto" table_mm="${PREPARE_GRASP_DEFAULT_TABLE_MM}"
    shift || true
    [ "${profile}" = "grasp" ] \
        || die "perfil desconocido: ${profile} (usa prepare grasp)"
    if [ "${1:-}" != "" ] && [[ "${1}" != --* ]]; then
        piece="$1"
        shift
    fi
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --table-height|--height) table_mm="${2:?falta valor para --table-height}"; shift 2 ;;
            *) die "opcion desconocida para prepare: $1" ;;
        esac
    done
    case "${piece}" in
        auto|engranaje|poste|rueda|estrella) ;;
        *) die "pieza desconocida: '${piece}'" ;;
    esac
    case "${table_mm}" in
        ''|*[!0-9]*) die "altura invalida: ${table_mm}" ;;
    esac

    say "Preparando cadena de grasp (pieza=${piece}, tabla=${table_mm} mm)"
    approach_stack
    arm
    if ! in_container 'python3 /workspace/scripts/check_aruco_approach_action.py --timeout 15'; then
        die "falta /aruco_lidar_approach; ejecuta en el portatil: ROBOT_IP=<ip> LAPTOP_IP=<ip> ${PREPARE_GRASP_REMOTE_COMMAND}"
    fi

    local stop_distance="${GRASP_STOP_DISTANCE}"
    if [ -z "${stop_distance}" ]; then
        case "${piece}" in
            # La rueda comparte el offset fisico del chasis: 0.20 m de
            # LiDAR-pared conserva aproximadamente 7 cm de despeje.
            rueda) stop_distance="0.20" ;;
            *) stop_distance="0.20" ;;
        esac
    fi
    if grasp_stack_ready true true "${table_mm}"; then
        printf 'object_grasp_server ya preparado para tabla=%s mm.\n' "${table_mm}"
    else
        if is_running '[o]bject_grasp_server'; then
            stop grasp
        fi
        start_grasp_server true true "${table_mm}" "${stop_distance}"
    fi
    if ! in_container 'python3 /workspace/scripts/check_grasp_ready.py --timeout 2'; then
        die "La preparacion no termino: alguna accion requerida no esta disponible."
    fi
    write_prepared_state "${PREPARE_GRASP_READY_FILE}" grasp "${piece}" "${table_mm}"
    printf '\nPreparacion completa. Las siguientes llamadas pueden reutilizar la pila:\n'
    printf '  ALLOW_MOTION=1 DISTRIBUTED=1 LAPTOP_IP=<ip> \\\n'
    printf '    ./scripts/tsummit.sh grasp %s --action pick --table-height %s\n' \
        "${piece}" "${table_mm}"
}

grasp_send() {
    local piece="${1:-auto}" action="${2:-}"
    [ -n "${action}" ] || die "la accion es obligatoria: --action pick|place"
    local elapsed_ms=$(( ($(date +%s%N) - GRASP_COMMAND_STARTED_NS) / 1000000 ))
    printf 'Infraestructura reutilizada; inicio de mision tras %s ms.\n' \
        "${elapsed_ms}"
    say "Enviando goal /grasp_object  (pieza: ${piece}, accion: ${action})"
    in_container "ros2 action send_goal /grasp_object \
        home_service_interfaces/action/PickPlace \
        '{operation: ${action}, target_pose_name: ${piece}, retreat_pose_name: home}' --feedback"
}

grasp() {
    confirm_motion
    local piece="${1:-auto}"
    [ -n "${1:-}" ] || die "uso: grasp <pieza> --action pick|place [--table-height mm]"
    shift
    local action="" table_mm=""
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --action) action="${2:?falta valor para --action}"; shift 2 ;;
            --table-height|--height) table_mm="${2:?falta valor para --table-height}"; shift 2 ;;
            *) die "opcion desconocida para grasp: $1" ;;
        esac
    done
    case "${action}" in pick|place) ;; *) die "la accion es obligatoria: --action pick|place" ;; esac
    case "${table_mm}" in '' ) die "indica --table-height mm; no se adivina la altura" ;; *[!0-9]*) die "altura invalida: ${table_mm}" ;; esac
    require_prepared_grasp "${piece}" "${table_mm}"
    grasp_send "${piece}" "${action}"
}

pick() {
    confirm_motion
    local piece="${1:-auto}" table_mm="${TABLE_HEIGHT_MM:-}"
    shift || true
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --table-height|--height) table_mm="${2:?falta valor para --table-height}"; shift 2 ;;
            *) die "opcion desconocida para pick: $1" ;;
        esac
    done
    [ -n "${table_mm}" ] || die "indica --table-height mm; no se adivina la altura"
    require_prepared_grasp "${piece}" "${table_mm}"
    grasp_send "${piece}" pick
}

grasp_dry() {
    # Ensayo: identifica la pieza y calcula el agarre, pero no manda
    # nada al brazo ni a las ruedas. Es la forma de validar el catalogo
    # y el mapa ArUco->pieza sin riesgo.
    say "ENSAYO en seco: sin brazo y sin mover la base"
    local piece="${1:-auto}"
    grasp_stack false false "${piece}"
    grasp_send "${piece}" pick
}

aruco_loss() {
    ensure_container
    require_distributed_aruco
    local marker_id="${1:?uso: aruco-loss <id_aruco> [segundos]}"
    local seconds="${2:-30}"
    in_container "python3 /workspace/scripts/measure_aruco_loss.py '${marker_id}' '${seconds}'"
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
    require_prepared_grasp auto "${PREPARE_GRASP_DEFAULT_TABLE_MM}"
    printf 'Pila lista. Lanza la toma con:\n'
    printf '%s\n' '  ALLOW_MOTION=1 DISTRIBUTED=1 LAPTOP_IP=<ip> \' \
        '  ./scripts/tsummit.sh grasp <engranaje|poste|rueda|auto>'
}

reto2() {
    confirm_motion
    say "Reto 2 — Kitting"
    require_prepared_grasp auto "${PREPARE_GRASP_DEFAULT_TABLE_MM}"
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
    local target="${1:-}"
    local pattern=""
    local invalidate_prepare=0

    if [ -n "${target}" ]; then
        if [ "${target}" = "help" ]; then
            printf '%s\n' \
                'Uso: ./scripts/tsummit.sh stop [nodo]' \
                'Sin nodo detiene todos los stacks locales.' \
                'Nodos: arm grasp approach detector camera lidar odom scan mux model slam rviz foxglove'
            return
        fi
        case "${target}" in
            arm|mecharm|mecharm_driver_node)
                pattern='[m]echarm_driver_node'
                invalidate_prepare=1
                ;;
            grasp|object_grasp_server)
                pattern='[o]bject_grasp_server'
                invalidate_prepare=1
                ;;
            approach|aruco_lidar_approach_server)
                pattern='[a]ruco_lidar_approach_server'
                invalidate_prepare=1
                ;;
            detector|aruco_detector|aruco_detector_node)
                pattern='[a]ruco_detector_node'
                invalidate_prepare=1
                ;;
            camera|csi_camera|csi_camera_node)
                pattern='[c]si_camera_node'
                invalidate_prepare=1
                ;;
            lidar|ydlidar|ydlidar_ros2_driver_node)
                pattern='[y]dlidar_ros2_driver_node'
                invalidate_prepare=1
                ;;
            odom|myagv_odometry_node)
                pattern='[m]yagv_odometry_node'
                invalidate_prepare=1
                ;;
            scan|scan_sanitizer|scan_sanitizer_node)
                pattern='[s]can_sanitizer_node'
                invalidate_prepare=1
                ;;
            mux|twist_mux)
                pattern='[r]os2 run twist_mux twist_mux'
                invalidate_prepare=1
                ;;
            model|robot_state_publisher)
                pattern='[r]obot_state_publisher'
                invalidate_prepare=1
                ;;
            slam|slam_toolbox)
                pattern='[s]lam_toolbox'
                ;;
            rviz|rviz2)
                pattern='[r]viz2'
                ;;
            foxglove|foxglove_bridge)
                pattern='[f]oxglove_bridge'
                ;;
            *)
                die "nodo desconocido: '${target}'. Usa 'tsummit.sh stop help'."
                ;;
        esac

        "${DOCKER[@]}" exec "${CONTAINER}" bash -lc \
            "if pgrep -f '${pattern}' >/dev/null; then \
                 pkill -INT -f '${pattern}'; sleep 2; \
                 pgrep -f '${pattern}' >/dev/null && pkill -KILL -f '${pattern}' || true; \
                 echo 'Nodo detenido: ${target}'; \
             else \
                 echo 'Nodo no estaba activo: ${target}'; \
              fi"
        [ "${invalidate_prepare}" -eq 1 ] && clear_prepare_state
        return
    fi

    "${DOCKER[@]}" exec "${CONTAINER}" bash -lc \
        "pkill -KILL -f '[o]bject_grasp_server' || true; \
          pkill -KILL -f '[m]echarm_driver_node' || true" 2>/dev/null || true
    clear_prepare_state
    "${MAZE}" stop 2>/dev/null || true
    routine stop
}

# =====================================================================

case "${1:-help}" in
    build)        shift; build "$@" ;;
    doctor|check-integrity) doctor ;;
    mapping|mapear) mapping ;;
    save-map)     shift; save_map "$@" ;;

    arm|brazo)    arm ;;
    calibrate-grasp|calibrar-agarre) shift; calibrate_grasp "$@" ;;
    test-grasp|probar-agarre) shift; test_grasp_calibration "$@" ;;
    test-pick-lift|probar-pick-lift) shift; test_pick_lift "$@" ;;
    perception|vision) perception ;;

    approach-check|aprox-check) approach_check ;;
    approach|aproximar) shift; approach "$@" ;;

    prepare|preparar) shift; prepare "$@" ;;

    pick|tomar)         shift; pick "$@" ;;
    grasp)              shift; grasp "$@" ;;
    grasp-dry|ensayo) shift; grasp_dry "${1:-auto}" ;;
    aruco-loss)      shift; aruco_loss "$@" ;;
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
    stop)         shift; stop "$@" ;;

    help|-h|--help)
        cat <<'EOF'
T-SUMMIT Challenge — consola unica

  MAPEO
    mapping                 base -> slam -> rviz -> teleop (todo en orden)
    save-map <nombre>       guarda /workspace/maps/<nombre>.{yaml,pgm}

  APROXIMACION A UN ARUCO
    prepare grasp [pieza] [--table-height mm]
                             arranca y comprueba la pila reutilizable;
                             no mueve el robot. Repite este comando para
                             recuperar una dependencia parada.
    approach-check         arranca base + camara y comprueba entradas, NO mueve
    approach <id> [stop_m] aproxima la BASE al marcador <id> (exige ALLOW_MOTION=1,
                            DISTRIBUTED=1 y LAPTOP_IP=<ip>)
                            stop_m = distancia LiDAR-pared, por defecto 0.20 m;
                                    la seguridad usa despeje del footprint

  TOMA DE PIEZA  (retos 1 y 2)
    grasp-dry [pieza]       ENSAYO: identifica y calcula, no mueve nada
     pick [pieza]            cadena completa de pick y termina en home (exige ALLOW_MOTION=1,
                              DISTRIBUTED=1 y LAPTOP_IP=<ip>)
     grasp <pieza> --action pick|place --table-height mm
                             cadena completa con accion y altura explicitas;
                             exige haber ejecutado prepare grasp antes
    aruco-loss <id> [seg]   mide flujo y ausencia de un ArUco sin mover
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
     calibrate-grasp <pieza> <altura_mm> --action pick|place
                             primero captura/guarda home y despues ensena
                             intermedio, preagarre y contacto para esa pieza
                             (engranaje|poste|rueda|estrella) sobre una
                             plataforma de <altura_mm> (100 o 200); los guarda
                             anidados en grasp_calibrations.yaml y termina en
                             home. Anade --capture-global-poses para ensenar
                              todas las poses de poses.yaml (exige
                              ALLOW_MOTION=1 y driver parado)
     calibrate-grasp --menu   menu para listar, ver, editar, recapturar o
                              eliminar calibraciones con confirmacion
     test-grasp <pieza> <altura_mm>
                              recorre las poses calibradas sin mover base ni pinza
                              (exige ALLOW_MOTION=1)
                              admite --action pick|place
    test-pick-lift <pieza> <altura_mm>
                             abre, baja a contacto, cierra y eleva a preagarre;
                             vuelve a home; solo brazo y pinza, deja el objeto
                             sujeto.
                             Usa por defecto torque=200 y corriente=500;
                             permite ajustar con --gripper-torque 150..980 y
                             --gripper-protect-current 1..500
                             (exige ALLOW_MOTION=1)
    perception              camara CSI + detector ArUco
    rviz / teleop           visualizacion y mando
    viz [rviz|foxglove|none]  abre la visualizacion elegida (o ninguna)

  UTIL
    build [paquetes]        colcon build dentro del contenedor
    doctor                  verifica dependencias, paquetes y archivos criticos
    status                  nodos, acciones y procesos clave
    logs <nombre> [n]       tail -f de un log
    stop [nodo]             sin nodo, para todos los stacks
                            nodo = arm|grasp|approach|detector|camera|lidar|odom|
                                   scan|mux|model|slam|rviz|foxglove

  Todo lo que mueve el robot exige ALLOW_MOTION=1 en cada llamada.
  El repo esta montado en /workspace: editar aqui = editar dentro del
  contenedor. Solo hay que 'build' al anadir nodos o entry points.
EOF
        ;;
    *) die "comando desconocido: $1  (prueba: help)" ;;
esac
