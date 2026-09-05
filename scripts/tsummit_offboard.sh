#!/usr/bin/env bash
# =====================================================================
#  T-SUMMIT — procesamiento FUERA del robot (portatil / servidor)
# ---------------------------------------------------------------------
#  Este script se ejecuta EN EL PORTATIL, no en la Jetson.
#
#  Por que existe: el FAQ oficial del T-SUMMIT recomienda cómputo
#  distribuido para exactamente este sintoma ("image recognition ...
#  insufficient computing power"). En la Nano el detector ArUco llegaba
#  a 1.6 nucleos y la deteccion caia a 0.1 Hz; el robot giraba buscando
#  un marcador que tenia delante.
#
#  REPARTO
#    Jetson  : camara CSI, LiDAR, odometria/motores, twist_mux,
#              scan_sanitizer, driver del MechArm.   (drivers + seguridad)
#    Portatil: detector ArUco, aproximacion, agarre, Nav2/SLAM, RViz.
#
#  LA IMAGEN VIAJA COMPRIMIDA. Medido en el robot:
#      cruda       246.8 Mbit/s   (1518 KB/frame)  -> imposible por WiFi
#      comprimida    6.1 Mbit/s   (  35 KB/frame)  -> trivial
#  y con la MISMA tasa de deteccion (verificado cuadro a cuadro).
#
#  USO
#      # 1. una vez, en ambas maquinas: mira 'check'
#      ROBOT_IP=192.168.43.10 LAPTOP_IP=192.168.43.20 \
#          ./scripts/tsummit_offboard.sh check
#
#      # 2. arrancar el procesamiento
#      ROBOT_IP=192.168.43.10 LAPTOP_IP=192.168.43.20 \
#          ./scripts/tsummit_offboard.sh run
#
#      # imprimir los exports para usar ros2 a mano en otra terminal
#      eval "$(ROBOT_IP=... LAPTOP_IP=... ./scripts/tsummit_offboard.sh env)"
# =====================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROS_DISTRO_USED="${ROS_DISTRO_USED:-humble}"
# OJO: el contenedor del robot se crea con ROS_DOMAIN_ID=30
# (docker/run_jetson_robot.sh). Si tu shell trae otro valor -muy comun,
# p.ej. 1- y lo heredamos, las dos maquinas quedan en dominios DDS
# distintos y NO se ven nunca, sin ningun error visible. Por eso manda
# el del robot; para cambiarlo de verdad usa ROBOT_DOMAIN_ID.
ROBOT_DOMAIN_ID="${ROBOT_DOMAIN_ID:-30}"
if [ -n "${ROS_DOMAIN_ID:-}" ] \
   && [ "${ROS_DOMAIN_ID}" != "${ROBOT_DOMAIN_ID}" ]; then
    printf 'AVISO: tu shell tiene ROS_DOMAIN_ID=%s pero el robot usa %s.\n' \
        "${ROS_DOMAIN_ID}" "${ROBOT_DOMAIN_ID}" >&2
    printf '       Se usara %s (si de verdad quieres otro: ROBOT_DOMAIN_ID=<n>\n' \
        "${ROBOT_DOMAIN_ID}" >&2
    printf '       aqui Y ROS_DOMAIN_ID=<n> al crear el contenedor del robot).\n' >&2
fi
ROS_DOMAIN_ID="${ROBOT_DOMAIN_ID}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# IPs en la red 5 GHz. Sin esto no hay descubrimiento fiable.
ROBOT_IP="${ROBOT_IP:-}"
LAPTOP_IP="${LAPTOP_IP:-}"

die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

# IPv4 propia por la que sale el trafico, o la de WIFI_IFACE si se indica.
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

has_local_ip() {
    ip -4 -o addr show 2>/dev/null | awk '{print $4}' | cut -d/ -f1 \
        | grep -qx "$1"
}

require_ips() {
    # LAPTOP_IP se detecta sola si no se pasa: el DHCP de un hotspot
    # cambia las IPs entre arranques y una IP obsoleta hace que Cyclone
    # no pueda hacer bind. Los nodos entonces mueren con "rcl node's rmw
    # handle is invalid", un error que no menciona la red por ningun
    # lado. Mejor detectarla y validarla que perseguir ese fantasma.
    if [ -z "${LAPTOP_IP}" ]; then
        LAPTOP_IP="$(own_ip)"
        [ -n "${LAPTOP_IP}" ] \
            && printf 'LAPTOP_IP detectada: %s\n' "${LAPTOP_IP}" >&2
    fi
    [ -n "${ROBOT_IP}" ] || die "define ROBOT_IP=<ip de la Jetson en la red 5 GHz>"
    [ -n "${LAPTOP_IP}" ] || die "define LAPTOP_IP=<ip de esta maquina>"

    if ! has_local_ip "${LAPTOP_IP}"; then
        printf 'ERROR: LAPTOP_IP=%s no esta en ninguna interfaz de esta maquina.\n' \
            "${LAPTOP_IP}" >&2
        printf '       IPs actuales: %s\n' \
            "$(ip -4 -o addr show | awk '{print $4}' | cut -d/ -f1 | grep -v '^127' | tr '\n' ' ')" >&2
        die "corrige LAPTOP_IP (o dejala vacia para autodetectarla)"
    fi
}

# ---------------------------------------------------------------------
# CycloneDDS distribuido
# ---------------------------------------------------------------------
# El contenedor del robot usa por defecto una config atada a 'lo'
# (loopback): perfecta para todo-en-la-Jetson y RADICALMENTE incompatible
# con esto. Aqui se declara la interfaz real y, sobre todo, PEERS
# UNICAST: el multicast sobre WiFi (y mas en un hotspot de movil) se
# pierde o lo filtra el AP, asi que no se depende de el.
dds_uri() {
    cat <<EOF
<CycloneDDS><Domain>
  <General>
    <Interfaces><NetworkInterface address="${LAPTOP_IP}"/></Interfaces>
    <AllowMulticast>false</AllowMulticast>
  </General>
  <Discovery>
    <ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
    <Peers>
      <Peer address="${ROBOT_IP}"/>
      <Peer address="${LAPTOP_IP}"/>
    </Peers>
  </Discovery>
</Domain></CycloneDDS>
EOF
}

# ---------------------------------------------------------------------
# bundle: empaqueta SOLO lo que el portatil necesita
# ---------------------------------------------------------------------
# No hace falta clonar el workspace entero (13 paquetes, la mayoria son
# drivers del robot: camara CSI, odometria, LiDAR, brazo). El lado
# externo solo necesita 3:
#
#   home_service_interfaces  <- OBLIGATORIO y debe ser IDENTICO al del
#                               robot: define los .msg/.action. Si las
#                               definiciones difieren, DDS calcula otro
#                               hash de tipo y los topics NO conectan,
#                               sin dar ningun error legible.
#   home_service_perception  <- detector ArUco
#   home_service_behaviors   <- aproximacion, agarre y offboard.launch.py
#
# El resto de dependencias salen de apt (rclpy, cv_bridge, tf2_ros,
# nav_msgs, sensor_msgs) y ya vienen con un ros-humble-desktop.
bundle() {
    local out="${1:-/tmp/tsummit_offboard.tar.gz}"
    local pkgs="home_service_interfaces home_service_perception home_service_behaviors"

    local p
    for p in ${pkgs}; do
        [ -d "${ROOT}/src/${p}" ] || die "falta ${ROOT}/src/${p}"
    done

    tar czf "${out}" -C "${ROOT}" \
        --exclude='__pycache__' --exclude='*.pyc' \
        $(for p in ${pkgs}; do printf 'src/%s ' "${p}"; done) \
        scripts/tsummit_offboard.sh

    say "Paquete listo: ${out}"
    cat <<EOF
Contiene: ${pkgs}
          scripts/tsummit_offboard.sh

En el PORTATIL:
    mkdir -p ~/tsummit_ws && cd ~/tsummit_ws
    tar xzf $(basename "${out}")
    sudo apt install ros-humble-cv-bridge ros-humble-tf2-ros \
                     ros-humble-rmw-cyclonedds-cpp python3-opencv
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    ROBOT_IP=<ip-jetson> ./scripts/tsummit_offboard.sh check

IMPORTANTE: home_service_interfaces debe ser el MISMO en las dos
maquinas. Si tocas un .msg o .action, vuelve a pasar el bundle y
recompila en ambas, o los topics dejaran de conectar en silencio.
EOF
}

print_env() {
    require_ips
    printf 'export ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID}"
    printf 'export RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION}"
    printf "export CYCLONEDDS_URI='%s'\n" "$(dds_uri | tr -d '\n')"
}

# ---------------------------------------------------------------------
# Comprobaciones previas: fallar aqui es barato, fallar en pista no.
# ---------------------------------------------------------------------
check() {
    require_ips
    local fail=0

    say "1. Alcance y latencia hacia el robot (${ROBOT_IP})"
    if ping -c 20 -i 0.2 -W 1 "${ROBOT_IP}" 2>/dev/null | tail -2; then
        printf '   Mira mdev/max: un mdev alto o max > 100 ms hara que Nav2\n'
        printf '   y la aproximacion vayan a tirones.\n'
    else
        printf '   FALLO: no hay ping.\n'
        printf '   Causa tipica en hotspot de movil: AISLAMIENTO DE CLIENTES\n'
        printf '   (AP isolation) activado; los dispositivos ven Internet pero\n'
        printf '   NO se ven entre si. Desactivalo en los ajustes del hotspot.\n'
        fail=1
    fi

    say "2. Banda y canal WiFi de esta maquina"
    if command -v iw >/dev/null 2>&1; then
        iw dev 2>/dev/null | awk '/Interface/{i=$2} /channel/{print "   "i": "$0}' || true
        printf '   Debe ser 5 GHz y, preferible, canal 36-48 (UNII-1).\n'
        printf '   EVITA canales DFS (52-144): el AP puede verse obligado a\n'
        printf '   cambiar de canal por deteccion de radar EN MITAD de una\n'
        printf '   carrera, y se corta el enlace.\n'
    else
        printf '   (instala "iw" para ver banda y canal)\n'
    fi

    say "3. Sincronia de reloj"
    if command -v chronyc >/dev/null 2>&1; then
        chronyc tracking 2>/dev/null | grep -E "System time|Last offset" || true
    else
        printf '   chrony NO instalado. TF entre dos maquinas necesita relojes\n'
        printf '   sincronizados o todo falla con "message too old".\n'
        printf '   sudo apt install chrony  (en ambas; el portatil hace de\n'
        printf '   servidor y la Jetson apunta a %s)\n' "${LAPTOP_IP}"
        fail=1
    fi

    say "4. Entorno ROS"
    printf '   ROS_DOMAIN_ID=%s  (el contenedor del robot usa %s)\n' \
        "${ROS_DOMAIN_ID}" "${ROBOT_DOMAIN_ID}"
    if [ "${ROS_DOMAIN_ID}" != "${ROBOT_DOMAIN_ID}" ]; then
        printf '   FALLO: dominios distintos. Con dominios DDS distintos las\n'
        printf '          dos maquinas NO se ven, y sin ningun mensaje de error.\n'
        fail=1
    fi
    printf '   RMW=%s\n' "${RMW_IMPLEMENTATION}"
    if [ "${RMW_IMPLEMENTATION}" != "rmw_cyclonedds_cpp" ]; then
        printf '   FALLO: el robot usa rmw_cyclonedds_cpp. Dos RMW distintos\n'
        printf '          tampoco se hablan.  sudo apt install ros-%s-rmw-cyclonedds-cpp\n' \
            "${ROS_DISTRO_USED}"
        fail=1
    fi
    if [ -f "/opt/ros/${ROS_DISTRO_USED}/setup.bash" ]; then
        printf '   ROS 2 %s encontrado.\n' "${ROS_DISTRO_USED}"
    else
        printf '   FALLO: no existe /opt/ros/%s\n' "${ROS_DISTRO_USED}"
        fail=1
    fi

    say "5. ¿Se ven los topics del robot?"
    printf '   (necesita que el robot ya este arrancado con DISTRIBUTED=1)\n'
    # shellcheck disable=SC1090
    if [ -f "/opt/ros/${ROS_DISTRO_USED}/setup.bash" ]; then
        (
            # Los setup.bash de ROS leen variables sin definir
            # (AMENT_TRACE_SETUP_FILES y compania): con 'set -u' revientan
            # con "unbound variable". Se desactiva solo para el source.
            set +u
            source "/opt/ros/${ROS_DISTRO_USED}/setup.bash"
            [ -f "${ROOT}/install/setup.bash" ] && source "${ROOT}/install/setup.bash"
            set -u
            export ROS_DOMAIN_ID RMW_IMPLEMENTATION
            CYCLONEDDS_URI="$(dds_uri | tr -d '\n')"
            export CYCLONEDDS_URI
            timeout 8 ros2 topic list 2>/dev/null | grep -E \
                "/camera/image_raw/compressed|/scan_filtered|/odom|/cmd_vel" \
                || printf '   (todavia no se ve nada del robot)\n'
        ) || true
    fi

    printf '\n'
    [ "${fail}" -eq 0 ] && say "Comprobaciones OK" || say "Hay fallos que arreglar antes de seguir"
    return "${fail}"
}

run() {
    require_ips
    say "Procesamiento externo (robot=${ROBOT_IP}  esta maquina=${LAPTOP_IP})"

    [ -f "/opt/ros/${ROS_DISTRO_USED}/setup.bash" ] \
        || die "no existe /opt/ros/${ROS_DISTRO_USED}"
    [ -f "${ROOT}/install/setup.bash" ] \
        || die "compila primero el workspace aqui: colcon build"

    # 'set +u' alrededor del source: los setup.bash de ROS leen
    # variables sin definir y con nounset abortan con "unbound variable".
    set +u
    # shellcheck disable=SC1090,SC1091
    source "/opt/ros/${ROS_DISTRO_USED}/setup.bash"
    # shellcheck disable=SC1090,SC1091
    source "${ROOT}/install/setup.bash"
    set -u

    export ROS_DOMAIN_ID RMW_IMPLEMENTATION
    CYCLONEDDS_URI="$(dds_uri | tr -d '\n')"
    export CYCLONEDDS_URI

    exec ros2 launch home_service_behaviors offboard.launch.py "$@"
}

case "${1:-help}" in
    check)  check ;;
    bundle) shift; bundle "${1:-/tmp/tsummit_offboard.tar.gz}" ;;
    run)    shift; run "$@" ;;
    env)    print_env ;;
    help|-h|--help)
        sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
        ;;
    *) die "comando desconocido: $1  (check | run | bundle | env | help)" ;;
esac
