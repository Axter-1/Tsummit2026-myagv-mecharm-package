#!/usr/bin/env bash
# =====================================================================
#  JARVIS — Lanzador de Estación Base (PC / Laptop) para myAGV
# ---------------------------------------------------------------------
#  Este script corre 100% en la PC Externa.
#  Conecta con el myAGV en la Jetson Nano mediante CycloneDDS Unicast
#  y ejecuta el asistente de voz con Gemini 1.5 Flash.
#
#  USO:
#      export GEMINI_API_KEY="AIzaSy..."
#      ROBOT_IP=10.24.15.48 ./scripts/run_jarvis.sh
#
#      # O en modo texto si no tienes micrófono a mano:
#      ROBOT_IP=10.24.15.48 ./scripts/run_jarvis.sh --mode text
# =====================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROS_DISTRO_USED="${ROS_DISTRO_USED:-humble}"
ROBOT_DOMAIN_ID="${ROBOT_DOMAIN_ID:-30}"
ROS_DOMAIN_ID="${ROBOT_DOMAIN_ID}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

ROBOT_IP="${ROBOT_IP:-}"
LAPTOP_IP="${LAPTOP_IP:-}"

die() { printf '\033[31m[ERROR] %s\033[0m\n' "$*" >&2; exit 1; }
say() { printf '\n\033[1;34m== %s\033[0m\n' "$*"; }

# Autodetección de la IP de la PC en la red local
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

if [ -z "${LAPTOP_IP}" ]; then
    LAPTOP_IP="$(own_ip)"
fi

[ -n "${ROBOT_IP}" ] || die "Debes definir ROBOT_IP=<ip_jetson_nano>. Ejemplo: ROBOT_IP=10.24.15.48 $0"
GEMINI_API_KEY="${GEMINI_API_KEY:-AIzaSyCbe1HyDs9kbXhoFpmiif26GfdTHdz6iAE}"
export GEMINI_API_KEY

say "1. Configurando CycloneDDS Unicast (PC=${LAPTOP_IP} <-> Robot=${ROBOT_IP})"

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

CYCLONEDDS_URI="$(dds_uri | tr -d '\n')"
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION
export CYCLONEDDS_URI

say "2. Cargando entorno de trabajo ROS 2"
set +u
if [ -f "/opt/ros/${ROS_DISTRO_USED}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO_USED}/setup.bash"
fi
if [ -f "${ROOT}/install/setup.bash" ]; then
    source "${ROOT}/install/setup.bash"
fi
set -u

# Modo chequeo de enlace DDS
if [ "${1:-}" = "check" ]; then
    say "Comprobando enlace DDS con el robot..."
    printf "\n1. Tópico /odom (debe tener Publisher count >= 1):\n"
    ros2 topic info /odom 2>/dev/null || printf "   [FALLO] No responde el daemon de ROS 2.\n"
    printf "\n2. Suscriptores de /cmd_vel_aruco (debe listar 'twist_mux'):\n"
    ros2 topic info -v /cmd_vel_aruco 2>/dev/null | grep -A2 -i "subscription" || printf "   [AVISO] No se detecta twist_mux aún.\n"
    exit 0
fi

if [ "${ALLOW_MOTION:-0}" = "1" ]; then
    say "AVISO DE SEGURIDAD: ALLOW_MOTION=1 detectado. Actuadores físicos habilitados."
    export ALLOW_MOTION=1
fi

say "3. Iniciando JARVIS Voice Assistant"
export PYTHONPATH="${ROOT}:${PYTHONPATH:-}"
exec python3 -m voice_assistant.main "$@"
