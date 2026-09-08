#!/usr/bin/env python3
"""Agarre inteligente: identifica, verifica, agarra y confirma.

CORRE EN EL PORTATIL. No añade ni un ciclo de CPU a la Jetson: consume la
imagen comprimida que el robot YA publica y manda al brazo por las
acciones que YA existen. No modifica ningun fichero del pipeline actual.

Que añade sobre scripts/test_pick_lift.py (del que hereda la secuencia):

  1. IDENTIFICA la pieza en vez de fiarse del argumento. Cruza tres
     senales: ArUco, geometria local (OpenCV, decenas de ms) y, solo si
     las dos primeras dudan o discrepan, UNA consulta a gemini-3.8-flash.
  2. VERIFICA el agarre tras cerrar: lectura de la pinza + confirmacion
     visual. Si se cerro al aire, lo dice en vez de seguir como si nada.
  3. REINTENTA con correccion en vez de abortar al primer fallo.
  4. REGISTRA por que tomo cada decision, para poder depurar en pista.

La IA NUNCA esta en un bucle de control. Como mucho 2 llamadas por
agarre, con timeout duro, y si falla o tarda se sigue con la decision
local. Con --no-ai el script funciona entero sin red.

Uso (con el driver del brazo arriba y la pila offboard corriendo):

    export GEMINI_API_KEY=...
    python3 scripts/smart_pick.py auto  100            # identifica el solo
    python3 scripts/smart_pick.py rueda 100            # se lo dices tu
    python3 scripts/smart_pick.py auto  100 --dry-run  # sin mover el brazo
    python3 scripts/smart_pick.py auto  100 --no-ai    # sin red
    python3 scripts/smart_pick.py --benchmark          # mide la latencia IA
"""

import argparse
import os
import sys
import time

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from smart_vision import (  # noqa: E402
    PIEZAS, ReconocedorLocal, VerificadorGemini, decide_pieza, decodifica,
)

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CALIBRACIONES = os.path.join(
    RAIZ, "src", "home_service_behaviors", "config", "grasp_calibrations.yaml"
)
CATALOGO = os.path.join(
    RAIZ, "src", "home_service_behaviors", "config", "grasp_catalog.yaml"
)
POSES = os.path.join(
    RAIZ, "src", "myagv_mecharm_service", "config", "poses.yaml"
)

CLAVES_ANGULOS = (
    "intermediate_joint_angles", "pregrasp_joint_angles",
    "contact_joint_angles",
)
CLAVES_COORDS = (
    "intermediate_coords", "pregrasp_coords", "contact_coords",
)

# Diametro fisico aparente visto desde arriba (mm), de los STL. Se usa
# para deducir la escala mm/pixel sin necesitar la calibracion de la
# camara: si la pieza mide 152.8 mm y ocupa D pixeles, ya tenemos la
# escala en el plano de la pieza.
DIAMETRO_MM = {
    "engranaje": 152.8,
    "rueda": 150.0,
    "poste": 110.0,
    "estrella": 150.0,
}

# Tope duro de la correccion de pose. Un mapeo de ejes mal configurado
# NUNCA puede mandar el brazo lejos: como mucho mueve esto.
CORRECCION_MAX_MM = 15.0

HOME_TOLERANCIA_DEG = 3.5
HOME_INTENTOS = 2


# =====================================================================
#  Carga de configuracion (mismo formato que test_pick_lift.py)
# =====================================================================

def carga_calibracion(ruta, pieza, altura_mm):
    if not os.path.isfile(ruta):
        raise RuntimeError(f"no existe el fichero de calibraciones: {ruta}")
    with open(ruta, "r", encoding="utf-8") as fh:
        datos = yaml.safe_load(fh) or {}
    cal = (datos.get("calibrations", {}) or {}).get(pieza)
    if not isinstance(cal, dict):
        disponibles = sorted((datos.get("calibrations", {}) or {}))
        raise RuntimeError(
            f"no hay calibracion para '{pieza}' (hay: {disponibles or 'ninguna'})"
        )
    if not any(k in cal for k in CLAVES_ANGULOS):
        cal = cal.get(str(altura_mm))
    if not isinstance(cal, dict):
        raise RuntimeError(
            f"no hay calibracion de '{pieza}' para {altura_mm} mm"
        )

    modo = str(cal.get("execution_mode", "coords")).lower()
    if modo == "coords" and all(k in cal for k in CLAVES_COORDS):
        poses = [[float(v) for v in cal[k]] for k in CLAVES_COORDS]
        return "coords", poses
    poses = []
    for k in CLAVES_ANGULOS:
        v = cal.get(k)
        if not isinstance(v, (list, tuple)) or len(v) != 6:
            raise RuntimeError(f"calibracion invalida: falta {k}")
        poses.append([float(x) for x in v])
    return "angles", poses


def carga_pinza(ruta, pieza):
    try:
        with open(ruta, "r", encoding="utf-8") as fh:
            cat = (yaml.safe_load(fh) or {}).get("grasp_catalog", {})
        g = cat.get("gripper", {})
        spec = (cat.get("objects", {}) or {}).get(pieza, {})
        stroke = float(g.get("stroke_mm", 45.0))
        abre = float(spec.get("open_mm", stroke))
        cierra = float(spec.get("close_mm", 2.0))
        clamp = lambda mm: int(round(max(0.0, min(100.0, 100.0 * mm / stroke))))
        return clamp(abre), clamp(cierra)
    except (OSError, TypeError, ValueError):
        return 100, 20


def carga_pose(ruta, nombre):
    with open(ruta, "r", encoding="utf-8") as fh:
        poses = (yaml.safe_load(fh) or {}).get("poses", {})
    ang = poses.get(nombre)
    if not isinstance(ang, (list, tuple)) or len(ang) != 6:
        raise RuntimeError(f"no hay pose '{nombre}' valida en {ruta}")
    return [float(v) for v in ang]


def carga_mapa_aruco(ruta):
    try:
        with open(ruta, "r", encoding="utf-8") as fh:
            cat = (yaml.safe_load(fh) or {}).get("grasp_catalog", {})
        return {int(k): str(v) for k, v in (cat.get("aruco_to_object") or {}).items()}
    except (OSError, TypeError, ValueError):
        return {}


def error_articular(indice, real, objetivo):
    d = float(real) - float(objetivo)
    if indice == 5:                      # J6 da la vuelta por +-180
        return abs((d + 180.0) % 360.0 - 180.0)
    return abs(d)


# =====================================================================
#  Nodo
# =====================================================================

def construye_nodo():
    """Importa ROS aqui para que --benchmark funcione sin entorno ROS."""
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    from home_service_interfaces.action import MoveArm
    from home_service_interfaces.msg import ArucoDetectionArray
    from home_service_interfaces.srv import SetGripper

    class AgarreInteligente(Node):

        def __init__(self, topico_imagen, topico_detecciones):
            super().__init__("smart_pick")
            self.cliente_brazo = ActionClient(self, MoveArm, "/mecharm/move_arm")
            self.cliente_pinza = self.create_client(
                SetGripper, "/mecharm/set_gripper"
            )
            self.ultima_imagen = None
            self.ultimas_detecciones = []
            self.create_subscription(
                CompressedImage, topico_imagen, self._imagen,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                ArucoDetectionArray, topico_detecciones, self._detecciones, 10
            )

        def _imagen(self, msg):
            self.ultima_imagen = msg

        def _detecciones(self, msg):
            self.ultimas_detecciones = list(msg.detections)

        # -- espera de datos -----------------------------------------

        def espera_imagen(self, timeout=10.0):
            fin = time.monotonic() + timeout
            while time.monotonic() < fin and self.ultima_imagen is None:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.ultima_imagen is None:
                raise RuntimeError(
                    "no llega imagen. Comprueba que la camara del robot "
                    "publica y que el DDS os ve (CYCLONEDDS_URI)."
                )
            return decodifica(self.ultima_imagen)

        def refresca(self, segundos=0.6):
            fin = time.monotonic() + segundos
            while time.monotonic() < fin:
                rclpy.spin_once(self, timeout_sec=0.05)
            return decodifica(self.ultima_imagen) if self.ultima_imagen else None

        # -- brazo ---------------------------------------------------

        def mueve(self, etiqueta, angulos=None, coords=None, velocidad=20.0):
            goal = MoveArm.Goal()
            goal.pose_name = "" if (angulos or coords) else "home"
            if angulos is not None:
                goal.joint_angles = angulos
            if coords is not None:
                goal.coords = coords
                goal.move_mode = 1
            goal.speed_percent = velocidad
            self.get_logger().info(f"Moviendo: {etiqueta}")
            fut = self.cliente_brazo.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            handle = fut.result()
            if handle is None or not handle.accepted:
                raise RuntimeError(f"objetivo rechazado en {etiqueta}")
            fut_res = handle.get_result_async()
            rclpy.spin_until_future_complete(self, fut_res)
            res = fut_res.result()
            if res is None or not res.result.success:
                msg = res.result.message if res else "sin respuesta"
                raise RuntimeError(f"fallo en {etiqueta}: {msg}")
            return list(res.result.final_joint_angles)

        def mueve_home(self, etiqueta, objetivo):
            ultimo = None
            for intento in range(1, HOME_INTENTOS + 1):
                real = self.mueve(etiqueta, velocidad=40.0)
                if len(real) == 6:
                    err = max(
                        error_articular(i, real[i], objetivo[i]) for i in range(6)
                    )
                    if err <= HOME_TOLERANCIA_DEG:
                        return
                    ultimo = f"{err:.2f} deg"
                    self.get_logger().warning(
                        f"{etiqueta}: home sin asentar ({ultimo}), "
                        f"intento {intento}/{HOME_INTENTOS}"
                    )
                else:
                    ultimo = "lectura invalida"
            raise RuntimeError(f"{etiqueta}: home no confirmado ({ultimo})")

        def pinza(self, etiqueta, valor, par=500, corriente=500):
            if not self.cliente_pinza.wait_for_service(timeout_sec=10.0):
                raise RuntimeError("/mecharm/set_gripper no disponible")
            req = SetGripper.Request()
            req.value = int(valor)
            req.speed_percent = 20.0
            req.torque = int(par)
            req.force_control = bool(par)
            req.protect_current = int(corriente)
            self.get_logger().info(f"Pinza: {etiqueta} ({valor})")
            fut = self.cliente_pinza.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            res = fut.result()
            if res is None or not res.success:
                msg = res.message if res else "sin respuesta"
                raise RuntimeError(f"fallo de pinza en {etiqueta}: {msg}")
            return res.current_value

    return rclpy, AgarreInteligente


# =====================================================================
#  Correccion de pose por vision
# =====================================================================

def calcula_correccion(deteccion, pieza, cfg, ancho_img, alto_img):
    """Desplazamiento (dx, dy) en mm para centrar la pieza bajo la pinza.

    La escala sale de la propia pieza: sabemos cuanto mide (STL) y cuantos
    pixeles ocupa, asi que no hacen falta los intrinsecos de la camara.

    El MAPEO DE EJES (que direccion de la imagen es +X del brazo) depende
    de como este montada la camara y NO se puede adivinar: va en el
    config y por defecto la correccion esta DESACTIVADA. Ver --refine.
    """
    diam_mm = DIAMETRO_MM.get(pieza)
    if not diam_mm or deteccion is None or deteccion.area_px <= 0:
        return None, "sin datos para estimar la escala"

    # Diametro equivalente del contorno: el del circulo de igual area.
    diam_px = 2.0 * (deteccion.area_px / 3.14159265) ** 0.5
    if diam_px < 10.0:
        return None, f"pieza demasiado pequeña en imagen ({diam_px:.0f} px)"
    mm_por_px = diam_mm / diam_px

    # Objetivo: donde deberia aparecer la pieza cuando la pose enseñada
    # es correcta. Por defecto, el centro del encuadre.
    obj_x = cfg.get("centro_objetivo_x", ancho_img / 2.0)
    obj_y = cfg.get("centro_objetivo_y", alto_img / 2.0)
    cx, cy = deteccion.centro_px

    err_px_x = cx - obj_x
    err_px_y = cy - obj_y
    err_mm_x = err_px_x * mm_por_px
    err_mm_y = err_px_y * mm_por_px

    # Mapeo imagen -> brazo, del config.
    sx = float(cfg.get("signo_x", 0.0))
    sy = float(cfg.get("signo_y", 0.0))
    if cfg.get("intercambia_ejes", False):
        dx_mm, dy_mm = sx * err_mm_y, sy * err_mm_x
    else:
        dx_mm, dy_mm = sx * err_mm_x, sy * err_mm_y

    # Tope duro: un mapeo mal puesto no puede tirar el brazo.
    norma = (dx_mm ** 2 + dy_mm ** 2) ** 0.5
    recortado = ""
    if norma > CORRECCION_MAX_MM:
        factor = CORRECCION_MAX_MM / norma
        dx_mm *= factor
        dy_mm *= factor
        recortado = f" (recortado de {norma:.1f} mm al tope {CORRECCION_MAX_MM})"

    detalle = (
        f"escala {mm_por_px:.3f} mm/px (pieza {diam_px:.0f} px = {diam_mm} mm); "
        f"error {err_px_x:+.0f},{err_px_y:+.0f} px -> "
        f"correccion {dx_mm:+.1f},{dy_mm:+.1f} mm{recortado}"
    )
    return (dx_mm, dy_mm), detalle


def aplica_correccion(poses, modo, correccion):
    """Suma la correccion XY al preagarre y al contacto. El intermedio no
    se toca: es un waypoint en el aire, no tiene por que ser exacto."""
    if correccion is None or modo != "coords":
        return poses, False
    dx, dy = correccion
    nuevas = [list(p) for p in poses]
    for i in (1, 2):                     # preagarre y contacto
        nuevas[i][0] += dx
        nuevas[i][1] += dy
    return nuevas, True


# =====================================================================
#  Benchmark de latencia de la IA
# =====================================================================

def benchmark(clave, modelo, repeticiones=3):
    import numpy as np
    print(f"Midiendo latencia de {modelo} ({repeticiones} llamadas)...\n")
    v = VerificadorGemini(clave, modelo, timeout=15.0,
                          presupuesto=repeticiones + 1, debug=False)
    if not v.disponible:
        print("ERROR: sin GEMINI_API_KEY.")
        return 1
    imagen = np.full((480, 640, 3), 60, dtype=np.uint8)
    import cv2
    cv2.circle(imagen, (320, 240), 110, (40, 90, 200), -1)
    cv2.circle(imagen, (320, 240), 38, (60, 60, 60), -1)

    tiempos = []
    for i in range(repeticiones):
        v._cache.clear()                 # forzar llamada real
        t0 = time.perf_counter()
        r = v.identifica(imagen)
        ms = (time.perf_counter() - t0) * 1000.0
        tiempos.append(ms)
        estado = "OK " if r else "FALLO"
        print(f"  {i + 1}. {estado} {ms:7.0f} ms  {r if r else ''}")
    if tiempos:
        print(
            f"\n  media {sum(tiempos) / len(tiempos):.0f} ms   "
            f"min {min(tiempos):.0f} ms   max {max(tiempos):.0f} ms"
        )
        print(
            "\nPara que esto NO afecte al agarre: se llama como mucho 2 "
            "veces por pieza, fuera del lazo de control, con timeout duro."
        )
    return 0


# =====================================================================
#  Programa principal
# =====================================================================

def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("pieza", nargs="?", choices=("auto",) + PIEZAS,
                   help="'auto' = identificarla por vision")
    p.add_argument("altura_mm", nargs="?", type=int,
                   help="altura de la plataforma (100 o 200)")
    p.add_argument("--no-ai", action="store_true",
                   help="sin ninguna llamada a Gemini (todo local)")
    p.add_argument("--dry-run", action="store_true",
                   help="identifica y decide, pero NO mueve el brazo")
    p.add_argument("--refine", action="store_true",
                   help="aplica la correccion XY por vision (exige "
                        "signo_x/signo_y calibrados en el config)")
    p.add_argument("--reintentos", type=int, default=1,
                   help="reintentos de agarre tras un fallo (por defecto 1)")
    p.add_argument("--modelo", default="gemini-3.8-flash")
    p.add_argument("--timeout-ia", type=float, default=8.0,
               help="MEDIDO: las respuestas buenas tardan 1.5-8 s")
    p.add_argument("--modelo-reserva", default="gemini-3.5-flash-lite",
               help="a usar cuando el principal devuelve 503")
    p.add_argument("--presupuesto-ia", type=int, default=2,
                   help="maximo de llamadas a Gemini por agarre")
    p.add_argument("--topico-imagen", default="/camera/image_raw/compressed")
    p.add_argument("--topico-detecciones", default="/aruco/detections")
    p.add_argument("--config", default=os.path.join(RAIZ, "config", "smart_pick.yaml"))
    p.add_argument("--par-pinza", type=int, default=500)
    p.add_argument("--corriente-pinza", type=int, default=500)
    p.add_argument("--benchmark", action="store_true",
                   help="mide la latencia real de la IA y sale")
    p.add_argument("--debug", action="store_true")
    args = p.parse_args()

    clave = os.environ.get("GEMINI_API_KEY", "")

    if args.benchmark:
        return benchmark(clave, args.modelo)

    if not args.pieza or args.altura_mm is None:
        p.error("hacen falta <pieza|auto> y <altura_mm> (o usa --benchmark)")

    cfg = {}
    if os.path.isfile(args.config):
        with open(args.config, "r", encoding="utf-8") as fh:
            cfg = (yaml.safe_load(fh) or {}).get("smart_pick", {}) or {}

    verificador = None
    if not args.no_ai:
        verificador = VerificadorGemini(
            clave, args.modelo, args.timeout_ia, args.presupuesto_ia,
            args.debug, args.modelo_reserva,
        )
        if not verificador.disponible:
            print(
                "AVISO: sin GEMINI_API_KEY -> se sigue solo con vision local.\n"
                "       export GEMINI_API_KEY=...  para activarla.\n"
            )

    reconocedor = ReconocedorLocal(debug=args.debug)
    mapa_aruco = carga_mapa_aruco(CATALOGO)

    rclpy, Clase = construye_nodo()
    rclpy.init()
    nodo = Clase(args.topico_imagen, args.topico_detecciones)
    bitacora = []

    try:
        # ---------- 1. mirar ----------
        print("== 1. MIRAR ==")
        imagen = nodo.espera_imagen()
        nodo.refresca(0.5)               # deja llegar detecciones ArUco
        alto, ancho = imagen.shape[:2]
        print(f"  imagen {ancho}x{alto}")

        deteccion = reconocedor.detecta(imagen)
        print(f"  geometria local ({reconocedor.ultimo_ms:.1f} ms): {deteccion}")

        pieza_aruco = None
        if nodo.ultimas_detecciones:
            ids = [d.id for d in nodo.ultimas_detecciones]
            for d in nodo.ultimas_detecciones:
                if d.id in mapa_aruco:
                    pieza_aruco = mapa_aruco[d.id]
                    break
            print(f"  ArUco visibles: {ids} -> pieza '{pieza_aruco}'")
        else:
            print("  ArUco: ninguno visible")

        # ---------- 2. decidir ----------
        print("\n== 2. IDENTIFICAR ==")
        home = carga_pose(POSES, "home")
        ya_en_home = False

        if args.pieza == "auto":
            # Si la IA va a hacer falta, se lanza AHORA en segundo plano
            # y mientras tanto el brazo se va a home, que es un
            # movimiento que la secuencia hace igual y tarda varios
            # segundos. Cuando el brazo llega, la respuesta ya esta: la
            # latencia de Gemini (medida, 1.8-4.1 s) deja de costar
            # tiempo de ciclo. Si aun no ha llegado, se sigue sin ella.
            asa = None
            hace_falta_ia = (
                verificador is not None and verificador.disponible
                and not (
                    pieza_aruco and deteccion
                    and pieza_aruco == deteccion.pieza
                )
            )
            if hace_falta_ia:
                print("  lanzando consulta a Gemini en segundo plano...")
                asa = verificador.lanza(
                    verificador.identifica, imagen,
                    deteccion.pieza if deteccion else None,
                )

            if not args.dry_run and asa is not None:
                if nodo.cliente_brazo.wait_for_server(timeout_sec=15.0):
                    nodo.mueve_home("home (mientras responde la IA)", home)
                    ya_en_home = True

            respuesta_ia = asa.recoge() if asa is not None else None
            if asa is not None:
                print(
                    f"  Gemini respondio en {verificador.ultimo_ms:.0f} ms"
                    if respuesta_ia else
                    "  Gemini no respondio a tiempo; se decide sin ella"
                )
            pieza, confianza, motivo = decide_pieza(
                deteccion, pieza_aruco, None, None,
                debug=args.debug, respuesta_ia=respuesta_ia,
            )
            if pieza is None:
                raise RuntimeError(f"no se pudo identificar la pieza: {motivo}")
            print(f"  -> {pieza}  (confianza {confianza:.2f})")
            print(f"     {motivo}")
            bitacora.append(f"identificacion: {motivo}")
        else:
            pieza = args.pieza
            confianza = 1.0
            print(f"  -> {pieza} (indicada a mano, sin identificar)")
            if deteccion and deteccion.pieza != pieza:
                print(
                    f"     AVISO: la geometria ve '{deteccion.pieza}' "
                    f"({deteccion.confianza:.2f}), no '{pieza}'."
                )
                bitacora.append(
                    f"discrepancia: pedida={pieza} vista={deteccion.pieza}"
                )

        # ---------- 3. cargar la calibracion ----------
        print("\n== 3. CALIBRACION ==")
        modo, poses = carga_calibracion(CALIBRACIONES, pieza, args.altura_mm)
        abre, cierra = carga_pinza(CATALOGO, pieza)
        print(f"  modo={modo}  pinza abre={abre} cierra={cierra}")

        correccion = None
        if args.refine:
            correccion, detalle = calcula_correccion(
                deteccion, pieza, cfg, ancho, alto
            )
            print(f"  correccion: {detalle}")
            if correccion and not (cfg.get("signo_x") or cfg.get("signo_y")):
                print(
                    "  AVISO: signo_x/signo_y sin calibrar en el config -> "
                    "correccion nula. Ver config/smart_pick.yaml."
                )
            poses, aplicada = aplica_correccion(poses, modo, correccion)
            if aplicada:
                bitacora.append(f"correccion aplicada: {detalle}")

        intermedio, preagarre, contacto = poses

        if args.dry_run:
            print("\n== DRY RUN: no se mueve el brazo ==")
            print(f"  intermedio: {[round(v, 1) for v in intermedio]}")
            print(f"  preagarre:  {[round(v, 1) for v in preagarre]}")
            print(f"  contacto:   {[round(v, 1) for v in contacto]}")
            if verificador:
                print(f"  {verificador.resumen()}")
            return 0

        # ---------- 4. ejecutar ----------
        if not nodo.cliente_brazo.wait_for_server(timeout_sec=15.0):
            raise RuntimeError(
                "/mecharm/move_arm no disponible; arranca el driver del brazo"
            )

        clave_mov = "coords" if modo == "coords" else "angles"
        def mv(etiqueta, pose):
            nodo.mueve(etiqueta, **{clave_mov: pose})

        exito = False
        for intento in range(args.reintentos + 1):
            etiqueta_intento = (
                "" if intento == 0 else f" (reintento {intento}/{args.reintentos})"
            )
            print(f"\n== 4. AGARRE{etiqueta_intento} ==")
            if not (ya_en_home and intento == 0):
                nodo.mueve_home("home", home)
            ya_en_home = False
            nodo.pinza("abrir", abre, args.par_pinza, args.corriente_pinza)
            mv("intermedio", intermedio)
            mv("preagarre", preagarre)
            mv("contacto", contacto)

            lectura = nodo.pinza(
                "cerrar", cierra, args.par_pinza, args.corriente_pinza
            )
            time.sleep(1.0)
            relectura = nodo.pinza(
                "reapriete", cierra, args.par_pinza, args.corriente_pinza
            )

            # ---------- 5. verificar ----------
            print("\n== 5. VERIFICAR ==")
            # La pinza parada ANTES del cierre pedido significa que hay algo
            # entre las mordazas. Que llegue al valor pedido significa que
            # se cerro al aire.
            hay_algo = relectura > cierra + 4
            print(
                f"  pinza: pedido={cierra} cierre={lectura} reapriete={relectura}"
                f"  -> {'algo entre las mordazas' if hay_algo else 'CERRO AL AIRE'}"
            )
            bitacora.append(
                f"intento {intento}: pinza {cierra}/{lectura}/{relectura}"
            )

            confirmado = hay_algo
            # La confirmacion visual se lanza en segundo plano: si la
            # pinza ya dice que hay algo, el brazo empieza a levantar
            # mientras Gemini mira. Si desmiente, se baja y reintenta.
            if verificador and verificador.disponible:
                img2 = nodo.refresca(0.5)
                if img2 is not None:
                    asa_v = verificador.lanza(
                        verificador.verifica_agarre, img2, pieza
                    )
                    r = asa_v.recoge()
                    if r is not None and "sujeta" in r:
                        visual = bool(r["sujeta"])
                        print(
                            f"  Gemini: sujeta={visual} ({verificador.ultimo_ms:.0f} ms)"
                            f" — {r.get('motivo', '')}"
                        )
                        bitacora.append(f"vision agarre: {r.get('motivo', '')}")
                        # La pinza manda si dice que NO hay nada: es una
                        # medida fisica. La IA solo puede desmentir un
                        # falso positivo (mordazas trabadas sin pieza).
                        confirmado = hay_algo and visual

            if confirmado:
                exito = True
                mv("levantar a preagarre", preagarre)
                nodo.mueve_home("volver a home", home)
                break

            print("  agarre NO confirmado.")
            nodo.pinza("reabrir", abre, args.par_pinza, args.corriente_pinza)
            mv("retirada a preagarre", preagarre)
            if intento < args.reintentos:
                print("  se reintenta.")
            else:
                nodo.mueve_home("volver a home", home)

        # ---------- resumen ----------
        print("\n" + "=" * 62)
        print("AGARRE CONSEGUIDO" if exito else "AGARRE FALLIDO")
        print(f"  pieza: {pieza} (confianza {confianza:.2f}) @ {args.altura_mm} mm")
        if verificador:
            print(f"  {verificador.resumen()}")
        print("  bitacora:")
        for linea in bitacora:
            print(f"    - {linea}")
        print("=" * 62)
        return 0 if exito else 2

    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrumpido.", file=sys.stderr)
        sys.exit(130)
