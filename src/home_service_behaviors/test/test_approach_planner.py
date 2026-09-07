#!/usr/bin/env python3
"""Pruebas del planificador de aproximacion.

Estas pruebas existen porque el control anterior SOLO se podia evaluar
en pista: cada hipotesis costaba montar el robot, y varias resultaron
falsas despues de horas de pruebas. El planificador es geometria pura,
sin ROS, asi que se puede castigar aqui en segundos.

La ultima prueba simula el lazo completo con los tres defectos reales
medidos en el robot: zona muerta de los motores, retardo de la vision
por WiFi y perdidas de deteccion.
"""

import math
import random

from home_service_behaviors.approach_planner import (
    TargetEstimate,
    build_path,
    carrot,
    corridor_coords,
    desired_heading,
    holonomic_command,
    normalize_angle,
    outward_normal,
    path_length,
    predict_pose,
    profile_speed,
    staging_pose,
    deadband_floor,
    stopping_distance,
    tolerance_is_reachable,
    plane_returns,
)


LIMITS = {
    'max_linear': 0.18,
    'max_lateral': 0.12,
    'max_angular': 0.8,
    'min_linear': 0.05,
    'min_lateral': 0.045,
    'min_angular': 0.15,
    'kp_angular': 1.5,
    'accel': 0.25,
    'distance_tolerance': 0.03,
    'yaw_tolerance': math.radians(4.0),
    'yaw_hysteresis': 1.5,
}


# ---------------------------------------------------------------------
# Geometria
# ---------------------------------------------------------------------

def test_staging_pose_esta_sobre_la_normal():
    sx, sy, syaw = staging_pose(2.0, 1.0, 1.0, 0.0, 0.5)

    assert abs(sx - 2.5) < 1e-9
    assert abs(sy - 1.0) < 1e-9

    # mirando HACIA el marcador, o sea en -x
    assert abs(abs(normalize_angle(syaw - math.pi))) < 1e-9


def test_normal_saliente_invierte_el_signo():
    assert outward_normal(1.0, 0.0) == (-1.0, -0.0)


def test_corridor_coords_separa_avance_y_lateral():
    # marcador en el origen, normal saliente hacia +x
    along, lateral = corridor_coords(0.8, 0.3, 0.0, 0.0, 1.0, 0.0)

    assert abs(along - 0.8) < 1e-9
    assert abs(lateral - 0.3) < 1e-9


def test_camino_rodea_por_el_encare_desde_fuera():
    # robot muy desviado: debe pasar por el punto de encare
    path = build_path(0.0, 2.0, 1.0, 0.0, 1.0, 0.0, 0.45, 0.20)

    assert len(path) == 3


def test_camino_va_recto_desde_dentro_del_pasillo():
    """El bug que estancaba la aproximacion a 13.7 cm del objetivo.

    Con el camino reconstruido desde el robot en cada ciclo, dentro del
    pasillo `remaining` incluia para siempre el rodeo por el punto de
    encare y no bajaba nunca de standoff - stop_distance, asi que la
    llegada no se declaraba jamas.
    """
    path = build_path(0.30, 0.01, 0.0, 0.0, 1.0, 0.0, 0.45, 0.20)

    assert len(path) == 2

    # y la distancia que queda es la real hasta el objetivo
    _, remaining, _ = carrot(path, 0.30, 0.01, 0.25)

    assert remaining < 0.15


def test_carrot_se_satura_en_el_final():
    path = [(0.0, 0.0), (1.0, 0.0)]

    point, remaining, _ = carrot(path, 0.95, 0.0, 0.5)

    assert abs(point[0] - 1.0) < 1e-9
    assert abs(remaining - 0.05) < 1e-9


def test_longitud_del_camino():
    assert abs(path_length([(0, 0), (3, 0), (3, 4)]) - 7.0) < 1e-9


# ---------------------------------------------------------------------
# Perfil de velocidad y zona muerta
# ---------------------------------------------------------------------

def test_perfil_frena_al_acercarse():
    """La rampa solo frena por debajo de v_max^2 / 2a.

    Con v_max 0.18 y a 0.25 eso son 6.5 cm: por encima de esa distancia
    la rampa satura al tope y NO frena todavia. Poner aqui una
    distancia mayor (0.10 m, por ejemplo) hace que la prueba compare
    0.18 contra 0.18 y falle sin que haya nada roto.
    """
    saturada = profile_speed(2.0, 0.18, 0.25)
    frenando = profile_speed(0.04, 0.18, 0.25)

    assert saturada == 0.18
    assert frenando < saturada
    assert abs(frenando - math.sqrt(2 * 0.25 * 0.04)) < 1e-9


def test_perfil_da_cero_dentro_de_tolerancia():
    assert profile_speed(0.01, 0.18, 0.25, v_min=0.05,
                         tolerance=0.03) == 0.0


def test_perfil_respeta_la_zona_muerta_fuera_de_tolerancia():
    """El fallo de raiz del control anterior.

    Con v = kp * error, cerca del objetivo el mando se hacia
    infinitesimal y caia bajo la zona muerta de los motores: se
    publicaba y las ruedas no giraban. El robot se paraba ANTES de
    llegar y el estado no cerraba nunca.
    """
    v = profile_speed(0.05, 0.18, 0.25, v_min=0.05, tolerance=0.03)

    assert v >= 0.05


# ---------------------------------------------------------------------
# Estimador
# ---------------------------------------------------------------------

def test_el_promedio_diluye_ruido_aleatorio():
    """Con ruido de media cero, promediar SI ayuda."""
    random.seed(7)

    est = TargetEstimate(alpha_position=0.20, alpha_normal=0.10)

    for _ in range(200):
        angle = random.gauss(0.0, math.radians(10.0))
        est.update(1.0, 0.0, math.cos(angle), math.sin(angle))

    assert abs(math.atan2(est.ny, est.nx)) < math.radians(4.0)


def test_el_promedio_NO_diluye_un_sesgo_sistematico():
    """La suposicion falsa que motivo la guarda de oblicuidad.

    El comentario original del servidor decia que meter la normal del
    LiDAR con doble peso era seguro porque "si el ajuste engancha una
    pared vecina, el promedio lo diluye". Es falso, y de dos maneras:

      1. El alfa tiene SUELO -- max(alpha, 1/n) -- asi que la media
         corriente 1/n solo dura las primeras muestras. Despues es un
         exponencial fijo, con memoria de unas pocas muestras.
      2. Promediar solo diluye error ALEATORIO. El fallo real es
         SISTEMATICO: la SVD engancha la pared contigua con coherencia
         1.00 y devuelve la MISMA respuesta erronea cada ciclo. La
         media de una constante es esa constante.

    Con doble peso el sesgo no se diluye: gana. Por eso hace falta
    RECHAZARLO por geometria (normal_obliquity en el servidor), no
    esperar que se promedie solo.
    """
    est = TargetEstimate(alpha_position=0.20, alpha_normal=0.10)

    for _ in range(40):
        # buena, a 0 grados, peso normal (ArUco)
        est.update(1.0, 0.0, 1.0, 0.0)
        # pared contigua, a 90 grados, peso doble (LiDAR)
        est.update(1.0, 0.0, 0.0, 1.0, alpha_scale=2.0)

    desviacion = abs(math.atan2(est.ny, est.nx))

    # Muy lejos de la buena y pegada a la mala: NO se diluyo.
    assert desviacion > math.radians(60.0), math.degrees(desviacion)


# ---------------------------------------------------------------------
# Control
# ---------------------------------------------------------------------

def test_nunca_manda_los_tres_ejes_a_la_vez():
    """La placa se queda QUIETA si recibe los tres ejes no nulos.

    Medido en el robot: uno o dos ejes siempre mueven; los tres dan cero
    absoluto a cualquier magnitud (probado a 1, 1/2 y 1/4). Esta prueba
    sustituye a `test_mueve_los_tres_ejes_a_la_vez`, que afirmaba lo
    contrario y pasaba porque nadie lo habia medido en el hardware.
    """
    # rumbo muy fuera de banda: toca ciclo de giro puro
    vx, vy, wz, _, _, _ = holonomic_command(
        0.0, 0.0, 0.0,
        (1.0, 1.0),
        math.radians(45),
        1.41,
        LIMITS,
        yaw_settled=False,
    )

    assert abs(wz) > 1e-6, 'con el rumbo fuera de banda tiene que girar'
    assert vx == 0.0 and vy == 0.0, 'giro puro: sin traslacion'


def test_traslacion_plena_cuando_el_rumbo_esta_asentado():
    """vx y vy van SIEMPRE juntos: separarlos torceria la direccion."""
    # robot a yaw 0 y objetivo en diagonal: el error de cuerpo tiene
    # componente en los DOS ejes. Con el robot a 45 grados caeria todo
    # sobre uno solo y la prueba no probaria nada.
    vx, vy, wz, _, _, settled = holonomic_command(
        0.0, 0.0, 0.0,
        (1.0, 1.0),
        0.0,
        1.41,
        LIMITS,
        yaw_settled=True,
    )

    assert settled
    assert wz == 0.0, 'asentado: no gira'
    assert abs(vx) > 1e-6 and abs(vy) > 1e-6, 'los dos ejes de traslacion'


def test_ninguna_combinacion_saca_tres_componentes():
    """Barrido: en ningun caso salen las tres no nulas a la vez."""
    for gx, gy in ((1.0, 1.0), (0.5, -0.8), (-1.2, 0.3), (0.05, 0.05)):
        for yaw_obj in (0.0, math.radians(30), math.radians(-120)):
            for ryaw in (0.0, math.radians(90)):
                for settled in (True, False):
                    vx, vy, wz, _, _, _ = holonomic_command(
                        0.0, 0.0, ryaw,
                        (gx, gy),
                        yaw_obj,
                        math.hypot(gx, gy),
                        LIMITS,
                        yaw_settled=settled,
                    )

                    activos = sum(
                        1 for v in (vx, vy, wz) if abs(v) > 1e-9
                    )

                    assert activos <= 2, (gx, gy, yaw_obj, ryaw,
                                          settled, vx, vy, wz)


def test_rumbo_mira_al_marcador_de_lejos():
    # marcador a la izquierda, normal saliente hacia +x
    yaw = desired_heading(0.0, 0.0, 1.0, 1.0, 1.0, 0.0,
                          remaining=2.0, blend_distance=0.9)

    assert abs(normalize_angle(yaw - math.radians(45))) < math.radians(5)


def test_rumbo_se_pone_perpendicular_al_llegar():
    yaw = desired_heading(0.5, 0.0, 0.0, 0.0, 1.0, 0.0,
                          remaining=0.0, blend_distance=0.9)

    assert abs(normalize_angle(yaw - math.pi)) < 1e-6


def test_interpolacion_de_angulos_cruza_pi():
    """Interpolar angulos linealmente falla en el cruce de +-pi."""
    yaw = desired_heading(0.0, 0.0, -1.0, -0.02, -1.0, 0.0,
                          remaining=1.0, blend_distance=2.0)

    assert -math.pi <= yaw <= math.pi


# ---------------------------------------------------------------------
# Lazo completo con los defectos reales
# ---------------------------------------------------------------------

def _simulate(marker, normal_yaw, start, seed):
    """Lazo cerrado con zona muerta, retardo, ruido y perdidas."""
    random.seed(seed)

    dt = 0.05
    deadband_lin, deadband_lat, deadband_ang = 0.042, 0.038, 0.10
    latency, pos_noise, nrm_noise, drop = 0.20, 0.010, math.radians(6.0), 0.25

    stop_distance, standoff, lookahead = 0.20, 0.45, 0.25

    mx, my = marker
    onx, ony = math.cos(normal_yaw), math.sin(normal_yaw)

    rx, ry, ryaw = start
    est = TargetEstimate(alpha_position=0.20, alpha_normal=0.10)

    history = []
    settled = False
    t = 0.0

    while t < 60.0:

        history.append((t, rx, ry, ryaw))

        if random.random() > drop and t >= latency:
            est.update(
                mx + random.gauss(0, pos_noise),
                my + random.gauss(0, pos_noise),
                math.cos(normal_yaw + random.gauss(0, nrm_noise)),
                math.sin(normal_yaw + random.gauss(0, nrm_noise)),
            )

        if not est.ready:
            t += dt
            continue

        ex, ey, enx, eny = est.pose

        path = build_path(rx, ry, ex, ey, enx, eny,
                          standoff, stop_distance)

        cxy, remaining, _ = carrot(path, rx, ry, lookahead)

        tyaw = desired_heading(rx, ry, ex, ey, enx, eny,
                               remaining, standoff * 2.0)

        vx, vy, wz, yaw_err, reached, settled = holonomic_command(
            rx, ry, ryaw, cxy, tyaw, remaining, LIMITS,
            yaw_settled=settled,
        )

        if reached and settled:
            break

        vx = vx if abs(vx) >= deadband_lin else 0.0
        vy = vy if abs(vy) >= deadband_lat else 0.0
        wz = wz if abs(wz) >= deadband_ang else 0.0

        rx, ry, ryaw = predict_pose(rx, ry, ryaw, vx, vy, wz, dt)

        t += dt

    goal_x = mx + onx * stop_distance
    goal_y = my + ony * stop_distance
    goal_yaw = math.atan2(-ony, -onx)

    return (
        math.hypot(rx - goal_x, ry - goal_y),
        abs(normalize_angle(ryaw - goal_yaw)),
        t,
    )


def test_converge_en_escenarios_dificiles():
    """Llega perpendicular y a distancia, pese a zona muerta y retardo."""
    casos = [
        ((1.0, 0.0), math.pi, (0.0, 0.0, 0.0)),
        ((1.0, 0.7), math.radians(200), (0.0, 0.0, 0.0)),
        ((1.0, 0.3), math.radians(250), (0.0, 0.0, 0.0)),
        ((0.6, 1.0), math.radians(270), (0.0, 0.0, 0.0)),
        ((1.2, 0.0), math.pi, (0.0, 0.0, math.radians(120))),
        ((0.35, 0.0), math.pi, (0.0, 0.0, 0.0)),
        ((-0.8, 0.4), math.radians(20), (0.0, 0.0, 0.0)),
    ]

    for marker, nyaw, start in casos:
        for seed in range(10):
            err_d, err_y, t = _simulate(marker, nyaw, start, seed)

            assert err_d <= 0.06, (marker, seed, err_d)
            assert err_y <= math.radians(8.0), (marker, seed, err_y)
            assert t < 30.0, (marker, seed, t)


# ---------------------------------------------------------------------
# Los dos fallos vistos en pista
# ---------------------------------------------------------------------

def test_histeresis_corta_el_casta_eo_del_giro():
    """El robot giraba a izquierda y derecha sin asentarse.

    Con un solo umbral, la zona muerta obliga a mandar min_angular en
    cuanto se sale de tolerancia; ese minimo se pasa de largo y al
    ciclo siguiente hay que corregir al otro lado. Ciclo limite.

    Con histeresis, una vez dentro de la tolerancia fina no se vuelve a
    mandar giro hasta superar el umbral GRANDE.
    """
    limits = dict(LIMITS)
    limits['yaw_hysteresis'] = 2.5

    # dentro de tolerancia: se asienta
    _, _, wz, _, _, settled = holonomic_command(
        0.0, 0.0, 0.0, (1.0, 0.0),
        math.radians(2.0), 1.0, limits,
    )

    assert settled
    assert wz == 0.0

    # error mayor que la tolerancia pero menor que el de salida:
    # SIGUE asentado, no manda nada. Aqui es donde castañeaba.
    _, _, wz, _, _, settled = holonomic_command(
        0.0, 0.0, 0.0, (1.0, 0.0),
        math.radians(7.0), 1.0, limits,
        yaw_settled=settled,
    )

    assert settled
    assert wz == 0.0

    # error grande de verdad: vuelve a engancharse
    _, _, wz, _, _, settled = holonomic_command(
        0.0, 0.0, 0.0, (1.0, 0.0),
        math.radians(25.0), 1.0, limits,
        yaw_settled=settled,
    )

    assert not settled
    assert abs(wz) >= limits['min_angular']


def test_sin_histeresis_hay_ciclo_limite():
    """El caso que distingue: la DERIVA despues de asentarse.

    Mientras el robot avanza, el rumbo objetivo se mueve unos grados.
    Con un umbral unico eso basta para volver a mandar giro, y como la
    zona muerta obliga a mandar min_angular, se pasa de largo y hay que
    corregir al otro lado: castañeo. Con dos umbrales el robot se queda
    callado mientras la deriva no salga de la banda.
    """
    def tras_derivar(hysteresis, deriva_deg):
        limits = dict(LIMITS)
        limits['yaw_hysteresis'] = hysteresis

        # entra en tolerancia y se asienta
        *_, settled = holonomic_command(
            0.0, 0.0, 0.0, (1.0, 0.0),
            math.radians(2.0), 1.0, limits,
        )

        assert settled

        # el rumbo objetivo deriva
        _, _, wz, _, _, settled = holonomic_command(
            0.0, 0.0, 0.0, (1.0, 0.0),
            math.radians(deriva_deg), 1.0, limits,
            yaw_settled=settled,
        )

        return wz, settled

    # Umbral unico (hysteresis 1.0): 6 grados ya lo despierta.
    wz, settled = tras_derivar(1.0, 6.0)

    assert wz != 0.0
    assert not settled

    # Con histeresis 1.5 la banda de salida son 6 grados: aguanta.
    wz, settled = tras_derivar(1.5, 5.0)

    assert wz == 0.0
    assert settled

    # Pero una deriva de verdad SI lo despierta: no es sordera.
    wz, settled = tras_derivar(1.5, 20.0)

    assert wz != 0.0
    assert not settled


def test_filtro_rechaza_el_salto_de_ambiguedad_del_aruco():
    """La pose de un ArUco plano salta entre dos soluciones.

    Sin filtro, esos saltos entran en la media, el rumbo objetivo se
    mueve con ellos y el robot los persigue a izquierda y derecha.
    """
    est = TargetEstimate(
        alpha_position=0.20,
        alpha_normal=0.10,
        max_normal_jump=math.radians(35.0),
        gate_after=3,
        relock_after=12,
    )

    for _ in range(10):
        est.update(1.0, 0.0, 1.0, 0.0)

    antes = math.atan2(est.ny, est.nx)

    # rama ambigua a 80 grados: debe rechazarse
    aceptada = est.update(1.0, 0.0, math.cos(math.radians(80.0)),
                          math.sin(math.radians(80.0)))

    assert not aceptada
    assert est.rejected == 1
    assert abs(math.atan2(est.ny, est.nx) - antes) < 1e-9


def test_el_filtro_no_se_atrinchera_en_una_estimacion_mala():
    """Si se rechaza sin parar, la equivocada es la estimacion."""
    est = TargetEstimate(
        alpha_position=0.20,
        alpha_normal=0.10,
        max_normal_jump=math.radians(35.0),
        gate_after=3,
        relock_after=6,
    )

    for _ in range(10):
        est.update(1.0, 0.0, 1.0, 0.0)

    # la realidad es otra, insistentemente
    for _ in range(20):
        est.update(1.0, 0.0, 0.0, 1.0)

    # acaba reenganchandose en vez de defender la vieja para siempre
    assert abs(math.atan2(est.ny, est.nx) - math.pi / 2) < math.radians(20)


def test_el_techo_de_velocidad_debe_superar_la_zona_muerta():
    """El fallo que dejo al robot parado publicando 0.08 m/s.

    Con max_linear_speed 0.08 y una zona muerta real por encima, el
    rango ENTERO de mando cae dentro de la zona muerta: ninguna salida
    del controlador puede mover las ruedas. Un techo que no supera
    holgadamente al suelo es una configuracion sin margen util.
    """
    limits = dict(LIMITS)

    assert limits['max_linear'] > limits['min_linear'] * 1.5
    assert limits['max_lateral'] > limits['min_lateral'] * 1.5
    assert limits['max_angular'] > limits['min_angular'] * 1.5


def test_la_zona_muerta_no_tuerce_la_direccion():
    """La zona muerta es una ELIPSE, no una caja.

    Aplicarla eje por eje destroza la direccion del movimiento: con el
    objetivo muy a un lado, vy es grande y vx minusculo, pero el minimo
    de avance eleva ese vx y el robot sale en diagonal en vez de de
    lado. Medido antes del arreglo: hasta 26 grados de desvio, con el
    robot abandonando el camino.
    """
    limits = dict(LIMITS)
    limits['min_linear'] = 0.12
    limits['min_lateral'] = 0.13
    limits['max_linear'] = 0.22
    limits['max_lateral'] = 0.20

    for grados in (5, 15, 30, 45, 60, 80, 120, 200, 330):

        a = math.radians(grados)

        vx, vy, _, _, _, _ = holonomic_command(
            0.0, 0.0, 0.0,
            (math.cos(a), math.sin(a)),
            0.0, 1.0, limits,
        )

        salida = math.degrees(math.atan2(vy, vx))

        desvio = abs(normalize_angle(math.radians(grados - salida)))

        assert desvio < math.radians(1.0), (grados, salida)


def test_la_zona_muerta_eleva_el_modulo_lo_justo():
    """Un mando pequeño se sube al borde de la elipse, y no mas.

    El modulo lo fija `remaining` por la rampa, no la distancia al
    carrot, que solo da DIRECCION. Con la aceleracion y la tolerancia
    por defecto la rampa nunca baja del suelo antes de entrar en
    tolerancia -- una propiedad buena, no un accidente -- asi que para
    ejercitar el suelo hay que apretar la tolerancia.
    """
    limits = dict(LIMITS)
    limits['min_linear'] = 0.12
    limits['min_lateral'] = 0.13
    limits['distance_tolerance'] = 0.005

    # a 1 cm la rampa pide sqrt(2*0.25*0.01) = 0.071, bajo el suelo
    vx, vy, _, _, reached, _ = holonomic_command(
        0.0, 0.0, 0.0, (1.0, 0.0), 0.0, 0.01, limits,
    )

    assert not reached
    assert abs(math.hypot(vx, vy) - 0.12) < 1e-6

    # y dentro de tolerancia se manda CERO de verdad, no el suelo
    vx, vy, _, _, reached, _ = holonomic_command(
        0.0, 0.0, 0.0, (1.0, 0.0), 0.0, 0.001, limits,
    )

    assert reached
    assert vx == 0.0 and vy == 0.0


def test_el_suelo_de_zona_muerta_cae_en_la_elipse():
    """El suelo debe caer EN el borde de la elipse, no cerca.

    hypot(a*dx, b*dy) parametriza la elipse por la direccion de la
    preimagen en el circulo unidad, no por la del rayo pedido. Con a y
    b parecidos se confunde con el radio real; en cuanto divergen se
    dispara.
    """
    for a, b in ((0.03, 0.035), (0.03, 0.20), (0.12, 0.13), (0.05, 0.005)):
        for deg in range(0, 91, 5):
            r = math.radians(deg)
            dx, dy = math.cos(r), math.sin(r)

            radio = deadband_floor(dx, dy, a, b)

            # el punto devuelto satisface la ecuacion de la elipse
            en_elipse = (radio * dx / a) ** 2 + (radio * dy / b) ** 2

            assert abs(en_elipse - 1.0) < 1e-9, (a, b, deg, en_elipse)


def test_el_suelo_no_sobrepasa_cuando_los_semiejes_divergen():
    """Regresion del x3.41.

    Con min_linear 0.03 y min_lateral 0.20, la formula anterior daba
    0.1430 a 45 grados donde la elipse vale 0.0420. Un suelo 3.4 veces
    mas alto del pedido es el tiron que bajar los minimos evita.
    """
    dx = dy = math.cos(math.radians(45.0))

    radio = deadband_floor(dx, dy, 0.03, 0.20)

    assert abs(radio - 0.04200) < 1e-4, radio
    assert radio < math.hypot(0.03 * dx, 0.20 * dy)


def test_sin_semieje_no_hay_suelo():
    """Un minimo a cero desactiva la zona muerta en vez de dividir por cero."""
    assert deadband_floor(1.0, 0.0, 0.0, 0.035) == 0.0
    assert deadband_floor(0.0, 1.0, 0.03, 0.0) == 0.0


# ---------------------------------------------------------------------
# El suelo de la base y la distancia de parada
# ---------------------------------------------------------------------

def test_distancia_de_parada():
    """Lo que sigue recorriendo tras mandarle cero."""
    # suelo de giro 0.37 rad/s, retardo 200 ms, ciclo a 20 Hz
    d = stopping_distance(0.37, 0.200, 0.05)

    assert abs(d - 0.0925) < 1e-9


def test_una_tolerancia_menor_que_la_parada_es_inalcanzable():
    """El baile izquierda-derecha, reducido a una desigualdad.

    Con suelo de giro 0.37 rad/s y 200 ms de retardo, el robot gira
    0.093 rad DESPUES de decidir pararse. Contra una tolerancia de
    0.08 rad no puede asentarse: sale por el otro lado y corrige al
    reves. No es un problema de ganancias, es aritmetica.
    """
    assert not tolerance_is_reachable(0.08, 0.37, 0.200, 0.05)
    assert tolerance_is_reachable(0.15, 0.37, 0.200, 0.05)


def test_por_debajo_del_suelo_solo_hay_suelo_o_cero():
    """La base no modula por debajo de su suelo: la rampa no existe."""
    # lejos: la rampa manda
    assert profile_speed(1.0, 0.18, 0.25, v_min=0.07,
                         tolerance=0.001, stop_margin=0.018) == 0.18

    # La rampa baja del suelo solo por debajo de v_min^2/(2a), aqui
    # 0.98 cm. A 0.8 cm pide sqrt(2*0.25*0.008)=0.063 < 0.07, y aun
    # queda mas que la distancia de parada: se manda el suelo.
    assert profile_speed(0.008, 0.18, 0.25, v_min=0.07,
                         tolerance=0.001, stop_margin=0.005) == 0.07

    # dentro de la distancia de parada: cero, o se pasaria de largo
    assert profile_speed(0.003, 0.18, 0.25, v_min=0.07,
                         tolerance=0.001, stop_margin=0.005) == 0.0

    # Con los valores REALES de hoy la banda ni existe: la rampa baja
    # del suelo a 0.98 cm y la distancia de parada son 1.8 cm, asi que
    # cuando la rampa flaquea ya toca parar. Inofensivo hoy, y por eso
    # mismo conviene que este escrito.
    assert profile_speed(0.009, 0.18, 0.25, v_min=0.07,
                         tolerance=0.001, stop_margin=0.018) == 0.0


def test_el_suelo_nunca_supera_el_techo():
    """Una configuracion sin margen no debe emitir mas que v_max."""
    v = profile_speed(0.05, 0.06, 0.25, v_min=0.20, tolerance=0.01)

    assert v <= 0.06


# =====================================================================
#  El sector frontal del LiDAR no mide "el marcador"
# =====================================================================


def test_la_mediana_del_sector_devuelve_la_pared():
    """Reproduce el fallo de pista: el goal expiro a 0.316 m.

    Escena real: ArUco de 8 cm sobre una caja, la pared 13 cm detras,
    sector de +-6 grados que a 0.5 m abarca +-5.3 cm. La caja no llena
    el sector, asi que la mayoria de los ecos son de la pared.
    """
    import statistics

    # 3 ecos en la caja, 8 en la pared: proporciones de la escena real.
    ecos = [0.384] * 3 + [0.511] * 8

    # Sin banda, la mediana se va con la mayoria y devuelve la pared.
    assert statistics.median(plane_returns(ecos, None, 0.08)) == 0.511

    # Con banda, la pared desaparece y queda el plano del marcador.
    gated = plane_returns(ecos, expected=0.384, band=0.08)
    assert statistics.median(gated) == 0.384
    assert 0.511 not in gated


def test_un_eco_mas_cerca_siempre_cuenta():
    """La banda es de un solo lado: un obstaculo delante no se filtra.

    Si se filtrara por los dos lados, una caja que se cruza en el camino
    quedaria invisible justo para la parada de seguridad, que es lo
    unico que evita el choque.
    """
    ecos = [0.12, 0.384, 0.390]

    gated = plane_returns(ecos, expected=0.384, band=0.08)

    assert 0.12 in gated


def test_sin_expectativa_no_se_filtra():
    """Sin geometria fiable no hay forma honesta de decidir que sobra."""
    ecos = [0.30, 0.51, 0.90]

    assert plane_returns(ecos, None, 0.08) == ecos
    assert plane_returns(ecos, 0.30, 0.0) == ecos


def test_la_banda_puede_dejar_el_sector_vacio():
    """Si todo cae fuera hay que devolver vacio, no inventar un numero.

    El llamante tiene que poder distinguir "no veo el plano" de "el
    plano esta a X", porque son decisiones distintas.
    """
    ecos = [0.90, 0.95]

    assert plane_returns(ecos, expected=0.30, band=0.08) == []
