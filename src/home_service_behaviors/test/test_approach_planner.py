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
# Control
# ---------------------------------------------------------------------

def test_mueve_los_tres_ejes_a_la_vez():
    """Sobre una base mecanum no hay que corregir un eje cada vez."""
    vx, vy, wz, _, _ = holonomic_command(
        0.0, 0.0, 0.0,
        (1.0, 1.0),
        math.radians(45),
        1.41,
        LIMITS,
    )

    assert abs(vx) > 1e-6
    assert abs(vy) > 1e-6
    assert abs(wz) > 1e-6


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

        vx, vy, wz, yaw_err, reached = holonomic_command(
            rx, ry, ryaw, cxy, tyaw, remaining, LIMITS
        )

        if reached and abs(yaw_err) <= LIMITS['yaw_tolerance']:
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
