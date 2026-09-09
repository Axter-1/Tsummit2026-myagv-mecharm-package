#!/usr/bin/env python3
"""Reglas puras para recuperar una aproximacion antes del agarre."""

import math
import re


def is_recoverable_approach_status(status):
    """SAFE_STOP y STALLED requieren verificacion fisica adicional.

    BLOCKED representa un despeje de chasis insuficiente y nunca se
    convierte en una autorizacion de movimiento del brazo.
    """
    return status in ('SAFE_STOP', 'STALLED')


def stop_distance_is_valid(measured, target, tolerance):
    """La parada LiDAR debe coincidir con la posicion de calibracion."""
    return (
        measured is not None
        and measured > 0.0
        and
        abs(measured - target) <= tolerance
    )


def scan_agrees_with_result(scan_distance, result_distance, tolerance):
    """Evita autorizar el brazo con un resultado LiDAR ya obsoleto."""
    return (
        scan_distance is not None
        and result_distance is not None
        and
        abs(scan_distance - result_distance) <= tolerance
    )


def chassis_clearance_is_valid(measured, minimum):
    """Requiere un despeje numerico y no menor que el minimo calibrado."""
    return (
        measured is not None
        and math.isfinite(measured)
        and measured >= minimum
    )


def chassis_clearance_from_message(message):
    """Respaldo para clientes con una interfaz de accion anterior.

    El servidor incluye el despeje de su mismo haz LiDAR en SAFE_STOP. Se
    usa solo cuando ``final_chassis_clearance`` no llego por una definicion
    de accion desactualizada; el cliente aun exige su scan frontal fresco.
    """
    match = re.search(r'despeje chasis=([0-9]+(?:\.[0-9]+)?) m', message)
    return float(match.group(1)) if match else None
