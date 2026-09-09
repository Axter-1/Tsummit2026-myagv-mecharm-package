#!/usr/bin/env python3
"""Reglas puras para recuperar una aproximacion antes del agarre."""


def is_recoverable_approach_status(status):
    """Solo STALLED puede estar fisicamente en una parada util.

    BLOCKED representa un despeje de chasis insuficiente y nunca se
    convierte en una autorizacion de movimiento del brazo.
    """
    return status == 'STALLED'


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
