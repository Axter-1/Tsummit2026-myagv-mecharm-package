#!/usr/bin/env python3
"""Lee los parametros de object_grasp_server desde su proceso local.

La consulta ROS de parametros depende del descubrimiento de servicios DDS,
que puede ser intermitente entre la Jetson y el portatil. El proceso de grasp
corre localmente en la Jetson y Launch deja su archivo de parametros en la
linea de comando; leerlo aqui verifica la configuracion sin depender de DDS.
"""

import argparse
import os
import sys

import yaml


def server_parameter_file():
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as handle:
                args = [part.decode() for part in handle.read().split(b'\0') if part]
        except OSError:
            continue
        if not any(arg.endswith('/object_grasp_server') for arg in args):
            continue
        try:
            index = args.index('--params-file')
            return args[index + 1]
        except (ValueError, IndexError):
            continue
    return None


def parameters_from_file(path):
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    for value in data.values():
        if not isinstance(value, dict):
            continue
        parameters = value.get('ros__parameters')
        if isinstance(parameters, dict):
            return parameters
    return {}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--table-height', type=int, required=True)
    parser.add_argument('--approach-stop-distance', type=float, required=True)
    args = parser.parse_args()

    path = server_parameter_file()
    if path is None:
        print('FALTA: object_grasp_server o su archivo de parametros')
        return 1
    try:
        parameters = parameters_from_file(path)
        table_height = int(parameters['table_height_mm'])
        stop_distance = float(parameters['approach_stop_distance'])
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f'FALTA: no se pudieron leer parametros de {path}: {exc}')
        return 1

    print(
        f'object_grasp_server: table_height_mm={table_height}, '
        f'approach_stop_distance={stop_distance:.3f} m'
    )
    if table_height != args.table_height:
        return 1
    return 0 if abs(stop_distance - args.approach_stop_distance) < 1e-6 else 1


if __name__ == '__main__':
    sys.exit(main())
