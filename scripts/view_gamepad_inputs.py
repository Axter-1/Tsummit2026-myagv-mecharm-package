#!/usr/bin/env python3
"""Muestra eventos evdev del mando por nombre, sin depender de eventX."""

import sys
import time

from evdev import InputDevice, categorize, ecodes, list_devices

RELEVANT_CODES = {
    "ABS_X", "ABS_Y", "ABS_HAT0X", "ABS_HAT0Y",
    "BTN_TL", "BTN_TR", "BTN_TR2"
}


def find_gamepad(name):
    wanted = name.lower()
    for path in list_devices():
        try:
            device = InputDevice(path)
            matches = wanted in device.name.lower()
            has_axes = ecodes.EV_ABS in device.capabilities()
            has_buttons = ecodes.EV_KEY in device.capabilities()
            if matches and has_axes and has_buttons:
                return device
            device.close()
        except OSError:
            continue
    return None


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "TGZ Controller"
    print("Buscando: %s (Ctrl+C para salir)" % name, flush=True)

    while True:
        device = find_gamepad(name)
        if device is None:
            print("Mando no encontrado; reintentando...", flush=True)
            time.sleep(1.0)
            continue

        print("Conectado: %s (%s)" % (device.name, device.path), flush=True)
        try:
            for event in device.read_loop():
                if event.type not in (ecodes.EV_ABS, ecodes.EV_KEY):
                    continue
                decoded = categorize(event)
                code = ecodes.bytype.get(event.type, {}).get(event.code, event.code)
                if code not in RELEVANT_CODES:
                    continue
                value = getattr(
                    decoded, "keystate", getattr(decoded, "value", event.value)
                )
                event_type = ecodes.EV.get(event.type, event.type)
                print(
                    "%-8s %-18s value=%s" % (event_type, code, value),
                    flush=True,
                )
        except (OSError, EOFError):
            print("Mando desconectado; reintentando...", flush=True)
        finally:
            device.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nFin.")
