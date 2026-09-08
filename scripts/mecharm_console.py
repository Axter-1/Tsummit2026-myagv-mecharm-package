#!/usr/bin/env python3
"""Consola interactiva del MechArm 270 M5 — pruebas y ensenanza de poses.

Habla DIRECTAMENTE con el brazo por el puerto serie (pymycobot), sin
pasar por ROS. Es la herramienta para:

  * comprobar que el brazo y la pinza responden,
  * calibrar los valores de apertura/cierre de la pinza,
  * ensenar poses moviendo el brazo A MANO y guardarlas en poses.yaml.

IMPORTANTE: no puede correr a la vez que mecharm_driver_node — solo un
proceso puede tener abierto /dev/ttyACM0. Para el nodo antes:

    pkill -f mecharm_driver_node

Uso dentro del contenedor:

    python3 /workspace/scripts/mecharm_console.py
    python3 /workspace/scripts/mecharm_console.py --port /dev/ttyACM0

Comandos (escribe 'help' dentro de la consola):

    a | angles          leer angulos actuales [J1..J6]
    c | coords          leer coordenadas [X,Y,Z,RX,RY,RZ]
    open                abrir pinza (valor 100)
    close               cerrar pinza (valor 20)
    g <0-100>           pinza a una apertura concreta
    gv                  leer el valor actual de la pinza
    sweep               barrido 100->0 en pasos, para ver el recorrido
    move <j1..j6>       mover a esos angulos
    goto <pose>         mover a una pose guardada
    free                LIBERAR servos (el brazo cae: sujetalo)
    lock                volver a alimentar los servos
    save <pose>         guardar la posicion ACTUAL con ese nombre
    list                listar poses guardadas
    speed <1-100>       velocidad por defecto de los movimientos
    home                ir a la pose 'home'
    q | quit            salir (deja el brazo alimentado)
"""

import argparse
import os
import sys
import time

try:
    import yaml
except ImportError:
    yaml = None

try:
    from pymycobot.mecharm270 import MechArm270
except ImportError:
    print("ERROR: falta pymycobot.  pip3 install pymycobot==4.0.6")
    sys.exit(1)


WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_POSES_FILE = os.path.join(
    WORKSPACE_ROOT,
    "src",
    "myagv_mecharm_service",
    "config",
    "poses.yaml",
)

# Limites reportados por el firmware del MechArm 270 M5.
JOINT_MIN = [-160.0, -75.0, -175.0, -155.0, -115.0, -180.0]
JOINT_MAX = [160.0, 120.0, 65.0, 155.0, 115.0, 180.0]


def clamp(value, low, high):
    return max(low, min(high, value))


class Console:

    def __init__(self, port, baud, poses_file, speed):
        self.poses_file = poses_file
        self.speed = speed
        print(f"Conectando a {port} @ {baud} ...")
        self.mc = MechArm270(port, baud)
        time.sleep(0.5)
        try:
            self.mc.power_on()
        except Exception as exc:  # noqa: BLE001
            print(f"  aviso: power_on fallo ({exc})")
        try:
            self.mc.set_fresh_mode(1)
        except Exception:  # noqa: BLE001
            pass

        angles = self.read_angles()
        if angles is None:
            print(
                "AVISO: el brazo no devuelve angulos. Revisa que este\n"
                "       encendido y que ningun otro proceso tenga el\n"
                "       puerto abierto (pkill -f mecharm_driver_node)."
            )
        else:
            print(f"  conectado. angulos actuales: {fmt(angles)}")

        self.poses = self.load_poses()
        print(f"  poses cargadas: {sorted(self.poses) or '(ninguna)'}")

    # -----------------------------------------------------------------
    # Lecturas robustas
    # -----------------------------------------------------------------

    def _read(self, method, length, retries=12, delay=0.15):
        for _ in range(retries):
            try:
                value = getattr(self.mc, method)()
            except Exception:  # noqa: BLE001
                value = None
            if isinstance(value, (list, tuple)) and len(value) == length:
                return [float(v) for v in value]
            time.sleep(delay)
        return None

    def read_angles(self):
        return self._read("get_angles", 6)

    def read_coords(self):
        return self._read("get_coords", 6)

    # -----------------------------------------------------------------
    # Poses
    # -----------------------------------------------------------------

    def load_poses(self):
        if yaml is None or not os.path.isfile(self.poses_file):
            return {}
        try:
            with open(self.poses_file, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            raw = data.get("poses", {}) or {}
            return {
                str(k): [float(x) for x in v]
                for k, v in raw.items()
                if isinstance(v, (list, tuple)) and len(v) == 6
            }
        except Exception as exc:  # noqa: BLE001
            print(f"  aviso: no se pudo leer {self.poses_file}: {exc}")
            return {}

    def save_poses(self):
        if yaml is None:
            print("  ERROR: falta PyYAML, no se puede guardar.")
            return False
        header = (
            "# Poses con nombre del MechArm 270 M5 (grados, [J1..J6]).\n"
            "# Editado por scripts/mecharm_console.py\n"
            "#\n"
            "# Para ensenar una pose:\n"
            "#   free            -> libera los servos (sujeta el brazo)\n"
            "#   (mueve a mano)\n"
            "#   save <nombre>   -> guarda la posicion actual\n"
            "#   lock            -> vuelve a alimentar\n\n"
        )
        try:
            os.makedirs(
                os.path.dirname(self.poses_file) or ".", exist_ok=True
            )
            body = yaml.safe_dump(
                {"poses": {k: [round(x, 2) for x in v]
                           for k, v in sorted(self.poses.items())}},
                default_flow_style=True,
                sort_keys=False,
            )
            with open(self.poses_file, "w", encoding="utf-8") as handle:
                handle.write(header + body)
            print(f"  guardado en {self.poses_file}")
            return True
        except Exception as exc:  # noqa: BLE001
            print(f"  ERROR guardando: {exc}")
            return False

    # -----------------------------------------------------------------
    # Movimiento
    # -----------------------------------------------------------------

    def move_angles(self, angles):
        safe = [
            clamp(a, JOINT_MIN[i], JOINT_MAX[i])
            for i, a in enumerate(angles)
        ]
        if safe != list(angles):
            print(f"  (recortado a limites articulares: {fmt(safe)})")
        self.mc.send_angles(safe, self.speed)
        self.wait_arrival(safe)

    def wait_arrival(self, target, tolerance=2.0, timeout=20.0):
        deadline = time.monotonic() + timeout
        stable = 0
        time.sleep(0.3)
        while time.monotonic() < deadline:
            current = self._read("get_angles", 6, retries=1, delay=0.0)
            if current is not None:
                error = max(
                    abs(current[i] - target[i]) for i in range(6)
                )
                if error <= tolerance:
                    stable += 1
                    if stable >= 3:
                        print(f"  llegado. error max {error:.2f} deg")
                        return True
                else:
                    stable = 0
            time.sleep(0.1)
        print("  AVISO: no confirmo llegada (timeout).")
        return False

    def set_gripper(self, value):
        value = int(clamp(value, 0, 100))
        try:
            self.mc.set_gripper_value(value, 50, 1)
        except Exception as exc:  # noqa: BLE001
            print(f"  set_gripper_value fallo ({exc}); pruebo state...")
            try:
                self.mc.set_gripper_state(0 if value >= 50 else 1, 50, 1)
            except Exception as exc2:  # noqa: BLE001
                print(f"  ERROR: {exc2}")
                return
        time.sleep(1.2)
        read = self.gripper_value()
        print(f"  pinza -> {value}   (lectura: {read})")

    def gripper_value(self):
        try:
            value = self.mc.get_gripper_value()
            return value if isinstance(value, (int, float)) else "?"
        except Exception:  # noqa: BLE001
            return "?"

    # -----------------------------------------------------------------
    # Bucle de comandos
    # -----------------------------------------------------------------

    def run(self):
        print("\nEscribe 'help' para ver los comandos. 'q' para salir.\n")
        while True:
            try:
                line = input("mecharm> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue

            parts = line.split()
            cmd, args = parts[0].lower(), parts[1:]

            try:
                if cmd in ("q", "quit", "exit"):
                    break

                elif cmd == "help":
                    print(__doc__.split("Comandos")[1])

                elif cmd in ("a", "angles"):
                    print(f"  angulos: {fmt(self.read_angles())}")

                elif cmd in ("c", "coords"):
                    print(f"  coords : {fmt(self.read_coords())}")

                elif cmd == "open":
                    self.set_gripper(100)

                elif cmd == "close":
                    self.set_gripper(20)

                elif cmd == "g":
                    if not args:
                        print("  uso: g <0-100>")
                    else:
                        self.set_gripper(int(float(args[0])))

                elif cmd == "gv":
                    print(f"  valor pinza: {self.gripper_value()}")

                elif cmd == "sweep":
                    print("  barrido de la pinza 100 -> 0 ...")
                    for value in (100, 80, 60, 40, 20, 0):
                        self.set_gripper(value)
                    print("  fin del barrido. Anota en que valor la "
                          "pinza sujeta la pieza sin forzarla.")

                elif cmd == "move":
                    if len(args) != 6:
                        print("  uso: move j1 j2 j3 j4 j5 j6")
                    else:
                        self.move_angles([float(x) for x in args])

                elif cmd == "goto":
                    if not args:
                        print(f"  uso: goto <pose>  {sorted(self.poses)}")
                    elif args[0] not in self.poses:
                        print(f"  pose desconocida. Hay: "
                              f"{sorted(self.poses)}")
                    else:
                        self.move_angles(self.poses[args[0]])

                elif cmd == "home":
                    if "home" in self.poses:
                        self.move_angles(self.poses["home"])
                    else:
                        self.move_angles([0.0] * 6)

                elif cmd == "free":
                    print("  !! SUJETA EL BRAZO: va a caer por su peso.")
                    if input("  escribe SI para continuar: ") == "SI":
                        self.mc.release_all_servos(1)
                        print("  servos liberados. Mueve el brazo y usa "
                              "'save <nombre>'.")
                    else:
                        print("  cancelado.")

                elif cmd == "lock":
                    self.mc.power_on()
                    self.mc.clear_error_information()
                    print("  servos alimentados y error limpiado.")

                elif cmd == "save":
                    if not args:
                        print("  uso: save <nombre>")
                    else:
                        angles = self.read_angles()
                        if angles is None:
                            print("  ERROR: no se pudieron leer angulos.")
                        else:
                            self.poses[args[0]] = angles
                            print(f"  {args[0]} = {fmt(angles)}")
                            self.save_poses()

                elif cmd == "list":
                    if not self.poses:
                        print("  (ninguna pose guardada)")
                    for name, angles in sorted(self.poses.items()):
                        print(f"  {name:<16} {fmt(angles)}")

                elif cmd == "speed":
                    if not args:
                        print(f"  velocidad actual: {self.speed}")
                    else:
                        self.speed = int(clamp(int(args[0]), 1, 100))
                        print(f"  velocidad -> {self.speed}")

                else:
                    print(f"  comando desconocido: {cmd}  ('help')")

            except Exception as exc:  # noqa: BLE001
                print(f"  ERROR: {exc}")

        print("Saliendo. El brazo queda alimentado.")


def fmt(values):
    if values is None:
        return "(sin lectura)"
    return "[" + ", ".join(f"{v:7.2f}" for v in values) + "]"


def main():
    parser = argparse.ArgumentParser(
        description="Consola interactiva del MechArm 270 M5."
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--poses-file", default=DEFAULT_POSES_FILE)
    parser.add_argument("--speed", type=int, default=30)
    args = parser.parse_args()

    if not os.path.exists(args.port):
        print(f"ERROR: {args.port} no existe. Comprueba el cable USB y")
        print("       'ls /dev/ttyACM*'.")
        return 1

    Console(args.port, args.baud, args.poses_file, args.speed).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
