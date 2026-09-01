#!/usr/bin/env python3
"""
Teleoperacion del myAGV con un mando Bluetooth (Xbox Series / Xbox One).

Este nodo NO usa el stack `joy` de ROS. Abre el mando directamente como
dispositivo de entrada del kernel (evdev, /dev/input/eventX), que es la forma
en la que Linux expone un mando emparejado por Bluetooth. A partir de los
eventos del mando publica `geometry_msgs/Twist` para el movimiento
omnidireccional del myAGV (ruedas mecanum: linear.x, linear.y, angular.z).

Mapeo de controles
------------------
  * Gatillo RT ............ avanzar        (linear.x > 0)
  * Gatillo LT ............ retroceder     (linear.x < 0)
  * Boton  LB ............. girar izquierda (angular.z > 0)
  * Boton  RB ............. girar derecha   (angular.z < 0)
  * Joystick izquierdo .... adelante/atras (linear.x) e izquierda/derecha
                            (linear.y, traslacion lateral omnidireccional)
  * Cruceta (D-Pad) ....... mismos ejes que el joystick izquierdo

Los aportes del joystick, la cruceta y los gatillos se suman y se recortan
al maximo configurado, de modo que cualquier combinacion es valida.

Requisitos
----------
  sudo apt install python3-evdev      (o: pip3 install evdev)

El usuario debe pertenecer al grupo `input` para leer /dev/input/event*:
  sudo usermod -aG input $USER   (y volver a iniciar sesion)
"""

import os
import select
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


# ============================================================
# Dependencia opcional: python-evdev
# ============================================================

try:
    from evdev import InputDevice, ecodes, list_devices

    EVDEV_AVAILABLE = True
except ImportError:  # pragma: no cover - depende del entorno
    EVDEV_AVAILABLE = False
    ecodes = None


# ============================================================
# Utilidades
# ============================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def apply_deadzone(value, deadzone):
    """
    Aplica zona muerta con reescalado a un valor normalizado en [-1, 1].

    Por debajo de `deadzone` devuelve 0.0 y el resto del recorrido se
    reescala a [0, 1] para no perder resolucion.
    """
    if deadzone <= 0.0:
        return value

    magnitude = abs(value)

    if magnitude <= deadzone:
        return 0.0

    scaled = (magnitude - deadzone) / (1.0 - deadzone)

    return scaled if value > 0.0 else -scaled


# ============================================================
# Nombres logicos de los controles del mando
# ============================================================
#
# Los codigos por defecto corresponden a un mando Xbox Series/One
# reconocido por el driver `xpad` / `xpadneo` en Linux. Si tu mando
# expone codigos distintos puedes sobreescribirlos por parametros.

DEFAULT_CODE_MAP = {
    # Ejes analogicos
    "axis_stick_x": "ABS_X",        # joystick izq. horizontal (+ derecha)
    "axis_stick_y": "ABS_Y",        # joystick izq. vertical   (+ abajo)
    "axis_trigger_lt": "ABS_Z",     # gatillo izquierdo  (reposo 0 -> max)
    "axis_trigger_rt": "ABS_RZ",    # gatillo derecho    (reposo 0 -> max)
    "axis_dpad_x": "ABS_HAT0X",     # cruceta horizontal (+ derecha)
    "axis_dpad_y": "ABS_HAT0Y",     # cruceta vertical   (+ abajo)
    # Botones
    "button_lb": "BTN_TL",
    "button_rb": "BTN_TR",
}

# Algunos drivers exponen la cruceta como botones en vez de como eje HAT.
DPAD_BUTTON_CODES = {
    "up": "BTN_DPAD_UP",
    "down": "BTN_DPAD_DOWN",
    "left": "BTN_DPAD_LEFT",
    "right": "BTN_DPAD_RIGHT",
}


# ============================================================
# Lectura del mando (hilo dedicado)
# ============================================================

class GamepadReader:
    """
    Mantiene la conexion con el mando y traduce sus eventos evdev.

    El estado resultante es normalizado y esta protegido por lock:

        stick_x, stick_y   in [-1, 1]
        dpad_x,  dpad_y     in [-1, 1]
        trigger_lt, trigger_rt in [0, 1]
        button_lb, button_rb   in {0, 1}

    Si el mando se desconecta el estado se pone a cero y el hilo intenta
    reconectar periodicamente.
    """

    def __init__(self, node, code_map):
        self._node = node
        self._log = node.get_logger()

        self._device_path = (
            node.get_parameter("device_path").value or ""
        )
        self._device_name = (
            node.get_parameter("device_name").value or ""
        )
        self._reconnect_period = float(
            node.get_parameter("reconnect_period").value
        )

        # Resolucion de codigos evdev (str -> int)
        self._code = {}
        for logical, default_name in DEFAULT_CODE_MAP.items():
            name = node.get_parameter(code_map[logical]).value or default_name
            self._code[logical] = ecodes.ecodes[name]

        self._dpad_btn = {}
        for direction, default_name in DPAD_BUTTON_CODES.items():
            self._dpad_btn[direction] = ecodes.ecodes.get(default_name)

        # Estado compartido
        self._lock = threading.Lock()
        self._reset_state()

        self._connected = False
        self._last_event_time = 0.0

        # Rangos de los ejes (se rellenan al abrir el dispositivo)
        self._absinfo = {}

        self._device = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    # ---------------------------------------------------------
    # API publica
    # ---------------------------------------------------------

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def get_state(self):
        with self._lock:
            return dict(self._state)

    @property
    def connected(self):
        return self._connected

    @property
    def seconds_since_last_event(self):
        if self._last_event_time == 0.0:
            return float("inf")
        return time.monotonic() - self._last_event_time

    # ---------------------------------------------------------
    # Interno
    # ---------------------------------------------------------

    def _reset_state(self):
        self._state = {
            "stick_x": 0.0,
            "stick_y": 0.0,
            "dpad_x": 0.0,
            "dpad_y": 0.0,
            "trigger_lt": 0.0,
            "trigger_rt": 0.0,
            "button_lb": 0,
            "button_rb": 0,
        }

    def _find_device(self):
        """Localiza el dispositivo del mando por ruta o por nombre."""
        if self._device_path:
            if os.path.exists(self._device_path):
                return self._device_path
            return None

        wanted = self._device_name.lower()

        for path in list_devices():
            try:
                dev = InputDevice(path)
            except OSError:
                continue

            caps = dev.capabilities()
            has_abs = ecodes.EV_ABS in caps
            has_key = ecodes.EV_KEY in caps

            name_match = (not wanted) or (wanted in dev.name.lower())

            # Un mando real tiene ejes analogicos y botones.
            if name_match and has_abs and has_key:
                self._log.info(
                    "Mando encontrado: '%s' en %s" % (dev.name, path)
                )
                dev.close()
                return path

            dev.close()

        return None

    def _open_device(self):
        path = self._find_device()

        if path is None:
            return False

        try:
            self._device = InputDevice(path)
        except OSError as exc:
            self._log.warning("No se pudo abrir %s: %s" % (path, exc))
            return False

        # Cachear rangos de los ejes
        self._absinfo = {}
        abs_caps = dict(
            self._device.capabilities().get(ecodes.EV_ABS, [])
        )
        for code, info in abs_caps.items():
            self._absinfo[code] = info

        self._connected = True
        self._log.info(
            "Conectado al mando '%s' (%s)"
            % (self._device.name, self._device.path)
        )
        return True

    def _close_device(self):
        if self._device is not None:
            try:
                self._device.close()
            except OSError:
                pass
        self._device = None
        self._connected = False
        with self._lock:
            self._reset_state()

    def _norm_axis(self, code, raw):
        """Normaliza un eje bipolar (joystick) a [-1, 1]."""
        info = self._absinfo.get(code)
        if info is None:
            return 0.0

        lo, hi = info.min, info.max
        if hi == lo:
            return 0.0

        # Mapea [lo, hi] -> [-1, 1]
        value = 2.0 * (raw - lo) / (hi - lo) - 1.0
        return clamp(value, -1.0, 1.0)

    def _norm_trigger(self, code, raw):
        """Normaliza un gatillo (reposo = minimo) a [0, 1]."""
        info = self._absinfo.get(code)
        if info is None:
            # Fallback razonable para xpad (0..255)
            return clamp(raw / 255.0, 0.0, 1.0)

        lo, hi = info.min, info.max
        if hi == lo:
            return 0.0

        return clamp((raw - lo) / (hi - lo), 0.0, 1.0)

    def _handle_event(self, event):
        now = time.monotonic()

        if event.type == ecodes.EV_ABS:
            with self._lock:
                if event.code == self._code["axis_stick_x"]:
                    self._state["stick_x"] = self._norm_axis(
                        event.code, event.value
                    )
                    self._last_event_time = now
                elif event.code == self._code["axis_stick_y"]:
                    self._state["stick_y"] = self._norm_axis(
                        event.code, event.value
                    )
                    self._last_event_time = now
                elif event.code == self._code["axis_trigger_lt"]:
                    self._state["trigger_lt"] = self._norm_trigger(
                        event.code, event.value
                    )
                    self._last_event_time = now
                elif event.code == self._code["axis_trigger_rt"]:
                    self._state["trigger_rt"] = self._norm_trigger(
                        event.code, event.value
                    )
                    self._last_event_time = now
                elif event.code == self._code["axis_dpad_x"]:
                    self._state["dpad_x"] = float(
                        clamp(event.value, -1, 1)
                    )
                    self._last_event_time = now
                elif event.code == self._code["axis_dpad_y"]:
                    self._state["dpad_y"] = float(
                        clamp(event.value, -1, 1)
                    )
                    self._last_event_time = now

        elif event.type == ecodes.EV_KEY:
            pressed = 1 if event.value else 0

            with self._lock:
                if event.code == self._code["button_lb"]:
                    self._state["button_lb"] = pressed
                    self._last_event_time = now
                elif event.code == self._code["button_rb"]:
                    self._state["button_rb"] = pressed
                    self._last_event_time = now
                elif event.code == self._dpad_btn.get("up"):
                    self._state["dpad_y"] = -1.0 if pressed else 0.0
                    self._last_event_time = now
                elif event.code == self._dpad_btn.get("down"):
                    self._state["dpad_y"] = 1.0 if pressed else 0.0
                    self._last_event_time = now
                elif event.code == self._dpad_btn.get("left"):
                    self._state["dpad_x"] = -1.0 if pressed else 0.0
                    self._last_event_time = now
                elif event.code == self._dpad_btn.get("right"):
                    self._state["dpad_x"] = 1.0 if pressed else 0.0
                    self._last_event_time = now

    def _run(self):
        while not self._stop.is_set():

            if self._device is None:
                if not self._open_device():
                    self._log.warning(
                        "Mando no disponible, reintentando en %.1fs "
                        "(empareja el control por Bluetooth)"
                        % self._reconnect_period,
                        throttle_duration_sec=10.0,
                    )
                    self._stop.wait(self._reconnect_period)
                    continue

            try:
                # Espera con timeout para poder atender la parada.
                r, _, _ = select.select([self._device.fd], [], [], 0.5)
                if not r:
                    continue

                for event in self._device.read():
                    self._handle_event(event)

            except (OSError, IOError) as exc:
                self._log.warning(
                    "Mando desconectado (%s). Reintentando..." % exc
                )
                self._close_device()
                self._stop.wait(self._reconnect_period)


# ============================================================
# Nodo ROS 2
# ============================================================

class BluetoothGamepadTeleop(Node):

    def __init__(self):
        super().__init__("bluetooth_gamepad_teleop")

        # ----------------------------------------------------
        # Parametros: dispositivo
        # ----------------------------------------------------
        self.declare_parameter("device_path", "")
        self.declare_parameter("device_name", "Xbox Wireless Controller")
        self.declare_parameter("reconnect_period", 2.0)

        # ----------------------------------------------------
        # Parametros: salida
        # ----------------------------------------------------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("publish_zero_when_idle", True)

        # ----------------------------------------------------
        # Parametros: limites de velocidad (m/s, rad/s)
        # ----------------------------------------------------
        self.declare_parameter("max_linear_x", 0.4)
        self.declare_parameter("max_linear_y", 0.4)
        self.declare_parameter("max_angular_z", 1.2)

        # ----------------------------------------------------
        # Parametros: forma de la respuesta
        # ----------------------------------------------------
        self.declare_parameter("stick_deadzone", 0.12)
        self.declare_parameter("trigger_deadzone", 0.05)
        self.declare_parameter("slew_rate", 2.0)  # fraccion de rango / s (0 = off)
        self.declare_parameter("controller_timeout", 1.0)

        # Inversiones opcionales (segun montaje / preferencia)
        self.declare_parameter("invert_linear_x", False)
        self.declare_parameter("invert_linear_y", False)
        self.declare_parameter("invert_angular_z", False)

        # ----------------------------------------------------
        # Parametros: nombres de codigos evdev (avanzado)
        # ----------------------------------------------------
        code_map = {}
        for logical, default_name in DEFAULT_CODE_MAP.items():
            pname = "code." + logical
            self.declare_parameter(pname, default_name)
            code_map[logical] = pname

        # ----------------------------------------------------
        # Leer parametros
        # ----------------------------------------------------
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.publish_zero_when_idle = bool(
            self.get_parameter("publish_zero_when_idle").value
        )

        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_linear_y = float(self.get_parameter("max_linear_y").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)

        self.stick_deadzone = float(
            self.get_parameter("stick_deadzone").value
        )
        self.trigger_deadzone = float(
            self.get_parameter("trigger_deadzone").value
        )
        self.slew_rate = float(self.get_parameter("slew_rate").value)
        self.controller_timeout = float(
            self.get_parameter("controller_timeout").value
        )

        self.invert_linear_x = bool(
            self.get_parameter("invert_linear_x").value
        )
        self.invert_linear_y = bool(
            self.get_parameter("invert_linear_y").value
        )
        self.invert_angular_z = bool(
            self.get_parameter("invert_angular_z").value
        )

        # ----------------------------------------------------
        # Estado de salida (para el limitador de aceleracion)
        # ----------------------------------------------------
        self._out = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._last_tick = time.monotonic()
        self._was_connected = False

        # ----------------------------------------------------
        # ROS
        # ----------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ----------------------------------------------------
        # Lector del mando
        # ----------------------------------------------------
        self.reader = GamepadReader(self, code_map)
        self.reader.start()

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            "bluetooth_gamepad_teleop listo. Publicando Twist en '%s' "
            "(max lin_x=%.2f, lin_y=%.2f m/s, ang_z=%.2f rad/s)"
            % (
                self.cmd_vel_topic,
                self.max_linear_x,
                self.max_linear_y,
                self.max_angular_z,
            )
        )

    # --------------------------------------------------------
    # Bucle de control
    # --------------------------------------------------------

    def _compute_target(self, state):
        """
        Combina joystick, cruceta y gatillos en un objetivo normalizado.

        Devuelve fracciones de rango en [-1, 1] con la convencion REP-103:
        x adelante, y izquierda, z antihorario.
        """
        # --- Joystick izquierdo -----------------------------
        # ABS_Y crece hacia abajo  -> adelante = -stick_y
        # ABS_X crece hacia la derecha -> izquierda (+y) = -stick_x
        stick_fwd = -apply_deadzone(state["stick_y"], self.stick_deadzone)
        stick_left = -apply_deadzone(state["stick_x"], self.stick_deadzone)

        # --- Cruceta (D-Pad) --------------------------------
        # HAT0Y: -1 arriba/adelante, +1 abajo/atras
        # HAT0X: -1 izquierda, +1 derecha
        dpad_fwd = -state["dpad_y"]
        dpad_left = -state["dpad_x"]

        # --- Gatillos --------------------------------------
        # RT -> adelante ; LT -> atras
        lt = apply_deadzone(state["trigger_lt"], self.trigger_deadzone)
        rt = apply_deadzone(state["trigger_rt"], self.trigger_deadzone)
        trigger_fwd = rt - lt

        # --- Giro -----------------------------------------
        # LB -> girar izquierda (+z) ; RB -> girar derecha (-z)
        turn = float(state["button_lb"]) - float(state["button_rb"])

        target_x = clamp(stick_fwd + dpad_fwd + trigger_fwd, -1.0, 1.0)
        target_y = clamp(stick_left + dpad_left, -1.0, 1.0)
        target_z = clamp(turn, -1.0, 1.0)

        if self.invert_linear_x:
            target_x = -target_x
        if self.invert_linear_y:
            target_y = -target_y
        if self.invert_angular_z:
            target_z = -target_z

        return target_x, target_y, target_z

    def _slew(self, current, target, dt):
        if self.slew_rate <= 0.0:
            return target

        max_step = self.slew_rate * dt
        delta = clamp(target - current, -max_step, max_step)
        return current + delta

    def _on_timer(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        connected = self.reader.connected
        stale = (
            self.reader.seconds_since_last_event > self.controller_timeout
        )

        if connected and not self._was_connected:
            self.get_logger().info("Mando activo.")
        elif not connected and self._was_connected:
            self.get_logger().warning(
                "Mando perdido: enviando parada."
            )
        self._was_connected = connected

        if not connected:
            # Sin mando -> frenar y no seguir publicando basura.
            self._out = {"x": 0.0, "y": 0.0, "z": 0.0}
            if self.publish_zero_when_idle:
                self._publish(0.0, 0.0, 0.0)
            return

        if stale:
            target_x = target_y = target_z = 0.0
        else:
            state = self.reader.get_state()
            target_x, target_y, target_z = self._compute_target(state)

        self._out["x"] = self._slew(self._out["x"], target_x, dt)
        self._out["y"] = self._slew(self._out["y"], target_y, dt)
        self._out["z"] = self._slew(self._out["z"], target_z, dt)

        moving = any(
            abs(self._out[k]) > 1e-3 for k in ("x", "y", "z")
        )

        if moving or self.publish_zero_when_idle:
            self._publish(
                self._out["x"] * self.max_linear_x,
                self._out["y"] * self.max_linear_y,
                self._out["z"] * self.max_angular_z,
            )

    def _publish(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    # --------------------------------------------------------

    def destroy_node(self):
        try:
            self.reader.stop()
        finally:
            # Ultimo mensaje de parada para dejar el robot quieto.
            try:
                self._publish(0.0, 0.0, 0.0)
            except Exception:
                pass
            super().destroy_node()


# ============================================================
# main
# ============================================================

def main(args=None):
    rclpy.init(args=args)

    if not EVDEV_AVAILABLE:
        print(
            "\n[bluetooth_gamepad_teleop] Falta la dependencia 'evdev'.\n"
            "  Instalala con:  sudo apt install python3-evdev\n"
            "             o :  pip3 install evdev\n"
        )
        rclpy.shutdown()
        return

    node = BluetoothGamepadTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
