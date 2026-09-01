# myagv_teleop_joy

Teleoperación **omnidireccional** del myAGV con un mando **Bluetooth**
(pensado para un **Xbox Series / Xbox One**, sirve cualquier mando reconocido
por Linux).

El nodo `bluetooth_gamepad_teleop` **no usa el stack `joy` de ROS**. Abre el
mando directamente como dispositivo de entrada del kernel (`evdev`,
`/dev/input/eventX`), que es la forma en que Linux expone un mando emparejado
por Bluetooth, y publica `geometry_msgs/Twist`.

## Mapeo de controles

| Control | Acción | Campo de `Twist` |
|---|---|---|
| **RT** (gatillo derecho) | avanzar | `linear.x > 0` |
| **LT** (gatillo izquierdo) | retroceder | `linear.x < 0` |
| **LB** | girar a la izquierda | `angular.z > 0` |
| **RB** | girar a la derecha | `angular.z < 0` |
| **Joystick izquierdo** ↑↓ | adelante / atrás | `linear.x` |
| **Joystick izquierdo** ←→ | traslación lateral | `linear.y` |
| **Cruceta (D-Pad)** ↑↓ | adelante / atrás | `linear.x` |
| **Cruceta (D-Pad)** ←→ | traslación lateral | `linear.y` |

Los aportes del joystick, la cruceta y los gatillos se **suman** y se recortan
al máximo configurado. Si no llegan eventos del mando durante
`controller_timeout` segundos, o si el mando se desconecta, el nodo publica
velocidad cero.

## Dependencias

```bash
sudo apt install python3-evdev        # o: pip3 install evdev
```

El usuario debe poder leer `/dev/input/event*` (grupo `input`):

```bash
sudo usermod -aG input $USER          # y volver a iniciar sesión
```

## Emparejar el mando Xbox por Bluetooth

```bash
bluetoothctl
> power on
> agent on
> scan on
# mantener pulsado el botón de sincronización del mando hasta que parpadee rápido
> pair <MAC_DEL_MANDO>
> trust <MAC_DEL_MANDO>
> connect <MAC_DEL_MANDO>
```

Comprueba que aparece como dispositivo de entrada:

```bash
cat /proc/bus/input/devices | grep -iA5 xbox
ls -l /dev/input/by-id/*event-joystick*
```

> **WSL:** el Bluetooth no pasa a WSL directamente. Empareja el mando (o su
> receptor USB inalámbrico) en Windows y adjunta el dispositivo con
> `usbipd attach --wsl --busid <BUSID>` como se hace con el resto de
> periféricos del proyecto.

## Uso

```bash
colcon build --packages-select myagv_teleop_joy
source install/setup.bash

# con parámetros por defecto (autodetección del mando, publica en /cmd_vel)
ros2 launch myagv_teleop_joy bluetooth_gamepad_teleop.launch.py

# o directamente el nodo con un fichero de parámetros
ros2 run myagv_teleop_joy bluetooth_gamepad_teleop \
    --ros-args --params-file src/myagv_teleop_joy/config/xbox_series.yaml
```

Publicar en otro topic (p. ej. para el `twist_mux` del workspace):

```bash
ros2 launch myagv_teleop_joy bluetooth_gamepad_teleop.launch.py \
    cmd_vel_topic:=/cmd_vel_joy
```

## Integración con `twist_mux`

Este workspace multiplexa velocidades con `twist_mux`
(`src/mobile_manipulator_sim/config/twist_mux.yaml`). Para que el mando tenga
prioridad sobre la navegación añade una entrada:

```yaml
twist_mux:
  ros__parameters:
    topics:
      joy:
        topic: /cmd_vel_joy
        timeout: 0.5
        priority: 120        # por encima de aruco (100) y navigation (50)
```

y lanza el nodo con `cmd_vel_topic:=/cmd_vel_joy`.

## Parámetros principales

| Parámetro | Def. | Descripción |
|---|---|---|
| `device_path` | `""` | Ruta fija (`/dev/input/eventX`). Vacío = autodetección. |
| `device_name` | `"Xbox Wireless Controller"` | Subcadena del nombre para autodetección. |
| `cmd_vel_topic` | `/cmd_vel` | Topic `Twist` de salida. |
| `publish_rate` | `20.0` | Frecuencia de publicación (Hz). |
| `max_linear_x` | `0.4` | Velocidad lineal máx. adelante/atrás (m/s). |
| `max_linear_y` | `0.4` | Velocidad lineal máx. lateral (m/s). |
| `max_angular_z` | `1.2` | Velocidad angular máx. (rad/s). |
| `stick_deadzone` | `0.12` | Zona muerta del joystick. |
| `trigger_deadzone` | `0.05` | Zona muerta de los gatillos. |
| `slew_rate` | `2.0` | Límite de aceleración (fracción de rango/s). `0` = desactivado. |
| `controller_timeout` | `1.0` | Segundos sin eventos → frena. |
| `invert_linear_x` / `_y` / `_angular_z` | `false` | Inversión de ejes. |
| `code.*` | ver `config/xbox_series.yaml` | Nombres de códigos `evdev` (solo si tu mando difiere del `xpad` estándar). |

## Diagnóstico

```bash
# ver qué códigos emite tu mando en tiempo real
python3 -m evdev.evtest

# comprobar la salida del nodo
ros2 topic echo /cmd_vel
```
