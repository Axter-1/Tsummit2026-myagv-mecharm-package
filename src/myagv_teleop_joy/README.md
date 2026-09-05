# myagv_teleop_joy

Teleoperación **omnidireccional** del myAGV con un gamepad genérico **USB**
(identificado por Linux como **TGZ Controller**).

El nodo `bluetooth_gamepad_teleop` **no usa el stack `joy` de ROS**. Abre el
mando directamente como dispositivo de entrada del kernel (`evdev`,
`/dev/input/eventX`), que es la forma en que Linux expone un mando emparejado
por Bluetooth, y publica `geometry_msgs/Twist`.

## Mapeo de controles

| Control | Acción | Campo de `Twist` |
|---|---|---|
| **Joystick izquierdo** ↑↓ | avance/retroceso | `linear.x` |
| **Joystick izquierdo** ←→ | desplazamiento lateral | `linear.y` |
| **Cruceta** ↑↓←→ | movimiento lineal alternativo | `linear.x` / `linear.y` |
| **LB / L1** | girar a la izquierda | `angular.z > 0` |
| **RB / R1 / R2** | girar a la derecha | `angular.z < 0` |

Solo el stick izquierdo y LB/RB generan movimiento. El stick derecho, gatillos
y cruceta se ignoran. Si no llegan eventos del mando durante
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

## Conectar el gamepad USB

```bash
ls -l /dev/input/by-id/*event-joystick*
cat /proc/bus/input/devices
```

El gamepad utilizado aparece como `TGZ Controller`. El evento puede cambiar
entre `/dev/input/event2`, `/dev/input/event7`, etc.; el nodo lo detecta por
nombre y capacidades, no por el número del evento.

Comprueba que aparece como dispositivo de entrada:

```bash
cat /proc/bus/input/devices | grep -iA5 xbox
ls -l /dev/input/by-id/*event-joystick*
```

El contenedor monta `/dev/input` y permite dinámicamente la clase evdev, por lo
que no es necesario configurar manualmente `eventX`.

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
| `device_name` | `"TGZ Controller"` | Subcadena del nombre para autodetección. |
| `cmd_vel_topic` | `/cmd_vel` | Topic `Twist` de salida. |
| `publish_rate` | `100.0` | Frecuencia de publicación (Hz). |
| `max_linear_x` | `0.4` | Velocidad lineal máx. adelante/atrás (m/s). |
| `max_linear_y` | `0.4` | Velocidad lineal máx. lateral (m/s). |
| `max_angular_z` | `1.2` | Velocidad angular máx. (rad/s). |
| `stick_deadzone` | `0.12` | Zona muerta del joystick. |
| `trigger_deadzone` | `0.05` | Zona muerta de los gatillos. |
| `slew_rate` | `0.0` | Respuesta inmediata. `0` = desactivado. |
| `controller_timeout` | `0.0` | `0` mantiene el último estado hasta soltar/desconectar. |
| `invert_linear_x` / `_y` / `_angular_z` | `false` | Inversión de ejes. |
| `code.*` | ver `config/xbox_series.yaml` | Nombres de códigos `evdev` (solo si tu mando difiere del `xpad` estándar). |

## Diagnóstico

```bash
# ver qué códigos emite el gamepad en tiempo real, sin fijar eventX
./scripts/myagv_commands.sh inputs

# comprobar la salida del nodo
./scripts/myagv_commands.sh cmd_vel
```

Si `ros2 topic echo /cmd_vel` muestra `RuntimeError: !rclpy.ok()`, el daemon
de la CLI quedo en un estado invalido. Reinicialo con:

```bash
ros2 daemon stop
ros2 daemon start
```
