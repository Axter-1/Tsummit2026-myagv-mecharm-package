# Guía de pruebas del brazo MechArm 270 y la garra

Procedimiento para verificar el brazo y **enseñar las poses** que necesitan
los retos 1, 2 y 3. Está pensada para hacerse con el robot **sobre la mesa,
sin moverse**, antes de tocar la pista.

> **Nada de esto se ha probado todavía con el robot.** Toda la guía está
> escrita a partir de la hoja de datos del MechArm 270 M5 y de la API de
> `pymycobot 4.0.6` verificada en el contenedor.

---

## 0. Seguridad — leer antes de empezar

| Riesgo | Precaución |
|---|---|
| Al **liberar los servos** (`free`) el brazo **cae por su propio peso** | Sujétalo con la mano *antes* de ejecutar el comando |
| El brazo puede golpear el chasis o el LiDAR | Empieza siempre con velocidad **≤ 30** |
| Carga máxima **250 g**, alcance **270 mm** | Una pose fuera de alcance hace que el brazo se quede empujando |
| El puerto serie es **exclusivo** | Solo un proceso a la vez puede usar `/dev/ttyACM0` |

Parada de emergencia: `Ctrl+C` en la consola y, si hace falta, apagar el
brazo por su interruptor.

---

## 1. Comprobaciones previas

```bash
# 1. El contenedor está corriendo (modo ligero: base + LiDAR)
START_BRINGUP=0 ./docker/run_jetson_robot.sh

# 2. En otra terminal: ¿existe el puerto del brazo?
./scripts/run_classification.sh shell
ls -l /dev/ttyACM*        # debe aparecer /dev/ttyACM0
python3 -c "from pymycobot.mecharm270 import MechArm270; print('pymycobot OK')"
```

Si `/dev/ttyACM0` no aparece: revisa el cable USB-C del brazo y que esté
encendido. Si aparece como `ttyACM1`, pásalo con `ARM_PORT=/dev/ttyACM1`.

---

## 2. Prueba de contacto con la consola directa

La consola habla con el brazo **sin pasar por ROS**. Es la forma más
rápida de saber si el hardware responde.

```bash
./scripts/run_classification.sh teach
```

Esto detiene `mecharm_driver_node` (para liberar el puerto) y abre:

```
mecharm> a                 # leer ángulos  -> [ 0.00, 0.00, ...]
mecharm> c                 # leer coordenadas [X,Y,Z,RX,RY,RZ] en mm/grados
```

**Si `a` devuelve `(sin lectura)`** el brazo no está respondiendo: revisa
alimentación, cable y que ningún otro proceso tenga el puerto.

---

## 3. Calibrar la garra ⭐

Este es el punto que más importa: hay que averiguar **con qué valor la
pinza sujeta la pieza sin forzarla**.

```
mecharm> open              # abre del todo (valor 100)
mecharm> close             # cierra (valor 20)
mecharm> gv                # lee el valor actual de la pinza
```

Barrido completo para ver todo el recorrido:

```
mecharm> sweep             # 100 -> 80 -> 60 -> 40 -> 20 -> 0
```

**Procedimiento con la pieza real:**

1. `g 100` → pinza abierta.
2. Coloca a mano una pieza impresa en 3D entre los dedos.
3. Baja el valor de 10 en 10: `g 60`, `g 50`, `g 40`…
4. Anota el **primer valor que sujeta la pieza firmemente** sin que los
   dedos se queden forzando (el motor no debe "zumbar").
5. Ese es tu `gripper_closed_value`. El `gripper_open_value` es el valor
   más alto en el que la pieza entra y sale sin rozar (normalmente 100).

Escribe los dos valores en
`src/myagv_mecharm_service/config/mecharm.yaml`:

```yaml
    gripper_open_value: 100      # <- tu valor
    gripper_closed_value: 20     # <- tu valor
```

> **Nota:** el driver usa `set_gripper_value(0..100)`, donde **0 = cerrada**
> y **100 = abierta**. Si esa llamada fallara, cae automáticamente a
> `set_gripper_state`, cuyo flag es al revés de lo que suele suponerse
> (**0 = abrir**, 1 = cerrar). Ese detalle ya está resuelto en el código.

---

## 4. Enseñar las poses

Los retos 1 y 2 necesitan **cinco poses**:

| Pose | Para qué |
|---|---|
| `home` | brazo plegado — se usa al navegar y es obligatoria en el laberinto |
| `observe` | mirando la pieza desde arriba, antes de bajar |
| `carry` | sujetando la pieza durante el traslado |
| `pick_table` | agarre sobre la plataforma de carga |
| `place_table` | soltado sobre la plataforma de destino |

### Procedimiento

```
mecharm> speed 25          # velocidad prudente
mecharm> free              # pide confirmación: escribe SI
                           # ¡SUJETA EL BRAZO ANTES!
```

Con los servos liberados, **mueve el brazo a mano** hasta la posición
deseada y guárdala:

```
mecharm> save home
mecharm> save observe
mecharm> save pick_table
...
mecharm> lock              # vuelve a alimentar los servos
mecharm> list              # revisa lo guardado
```

Cada `save` **reescribe** `src/myagv_mecharm_service/config/poses.yaml`.

La calibracion de una pieza tambien puede ser la fuente de la pose `home`:
`calibrate-grasp` la captura al principio antes de abrir la pinza. Para
reensenar todas las poses globales (`home`, `safe_navigation`, `carry`,
`observe`, `pick_table` y `place_table`) usa:

```bash
ALLOW_MOTION=1 ./scripts/tsummit.sh calibrate-grasp poste 100 \
  --capture-global-poses
```

Todos los nodos y ensayos leen esas poses desde el mismo `poses.yaml`.

### Verificar que una pose se alcanza sola

```
mecharm> home              # vuelve a home
mecharm> goto pick_table   # debe llegar y decir "llegado. error max X deg"
```

Si dice `AVISO: no confirmó llegada (timeout)` la pose está fuera de
alcance o hay una obstrucción.

> **Importante para `pick_table` y `place_table`:** enséñalas con el robot
> colocado **exactamente donde lo deja el paso `aruco`** (a
> `stop_distance` del marcador). Si las enseñas con el robot en otro
> sitio, el agarre no coincidirá.

### Instalar las poses nuevas

`poses.yaml` vive en `src/`, pero el nodo lee la copia instalada:

```bash
./scripts/run_classification.sh shell
colcon build --packages-select myagv_mecharm_service
```

*(con `--symlink-install`, que es como se compila aquí, el enlace ya
apunta al archivo de `src/` y suele bastar con reiniciar el nodo)*

---

## 5. Probar el brazo por ROS

Sal de la consola (`q`) y arranca la pila para probar el camino real
(el que usará la misión):

```bash
./scripts/run_classification.sh stack     # pila SIN misión
```

En otra terminal:

```bash
# Garra por servicio
./scripts/run_classification.sh gripper 100     # abrir
./scripts/run_classification.sh gripper 20      # cerrar

# Ir a una pose guardada
./scripts/run_classification.sh arm home
./scripts/run_classification.sh arm observe
./scripts/run_classification.sh arm pick_table
```

Equivalentes crudos si prefieres escribirlos a mano:

```bash
ros2 service call /mecharm/set_gripper \
  home_service_interfaces/srv/SetGripper '{value: 100, speed_percent: 40.0}'

ros2 action send_goal /mecharm/move_arm \
  home_service_interfaces/action/MoveArm \
  '{pose_name: "observe", speed_percent: 25.0}' --feedback

# Liberar servos por ROS (para enseñar sin cerrar el nodo)
ros2 service call /mecharm/free_move std_srvs/srv/SetBool '{data: true}'
ros2 service call /mecharm/free_move std_srvs/srv/SetBool '{data: false}'

# Ver los ángulos en vivo
ros2 topic echo /mecharm/joint_states
```

---

## 6. Probar el ciclo completo pick → place

Con una pieza colocada en la plataforma:

```bash
ros2 action send_goal /mecharm/pick_place \
  home_service_interfaces/action/PickPlace \
  '{operation: "pick",
    target_pose_name: "pick_table",
    approach_height: 70.0,
    retreat_pose_name: "carry",
    speed_percent: 25.0}' --feedback
```

El feedback pasa por los estados `DESCEND → GRIP → LIFT → RETREAT`.
Para soltarla:

```bash
ros2 action send_goal /mecharm/pick_place \
  home_service_interfaces/action/PickPlace \
  '{operation: "place",
    target_pose_name: "place_table",
    approach_height: 70.0,
    retreat_pose_name: "home",
    speed_percent: 25.0}' --feedback
```

**Qué observar:**
- La aproximación baja recta (los waypoints se calculan en cartesianas).
- La pinza se cierra *después* de llegar, no durante el movimiento.
- Al levantar, la pieza no se desliza (si se desliza, baja
  `gripper_closed_value`, es decir cierra más).

---

## 7. Diagnóstico de fallos

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| `ARM_FAULT: Brazo no conectado` | otro proceso tiene el puerto | `pkill -f mecharm_driver_node` |
| `TIMEOUT` en un movimiento | pose fuera del alcance de 270 mm, u obstrucción | reduce el alcance o re-enseña la pose |
| `El brazo dejo de acercarse al objetivo` | límite articular o choque | revisa los límites en `mecharm.yaml` |
| La pinza no se mueve | `gripper_type` incorrecto o pinza no inicializada | `mecharm> gv` — si devuelve `?`, revisa el conector de la pinza |
| La pieza se cae al levantar | cierre insuficiente | baja `gripper_closed_value` |
| El motor de la pinza zumba | cierre excesivo | sube `gripper_closed_value` |
| Movimientos a tirones | velocidad demasiado alta | `speed 20` |

Parámetros de ajuste fino en `src/myagv_mecharm_service/config/mecharm.yaml`:

```yaml
    angle_tolerance_deg: 2.0     # sube si aborta por TIMEOUT sin motivo
    coord_tolerance_mm: 6.0
    stall_timeout_sec: 4.0       # sube si los movimientos son muy lentos
    max_speed_percent: 60.0      # tope duro de velocidad
```

---

## 8. Checklist antes de la pista

- [ ] `/dev/ttyACM0` visible y `pymycobot` importa
- [ ] `gripper_open_value` y `gripper_closed_value` calibrados con la pieza real
- [ ] Las 5 poses enseñadas y verificadas con `goto`
- [ ] `home` deja el brazo dentro del contorno del robot (crítico: pasillos de 60 cm)
- [ ] `pick_place` completo funciona con una pieza real
- [ ] `poses.yaml` compilado/instalado y el driver reiniciado
