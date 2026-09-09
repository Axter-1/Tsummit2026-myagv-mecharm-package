# ALIGN_PERPENDICULAR — alineación previa a la aproximación

Etapa nueva entre `SEARCHING` y la aproximación. Estado del árbol:
sin verificar en pista todavía (ver *Validación*).

```
SEARCHING ─→ ALIGN_PERPENDICULAR ─→ APPROACH (PURSUING) ─→ REACHED
                    ↑    │                   │
                    │    └── REACQUIRING ────┘
                    └──── realineación con histéresis ────┘
```

## Por qué, si `build_path` ya llegaba perpendicular

El camino de dos tramos (robot → encare → parada) deja la llegada
perpendicular **por construcción**, y eso sigue siendo cierto. Lo que no
da es perpendicularidad **al empezar**: el primer tramo se recorre
mirando al marcador, porque `desired_heading()` mezcla rumbo y normal con
`remaining` para no perderlo de vista. El robot ataca el pasillo oblicuo
y toda la corrección de encare se paga al final, ya cerca, donde el
margen de maniobra y el encuadre son mínimos.

Alinear antes cuesta unos segundos parado y quita ese pago tardío.

**No es la vieja máquina `ALIGN_HEADING → ALIGNING_LATERAL`.** Aquella
corregía un grado de libertad cada vez contra el error instantáneo de
cámara, y en mecanum eso se persigue la cola. Aquí los dos lazos trabajan
contra la estimación fijada en `odom`, y la traslación sale siempre como
**vector** (`vx` y `vy` juntos, en diagonal si hace falta).

## Restricción de hardware que condiciona el diseño

La placa del myAGV se queda a **cero absoluto** cuando los tres ejes son
no nulos. Medido sobre `/odom`, ventanas de 2 s (tabla completa en
`holonomic_command`):

```
avance + giro      0.302 m / 0.273 rad
lateral + giro     0.217 m / 0.277 rad
avance + lateral   0.271 m
LOS TRES           0.000 m / 0.000 rad   <-- ni un mm, a cualquier magnitud
```

Así que la etapa **no puede** mandar `vx`, `vy` y `wz` a la vez. Lo que
hace es multiplexar por ciclo con la histéresis como puerta: fuera de
banda angular el ciclo es **giro puro**; dentro, **traslación plena** con
los dos ejes. La simultaneidad que sí se conserva —y es la que evita
perseguirse la cola— es la de `vx` con `vy`.

`test_alineacion_nunca_manda_los_tres_ejes` barre 200 poses y lo verifica.

## Los tres errores son distintos

| error | qué mide | cero cuando |
|---|---|---|
| `center_x_normalized` | centrado en **imagen** | el robot **apunta** al marcador |
| `yaw_error` | **perpendicularidad** al plano | el eje óptico es paralelo a la normal |
| `lateral` | **centrado sobre el eje** normal | el robot está en la recta de la normal |

Un robot a 45° del plano puede tener el ArUco perfectamente centrado en
el encuadre. `perpendicular_errors()` calcula los tres últimos por
separado; `test_perpendicularidad_y_centrado_son_errores_distintos` fija
ese caso.

## Última pose confiable

`TargetEstimate` ya fijaba el marcador en `odom` con filtro exponencial y
rechazo de atípicos. Lo que se añade:

- **`TargetEstimate.quality`** ∈ [0, 1] = *madurez* (muestras fusionadas,
  saturando en `gate_after`) × *racha* (penaliza rechazos consecutivos).
- **`ReliablePose`** (`estimate.snapshot()`): pose + `frame='odom'` +
  `stamp_ns` + `samples` + `rejected` + `quality`, con
  `is_usable(edad, calidad_min, edad_max)`.
- El servidor **solo refresca** `reliable` mientras
  `quality >= align_min_quality`.

Ese último punto es el que importa: el modo de fallo típico no es que el
marcador desaparezca de golpe, sino que la pose se degrada unos ciclos
(marcador de perfil, desenfoque de movimiento) y **después** desaparece.
Guardando "la última" se guardaría justo la lectura aberrante.
`test_la_calidad_se_hunde_con_una_racha_de_rechazos` fija ese caso: pose
reciente, calidad 0.33, declarada no usable.

**El marco es `odom`, y por eso no hace falta "actualizar" la pose con la
odometría.** No se guarda un `tvec` de cámara —que caduca en cuanto la
base se desplaza— sino un punto fijo del mundo. Es la cadena TF
`odom → base_link` la que actualiza la relación robot↔marcador ciclo a
ciclo, sin tocar el valor almacenado.

## Pérdida visual durante la alineación

La referencia se juzga por **tres** cosas, no por una:

```
pose_usable =  quality      >= align_min_quality
           and edad_pose    <= align_max_pose_age
           and recorrido    <= align_max_blind_travel   (METROS, no segundos)
```

El límite en metros y no en segundos es deliberado: la deriva de
odometría crece con la distancia recorrida, y un robot parado esperando
no deriva nada.

Si falla alguna → `REACQUIRING`: **giro puro** hacia la posición
*recordada* en `odom` (`reacquire_heading()`, que apunta al marcador, no
a la normal — para recuperar la visión hay que apuntar). Rotar no cambia
la posición, así que no acumula deriva de traslación.

Agotados `align_max_attempts` o `align_recovery_timeout_sec`, se
**descarta la estimación** y se vuelve a `SEARCHING`. Nunca se sigue
corrigiendo contra una pose que ha dejado de describir el mundo.

## Transición y anti-pinpón

**Entrega a APPROACH**: al completar, se fija el rumbo perpendicular
como referencia ya asentada (`yaw_settled=True`) y se abre una ventana
`align_handoff_grace_sec` en la que el bloque de centrado por cámara
**no puede** reevaluar `yaw_settled`. Sin eso, APPROACH lo recalcula con
`center_x` en el primer ciclo y deshace el encare con un giro en seco,
que es justo lo que la etapa venía a quitar.

**Vuelta a ALIGN**: tres guardas.

1. *Histéresis*: `realign_yaw_threshold` (0.35) > `align_yaw_tolerance` ×
   `align_yaw_hysteresis` (0.13 × 1.6 = 0.208). Salir de la alineación no
   puede disparar la vuelta. Verificado en
   `test_la_histeresis_angular_no_dispara_la_vuelta_a_alinear`.
2. *Persistencia*: el error debe mantenerse `realign_persist_sec`. Un
   pico de un ciclo es ruido del estimador.
3. *Tope duro*: `max_realign_cycles`. Un bucle estable de dos etapas es
   peor que una aproximación mediocre.

No se realinea en el endgame (`along <= align_min_distance`) ni con la
referencia angular congelada (`angular_frozen`).

## Parámetros

Todos en `offboard.launch.py` (el servidor corre en el portátil) y con
default idéntico en el nodo.

| parámetro | def. | qué es |
|---|---|---|
| `align_enabled` | `true` | `false` = comportamiento anterior, para comparar |
| `align_yaw_tolerance` | 0.13 rad | perpendicularidad. Suelo alcanzable: 0.37 × (0.27 + 0.05) = 0.118 rad |
| `align_yaw_hysteresis` | 1.6 | umbral de salida = tol × esto |
| `align_lateral_tolerance` | 0.05 m | centrado sobre el eje normal |
| `align_lateral_hysteresis` | 1.6 | |
| `align_regulate_distance` | `true` | si la etapa coloca también en el punto de encare |
| `align_standoff_tolerance` | 0.10 m | banda ancha, para no pelearse con el frenado de APPROACH |
| `align_kp_angular` / `_linear` / `_lateral` | 1.2 / 0.6 / 0.9 | ganancias |
| `align_max_angular_speed` | 0.45 rad/s | |
| `align_max_linear_speed` | 0.09 m/s | |
| `align_max_lateral_speed` | 0.10 m/s | |
| `align_settle_sec` | 0.35 s | estabilidad exigida antes de dar por buena la alineación |
| `align_timeout_sec` | 25.0 s | presupuesto; agotado se pasa a APPROACH con lo que haya |
| `align_handoff_grace_sec` | 1.5 s | ventana en la que APPROACH no rehace el rumbo |
| `align_min_distance` | 0.30 m | por debajo no se alinea: lo termina el endgame |
| `align_max_pose_age` | 2.0 s | edad máxima para corregir sin ver |
| `align_max_blind_travel` | 0.15 m | metros a ciegas |
| `align_min_quality` | 0.5 | calidad mínima para fiarse |
| `align_recovery_angular_speed` | 0.40 rad/s | |
| `align_recovery_timeout_sec` | 6.0 s | por intento |
| `align_max_attempts` | 3 | intentos antes de volver a SEARCHING |
| `realign_yaw_threshold` | 0.35 rad | vuelta a ALIGN desde APPROACH |
| `realign_persist_sec` | 0.6 s | persistencia exigida |
| `max_realign_cycles` | 2 | tope de vueltas |
| `align_log_period` | 0.5 s | throttle del diagnóstico; 0.0 lo apaga |

`check_tolerances()` verifica al arrancar que `align_yaw_tolerance` y
`align_lateral_tolerance` sean alcanzables con el suelo de velocidad de
la base, y grita si no.

## Diagnóstico

Una línea por volcado, con throttle (`align_log_period`), en ALIGN,
REACQUIRING y APPROACH:

```
[ALIGN_PERPENDICULAR/YAW] aruco_actual: centro=-0.043 z=0.812 m |
ultima_pose_fiable(odom): (+1.204, +0.318) n=(-0.99, +0.11) q=1.00 n_muestras=37 |
edad_deteccion=0.05 s recorrido_ciego=0.000 m |
err_angular=-11.4 deg err_lateral=+0.082 m perpendicular=0.812 m |
cmd=(+0.000, +0.000, -0.370) | lidar=0.798 m | perdida=- | ultima_transicion=-
```

Cubre lo pedido: estado, pose actual, última pose fiable, edad, error
angular, error lateral, `vx/vy/wz`, distancia LiDAR, motivo de pérdida y
motivo de transición.

## Validación

Hecho (2026-09-09, en la Jetson):

- `test_approach_planner.py`: **60/60** (48 previos + 12 nuevos).
  Geometría, signos de movimiento, invariante de tres ejes, convergencia
  del lazo simulado con la zona muerta real, y la semántica de
  `quality`/`ReliablePose`.
- `pyflakes` limpio; `flake8` sin regresiones (los 2 avisos que quedan
  son previos, verificado contra `HEAD`).
- Comprobación estática: los 26 parámetros nuevos están declarados y
  usados; ningún `LaunchConfiguration` sin `DeclareLaunchArgument`.

**Pendiente y sin sustituto:**

- Instanciar el nodo (`ArucoLidarApproachServer()`) para validar las
  declaraciones contra rclpy. **No se puede hacer en la Jetson**: aquí
  hay ROS galactic / Python 3.8 y `install/` está compilado para
  humble / Python 3.10 (portátil). Hay que hacerlo en el portátil.
- Pista con ArUco estático, incluyendo tapar el marcador a mano para
  provocar `REACQUIRING`.
- Corrida oblicua de 1 m comparando `align_enabled:=true` contra `false`.

## Comandos

**Portátil** (levantar la pila; el servidor de aproximación va aquí):

```bash
cd ~/myagv_home_service_ws
colcon build --packages-select home_service_behaviors --symlink-install
source install/setup.bash

ROBOT_IP=<jetson> LAPTOP_IP=<portatil> ./scripts/tsummit_offboard.sh run
```

**Solo la alineación** (sin dejar que llegue a agarrar): pedir una
parada lejana, de modo que APPROACH no tenga casi nada que hacer después
de alinear.

```bash
ALLOW_MOTION=1 ROBOT_IP=<jetson> LAPTOP_IP=<portatil> \
    ./scripts/tsummit_offboard.sh approach 4 0.45 60
```

Mirar en el log del servidor las líneas `[ALIGN_PERPENDICULAR/...]` y la
de cierre `ALIGN_PERPENDICULAR completada (...)`.

**Comparación A/B** contra el comportamiento anterior:

```bash
ros2 param set /aruco_lidar_approach_server align_enabled false   # antes
ros2 param set /aruco_lidar_approach_server align_enabled true    # ahora
```

**Aproximación completa** (alineación + aproximación, sin brazo):

```bash
ALLOW_MOTION=1 ROBOT_IP=<jetson> LAPTOP_IP=<portatil> \
    ./scripts/tsummit_offboard.sh approach 4 0.20
```

**Pick completo — solo cuando la aproximación esté validada en pista.**
No se toca la infraestructura desde `grasp`: la pila ya está levantada
por `run`, y las calibraciones de altura siguen en
`src/home_service_behaviors/config/grasp_calibrations.yaml`.

```bash
export GEMINI_API_KEY=...            # o usa --no-ai
python3 scripts/smart_pick.py auto 100
```
