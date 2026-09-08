# Checkpoint — aproximación ArUco + LiDAR funcionando

Fecha: 2026-09-07. Rama `feature/real-hardware-infra`, HEAD `99f9c42`.
Validado en pista por el operador: la base llega y se alinea al ArUco 2
de forma repetible, incluida una escena oblicua (marcador casi en el
borde del cuadro, ~1.09 m, entrada perpendicular a la normal).

## Qué quedó resuelto

Cómputo distribuido: los **drivers** (odom, YDLIDAR, cámara CSI,
`mecharm_driver_node`) corren en la Jetson; la **detección ArUco** y el
**servidor de aproximación** corren en el portátil. Enlace por Tailscale
(Jetson `100.86.172.41`, portátil `100.91.114.36`), CycloneDDS por IP
con `AllowMulticast=false` y peers unicast. `ROS_DOMAIN_ID=30`.

### 1. Detección ArUco a ritmo fijo (`9fc25ed`, `db8c444`)

El callback de imagen solo guarda el fotograma; un timer procesa a
`process_hz` fijo (15 Hz portátil, 8 Hz Nano) con `MultiThreadedExecutor`.

- Verificado: **15 Hz plano**, 15-21 ms/frame contra cámara real, 76/76
  detecciones, sin una sola ráfaga en dos corridas.
- A 15 Hz eso es ~1/4 de un núcleo de los 12 del portátil.

### 2. Frenada sin punto muerto (`ffc4113`, `a9ad8b2`, `99f9c42`)

`brake_target` calculaba el "coast" realimentando la velocidad anterior
y entraba en **ciclo límite**: mandaba cero, coast cero, mandaba, cero…
El **52 % de los mensajes de `/cmd_vel_aruco` eran cero exacto**. En
crucero el robot rodaba sobre los ceros por inercia; en el endgame, sin
momento y con la zona muerta de la base, se quedaba clavado ~3 cm corto
(STALLED a 0.231 pidiendo 0.20).

Arreglo final: `brake_target` resuelve el **punto fijo** de la rampa
compensada

    v = -a·T + sqrt(a²·T² + 2·a·d)

función monótona de la distancia, sin velocidad de entrada. Cero
oscilación. En el endgame la velocidad la fija solo el LiDAR (no el
`remaining` del camino, que plantaba el robot antes de tiempo por la
geometría de la cámara).

### 3. Una sola medida de distancia (`ce4ab66`)

En el endgame (<0.60 m) frenar y declarar llegada usan las dos el eco
más cercano del sector, no la mediana. Antes se frenaba con uno y se
llegaba con otro.

## Parámetros calibrados (medidos, no supuestos)

| Parámetro | Valor | Cómo se obtuvo |
|---|---|---|
| `command_latency` | **0.27 s** | bag en la Jetson: primer `cmd vx>0.005` (t=1.427) → primer `odom vx>0.01` (t=1.650) = 0.223 s (actuación + encoder, sin red de vuelta). + Tailscale portátil→Jetson ≈ 0.05. |
| coast-down desde 0.19 m/s | ~0.35-0.40 s | transición del rodeo: `cmd vx` 0.180→0 en t=1.831, `odom vx` 0.190→0.010 hacia t=2.21. |
| `distance_tolerance` | **0.045 m** | con `v_min` 0.07 y latencia 0.27 la parada mínima física es ~0.031 m; 0.03 estaba por debajo del límite. El brazo corrige el resto vía `target_coords_stop_m`. |
| `process_hz` detección | 15 (portátil) / 8 (Nano) | medido: 15 Hz estable, 16 ms/frame. |

`command_latency` **angular** sigue sin medir: en el bag de la corrida
oblicua el primer `wz` salió clipado (el servidor ya publicaba cuando
arrancó la grabación). Se saca en la próxima corrida si el bag arranca
> 5 s antes del goal.

## Cómo reproducir la medida de latencia

```bash
# en la Jetson, dentro del contenedor, con CYCLONEDDS_URI por IP:
ros2 bag record -o /workspace/log/approach_lat_<ts> /cmd_vel_aruco /odom
# lanzar approach 2 0.20 desde el portátil
# analizar: primer cmd linear.x>0 (T0) vs primer odom twist.linear.x>0.01 (T1)
```

## Siguiente

Calibración de agarre por pieza y por altura de plataforma (10 cm /
20 cm) para los 4 objetos. Ver `docs/HANDOFF_POSTE.md` y
`scripts/calibrate_grasp.py`.
