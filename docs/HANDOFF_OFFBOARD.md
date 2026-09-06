# Traspaso — procesamiento externo (portátil)

Estado a 2026-09-05. Léelo antes de tocar nada.

## Reparto

| máquina | qué corre |
|---|---|
| Jetson (contenedor `myagv-robot`, Humble) | cámara CSI, LiDAR, odometría/motores, twist_mux, scan_sanitizer, MechArm, foxglove_bridge |
| Portátil | `aruco_detector`, `aruco_lidar_approach_server`, `object_grasp_server` |

`home_service_bringup` **solo existe dentro del contenedor**. No se lanza
a mano ni en el host de la Jetson (tiene Galactic) ni en el portátil.

## Red

```
Jetson    10.24.15.48   wlan0    ssh puerto 2222
Portátil  10.24.15.54   eth3 (WSL)
ROS_DOMAIN_ID=30 · CycloneDDS · UDP 14900-14980
Foxglove  ws://10.24.15.48:8765  (app en Windows, no en WSL)
```

## Comandos

Portátil:
```bash
WIFI_IFACE=eth3 ROBOT_IP=10.24.15.48 ./scripts/tsummit_offboard.sh check
WIFI_IFACE=eth3 ROBOT_IP=10.24.15.48 ./scripts/tsummit_offboard.sh run
eval "$(WIFI_IFACE=eth3 ROBOT_IP=10.24.15.48 ./scripts/tsummit_offboard.sh env)"

# aproximación a un ArUco — MUEVE EL ROBOT
ALLOW_MOTION=1 WIFI_IFACE=eth3 ROBOT_IP=10.24.15.48 LAPTOP_IP=10.24.15.54 \
    ./scripts/tsummit_offboard.sh approach 4 0.20
```

Lanzar la aproximación **desde el portátil** es lo correcto: el servidor es
local, así que el goal no cruza la red. Solo la cruzan las imágenes (hacia
el portátil) y `/cmd_vel` (hacia el robot).

Jetson (todo exige `DISTRIBUTED=1 WIFI_IFACE=wlan0 ROBOT_IP=... LAPTOP_IP=...`):
```bash
./scripts/run_robot_routine.sh base     # drivers
./scripts/run_robot_routine.sh aruco    # solo cámara en distribuido
./scripts/tsummit.sh viz foxglove
./scripts/tsummit.sh stop               # SIEMPRE así, no pkill a mano
ALLOW_MOTION=1 ./scripts/tsummit.sh approach 4 0.20
```

## Verificado funcionando

- JPEG comprimido 21 Hz por WiFi (246.8 → 6.1 Mbit/s, sin pérdida de detección).
- Detección ArUco 20.8 Hz en el portátil (en la Nano eran 0.13 Hz).
- Cadena TF completa: `odom → base_footprint → base_link → camera_link →
  camera_optical_frame → aruco_N`.
- Cadena de velocidad completa: servidor → `/cmd_vel_aruco` → twist_mux →
  `/cmd_vel` → motores. 182 mensajes medidos en 25 s.
- Watchdog de motores 300 ms.

## Trampas que ya costaron una sesión entera

1. **`ros2 topic list` / `node list` consultan al DEMONIO.** Si arrancó con
   otra config, responde con SU vista y parece que no hay topics. Usa
   `--no-daemon`, o `ros2 daemon stop` antes. **`topic hz`, `topic echo` y
   `action list` NO aceptan `--no-daemon`** (ni lo necesitan).
2. **`tsummit.sh` sin `DISTRIBUTED=1` usa DDS solo por loopback.** Un
   `action send_goal` no alcanza al servidor del portátil y el robot no se
   mueve, sin ningún error.
3. **ArUco sin margen blanco no se detecta.** Medido: el marcador oficial se
   detecta a 65° de inclinación y a 48 px de lado, pero falla si el borde
   negro llega al filo del papel o el soporte es tan oscuro como el borde.
   Hace falta ≥ media celda de margen claro (~1 cm en uno de 8 cm).
4. **Humble solo existe para jammy.** El host de la Jetson es focal: ningún
   `apt install ros-humble-*` funcionará ahí. Todo va en el contenedor.
5. **`home_service_interfaces` debe ser IDÉNTICO en ambas máquinas** o DDS
   calcula otro hash de tipo y los topics dejan de conectar en silencio.
6. **Reiniciar con `pkill` a mano deja `static_transform_publisher`
   huérfanos** peleando por el mismo frame. Usa `tsummit.sh stop`.

## Sin resolver

- **Montaje de la cámara: MEDIDO Y VALIDADO** — `CAMERA_X=0.16 CAMERA_Y=0.0
  CAMERA_Z=0.07 CAMERA_PITCH=0.0`. Se pasa por entorno a
  `run_robot_routine.sh aruco`. `CAMERA_X/Y/Z/PITCH`
  se pasan por entorno a `run_robot_routine.sh aruco`. Origen: `base_link`,
  centro de la base a ras de suelo; X delante, Y izquierda, Z arriba desde
  el suelo; pitch positivo = mirando abajo. Verificación: la Z de
  `tf2_echo odom aruco_N` debe dar la altura real del marcador.
- **Aproximación con movimiento: nunca completada.** La última prueba salió
  TIMEOUT con el robot bloqueado a propósito; toda la cadena respondió pero
  el lazo no puede cerrarse sin movimiento real.
- **`distance: -1.0`** en el feedback: el LiDAR no da distancia al marcador.
  Sin diagnosticar.
- `GOAL_X`/`GOAL_Y` del laberinto caen sobre una pared.
- `table_z_mm` (90.0) y `aruco_to_object` del catálogo de agarre: sin medir.
- `poses.yaml` del brazo: placeholders.

---

## Bucle de alineacion (diagnosticado 2026-09-05)

Sintoma: el robot detecta el ArUco, gira ~45 grados a un lado y entra en
un ciclo de "un poco a la izquierda, un poco a la derecha" sin llegar a
aproximarse.

El servidor no hace 90 grados ni avanza en paralelo. Su secuencia es:
girar hasta quedar PERPENDICULAR al plano del marcador
(`ALIGN_HEADING_TO_ARUCO`) -> desplazarse en LATERAL, que es el tramo
paralelo al marcador (`ALIGNING_LATERAL`) -> avanzar (`APPROACHING`).
Se quedaba encallado en el primer estado, por eso el tramo paralelo no
se llegaba a ver nunca.

Tres causas encadenadas, las tres corregidas:

1. **Normal del marcador a 45 grados.** `get_marker_normal()` usa el eje
   +Z del ArUco. La orientacion de un marcador pequeno visto casi de
   frente sufre ambiguedad planar: la pose salta entre dos soluciones
   simetricas. Promediar muestras de las dos ramas da un rumbo a mitad
   de camino. Corregido con `lock_min_coherence` (0.93): si las muestras
   no coinciden entre si, no se confia en el normal; tras
   `lock_max_attempts` se RENUNCIA a la perpendicularidad y se aproxima
   solo por centrado de camara, que es estable (center_x es un centroide
   en pixeles, no una pose 3D).

2. **`heading_tolerance` 0.02 rad = 1.15 grados, inalcanzable.** En ese
   borde `wz = 1.5 * 0.02 = 0.03 rad/s`, por debajo de la friccion
   estatica de los motores. Ahora 0.12 rad (7 grados) y se anadio
   `min_heading_speed` (0.12 rad/s): cualquier wz no nulo se eleva al
   minimo que de verdad mueve el robot. TUNEAR si el robot se pasa de
   largo o si sigue sin arrancar.

3. **`heading_realign_threshold` 0.05 rad = 2.9 grados.** El
   desplazamiento lateral en mecanum deriva mas que eso, asi que
   `ALIGNING_LATERAL` rebotaba a `ALIGN_HEADING_TO_ARUCO` nada mas
   empezar a moverse. Ahora 0.25 rad.

Ademas, dos numeros de montaje que estaban mal:

- `base_link -> camera_link` se lanzaba con (0.10, 0.0, 0.12). Lo medido
  es (0.16, 0.0, 0.07). Corregido en `robot.launch.py` Y en los defaults
  de `run_robot_routine.sh` (el script pisaba los del launch).
- `camera_x_minus_lidar_x` valia 0.16, que es la X de la camara, no la
  separacion. `laser_frame` esta a x=0.065, asi que la separacion real
  es **0.095**. El error metia 6.5 cm en la distancia de parada.

---

## Segunda ronda (2026-09-05, tras probar en el robot)

Tres observaciones del robot real y lo que se hizo con cada una.

### La busqueda nunca llegaba a detectar

Girando en continuo a 0.22 rad/s el marcador cruzaba el campo de vision
sin dejar un solo fotograma nitido Y quieto: desenfoque de movimiento de
la CSI mas la latencia de la tuberia (JPEG -> WiFi -> portatil). A mano
si se detectaba, porque a mano el marcador se sostiene parado.

Ahora la busqueda es **paso-y-mira**: gira `search_step_sec` (0.45 s) y
se PARA `search_dwell_sec` (0.70 s) a mirar. Un ciclo son ~1.15 s y
cubre ~5.7 grados, asi que un barrido de 360 grados lleva ~72 s. De ahi
que el timeout tuviera que subir.

### El giro de 45 grados persistia

La comprobacion de coherencia no bastaba porque el problema es
GEOMETRICO, no de ruido. Con la camara a 7 cm mirando los marcadores
desde muy abajo, la normal estimada sale casi VERTICAL. Su proyeccion
horizontal -- la unica parte que da rumbo -- es minuscula, y `atan2` de
un vector minusculo amplifica pocos grados de error de pose en decenas
de grados de rumbo. Normalizar en 2D borraba la prueba del delito: una
normal a 5 grados de la vertical produce un vector unitario con toda la
pinta de fiable.

Dos cambios:

- `get_marker_normal()` calcula la fraccion horizontal de la normal
  ANTES de normalizar y descarta la muestra si baja de
  `normal_min_horizontal` (0.5, o sea mas de 60 grados fuera de la
  horizontal).
- **`use_marker_normal` es False por defecto.** El robot ya no intenta
  ponerse perpendicular: se centra en el marcador y avanza. El centrado
  usa `center_x_normalized`, un centroide en pixeles, que es estable.
  Con esto el giro de 45 grados no puede ocurrir: no hay rumbo que
  inventar. Poner a True solo si sube la camara Y hace falta atacar el
  marcador de frente.

### Timeout

`timeout_sec` era 30 s (Jetson) / 40 s (portatil), ni para una vuelta de
busqueda. Ahora es el tercer argumento de `approach` / `aruco-goal`, o
la variable `APPROACH_TIMEOUT`, con **180 s** por defecto.

    ALLOW_MOTION=1 ./scripts/tsummit_offboard.sh approach 4 0.20 240

### Ademas

- Si se pierde el marcador durante `ALIGNING_LATERAL` y no hay rumbo
  fijado, el robot volvia a quedarse clavado hasta el timeout (la unica
  orden que se publicaba era la correccion de rumbo, que vale cero).
  Ahora vuelve a `SEARCHING`.
- `main()` llamaba a `stop_robot()` con el contexto ya invalidado por
  SIGTERM y escupia un RCLError que parecia un fallo y no lo era.

### Recordatorio: DOS sistemas de archivos

El servidor de aproximacion corre en el PORTATIL. Editar
`aruco_lidar_approach_server.py` en la Jetson NO cambia nada de lo que
ejecuta el robot hasta que el bundle se copia y se recompila alli:

    scp -P 2222 er@10.24.15.48:/tmp/tsummit_offboard.tar.gz ~
    cd ~/myagv_home_service_ws && tar xzf ~/tsummit_offboard.tar.gz
    colcon build --symlink-install --packages-select home_service_behaviors

Lo que SI vive en la Jetson: `robot.launch.py` (TF de la camara),
`run_robot_routine.sh`, la camara y el lidar.

---

## Tercera ronda (2026-09-05): el lidar miraba hacia atras

### "Waiting for lidar" para siempre

Dos fallos encadenados, los dos medidos en el robot.

**El 0 del scan es la TRASERA.** `base_link -> laser_frame` tiene
yaw = 180 grados: el YDLidar va montado del reves. El servidor
promediaba el sector `abs(angle) <= 4 grados`, que en el frame del laser
es el culo del robot, tapado por el propio chasis. Medido con el robot
quieto:

    sector    0 +-4 deg:   0 puntos   VACIO      <- lo que se leia
    sector +135 +-4 deg:  11 puntos   0.557 m
    sector +180 +-4 deg:  11 puntos   0.909 m    <- el frente de verdad
    sector  -90 +-4 deg:  10 puntos   1.019 m

Cero puntos validos incluso en `/scan` CRUDO, asi que no era el
sanitizer: es oclusion fisica. Nunca habia distancia, `final_distance`
se quedaba en -1.0 y el estado en `WAITING_LIDAR`. Ese -1.0 que llevaba
sesiones sin explicacion era esto.

**La comparacion angular se rompia en la frontera.** `abs(angle) > half`
no envuelve: +179 y -179 grados son vecinos y la resta cruda los separa
358. Justo donde cae el frente de este robot.

Arreglado con `lidar_front_angle_deg` (999.0 = deducirlo de la TF
`base_link -> laser_frame`, que es lo correcto porque sobrevive a que
alguien remonte el sensor) y diferencia angular con envolvente via
`normalize_angle`. Verificado: el nodo loguea

    Frente del robot en el frame laser_frame: -180.0 grados (deducido de la TF)

y devuelve 0.91 m de forma estable donde antes devolvia None.

### La perpendicularidad vuelve a intentarse

`use_marker_normal` vuelve a **True**. Lo que protege del giro de 45
grados no es desactivarlo, son los dos filtros: `normal_min_horizontal`
descarta normales casi verticales y `lock_min_coherence` descarta lotes
de muestras que no se ponen de acuerdo. Si rechazan el lock, se cae a
centrado + avance en vez de girar hacia un rumbo inventado.

Ojo, un fallo que aparecio al combinar ambas cosas: si el filtro de
verticalidad descartaba TODAS las muestras, `LOCK_TARGET` esperaba para
siempre muestras que no iban a llegar. Ahora agotar la ventana sin
muestras suficientes cuenta como intento fallido.

### Herramienta de analisis

    ./scripts/tsummit_offboard.sh analyze 4 12

Con el marcador delante y `run` en otra terminal, informa de la fraccion
horizontal de la normal, la coherencia entre muestras y el rango de
rumbos, y da un veredicto sobre si la alineacion perpendicular es viable
con esta geometria. No mueve el robot. **Ejecutalo antes de dar por
buena o mala la perpendicularidad**: distingue "normal casi vertical"
(sube la camara) de "ambiguedad planar" (marcador mas grande o mas
cerca), que piden arreglos distintos.

### Busqueda mas rapida

`search_angular_speed` 0.22 -> 0.35 rad/s. Sigue siendo paso-y-mira, asi
que cada paso cubre ahora ~9 grados en vez de ~5.7 y un barrido de 360
grados baja de ~72 s a ~46 s.

---

## Cuarta ronda (2026-09-05): la normal la da el LIDAR, no el ArUco

### El cambio de fondo

Se estaba pidiendo al ArUco algo para lo que no sirve. La normal sacada
de su POSE tiene dos problemas insalvables con esta geometria:
ambiguedad planar, y con la camara a 7 cm sale casi vertical, asi que su
proyeccion horizontal -- la unica que da rumbo -- es ruido amplificado.
De ahi el giro de 45 grados, y de ahi que al fallar el lock el robot se
"alineara donde encontro el marcador".

El lidar no tiene ninguno de los dos problemas. El marcador esta pegado
a una superficie plana, el lidar VE esa superficie, y una recta ajustada
a esos puntos da la orientacion del plano directamente en horizontal y
en metros de verdad.

**Reparto nuevo: el ArUco dice CUAL es el objetivo y EN QUE DIRECCION
esta; el lidar da la geometria.**

`get_lidar_surface_normal()` toma el rumbo al marcador de la TF
`base_link -> aruco_N` (solo la direccion: la distancia del ArUco
depende de que marker_size sea correcto, la direccion no), recoge los
puntos del scan en un sector de +-30 grados alrededor, se queda con la
superficie MAS CERCANA (banda de 0.30 m, para no coger la pared del
fondo) y ajusta una recta por componentes principales. Rechaza el
resultado si el residuo pasa de 2 cm o si la nube no tiene extension.

Medido en el robot contra una superficie real, seis muestras seguidas:

    rumbo perpendicular = -45.5, -45.1, -45.0, -47.2, -44.3, -46.1 deg

Dispersion de 2.9 grados. La pose del ArUco producia errores de 45.

La normal del ArUco sigue ahi como respaldo, solo si el lidar no da
nada (`use_lidar_normal:=false` la fuerza).

### La secuencia que resulta

1. `SEARCHING` paso-y-mira hasta ver el marcador.
2. `LOCK_TARGET` fija la normal de la superficie con el lidar.
3. `ALIGN_HEADING_TO_ARUCO` gira hasta quedar PERPENDICULAR al plano.
4. `ALIGNING_LATERAL` desplaza en vy. Como el rumbo ya esta sobre la
   normal, ese movimiento es PARALELO al marcador, y centrar el
   marcador en la imagen equivale a ponerse sobre su eje normal.
5. `APPROACHING` avanza con distancia de lidar hasta `stop_distance`.

### WAITING_LIDAR ya dice POR QUE

Era una caja negra: no distinguia "el scan no llega" de "llega pero el
sector que miro esta vacio", que piden arreglos opuestos. Ahora el
feedback trae el motivo:

    WAITING_LIDAR:SIN_SCAN                    -> /scan_filtered no llega
    WAITING_LIDAR:SCAN_VIEJO(1.42s)           -> llega tarde o a saltos
    WAITING_LIDAR:SECTOR_VACIO(frente=-180deg) -> nada que medir ahi

Si sale `SECTOR_VACIO`, el robot no tiene nada plano delante en +-6
grados. Si sale `SIN_SCAN` en el portatil pero el topic existe en la
Jetson, es descubrimiento DDS, no el lidar.

### Marcador perdido durante el centrado

Con rumbo fijado, perder el marcador dejaba al robot publicando solo la
correccion de rumbo hasta el timeout. Ahora aguanta
`lost_marker_timeout` (3 s, porque al girar hacia la perpendicular el
marcador se sale del encuadre un momento y vuelve) y despues vuelve a
`SEARCHING`.

---

## Quinta ronda (2026-09-05): la normal del lidar iba en el marco equivocado

Sintoma: al detectar el marcador el robot entraba en
`ALIGN_HEADING_TO_ARUCO`, giraba ~90 grados A LA DERECHA, en direccion
CONTRARIA al ArUco, pasaba a `ALIGNING_LATERAL`, se movia a tirones de
izquierda a derecha, perdia el marcador y volvia a buscar. En bucle.

**Causa: mezcla de marcos, introducida al meter la normal por lidar.**

`get_marker_normal()` (pose del ArUco) devolvia la normal en **odom**,
que es lo que `heading_control` necesita, porque compara contra el yaw
del robot en odom. `get_lidar_surface_normal()` la devolvia en
**base_link**, porque el calculo natural es en el cuerpo. Al sustituir
una fuente por la otra, el rumbo de cuerpo se trataba como rumbo de
mundo.

El error de rumbo resultante es exactamente **el yaw acumulado del
robot**, y de signo contrario. Medido en el robot:

    normal en base_link = -61.4 deg   yaw en odom = -21.2 deg
    -> rumbo correcto    = -82.6 deg
    -> error aplicado    = +21.2 deg   (= -yaw, en direccion contraria)

Por eso giraba al reves, y por eso giraba MAS cuanto mas hubiera girado
buscando el marcador: tras varios pasos de busqueda el yaw acumulado
llega a los 90 grados que se observaron.

Arreglado rotando la normal a odom antes de devolverla. Verificado con
la superficie quieta: rumbo en odom constante en -82.5 deg y error de
rumbo -61.4 deg, que es justo la normal en base_link, como debe ser.

### Tirones de izquierda a derecha en el centrado

Ademas del rumbo equivocado, el centrado lateral tenia la misma zona
muerta que ya se corrigio en el rumbo. Desplazarse de lado en mecanum
exige MAS par que girar: las cuatro ruedas empujan en diagonal y la
friccion transversal de los rodillos se suma. Con `kp_lateral` 0.08 y
errores pequenos, vy salia de 0.01 m/s: se publicaba y no movia nada,
hasta que la friccion cedia de golpe. Anadido `min_lateral_speed`
(0.035 m/s) y subido `max_lateral_speed` de 0.04 a 0.06.

### Busqueda

`search_angular_speed` 0.35 -> 0.45 rad/s.

### Nota de metodo

Estas dos rondas dejan una leccion barata de aprender aqui: **al cambiar
la FUENTE de una magnitud geometrica, comprobar el MARCO en el que la
espera quien la consume**. El fallo no daba ningun error, solo un robot
girando hacia el lado contrario.

---

## Sexta ronda (2026-09-05): el bucle de las 11 alineaciones

Sintoma: 11 veces seguidas el mismo patron, sin llegar nunca a
`APPROACHING`:

    Target found. Locking marker normal.
    Normal fijada por LIDAR (coherencia 1.00). Rumbo perpendicular = -82.5 deg
    Heading aligned with marker normal.
    [WARN] Marcador perdido 3.0 s. Volviendo a buscar.

### Lo que NO era

Se propuso que `get_lidar_surface_normal()` devolvia la TANGENTE en vez
de la normal (le faltaria un giro de 90 grados, o habria un x/y
intercambiado). **Comprobado y descartado.** Contra paredes SINTETICAS
de orientacion conocida, error 0.0 grados en las cinco:

    normal REAL | devuelto | error
       0.0 deg  |   0.0deg | +0.0
      20.0 deg  |  20.0deg | -0.0
     -20.0 deg  | -20.0deg | +0.0
      45.0 deg  |  45.0deg | +0.0
     -45.0 deg  | -45.0deg | -0.0

`np.linalg.svd` devuelve `vh`, cuyas FILAS son los vectores singulares
por la derecha: `vectors[0]` es el eje mayor (la direccion de la recta)
y `vectors[1]` el eje menor (la normal). El codigo usaba el correcto.

### Lo que SI era: dos fallos, ninguno en el ajuste

**1. La normal fijada no era la del marcador.** Un ajuste con coherencia
1.00 y residuo de milimetros solo dice que la nube ES una recta; no dice
que sea la recta correcta. Con un sector de +-30 grados, el ajuste puede
coger una pared lateral o el canto de un mueble. Una normal a 82 grados
de la linea de vision es imposible para una superficie que el robot esta
VIENDO: a esa oblicuidad el ArUco no se detectaria.

Anadida la guarda `normal_max_obliquity_deg` (60 grados). Verificada:
-82.5 y -90.0 se rechazan, 0.0 y -30.0 se aceptan. Las 11 iteraciones
del log habrian sido rechazadas en la primera.

**2. Girar a la normal pierde el marcador POR GEOMETRIA.** Este es el
fallo de diseno de fondo, y era mio. Si el robot no esta ya sobre el eje
normal, encararse a la normal aparta la camara del marcador exactamente
el angulo que le falta para estar en el eje. Con 90 grados de desfase el
marcador sale del encuadre, y el unico camino de vuelta era `SEARCHING`.
El bucle no era mala suerte: era inevitable.

Reordenado el maniobrar:

  * `ALIGN_HEADING_TO_ARUCO` encara al MARCADOR (no a la normal), que lo
    mantiene a la vista.
  * `ALIGNING_LATERAL` RODEA: con el robot encarado al marcador, un vy
    puro lo desplaza tangencialmente, en arco paralelo al plano del
    marcador, mientras el giro lo mantiene centrado. El error que se
    anula es el angulo entre la linea de vision y la normal fijada
    (`axis_error`), que vale cero exactamente sobre el eje normal.
  * `APPROACHING` avanza de frente, que ya es la perpendicular, con la
    distancia del lidar y siguiendo al marcador con el giro.

La normal ya no se usa como rumbo al que girar, sino como referencia de
DONDE colocarse. Signos verificados: robot desplazado a la izquierda del
eje -> vy a la derecha, y al reves.

### Builds desincronizados

    [WARN] Failed to get parameters:
           ('Invalid access to undeclared parameter(s)', 'min_lateral_speed')

El nodo que corria venia de un fuente sin ese parametro. **Recompilar en
el portatil ANTES de fiarse de cualquier prueba.** El servidor corre
alli; lo que se edite en la Jetson no cambia nada hasta que se copia el
bundle y se compila.
