# Parches sobre paquetes externos (`vcs import`)

`src/elephant_myagv_ros2/` **no forma parte de este repo**: está en
`.gitignore` y se restaura con `vcs import` desde `dependencies.repos`
(lo hace solo `docker/robot-entrypoint.sh` la primera vez).

Eso significa que **cualquier cambio hecho ahí se pierde** al reclonar o
al reimportar. Por eso los cambios locales viven aquí como parche.

## `myagv_odometry-local-changes.patch`

Lo importante que contiene es el **watchdog de `/cmd_vel`**.

### Por qué existe

`myAGVSub.cpp` guardaba el último `/cmd_vel` recibido en unas variables
atómicas y lo reenviaba a los motores a 100 Hz **indefinidamente**:

```cpp
void cmdCallback(msg) { linearX.store(msg->linear.x); ... }   // solo guarda

while (rclcpp::ok()) {
    node->sendCommand(linearX.load(), ...);   // reenvia a 100 Hz
    command_rate.sleep();                      // ...para siempre
}
```

Si el emisor callaba —nodo muerto, `twist_mux` con todas sus entradas en
timeout, o el enlace WiFi caído cuando el procesamiento corre en un
portátil externo— **el robot seguía rodando a la última velocidad
ordenada**. Un AMR no puede depender de que alguien siga hablando para
frenar.

El parche añade un watchdog de **300 ms**: si no llega `/cmd_vel`, manda
ceros a los motores. Verificado en el robot:

```
[WARN] [myagv_odometry_node]: WATCHDOG: sin /cmd_vel en 300 ms. Motores a cero.
```

300 ms es deliberado: por encima del timeout de 0.5 s de `twist_mux`
sería inútil, y por debajo de ~0.2 s un hipo normal de red frenaría el
robot a tirones. Los emisores publican a >= 10 Hz, así que son 3
mensajes perdidos seguidos.

### `restoreRun()` sin terminal: el bucle de "press enter"

`readSpeed()` llama a `restoreRun()` cuando la placa manda un frame de
fallo de rueda (sobre-corriente), o cuando el serie mete basura que da
la casualidad de cumplir el checksum de 5 bytes de ese frame. El código
original de ahí pedía por `stdin`:

```cpp
std::cout << "if you want restore run,pls input 1,then press enter";
while (res != 1) { std::cin >> res; std::cout << "press enter"; }
```

El nodo corre **siempre headless**. `std::cin` da EOF al instante, el
stream se queda en `failbit`, y a partir de ahí cada `std::cin >> res`
vuelve sin bloquear: el `while` gira a millones de vueltas por segundo
escribiendo `press enter`. **Un núcleo entero al 100 % y ~450 MB de log
en pocos minutos.** Pasó tres veces en un solo día de pruebas.

El parche: si no hay TTY (`isatty(fileno(stdin))`), auto-recupera
(`restore()`) con un techo de 1 Hz para que un serie ruidoso que falla
en bucle no vuelva a quemar CPU ni disco — ahora acotado y con un
`RCLCPP_WARN` visible. El camino interactivo (por si alguien lo arranca
desde una terminal) queda también guardado contra EOF.

Log esperado cuando la placa falla:

```
[WARN] [myagv_odometry_node]: Placa en fallo; auto-restore (sin terminal interactiva).
```

### Cómo aplicarlo tras un `vcs import` limpio

```bash
cd src/elephant_myagv_ros2
git apply ../../patches/myagv_odometry-local-changes.patch
cd ../..
# recompilar (es C++, no basta con el symlink-install)
docker exec myagv-robot bash -lc \
  'source /opt/ros/humble/setup.bash; cd /workspace; \
   MAKEFLAGS="-j2" colcon build --packages-select myagv_odometry --symlink-install'
```

Comprobar que está activo:

```bash
# publica cmd_vel un momento y para; el watchdog debe saltar
docker exec myagv-robot bash -lc \
  'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash;
   timeout 3 ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.01}}"'
docker exec myagv-robot grep WATCHDOG /workspace/log/robot_routine/base.log
```

### Regenerar el parche

Si vuelves a tocar `myagv_odometry`:

```bash
(cd src/elephant_myagv_ros2 && git diff myagv_odometry/) \
    > patches/myagv_odometry-local-changes.patch
```
