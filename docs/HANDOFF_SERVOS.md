# Handoff — verificar la integridad de los servos del MechArm 270

Documento para una sesion NUEVA. El brazo esta en proteccion por
sobrecarga y enfriandose. Esta sesion NO debe mover el brazo.

## 1. Estado en el momento del handoff

```
brazo    plegado, angulos [3.69, -39.02, 6.41, -1.05, 61.96, 5.18]
         postura segura, pero el driver no la da por buena
base     parada a 0.270 m del ArUco 2
pieza    el poste esta RETIRADO del escenario
driver   mecharm_driver_node reiniciado e idle
```

## 2. El fallo, leido del firmware

```
read_next_error      [0, [0,1], 0, 0, 0, 0, 0]
                     Joint2 - Emergency stop pressed
                     Joint2 - Communication problem
get_servo_status     [0, [5], 0, 0, 0, 0]      codigo 5 = over-load
get_servo_temps      [33, 40, 58, 33, 40, 33]  J3 a 58 C
get_servo_voltages   [7.3, 7.3, 7.2, 7.5, 7.5, 7.3]   nominal
is_all_servo_enable  1
is_power_on          1
```

`clear_error_information()` devolvio 1 (exito) y **el fallo no se fue**:
las mismas lecturas despues, y J3 subio a 60 C estando parado.

O sea: proteccion dura del servo 2 (hombro). No se recupera por
software. Hace falta apagar y encender el brazo.

## 3. Que lo provoco

Se encadenaron ~12 comandos de movimiento buscando por que fallaba una
pose, varios con el brazo muy extendido (X = 222 mm, casi todo el
alcance de 270, con la pinza montada). Ahi el par de sostener sobre J2
y J3 es maximo.

La degradacion se veia en los datos y no se leyo a tiempo:

```
safe_navigation al principio   0.79 grados de desviacion, OK
safe_navigation al final       4.66 grados, TIMEOUT
hundimiento de J3              -4.2  ->  -18.6  ->  -33 grados
un goal de J2 = +40 acabo en J2 = -38.9
```

Todo eso se interpreto como fallos de cinematica inversa, de
`send_coords`, de convenio de orientacion y de limites articulares. Era
el servo protegiendose. **La leccion operativa: cuando la precision
repetible se degrada, leer el hardware antes de seguir probando
teorias de software.**

## 4. Lo que hay que verificar en esta sesion

Despues de que el operador APAGUE Y ENCIENDA el brazo:

```
get_servo_status     los seis a 0
read_next_error      sin entradas de Joint2
get_servo_temps      J3 por debajo de 45 C, idealmente ~35
get_servo_voltages   los seis entre 7.0 y 7.6
is_servo_enable 1..6 todos a 1
```

Criterio: **los seis codigos a cero y J3 por debajo de 45 C.** Si el
codigo 5 persiste tras el ciclo de alimentacion, el servo puede estar
dañado y hay que decirlo claramente en vez de seguir.

Prueba de movimiento, SOLO si lo anterior pasa, y en este orden:

1. `move_arm pose_name safe_navigation` a velocidad 20.
   Debe quedar a menos de 1 grado. Al principio de la noche daba 0.79.
   Si da 4 grados, el brazo sigue tocado.
2. Releer temperaturas. Si J3 sube mas de 5 C con un solo movimiento
   pequeño, parar.

**No probar poses extendidas.** Nada por encima de X = 180 mm hasta que
se decida como agarrar el poste mas recogido.

## 5. Como conectarse

Computo distribuido: los drivers estan en la Jetson, el detector y el
control en el portatil.

```
ROS_DOMAIN_ID=30
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
Jetson    100.86.172.41   (Tailscale)
portatil  100.91.114.36   (Tailscale)
```

Trampas que ya costaron tiempo esta noche:

- Una terminal recien abierta trae `ROS_DOMAIN_ID=5` del `~/.bashrc` y
  no ve nada. Cargar el entorno con
  `eval "$(ROBOT_IP=... LAPTOP_IP=... ./scripts/tsummit_offboard.sh env)"`.
- `ros2 topic list` muestra un topic aunque nadie publique: basta un
  suscriptor. Mirar `Publisher count`.
- La consola `mecharm_console.py` y `mecharm_driver_node` no pueden
  correr a la vez: `/dev/ttyACM0` es exclusivo.
- Los limites de junta del `mecharm.yaml` eran incorrectos. Los buenos,
  leidos del firmware:
  `min [-160,-75,-175,-155,-115,-180]  max [160,120,65,155,115,180]`

## 6. Lo que NO hay que hacer

- No mover el brazo antes del ciclo de alimentacion.
- No usar `clear_error_information` como si arreglara algo: ya se probo
  y no quita el overload.
- No probar la pose enseñada del poste
  `[222.30, 21.20, 25.60, -15.26, 83.99, -14.38]`. Es la que provoco
  esto. Hay que re-enseñarla con la base mas cerca, y esa vez grabar
  tambien `get_angles`.
