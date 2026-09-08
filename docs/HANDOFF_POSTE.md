# Handoff — re-enseñar el agarre del poste

Para una sesion nueva. Lee la seccion 2 antes que ninguna otra: hay
cuatro numeros del contexto que circula que estan desactualizados, y uno
que es una contradiccion sin resolver.

## 1. Objetivo

Dejar `grasp poste` funcionando con la pose enseñada nueva, con el brazo
recogido para no repetir la sobrecarga del servo del hombro.

## 2. AVISOS: lo que hay que verificar antes de fiarse del contexto

**Los cinco commits `a9c3195..41a8029` NO son ajenos.** Los hizo la
sesion del portatil, estan probados y son la fuente de verdad actual. El
contexto que circula los describe como "cambios concurrentes a auditar";
audita si quieres, pero no los revierta nadie por creerlos intrusos.

**Cuatro valores del contexto estan desfasados.** Los del arbol mandan:

```
                          contexto      arbol (correcto)
lidar_to_front_bumper_m     0.195           0.09
table_z_mm                  -49.8          -42.70
approach_stop_distance       0.30      revisar, ver seccion 4
arm_x_offset_mm              142.0     NO ES CONSTANTE, ver abajo
```

**EL DESFASE NO ES CONSTANTE.** Cuatro lecturas dan cuatro numeros:

```
lectura                       parada   X brazo   desfase
tocando la superficie (1a)      300      149.8     150.2
tocando la superficie (2a)      280      138.0     142.0
agarre del poste (1a)           280      222.3      57.7
agarre del poste (2a, nueva)    184      163.5      20.5
```

Si `arm_x_offset_mm` fuera una constante geometrica del robot saldria el
mismo numero siempre. No lo es. **El modelo generico
`X = parada*1000 - arm_x_offset_mm` no describe lo que creemos**, y el
142 que hay en el codigo salio de una lectura de tocar la superficie,
que es otro punto del escenario. Para el poste da igual (usa pose
enseñada), pero esta MAL para engranaje y rueda.

**CONTRADICCION SIN RESOLVER, mirar antes de mover el brazo:**

```
angulos de la pose nueva  [7.29, 132.01, -50.62, 4.74, -90.17, -0.08]
limite de J2 del firmware  -75 .. 120
                                   132.01 > 120
```

El brazo reporto un angulo por encima de su propio maximo declarado. O
la lectura de limites es falsa, o la de angulos lo es. Zanjarlo con
`get_joint_max_angle(2)` y `get_angles()` seguidos antes de dar la pose
por buena.

## 3. La pose enseñada nueva

```
target_coords  [163.50, 15.50, -25.30, 4.08, 81.21, 6.62]
angulos        [7.29, 132.01, -50.62, 4.74, -90.17, -0.08]
parada LiDAR   0.184 m   (goal pedia 0.20)
radio          166.2 mm = 62% del alcance
```

El 62% es lo que importa: **la pose que quemo el servo estaba al 83%**.
Ese fue el problema, no la cinematica.

Los angulos son para VALIDAR la rama, no para mandarlos como goal
articular. Las coordenadas siguen gobernando el modelo.

## 4. El historial que hay que respetar

`target_coords_stop_m` guarda a que parada se enseño la pose, y el
servidor desplaza la X por la diferencia con la parada real medida. Al
meter la pose nueva hay que actualizar ese valor a **0.184**, o la
correccion aplicara el delta equivocado.

`approach_stop_distance` (0.30) ya no cuadra con esta pose. La
aproximacion se queda unos 27 mm larga de forma consistente, asi que
para aterrizar cerca de 0.184 hay que pedir ~0.157. Comprobarlo en vez
de suponerlo.

## 5. Estado del hardware

El servo 2 sufrio `over-load` por trabajar al 83% del alcance. Se limpio
con ciclo de alimentacion. Tras la verificacion:

```
get_servo_status   seis ceros
read_next_error    sin errores
J3                 31 C antes, 37 C tras un movimiento pequeño
safe_navigation    SUCCEEDED, error maximo 1.58 grados en J5
```

Los dos criterios que "no cumplen" (1 grado y 5 C) los puse yo con
umbrales mal calibrados: 1.58 cae entre los 0.79 y 1.76 medidos con el
brazo sano, y subir 6 C desde frio no es comparable a subir 2 C estando
parado a 58 C. **No hay evidencia de daño.** Lo que falta es una linea
base termica: tres o cuatro movimientos pequeños seguidos leyendo
temperatura. Si se estabiliza hacia 40-45 C es normal; si sube de forma
monotona, no.

## 6. Estado de la red AHORA MISMO

```
100.86.172.41  (Tailscale Jetson)   NO RESPONDE, 100% de perdida
10.53.98.48    (LAN Jetson)         responde
portatil       100.91.114.36 y 10.53.98.54, las dos arriba
```

**Tailscale esta caido en la Jetson** y toda la pila va por ahi. Hay que
levantarlo (`systemctl restart tailscaled`) antes de nada.

NO pasarse a la LAN sin depurar: se probo, y el descubrimiento DDS por
10.53.x no funcionaba. 239 ventanas seguidas, 20 minutos, cero
publicadores desde el portatil con la pila del robot sana.

## 7. Foxglove: NO esta roto

Verificado en el portatil con `FOXGLOVE=1`:

```
foxglove_bridge arranca, PID vivo
Started server on 0.0.0.0:8765
ss -tlnp   LISTEN 0.0.0.0:8765
conecta desde 127.0.0.1 y desde 100.91.114.36
```

El script y el puente estan bien. Si no conecta, es del lado del cliente:

- **Usa la app de escritorio, no el navegador.** `app.foxglove.dev` va
  por HTTPS y bloquea en silencio un `ws://` inseguro a una IP remota.
- **Esto es WSL.** Desde Windows hay que conectar a
  `ws://localhost:8765`, que WSL2 reenvia solo. La IP de Tailscale
  `100.91.114.36` es de dentro de WSL y Windows no la alcanza. El script
  imprime esa IP primero, lo cual despista: es un mensaje a mejorar.

## 8. Orden de trabajo

1. Levantar tailscaled en la Jetson y confirmar que 100.86.172.41 responde.
2. Resolver la contradiccion de J2 (seccion 2) sin mover el brazo.
3. Meter `target_coords` y `target_coords_stop_m: 0.184` en el catalogo.
4. Ajustar `approach_stop_distance` para aterrizar cerca de 0.184.
5. Aproximar la base y comprobar donde para de verdad.
6. Pre-agarre SIN descenso ni cierre. Leer temperatura antes y despues.
7. Solo si eso pasa, descenso y cierre.

## 9. No hacer

- No usar la calibracion vieja `[219.10, 5.90, -9.50, ...]`.
- No mandar los angulos enseñados como goal articular.
- No probar la pose vieja `[222.30, 21.20, 25.60, ...]`: es la del 83%
  de alcance, la que provoco el over-load.
- No dejar que un limitador recorte angulos contra los limites viejos
  del `mecharm.yaml` (J2<=90): eran falsos, el firmware da 120.
