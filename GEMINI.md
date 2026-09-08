# Memoria Técnica y de Contexto: T-SUMMIT Challenge

Este documento consolida y estructura todo el conocimiento contenido en la carpeta `Contexto/`, manuales técnicos, hojas de especificación y scripts del workspace para la operación del robot móvil **myAGV** y el brazo robótico **MechArm 270 M5**.

---

## 1. Características de la Plataforma Robótica

### 1.1 Base Móvil: Elephant Robotics myAGV
- **Cinemática:** Plataforma omnidireccional con 4 ruedas Mecanum.
- **Carga máxima superior:** 5 kg.
- **Límites de superficie:** Pendiente máxima del terreno **≤ 5%** (riesgo de deslizamiento de rodillos Mecanum). Operación exclusiva en interiores sobre superficies planas y secas.
- **Alimentación y Batería:**
  - Batería principal y secundaria.
  - Para cargar eficientemente, apagar el botón de energía de la AGV.
  - La batería secundaria inicia carga cuando su nivel desciende de 11V.
- **LiDAR:**
  - Modelo: **YDLIDAR X2L**.
  - Rango angular: 360° horizontal, FOV vertical: 0 a 1.75°, tasa de muestreo: 4000 puntos/s.
  - Script de arranque nativo: `start_ydlidar.sh`. Topic ROS 2: `/scan`, filtrado en `/scan_filtered`.

### 1.2 Brazo Robótico: MechArm 270 M5
- **Grados de libertad:** 6 ejes (J1 a J6), estructura centrosimétrica industrial.
- **Controlador base:** M5Stack-basic.
- **Peso propio:** 1.0 kg.
- **Capacidad de carga (Payload):** 250 g.
- **Radio de alcance útil:** 270 mm. Repetibilidad de posicionamiento: ±0.5 mm.
- **Límites angulares articulares:**
  - J1: -160° a +160°
  - J2: -85° a +90°
  - J3: -180° a +45°
  - J4: -160° a +160°
  - J5: -100° a +100°
  - J6: rotación continua (-∞ a +∞)
  - Velocidad angular máxima: 120°/s.
- **Comunicación:** Puerto serie USB `/dev/ttyACM0` a 115200 baudios con protocolo `pymycobot.mecharm270.MechArm270`.
- **End-Effector (Garra Adaptativa):**
  - Carrera total de pinza: ~45 mm (`stroke_mm`).
  - Rango de comando ROS/Python: 0 a 100 (0 = cerrado, 100 = abierto).
  - En llamadas de bajo nivel `set_gripper_state`: `0 = abrir`, `1 = cerrar`.

### 1.3 Visión Artificial: Cámara Frontal CSI IMX219
- **Sensor:** Sony IMX219 vía conector Ribbon CSI conectado a Jetson.
- **Pipeline GStreamer:**
  ```python
  "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=3264, height=2464, framerate=21/1 ! nvvidconv flip-method=0 ! video/x-raw, width=960, height=540, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=True max-buffers=1 emit-signals=True"
  ```
- **Resolución efectiva de trabajo:** 960 × 540 píxeles.
- **Matriz de calibración intrínseca:**
  - $f_x = 785.855$, $f_y = 584.820$, $c_x = 451.671$, $c_y = 259.057$.
  - Coeficientes de distorsión: `[0.095135, -0.109279, -0.002513, -0.002418, 0.0]`.

---

## 2. Sistema de Marcadores ArUco y Navegación Guiada

- **Diccionario:** `cv2.aruco.DICT_6X6_250`.
- **Dimensiones físicas de los marcadores:**
  - **Robot real en pista:** **0.08 m (8.0 cm x 8.0 cm)** (según hoja oficial `ArUcos_6x6_250_ID0-9_8cm_Carta.pdf`).
  - **Simulador Gazebo:** 0.05 m (5.0 cm).
- **Proceso de aproximación multimodal:**
  1. `SEARCHING`: Búsqueda angular del marcador con cámara CSI.
  2. `LOCK_TARGET`: Fijación de ID y cálculo de pose 3D ($X, Y, Z$, matriz Rodrigues).
  3. `ALIGN_HEADING_TO_ARUCO`: Orientación frontal perpendicular al plano del marcador.
  4. `ALIGNING_LATERAL`: Desplazamiento lateral omnidireccional (strafe) mediante Mecanum.
  5. `APPROACHING`: Avance frontal regulando velocidad y midiendo distancia residual por LiDAR.
  6. `REACHED`: Parada de precisión a distancia de trabajo (`stop_distance`, p. ej. 0.18 m).

---

## 3. Especificaciones del Torneo T-SUMMIT Challenge

### 3.1 Pista y Logística
- **Dimensiones totales:** 9.00 m de largo × 6.00 m de ancho × 0.50 m de altura perimetral.
- **División:** 4 cuadrantes independientes de 4.50 m × 3.00 m cada uno.
- **Dinámica:** Dos pistas gemelas; rotación de 5 equipos (4 compitiendo simultáneamente en cada cuadrante, 1 en pits) en bloques cronometrados de **6 minutos**.

```
+---------------------------+---------------------------+
|                           |                           |
|   Cuadrante 1 (A1)        |   Cuadrante 2 (B1)        |
|   CLASIFICACIÓN (KOSTAL)  |   KITTING (DENSO)         |
|                           |                           |
+---------------------------+---------------------------+
|                           |                           |
|   Cuadrante 3 (A2)        |   Cuadrante 4 (B2)        |
|   ENSAMBLAJE (MICHELIN)   |   LABERINTO (VCST)        |
|                           |                           |
+---------------------------+---------------------------+
```

### 3.2 Cuadrante 1: Clasificación (Caso KOSTAL)
- **Escenario:** 4 plataformas/mesas bajas (≤ 10 cm altura, 1.20 m × 0.50 m), pasillos estrechos de 1.00 m y 0.50 m.
- **Misión obligatoria estandarizada:**
  $$\text{START} \longrightarrow \text{Verde (ArUco 0)} \longrightarrow \text{Entrega Verde (ArUco 2)} \longrightarrow \text{Azul (ArUco 1)} \longrightarrow \text{Entrega Azul (ArUco 3)} \longrightarrow \text{FINISH}$$
- **Requisitos:** Ejecución 100% autónoma. Sin asistencia física. Respeto estricto del orden sin saltos.

### 3.3 Cuadrante 2: Kitting (Caso DENSO)
- **Escenario:** Estanterías y tolvas horizontales y verticales.
- **Secuencia requerida:**
  $$\text{START} \longrightarrow \text{Verde (ArUco 0)} \longrightarrow \text{Entrega Verde (ArUco 2)} \longrightarrow \text{Azul (ArUco 1)} \longrightarrow \text{Entrega Azul (ArUco 3)} \longrightarrow \text{FINISH}$$
- **Regla de Oro ("Lógica de Fallo" / Keep-it-simple):**
  - Implementación obligatoria de **Timeout de Detección de 10 segundos**.
  - Si el marcador/pieza no se detecta en 10 s, el sistema no debe bloquearse. Debe registrar en el log `"Pieza Omitida"` (`on_failure: skip`) y continuar a la siguiente coordenada de la secuencia.

### 3.4 Cuadrante 3: Ensamblaje (Caso MICHELIN)
- **Escenario:** 5 plataformas con componentes y una estación con un poste vertical (`POSTE2Nuevo.STL`).
- **Dinámica:**
  1. Recorrer en orden las estaciones de suministro identificadas con marcadores ArUco 0, 1 y 2.
  2. Recoger cada componente sucesivamente (Engranaje, Rueda, etc.).
  3. Trasladar las piezas a la base de montaje.
  4. Apilar concéntricamente las 3 piezas una sobre otra alrededor del poste formando una torre centrada y estable.

### 3.5 Cuadrante 4: Laberinto (Caso VCST)
- **Dimensiones:** Pasillos críticos de **60 cm (0.60 m)** de ancho exacto. Muros con grosor ~12 cm.
- **Navegación:** Desplazamiento desde punto de partida `START` (esquina superior izq, 1.0m × 1.0m) hasta `FINISH` (esquina inferior dcha).
- **Desafío:** Obstáculos dinámicos no mapeados en los pasillos de 60 cm. Nav2 debe gestionar el mapa dinámico de costes (Dynamic Costmap con LiDAR) para planificar rutas alternativas en tiempo real.
- **Restricción de postura:** El brazo debe permanecer plegado en pose `home` para respetar el gálibo del robot.

---

## 4. Catálogo Geométrico de Piezas y Estrategia de Agarre (STL)

Las piezas modeladas en `Contexto/` y analizadas en `src/home_service_behaviors/config/grasp_catalog.yaml`:

| Pieza | Archivo STL | Dimensiones Clave | Estrategia de Agarre | Parámetros de Agarre |
|---|---|---|---|---|
| **Engranaje** | `ENGRANAJE.STL` | Ø exterior punta: 152.8 mm<br>Ø raíz: 114.8 mm<br>Ø agujero: 49.2 mm<br>Espesor: 15.0 mm | Agarre radial sobre el alma anular maciza. Una mordaza baja en el agujero central y la otra en la raíz. | `span_mm`: 32.8<br>`open_mm`: 40.0<br>`close_mm`: 28.0<br>`grasp_z_mm`: 7.5 mm |
| **Poste / Base** | `POSTE2Nuevo.STL` | Disco base: Ø 110 mm × 20 mm espesor<br>Tubo cilíndrico: Ø 20 mm × 100 mm alt. | Pinzado superior del tubo cilíndrico en su tercio superior, librando la base. | `span_mm`: 20.0<br>`open_mm`: 32.0<br>`close_mm`: 15.0<br>`grasp_z_mm`: 70.0 mm |
| **Rueda** | `RUEDA15Nueva.STL` | Aro: Ø 150 mm × 5 mm espesor axial<br>Pared radial: 20 mm<br>Pie de apoyo: 20 × 30 mm | Pinzar el pie rectangular por el lado de 20 mm a 15 mm sobre la mesa (evita rebasar el alcance máximo de 270 mm del brazo). | `span_mm`: 20.0<br>`open_mm`: 32.0<br>`close_mm`: 15.0<br>`grasp_z_mm`: 15.0 mm |
| **Plataforma Start** | `Startazul20por15Nuevo.STL` | Dimensiones base: 20 cm × 15 cm | Base de inicio/estacionamiento. | - |

---

## 5. Arquitectura Software y Operación

### 5.1 Enrutamiento de Velocidad (`twist_mux`)
El control de la base móvil unifica tres fuentes de comando mediante prioridades:
1. **Teleoperación manual (Gamepad):** `/cmd_vel_joy` — Prioridad 200
2. **Alineación visual ArUco + LiDAR:** `/cmd_vel_aruco` — Prioridad 100
3. **Navegación global / local Nav2:** `/cmd_vel_nav` — Prioridad 50
4. **Salida final:** `/cmd_vel` hacia `/mecanum_drive_controller`

### 5.2 Scripts Clave de Lanzamiento en Producción
- `./scripts/tsummit.sh help`: Menú principal del robot.
- `ALLOW_MOTION=1 ./scripts/tsummit.sh reto1`: Ejecución del Reto 1 (Clasificación).
- `ALLOW_MOTION=1 ./scripts/tsummit.sh reto2`: Ejecución del Reto 2 (Kitting).
- `ALLOW_MOTION=1 ./scripts/tsummit.sh reto4`: Ejecución del Reto 4 (Laberinto con Nav2).
- `./scripts/tsummit.sh grasp-dry`: Verificación de cinemática y visión sin mover actuadores.
- `./scripts/run_classification.sh teach`: Calibración manual de poses con `mecharm_console.py`.
