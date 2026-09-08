#!/usr/bin/env python3

import math

import cv2
import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from tf2_ros import TransformBroadcaster

from home_service_interfaces.msg import (
    ArucoDetection,
    ArucoDetectionArray,
)


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        # ----------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------

        self.declare_parameter(
            'image_topic',
            '/camera/image_raw'
        )

        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera_info'
        )

        self.declare_parameter(
            'detections_topic',
            '/aruco/detections'
        )

        self.declare_parameter(
            'annotated_image_topic',
            '/aruco/image_annotated'
        )

        # Tamano real del lado impreso del marcador, en metros.
        # En esta competencia los ArUco se imprimen a 8 cm.
        self.declare_parameter(
            'marker_length',
            0.08
        )

        # Alias obsoleto: si es > 0 tiene prioridad sobre marker_length.
        self.declare_parameter(
            'marker_size',
            0.0
        )

        self.declare_parameter(
            'equalize_hist',
            True
        )

        self.declare_parameter(
            'publish_tf',
            True
        )

        self.image_topic = (
            self.get_parameter('image_topic')
            .get_parameter_value()
            .string_value
        )

        self.camera_info_topic = (
            self.get_parameter('camera_info_topic')
            .get_parameter_value()
            .string_value
        )

        self.detections_topic = (
            self.get_parameter('detections_topic')
            .get_parameter_value()
            .string_value
        )

        self.annotated_image_topic = (
            self.get_parameter('annotated_image_topic')
            .get_parameter_value()
            .string_value
        )

        marker_length = (
            self.get_parameter('marker_length')
            .get_parameter_value()
            .double_value
        )

        marker_size_alias = (
            self.get_parameter('marker_size')
            .get_parameter_value()
            .double_value
        )

        # marker_size (obsoleto) gana solo si se fijo explicitamente > 0.
        self.marker_length = (
            marker_size_alias
            if marker_size_alias > 0.0
            else marker_length
        )

        self.equalize_hist = bool(
            self.get_parameter('equalize_hist')
            .get_parameter_value()
            .bool_value
        )

        self.publish_tf = bool(
            self.get_parameter('publish_tf')
            .get_parameter_value()
            .bool_value
        )

        # ----------------------------------------------------------
        # Camera calibration
        # ----------------------------------------------------------

        self.camera_matrix = None
        self.dist_coeffs = None

        # ----------------------------------------------------------
        # OpenCV / ROS
        # ----------------------------------------------------------

        self.bridge = CvBridge()

        self.tf_broadcaster = TransformBroadcaster(self)

        # ----------------------------------------------------------
        # Diccionarios ArUco
        #
        # Detectar un diccionario extra casi DUPLICA el coste del
        # callback (es lo que tenia la deteccion a ~2.6 Hz en la Nano,
        # justo en el borde del detection_timeout de la aproximacion).
        # El T-SUMMIT usa 6x6_250 (ArUcos_6x6_250_ID0-9); el 5x5 era
        # del Home Service Challenge. Configurable por si acaso.
        # ----------------------------------------------------------

        # Modo distribuido: cuando este nodo corre en un portatil y la
        # camara en el robot, la imagen viaja comprimida por WiFi (la
        # cruda serian ~186 Mbit/s). Ver scripts/tsummit_offboard.sh.
        self.declare_parameter('use_compressed', False)
        self.use_compressed = bool(
            self.get_parameter('use_compressed').value
        )

        self.declare_parameter('use_dict_6x6_250', True)
        self.declare_parameter('use_dict_5x5_1000', False)

        # Tope de proceso (Hz). La camara puede publicar a 15-20 Hz pero
        # detectar a mas de ~8 Hz solo sirve para saturar la Nano.
        self.declare_parameter('max_process_hz', 8.0)
        self.max_process_hz = float(
            self.get_parameter('max_process_hz').value
        )
        self._last_process_t = 0.0

        # ------------------------------------------------------------
        # DESACOPLAR CAPTURA DE PROCESO
        #
        # El callback de imagen hacia TODO -- decodificar el JPEG,
        # equalizar, detectMarkers, solvePnP por marcador, TF, publicar
        # -- de forma sincrona en el unico hilo del executor. Con una
        # cola de 5 y un coste por fotograma que varia mucho (detectar
        # depende de cuantos cuadrilateros candidatos encuentre, y eso
        # se dispara con ruido o clutter), unos fotogramas se
        # amontonaban y salian a rafaga: de ahi las CAIDAS BRUSCAS de Hz
        # que impedian que la aproximacion se asentara.
        #
        # Ahora el callback solo GUARDA el ultimo fotograma (coste casi
        # nulo) y un timer lo procesa a ritmo fijo. Siempre el mas
        # fresco, nunca uno rancio de la cola, y a tasa determinista.
        # process_hz <= 0 -> comportamiento antiguo (procesar en el
        # callback), por si hace falta volver atras.
        self.declare_parameter('process_hz', 15.0)
        self.process_hz = float(self.get_parameter('process_hz').value)

        # Hilos que OpenCV usa en detectMarkers / imdecode. 0 = deja
        # que OpenCV decida (suele coger todos). En la Nano conviene
        # limitarlo para no pisar el nucleo del laser; en el portatil,
        # cuantos mas mejor.
        self.declare_parameter('opencv_threads', 0)
        _cvt = int(self.get_parameter('opencv_threads').value)
        if _cvt > 0:
            cv2.setNumThreads(_cvt)

        # La imagen anotada (dibujo + axes + encode + publicar) es cara
        # y nadie necesita verla a 30 fps. Se limita aparte aunque haya
        # suscriptor.
        self.declare_parameter('annotated_hz', 5.0)
        self.annotated_hz = float(self.get_parameter('annotated_hz').value)
        self._last_annotated_t = 0.0

        # Estado de la captura desacoplada.
        self._latest_frame = None      # (header, image_bgr) ya decodificado
        self._latest_stamp_ns = None   # sello del ultimo procesado
        self._frame_lock = None        # se crea abajo (threading)

        # Escala a la que se corre detectMarkers (1.0 = resolucion real).
        # detectMarkers es O(pixeles): a 960x540 tardaba ~250 ms en la
        # Nano. A 0.6 (~576x324) baja a ~90 ms y un ArUco de 8 cm a 1 m
        # aun mide ~45 px/lado. Las esquinas se reescalan a resolucion
        # real antes de estimar la pose, la precision no cambia.
        self.declare_parameter('detect_scale', 0.6)
        self.detect_scale = float(
            self.get_parameter('detect_scale').value
        )

        self._dictionaries = []
        if bool(self.get_parameter('use_dict_6x6_250').value):
            self._dictionaries.append((
                cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250),
                '6X6_250',
            ))
        if bool(self.get_parameter('use_dict_5x5_1000').value):
            self._dictionaries.append((
                cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000),
                '5X5_1000',
            ))

        self.detector_parameters = (
            cv2.aruco.DetectorParameters_create()
        )

        # Un ArUco de 8 cm entre 0.15 y 1.5 m ocupa una fraccion GRANDE
        # del encuadre. El 0.03 de fabrica acepta candidatos minusculos
        # -> muchos cuadrilateros que rechazar, coste alto y sobre todo
        # MUY VARIABLE con el ruido. Subir el minimo recorta ese trabajo
        # inutil y estabiliza el tiempo por fotograma.
        self.declare_parameter('min_marker_perimeter_rate', 0.06)
        self.detector_parameters.minMarkerPerimeterRate = float(
            self.get_parameter('min_marker_perimeter_rate').value
        )

        # Refinado subpixel de esquinas: ~1 ms con pocos marcadores y
        # mejora center_x_normalized y la normal del ArUco, que es lo
        # que usa el alineamiento. CORNER_REFINE_SUBPIX = 1.
        self.declare_parameter('corner_refine', True)
        if bool(self.get_parameter('corner_refine').value):
            self.detector_parameters.cornerRefinementMethod = 1
            self.detector_parameters.cornerRefinementWinSize = 4
            self.detector_parameters.cornerRefinementMaxIterations = 20

        import threading as _threading
        self._frame_lock = _threading.Lock()

        # Estadisticas de rendimiento: cada 5 s se registra la tasa real
        # de proceso y el coste medio, para ver de un vistazo si el
        # portatil va sobrado o pega tirones.
        self._stat_n = 0
        self._stat_ms = 0.0
        self._stat_hits = 0
        self._stat_t0 = None

        # ----------------------------------------------------------
        # Subscribers
        # ----------------------------------------------------------

        # Grupos: la ingesta de imagen y camera_info NO deben esperar a
        # que termine el procesado pesado. El timer de proceso va en su
        # propio grupo exclusivo (un solo procesado a la vez).
        self._io_group = ReentrantCallbackGroup()
        self._proc_group = MutuallyExclusiveCallbackGroup()

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
            callback_group=self._io_group,
        )

        # BEST_EFFORT + KEEP_LAST 1: la camara publica asi (perfil sensor
        # data). Con RELIABLE aqui no llegaria NI UN frame (QoS
        # incompatible), y aunque llegara, encolar frames viejos no
        # sirve para deteccion en vivo: siempre queremos el ultimo.
        if self.use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.image_topic + '/compressed',
                self.compressed_callback,
                qos_profile_sensor_data,
                callback_group=self._io_group,
            )
        else:
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                qos_profile_sensor_data,
                callback_group=self._io_group,
            )

        if self.process_hz > 0.0:
            self._proc_timer = self.create_timer(
                1.0 / self.process_hz,
                self._process_timer,
                callback_group=self._proc_group,
            )

        # ----------------------------------------------------------
        # Publishers
        # ----------------------------------------------------------

        self.detections_pub = self.create_publisher(
            ArucoDetectionArray,
            self.detections_topic,
            10
        )

        self.annotated_pub = self.create_publisher(
            Image,
            self.annotated_image_topic,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            'Aruco detector started'
        )

        self.get_logger().info(
            f'Image topic: {self.image_topic}'
        )

        self.get_logger().info(
            f'Camera info topic: {self.camera_info_topic}'
        )

        self.get_logger().info(
            f'Marker size: {self.marker_length:.3f} m'
        )

        self.get_logger().info(
            'Diccionarios activos: '
            + (', '.join(name for _, name in self._dictionaries)
               or 'NINGUNO (revisa use_dict_*)')
        )

    # ==============================================================
    # Camera info
    # ==============================================================

    def camera_info_callback(self, msg):

        self.camera_matrix = np.array(
            msg.k,
            dtype=np.float64
        ).reshape(3, 3)

        self.dist_coeffs = np.array(
            msg.d,
            dtype=np.float64
        )

    # ==============================================================
    # Rotation matrix -> quaternion
    # ==============================================================

    def rotation_matrix_to_quaternion(self, matrix):

        m00 = matrix[0, 0]
        m01 = matrix[0, 1]
        m02 = matrix[0, 2]

        m10 = matrix[1, 0]
        m11 = matrix[1, 1]
        m12 = matrix[1, 2]

        m20 = matrix[2, 0]
        m21 = matrix[2, 1]
        m22 = matrix[2, 2]

        trace = m00 + m11 + m22

        if trace > 0.0:

            s = math.sqrt(trace + 1.0) * 2.0

            qw = 0.25 * s
            qx = (m21 - m12) / s
            qy = (m02 - m20) / s
            qz = (m10 - m01) / s

        elif m00 > m11 and m00 > m22:

            s = math.sqrt(
                1.0 + m00 - m11 - m22
            ) * 2.0

            qw = (m21 - m12) / s
            qx = 0.25 * s
            qy = (m01 + m10) / s
            qz = (m02 + m20) / s

        elif m11 > m22:

            s = math.sqrt(
                1.0 + m11 - m00 - m22
            ) * 2.0

            qw = (m02 - m20) / s
            qx = (m01 + m10) / s
            qy = 0.25 * s
            qz = (m12 + m21) / s

        else:

            s = math.sqrt(
                1.0 + m22 - m00 - m11
            ) * 2.0

            qw = (m10 - m01) / s
            qx = (m02 + m20) / s
            qy = (m12 + m21) / s
            qz = 0.25 * s

        norm = math.sqrt(
            qx * qx +
            qy * qy +
            qz * qz +
            qw * qw
        )

        if norm > 0.0:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm

        return qx, qy, qz, qw

    # ==============================================================
    # Detect one dictionary
    # ==============================================================

    def detect_dictionary(
        self,
        gray,
        dictionary,
        dictionary_name
    ):

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            dictionary,
            parameters=self.detector_parameters
        )

        detections = []

        if ids is None:
            return detections

        for i, marker_id in enumerate(ids.flatten()):

            detections.append({
                'id': int(marker_id),
                'corners': corners[i],
                'dictionary': dictionary_name,
            })

        return detections

    # ==============================================================
    # Image callback
    # ==============================================================

    def compressed_callback(self, msg):
        """JPEG -> BGR. Con process_hz>0 solo guarda; procesa el timer."""
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'jpeg decode: {exc}', throttle_duration_sec=5.0
            )
            return

        if image is None:
            return

        self._ingest(msg.header, image)

    def image_callback(self, msg):
        """Imagen cruda -> BGR. Con process_hz>0 solo guarda."""
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(
                f'cv_bridge error: {exc}'
            )
            return

        self._ingest(msg.header, image)

    def _ingest(self, header, image):
        """Guarda el fotograma para que lo procese el timer.

        Si process_hz<=0 se procesa aqui mismo (modo antiguo).
        """
        if self.process_hz <= 0.0:
            self._process(header, image)
            return
        with self._frame_lock:
            self._latest_frame = (header, image)

    def _process_timer(self):
        """Toma el ultimo fotograma disponible y lo procesa. Salta si no
        ha llegado ninguno nuevo desde la ultima vez (no reprocesa)."""
        with self._frame_lock:
            frame = self._latest_frame
        if frame is None:
            return
        header, image = frame
        stamp_ns = (
            int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
        )
        if self._latest_stamp_ns == stamp_ns:
            return
        self._latest_stamp_ns = stamp_ns
        self._process(header, image)

    # ==============================================================
    # Camino comun (lo alimentan image_callback y compressed_callback)
    # ==============================================================

    def _process(self, header, image):

        _t_start = self.get_clock().now().nanoseconds * 1e-9
        if self._stat_t0 is None:
            self._stat_t0 = _t_start

        if self.camera_matrix is None:
            self.get_logger().warn(
                'Waiting for camera_info...',
                throttle_duration_sec=2.0
            )
            return

        # Limita el ritmo de PROCESO. Con process_hz>0 el ritmo ya lo
        # marca el timer y este gate sobra (ademas su return temprano se
        # saltaba las estadisticas). Solo actua en modo antiguo
        # (process_hz<=0, procesar en el callback).
        if self.process_hz <= 0.0 and self.max_process_hz > 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
            if (now - self._last_process_t) < (1.0 / self.max_process_hz):
                return
            self._last_process_t = now

        # ¿Alguien mira la imagen anotada? Si no, no dibujamos ni la
        # codificamos: es lo que mas frena el callback. Y si la mira,
        # aun asi la limitamos a annotated_hz -- nadie necesita el
        # recuadro a 30 fps y encode+publish de la imagen entera compite
        # con la deteccion.
        draw_annotated = self.annotated_pub.get_subscription_count() > 0
        if draw_annotated and self.annotated_hz > 0.0:
            _now_a = self.get_clock().now().nanoseconds * 1e-9
            if (_now_a - self._last_annotated_t) < (1.0 / self.annotated_hz):
                draw_annotated = False
            else:
                self._last_annotated_t = _now_a

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )

        # Realza el contraste: mejora la deteccion con luz irregular
        # (recomendado por la guia de camara del myAGV).
        if self.equalize_hist:
            gray = cv2.equalizeHist(gray)

        # Detectar sobre una imagen reducida: detectMarkers es O(pixeles)
        # y a 960x540 tardaba ~250 ms en la Nano (deteccion a 4 Hz). A
        # escala 0.6 (576x324) baja a ~90 ms y un ArUco de 8 cm a 1 m
        # sigue con ~45 px/lado. Las esquinas se reescalan de vuelta
        # antes de estimar la pose, asi que la precision no cambia.
        if self.detect_scale < 0.999:
            small = cv2.resize(
                gray, None,
                fx=self.detect_scale, fy=self.detect_scale,
                interpolation=cv2.INTER_AREA,
            )
        else:
            small = gray

        # ----------------------------------------------------------
        # Deteccion (diccionarios activos, ver __init__)
        # ----------------------------------------------------------

        detections_raw = []
        for dictionary, name in self._dictionaries:
            detections_raw.extend(
                self.detect_dictionary(small, dictionary, name)
            )

        # Esquinas de la imagen reducida -> resolucion real.
        if self.detect_scale < 0.999:
            inv = 1.0 / self.detect_scale
            for d in detections_raw:
                d['corners'] = d['corners'] * inv

        # ----------------------------------------------------------
        # Detection array
        # ----------------------------------------------------------

        detection_array = ArucoDetectionArray()

        detection_array.header = header
        detection_array.detections = []

        image_height, image_width = image.shape[:2]

        # ----------------------------------------------------------
        # Process every marker
        # ----------------------------------------------------------

        for detection_raw in detections_raw:

            marker_id = detection_raw['id']
            marker_corners = detection_raw['corners']
            dictionary_name = detection_raw['dictionary']

            # ------------------------------------------------------
            # Pose estimation
            # ------------------------------------------------------

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [marker_corners],
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs
            )

            if rvecs is None or tvecs is None:
                continue

            rvec = rvecs[0][0]
            tvec = tvecs[0][0]

            # ------------------------------------------------------
            # Marker center
            # ------------------------------------------------------

            points = marker_corners.reshape(
                4,
                2
            )

            center_x = float(
                np.mean(points[:, 0])
            )

            center_y = float(
                np.mean(points[:, 1])
            )

            center_x_normalized = (
                center_x -
                (image_width / 2.0)
            ) / (
                image_width / 2.0
            )

            # ------------------------------------------------------
            # Orientation
            # ------------------------------------------------------

            rotation_matrix, _ = cv2.Rodrigues(
                rvec
            )

            qx, qy, qz, qw = (
                self.rotation_matrix_to_quaternion(
                    rotation_matrix
                )
            )

            # ------------------------------------------------------
            # ArucoDetection message
            # ------------------------------------------------------

            detection = ArucoDetection()

            detection.header = header
            detection.id = marker_id

            detection.pose.position.x = float(
                tvec[0]
            )

            detection.pose.position.y = float(
                tvec[1]
            )

            detection.pose.position.z = float(
                tvec[2]
            )

            detection.pose.orientation.x = qx
            detection.pose.orientation.y = qy
            detection.pose.orientation.z = qz
            detection.pose.orientation.w = qw

            detection.center_x_px = center_x
            detection.center_x_normalized = (
                center_x_normalized
            )

            detection.distance_z = float(
                tvec[2]
            )

            detection.marker_size = float(
                self.marker_length
            )

            detection_array.detections.append(
                detection
            )

            # ------------------------------------------------------
            # TF
            #
            # IMPORTANTE:
            # mantenemos aruco_<id> para que siga funcionando
            # ArucoApproach sin tocarlo todavía.
            # ------------------------------------------------------

            tf_msg = TransformStamped()

            # El padre y el sello salen de la cabecera de la imagen
            # (camera_link). Sin esto TF descarta la transformada con
            # "TF_NO_FRAME_ID: ... because frame_id not set" y la
            # aproximacion nunca localiza el marcador.
            tf_msg.header.stamp = header.stamp
            tf_msg.header.frame_id = header.frame_id

            tf_msg.child_frame_id = (
                f'aruco_{marker_id}'
            )

            tf_msg.transform.translation.x = float(
                tvec[0]
            )

            tf_msg.transform.translation.y = float(
                tvec[1]
            )

            tf_msg.transform.translation.z = float(
                tvec[2]
            )

            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw

            if self.publish_tf:
                self.tf_broadcaster.sendTransform(
                    tf_msg
                )

            # ------------------------------------------------------
            # Draw marker (solo si alguien mira /aruco/image_annotated:
            # dibujar + codificar la imagen entera es lo mas caro del
            # callback y durante una aproximacion real nadie la mira).
            # ------------------------------------------------------

            if draw_annotated:
                cv2.aruco.drawDetectedMarkers(
                    image,
                    [marker_corners],
                    np.array([[marker_id]], dtype=np.int32)
                )

                cv2.drawFrameAxes(
                    image,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_length * 0.5
                )

                label = (
                    f'{dictionary_name} '
                    f'ID={marker_id} '
                    f'z={tvec[2]:.2f}m'
                )
                cv2.putText(
                    image,
                    label,
                    (int(center_x) - 70, int(center_y) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA
                )

        # ----------------------------------------------------------
        # Publish detections
        # ----------------------------------------------------------

        self.detections_pub.publish(
            detection_array
        )

        # ----------------------------------------------------------
        # Publish annotated image (solo si hay quien la mire)
        # ----------------------------------------------------------

        if draw_annotated:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(
                    image,
                    encoding='bgr8'
                )
                annotated_header = header
                self.annotated_pub.publish(annotated_msg)

            except Exception as exc:
                self.get_logger().error(
                    f'Annotated image publish error: {exc}'
                )

        # --- estadisticas de rendimiento ---
        _now = self.get_clock().now().nanoseconds * 1e-9
        self._stat_n += 1
        self._stat_ms += (_now - _t_start) * 1000.0
        if detection_array.detections:
            self._stat_hits += 1
        _win = _now - self._stat_t0
        if _win >= 5.0:
            hz = self._stat_n / _win
            avg_ms = self._stat_ms / max(1, self._stat_n)
            self.get_logger().info(
                f'deteccion {hz:.1f} Hz  ({avg_ms:.0f} ms/frame, '
                f'{self._stat_hits}/{self._stat_n} con marcador)'
            )
            self._stat_n = 0
            self._stat_ms = 0.0
            self._stat_hits = 0
            self._stat_t0 = _now


def main(args=None):

    rclpy.init(args=args)

    node = ArucoDetector()

    # Multihilo: la ingesta de fotogramas y camera_info no se quedan
    # bloqueadas detras del procesado pesado. 3 hilos bastan (io + proc
    # + margen); mas no ayuda porque el proc_group es exclusivo.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
