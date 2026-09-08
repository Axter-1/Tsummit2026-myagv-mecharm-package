#!/usr/bin/env python3
"""Publicador de la camara frontal del myAGV hacia ROS 2.

La camara frontal del myAGV es un modulo CSI (IMX219). En la Jetson NO
aparece como un /dev/video normal: hay que abrirla mediante el pipeline
GStreamer ``nvarguscamerasrc``. Un ``cv2.VideoCapture(0)`` devuelve
fotogramas negros o falla.

Este nodo soporta tres fuentes (parametro ``source``):

* ``nvargus``  (por defecto): pipeline ``nvarguscamerasrc`` de la Jetson.
* ``v4l2``:     ``cv2.VideoCapture(<device_index>)`` para una webcam USB
                o un puente v4l2 de la CSI.
* ``custom``:   se usa tal cual la cadena del parametro ``gst_pipeline``.

Publica:

* ``<camera>/image_raw``   (sensor_msgs/Image, bgr8)
* ``<camera>/camera_info`` (sensor_msgs/CameraInfo)

La informacion de calibracion se carga de ``camera_info_url`` (formato
estandar de ``camera_calibration``). Si no se indica o falla la carga, se
usa la calibracion de referencia del myAGV a 960x540 documentada en la
guia de camara/brazo.
"""

import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from sensor_msgs.msg import CameraInfo, CompressedImage, Image

try:
    import cv2
except ImportError as exc:  # pragma: no cover - depende del entorno
    raise ImportError(
        "OpenCV (cv2) no esta disponible. En la Jetson debe usarse el "
        "OpenCV del sistema compilado con soporte GStreamer."
    ) from exc

try:
    from cv_bridge import CvBridge
except ImportError as exc:  # pragma: no cover
    raise ImportError("cv_bridge no esta disponible.") from exc

import yaml


# Calibracion de referencia del myAGV (CSI IMX219) a 960x540.
# Fuente: "myAGV — ArUco Camera Detection & Robotic Arm Control".
DEFAULT_WIDTH = 960
DEFAULT_HEIGHT = 540

DEFAULT_CAMERA_MATRIX = [
    785.855437, 0.0, 451.670922,
    0.0, 584.820336, 259.056856,
    0.0, 0.0, 1.0,
]

DEFAULT_DIST_COEFFS = [
    0.095135, -0.109279, -0.002513, -0.002418, 0.0,
]


def build_gstreamer_pipeline(
    sensor_id,
    capture_width,
    capture_height,
    output_width,
    output_height,
    framerate,
    flip_method,
    exposure_time_us=0,
    gain=0.0,
):
    """Pipeline ``nvarguscamerasrc`` para la Jetson (guia de camara)."""

    manual_controls = ""
    if exposure_time_us > 0:
        manual_controls += (
            " aelock=true exposuretimerange=\"{0} {0}\"".format(
                int(exposure_time_us)
            )
        )
    if gain > 0.0:
        manual_controls += " gainrange=\"{0:.3f} {0:.3f}\"".format(gain)

    return (
        "nvarguscamerasrc{controls} sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM), width=(int){cw}, height=(int){ch}, "
        "framerate=(fraction){fr}/1 ! "
        "nvvidconv flip-method={flip} ! "
        "video/x-raw, width=(int){ow}, height=(int){oh}, "
        "format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! "
        "appsink drop=True max-buffers=1 emit-signals=True"
    ).format(
        controls=manual_controls,
        sensor_id=int(sensor_id),
        cw=int(capture_width),
        ch=int(capture_height),
        fr=int(framerate),
        flip=int(flip_method),
        ow=int(output_width),
        oh=int(output_height),
    )


class CsiCameraNode(Node):
    """Abre la camara y publica image_raw + camera_info."""

    def __init__(self):
        super().__init__("csi_camera_node")

        # -----------------------------------------------------------------
        # Parametros
        # -----------------------------------------------------------------
        self.declare_parameter("source", "nvargus")          # nvargus|v4l2|custom
        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("device_index", 0)            # solo v4l2
        self.declare_parameter("gst_pipeline", "")           # solo custom

        # Modo de sensor: 3264x2464@21 era la carga que mataba a la Nano
        # (el ISP + nvvidconv reescalando cada frame desde 8 MP). El
        # IMX219 tiene un modo 1280x720 nativo mucho mas ligero y sobra
        # para ArUcos de 8 cm a ~1 m.
        self.declare_parameter("capture_width", 1280)
        self.declare_parameter("capture_height", 720)
        # 960x540: por debajo, un ArUco de 8 cm a 1 m no llega a los
        # ~60 px/lado que un 6x6 necesita para detectarse fiable.
        self.declare_parameter("output_width", DEFAULT_WIDTH)
        self.declare_parameter("output_height", DEFAULT_HEIGHT)
        self.declare_parameter("framerate", 21)
        self.declare_parameter("exposure_time_us", 0)
        self.declare_parameter("gain", 0.0)
        # 2 = rot 180: el modulo CSI del myAGV va montado boca abajo.
        self.declare_parameter("flip_method", 2)

        # Publicacion. En modo distribuido (procesamiento en un
        # portatil) se apaga 'publish_raw' en el robot y solo viaja el
        # JPEG por WiFi.
        self.declare_parameter("publish_raw", True)
        self.declare_parameter("publish_compressed", True)
        self.declare_parameter("jpeg_quality", 80)

        self.declare_parameter("camera_name", "camera")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("camera_info_url", "")

        # 0.0 -> publica a la cadencia con la que llegan los fotogramas.
        self.declare_parameter("publish_rate", 0.0)
        self.declare_parameter("reopen_period_sec", 2.0)

        self.source = str(self.get_parameter("source").value).lower()
        self.sensor_id = int(self.get_parameter("sensor_id").value)
        self.device_index = int(self.get_parameter("device_index").value)
        self.custom_pipeline = str(self.get_parameter("gst_pipeline").value)

        self.capture_width = int(self.get_parameter("capture_width").value)
        self.capture_height = int(self.get_parameter("capture_height").value)
        self.publish_raw = bool(self.get_parameter("publish_raw").value)
        self.publish_compressed = bool(
            self.get_parameter("publish_compressed").value
        )
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.output_width = int(self.get_parameter("output_width").value)
        self.output_height = int(self.get_parameter("output_height").value)
        self.framerate = int(self.get_parameter("framerate").value)
        self.exposure_time_us = int(
            self.get_parameter("exposure_time_us").value
        )
        self.gain = float(self.get_parameter("gain").value)
        self.flip_method = int(self.get_parameter("flip_method").value)

        camera_name = str(self.get_parameter("camera_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        camera_info_url = str(self.get_parameter("camera_info_url").value)

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.reopen_period = float(
            self.get_parameter("reopen_period_sec").value
        )

        # -----------------------------------------------------------------
        # Calibracion
        # -----------------------------------------------------------------
        self.camera_info = self._load_camera_info(camera_info_url)

        # -----------------------------------------------------------------
        # ROS
        # -----------------------------------------------------------------
        self.bridge = CvBridge()

        # Imagen en BEST_EFFORT + KEEP_LAST 1 (perfil "sensor data"): es
        # lo estandar para video y evita el problema real que teniamos:
        # con RELIABLE, un suscriptor lento (el detector saturando la
        # Nano, o Foxglove por red) mete contrapresion y el publicador se
        # frena -> la imagen "no se publica" o va a tirones. En
        # BEST_EFFORT cada consumidor coge el ultimo frame y ya.
        # OJO: el suscriptor tambien debe ser BEST_EFFORT
        # (aruco_detector_node y foxglove_bridge ya lo hacen).
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # camera_info es pequeno y de baja tasa: RELIABLE + latch para que
        # cualquiera que llegue tarde reciba la calibracion igualmente.
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_pub = self.create_publisher(
            Image, f"{camera_name}/image_raw", image_qos
        )
        self.info_pub = self.create_publisher(
            CameraInfo, f"{camera_name}/camera_info", info_qos
        )
        self.compressed_pub = self.create_publisher(
            CompressedImage, f"{camera_name}/image_raw/compressed", image_qos
        )

        self.get_logger().info(
            f"publicando: raw={self.publish_raw} "
            f"comprimido={self.publish_compressed} "
            f"(jpeg q={self.jpeg_quality})"
        )

        self.capture = None

        # Timer de lectura de fotogramas.
        if publish_rate > 0.0:
            read_period = 1.0 / publish_rate
        else:
            read_period = 1.0 / max(1, self.framerate)

        self.read_timer = self.create_timer(read_period, self._on_read_timer)

        # Timer de (re)apertura de la camara.
        self.open_timer = self.create_timer(
            self.reopen_period, self._on_open_timer
        )

        self._warned_no_gstreamer = False

        self.get_logger().info(
            f"csi_camera_node -> source={self.source}, "
            f"{self.output_width}x{self.output_height}, "
            f"topic={camera_name}/image_raw, frame_id={self.frame_id}"
        )

        # Intento inmediato de apertura.
        self._on_open_timer()

    # =====================================================================
    # Calibracion
    # =====================================================================

    def _default_camera_info(self):
        info = CameraInfo()
        info.width = self.output_width
        info.height = self.output_height
        info.distortion_model = "plumb_bob"

        # Escala la calibracion de referencia (medida a 960x540) si la
        # resolucion de salida es distinta.
        sx = self.output_width / float(DEFAULT_WIDTH)
        sy = self.output_height / float(DEFAULT_HEIGHT)

        k = list(DEFAULT_CAMERA_MATRIX)
        k[0] *= sx  # fx
        k[2] *= sx  # cx
        k[4] *= sy  # fy
        k[5] *= sy  # cy

        info.k = k
        info.d = list(DEFAULT_DIST_COEFFS)
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            k[0], 0.0, k[2], 0.0,
            0.0, k[4], k[5], 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    def _load_camera_info(self, url):
        if not url:
            self.get_logger().warn(
                "camera_info_url vacio: se usa la calibracion de "
                "referencia del myAGV (recalibra para medir distancias "
                "con precision)."
            )
            return self._default_camera_info()

        path = url
        if path.startswith("file://"):
            path = path[len("file://"):]
        path = os.path.expanduser(path)

        if not os.path.isfile(path):
            self.get_logger().error(
                f"camera_info_url no encontrado: {path}. Se usa la "
                f"calibracion de referencia."
            )
            return self._default_camera_info()

        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle)

            cal_w = int(data["image_width"])
            cal_h = int(data["image_height"])
            k = [float(v) for v in data["camera_matrix"]["data"]]
            p = [float(v) for v in data["projection_matrix"]["data"]]

            # La calibracion suele estar a otra resolucion que la de
            # salida (p.ej. .yaml a 960x540, salida a 640x360). fx, fy,
            # cx, cy escalan lineal con la resolucion; d/r no cambian.
            sx = self.output_width / float(cal_w)
            sy = self.output_height / float(cal_h)
            if abs(sx - 1.0) > 1e-6 or abs(sy - 1.0) > 1e-6:
                for idx, s in ((0, sx), (2, sx), (4, sy), (5, sy)):
                    k[idx] *= s
                for idx, s in ((0, sx), (2, sx), (5, sy), (6, sy)):
                    p[idx] *= s
                self.get_logger().info(
                    f"Calibracion {cal_w}x{cal_h} escalada a "
                    f"{self.output_width}x{self.output_height}."
                )

            info = CameraInfo()
            info.width = self.output_width
            info.height = self.output_height
            info.distortion_model = data.get(
                "distortion_model", "plumb_bob"
            )
            info.k = k
            info.d = [
                float(v)
                for v in data["distortion_coefficients"]["data"]
            ]
            info.r = [
                float(v)
                for v in data["rectification_matrix"]["data"]
            ]
            info.p = p

            self.get_logger().info(f"Calibracion cargada de {path}")
            return info

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"No se pudo leer la calibracion ({exc}). Se usa la "
                f"calibracion de referencia."
            )
            return self._default_camera_info()

    # =====================================================================
    # Apertura de la camara
    # =====================================================================

    def _build_capture(self):
        if self.source == "custom":
            if not self.custom_pipeline:
                self.get_logger().error(
                    "source=custom pero gst_pipeline esta vacio."
                )
                return None
            return cv2.VideoCapture(
                self.custom_pipeline, cv2.CAP_GSTREAMER
            )

        if self.source == "v4l2":
            cap = cv2.VideoCapture(self.device_index)
            if cap is not None and cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.output_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.output_height)
            return cap

        # nvargus (por defecto).
        #
        # nvarguscamerasrc necesita un EGLDisplay para su FrameConsumer.
        # Si DISPLAY apunta a un X server al que este proceso NO esta
        # autorizado (caso tipico dentro del contenedor: el host exporta
        # DISPLAY=:0 pero sin xhost para 'root'), Argus falla con
        #   "No protocol specified"
        #   "(Argus) Error NotSupported: Failed to initialize EGLDisplay"
        # y la camara nunca entrega un fotograma ("Fallo al leer...").
        # Sin DISPLAY, Argus usa el EGL headless de Tegra y funciona.
        # Este nodo publica por 'appsink', nunca dibuja nada, asi que
        # no perdemos nada quitando DISPLAY.
        if os.environ.pop("DISPLAY", None) is not None:
            self.get_logger().info(
                "DISPLAY desactivado para nvarguscamerasrc (EGL headless)."
            )

        if not self._gstreamer_available():
            if not self._warned_no_gstreamer:
                self.get_logger().error(
                    "Este OpenCV no tiene backend GStreamer: no se puede "
                    "abrir la camara CSI con nvarguscamerasrc. En la "
                    "Jetson usa el contenedor con la pila L4T/NVIDIA, o "
                    "cambia 'source' a 'v4l2'."
                )
                self._warned_no_gstreamer = True
            return None

        pipeline = build_gstreamer_pipeline(
            self.sensor_id,
            self.capture_width,
            self.capture_height,
            self.output_width,
            self.output_height,
            self.framerate,
            self.flip_method,
            self.exposure_time_us,
            self.gain,
        )
        return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    @staticmethod
    def _gstreamer_available():
        try:
            info = cv2.getBuildInformation()
        except Exception:  # noqa: BLE001
            return False
        for line in info.splitlines():
            if "GStreamer" in line:
                return "YES" in line.upper()
        return False

    def _on_open_timer(self):
        if self.capture is not None and self.capture.isOpened():
            return

        if self.capture is not None:
            try:
                self.capture.release()
            except Exception:  # noqa: BLE001
                pass
            self.capture = None

        cap = self._build_capture()
        if cap is not None and cap.isOpened():
            self.capture = cap
            self.get_logger().info("Camara abierta correctamente.")
        else:
            if cap is not None:
                try:
                    cap.release()
                except Exception:  # noqa: BLE001
                    pass
            self.get_logger().warn(
                "No se pudo abrir la camara; reintentando en "
                f"{self.reopen_period:.1f} s."
            )

    # =====================================================================
    # Publicacion de fotogramas
    # =====================================================================

    def _on_read_timer(self):
        if self.capture is None or not self.capture.isOpened():
            return

        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.get_logger().warn(
                "Fallo al leer un fotograma; se reabrira la camara.",
                throttle_duration_sec=2.0,
            )
            try:
                self.capture.release()
            except Exception:  # noqa: BLE001
                pass
            self.capture = None
            return

        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        stamp = self.get_clock().now().to_msg()

        info_msg = self.camera_info
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self.frame_id

        # --- imagen cruda (consumidores LOCALES) -------------------
        if self.publish_raw:
            try:
                image_msg = self.bridge.cv2_to_imgmsg(
                    np.ascontiguousarray(frame), encoding="bgr8"
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"cv_bridge: {exc}")
                return

            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            self.image_pub.publish(image_msg)

        # --- imagen comprimida (consumidores REMOTOS) --------------
        # 960x540 BGR crudo = 1.55 MB por frame: a 15 Hz son 186 Mbit/s,
        # que ahogan cualquier WiFi y meten latencia. En JPEG son ~60 KB
        # (~7 Mbit/s). Imprescindible si el detector corre en otra
        # maquina; ver scripts/tsummit_offboard.sh.
        if self.publish_compressed:
            try:
                ok, buf = cv2.imencode(
                    ".jpg", frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
                )
                if ok:
                    comp = CompressedImage()
                    comp.header.stamp = stamp
                    comp.header.frame_id = self.frame_id
                    comp.format = "jpeg"
                    comp.data = buf.tobytes()
                    self.compressed_pub.publish(comp)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"jpeg: {exc}", throttle_duration_sec=5.0
                )

        self.info_pub.publish(info_msg)

    # =====================================================================
    # Cierre
    # =====================================================================

    def destroy_node(self):
        if self.capture is not None:
            try:
                self.capture.release()
            except Exception:  # noqa: BLE001
                pass
            self.capture = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CsiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
