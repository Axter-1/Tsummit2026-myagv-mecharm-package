#!/usr/bin/env python3
"""Resolucion de posiciones guardadas junto al mapa.

Estas pruebas existen porque el fallo que cubren es caro y silencioso:
cargar el <mapa>.poses.yaml equivocado manda al robot a coordenadas de
otra pista, y eso solo se descubre viendolo arrancar hacia una pared.

Se importa el mission_manager real con las interfaces stubeadas. En la
Jetson (galactic / Python 3.8) home_service_interfaces esta compilado
para humble / Python 3.10 y no se puede importar, pero nada de lo que
se prueba aqui toca esos mensajes.
"""

import math
import os
import sys
import tempfile
import time
import types

import pytest
import yaml


def _import_mission_manager():
    for name, attrs in (
        ("home_service_interfaces", ()),
        (
            "home_service_interfaces.action",
            ("MoveArm", "PickPlace", "ArucoApproach"),
        ),
    ):
        module = types.ModuleType(name)
        for attr in attrs:
            setattr(
                module, attr,
                type(attr, (), {"Goal": type("Goal", (), {})}),
            )
        sys.modules.setdefault(name, module)

    from home_service_mission import mission_manager

    return mission_manager.MissionManager


try:
    MissionManager = _import_mission_manager()
except ImportError as exc:  # rclpy/nav2_msgs ausentes: no es un fallo
    pytest.skip(f"entorno ROS incompleto: {exc}", allow_module_level=True)


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, message):
        self.lines.append(("info", message))

    def warn(self, message):
        self.lines.append(("warn", message))

    def error(self, message):
        self.lines.append(("error", message))


class _Harness:
    """Solo lo que las funciones bajo prueba usan de `self`.

    Instanciar el nodo de verdad levantaria clientes de accion y un
    executor; aqui se toman los metodos sin ceremonia.
    """

    def __init__(self, mission, poses=None):
        self.mission = mission
        self.poses = poses or {}
        self._logger = _Logger()

    def get_logger(self):
        return self._logger


for _name in (
    "_resolve_poses_file", "_default_maps_dir", "_load_poses", "resolve_pose",
    "_resolve_vars", "_var",
):
    setattr(_Harness, _name, getattr(MissionManager, _name))

_Harness._VAR = MissionManager._VAR


POSES = {
    "map": "pista_reto1",
    "frame_id": "map",
    "poses": {
        "start": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        "carga_0": {"x": 1.25, "y": -0.40, "yaw_deg": 90.0},
        "posicion_2": {"x": 2.10, "y": 0.55, "yaw_deg": -45.0},
    },
}


@pytest.fixture
def workspace():
    root = tempfile.mkdtemp()

    maps = os.path.join(root, "maps")
    os.makedirs(maps)

    mission_file = os.path.join(
        root, "src", "home_service_mission", "config", "m.yaml"
    )
    os.makedirs(os.path.dirname(mission_file))
    with open(mission_file, "w", encoding="utf-8") as handle:
        handle.write("mission: {}\n")

    path = os.path.join(maps, "pista_reto1.poses.yaml")
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(POSES, handle)

    return maps, mission_file, path


def test_map_explicito_elige_su_fichero_de_poses(workspace):
    maps, mission_file, path = workspace

    harness = _Harness({"map": "pista_reto1", "maps_dir": maps})

    assert harness._resolve_poses_file(mission_file) == path


def test_sin_map_se_coge_el_mas_reciente(workspace):
    """Es lo que se hace a diario: mapear, guardar poses, correr."""
    maps, mission_file, path = workspace

    time.sleep(0.02)
    nuevo = os.path.join(maps, "otra_pista.poses.yaml")
    with open(nuevo, "w", encoding="utf-8") as handle:
        yaml.safe_dump({"map": "otra_pista", "poses": {}}, handle)

    harness = _Harness({"maps_dir": maps})

    assert harness._resolve_poses_file(mission_file) == nuevo


def test_poses_file_manda_sobre_map(workspace):
    maps, mission_file, _ = workspace

    harness = _Harness({
        "poses_file": "/otro/sitio.yaml",
        "map": "pista_reto1",
        "maps_dir": maps,
    })

    assert harness._resolve_poses_file(mission_file) == "/otro/sitio.yaml"


def test_fichero_ausente_avisa_y_no_revienta(workspace):
    maps, _, _ = workspace

    harness = _Harness({})

    assert harness._load_poses(os.path.join(maps, "no.poses.yaml")) == {}
    assert any(level == "warn" for level, _ in harness._logger.lines)


def test_resolver_una_posicion_por_nombre(workspace):
    _, _, path = workspace

    harness = _Harness({})
    harness.poses = harness._load_poses(path)

    x, y, yaw, origen = harness.resolve_pose({"pose": "carga_0"})

    assert abs(x - 1.25) < 1e-9
    assert abs(y + 0.40) < 1e-9
    assert abs(math.degrees(yaw) - 90.0) < 1e-9
    assert "carga_0" in origen


def test_desplazamiento_relativo_sobre_una_posicion_guardada(workspace):
    """Colocar una variante sin volver a conducir el robot hasta alli."""
    _, _, path = workspace

    harness = _Harness({})
    harness.poses = harness._load_poses(path)

    x, y, yaw, _ = harness.resolve_pose({
        "pose": "carga_0", "dx": 0.10, "dy": -0.05, "dyaw_deg": 15.0,
    })

    assert abs(x - 1.35) < 1e-9
    assert abs(y + 0.45) < 1e-9
    assert abs(math.degrees(yaw) - 105.0) < 1e-6


def test_las_coordenadas_sueltas_siguen_valiendo(workspace):
    """Las misiones anteriores no se rompen."""
    harness = _Harness({})

    x, y, yaw, origen = harness.resolve_pose({
        "x": 0.8, "y": 0.2, "yaw_deg": -90.0,
    })

    assert abs(x - 0.8) < 1e-9
    assert abs(math.degrees(yaw) + 90.0) < 1e-9
    assert "YAML" in origen


def test_una_posicion_ausente_se_detecta_antes_de_mover_el_robot(workspace):
    """La misma comprobacion que hace __init__, con los mismos datos.

    Enterarse a mitad de rutina, con una pieza en la pinza, es el caso
    que esto evita.
    """
    _, _, path = workspace

    harness = _Harness({})
    poses = harness._load_poses(path)

    steps = [
        {"type": "navigate", "pose": "start"},
        {"type": "navigate", "pose": "no_existe"},
        {"type": "grasp", "action": "pick"},
    ]

    faltan = sorted({
        str(step["pose"])
        for step in steps
        if str(step.get("type", "")).strip().lower() == "navigate"
        and "pose" in step
        and str(step["pose"]) not in poses
    })

    assert faltan == ["no_existe"]


# ---------------------------------------------------------------------
# Variables de la mision
# ---------------------------------------------------------------------

def _con_vars(vars_):
    harness = _Harness({})
    harness.vars = dict(vars_)
    return harness


def test_sustituye_la_pieza_en_un_paso():
    """Los ArUco marcan UBICACIONES; que pieza hay en cada una no se
    sabe hasta la pista, asi que va como variable."""
    harness = _con_vars({"pieza_verde": "poste"})

    paso = harness._resolve_vars(
        {"type": "grasp", "action": "pick", "object": "${pieza_verde}"},
        "paso 1",
    )

    assert paso["object"] == "poste"
    assert paso["action"] == "pick"


def test_una_variable_sola_conserva_su_tipo():
    """`altura: "${altura_mm}"` tiene que seguir siendo un entero.

    Devolverlo como texto reventaria el int() del paso.
    """
    harness = _con_vars({"altura_mm": 100})

    assert harness._resolve_vars("${altura_mm}", "x") == 100
    assert isinstance(harness._resolve_vars("${altura_mm}", "x"), int)


def test_variable_dentro_de_un_texto_se_interpola():
    harness = _con_vars({"pieza_azul": "estrella"})

    assert harness._resolve_vars(
        "tomar_${pieza_azul}_del_aruco_1", "x"
    ) == "tomar_estrella_del_aruco_1"


def test_una_variable_sin_declarar_es_un_error_duro():
    """Fallar al cargar es mucho mejor que mandar el brazo a la pieza ''."""
    harness = _con_vars({"pieza_verde": "rueda"})

    with pytest.raises(RuntimeError) as exc:
        harness._resolve_vars({"object": "${pieza_azul}"}, "paso 7")

    assert "pieza_azul" in str(exc.value)
    assert "paso 7" in str(exc.value)


def test_la_sustitucion_baja_por_listas_y_diccionarios():
    harness = _con_vars({"a": "uno", "b": 2})

    resuelto = harness._resolve_vars(
        {"lista": ["${a}", {"anidado": "${b}"}], "intacto": 5.5},
        "x",
    )

    assert resuelto == {"lista": ["uno", {"anidado": 2}], "intacto": 5.5}


def test_los_retos_reales_declaran_las_variables_que_usan():
    """Ningun YAML de reto puede referirse a una variable inexistente.

    Es la comprobacion que evita descubrirlo con el robot en la pista.
    """
    import glob

    raiz = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ficheros = sorted(glob.glob(os.path.join(raiz, "config", "reto*.yaml")))

    assert ficheros, "no se encontro ningun YAML de reto"

    for ruta in ficheros:
        with open(ruta, "r", encoding="utf-8") as handle:
            mission = yaml.safe_load(handle)["mission"]

        harness = _Harness({})
        harness.vars = dict(mission.get("vars", {}) or {})

        for i, paso in enumerate(mission["steps"]):
            harness._resolve_vars(paso, f"{os.path.basename(ruta)} paso {i}")


def test_los_retos_reales_solo_usan_tipos_de_paso_conocidos():
    import glob

    raiz = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    conocidos = {"navigate", "aruco", "arm_pose", "pick", "place", "grasp"}

    for ruta in sorted(glob.glob(os.path.join(raiz, "config", "*.yaml"))):
        with open(ruta, "r", encoding="utf-8") as handle:
            mission = yaml.safe_load(handle)["mission"]

        for paso in mission["steps"]:
            assert paso["type"] in conocidos, (ruta, paso["type"])
