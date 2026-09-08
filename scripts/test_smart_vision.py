#!/usr/bin/env python3
"""Comprueba que el reconocedor local distingue las cuatro piezas.

Dibuja vistas cenitales sinteticas a la escala real de los STL y las pasa
por ReconocedorLocal. No necesita robot, camara ni red: se puede correr
en cualquier sitio para validar un cambio en las firmas geometricas.

    python3 scripts/test_smart_vision.py
    python3 scripts/test_smart_vision.py --guarda /tmp/piezas  # ver PNGs
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from smart_vision import ReconocedorLocal  # noqa: E402

PX_POR_MM = 3.0
LADO = 700
FONDO = (150, 150, 150)          # plataforma gris, sin saturacion
COLOR = {                        # las piezas son de colores saturados
    "engranaje": (40, 90, 200),
    "rueda": (60, 170, 70),
    "poste": (40, 190, 210),
    "estrella": (190, 90, 40),
}


def _lienzo():
    img = np.full((LADO, LADO, 3), FONDO, dtype=np.uint8)
    # Ruido suave: sin el, Canny da contornos irrealmente perfectos.
    ruido = np.random.normal(0, 4, img.shape).astype(np.int16)
    return np.clip(img.astype(np.int16) + ruido, 0, 255).astype(np.uint8)


def _mm(valor):
    return int(round(valor * PX_POR_MM))


def dibuja_engranaje():
    """Disco 152.8 con 20 dientes y agujero central 49.2."""
    img = _lienzo()
    c = LADO // 2
    r_punta, r_raiz = _mm(152.8 / 2), _mm(114.8 / 2)
    puntos = []
    dientes = 20
    for i in range(dientes * 4):
        ang = 2 * math.pi * i / (dientes * 4)
        # Cuadrado por diente: punta, punta, raiz, raiz.
        r = r_punta if (i % 4) in (0, 1) else r_raiz
        puntos.append([c + r * math.cos(ang), c + r * math.sin(ang)])
    cv2.fillPoly(img, [np.array(puntos, dtype=np.int32)], COLOR["engranaje"])
    cv2.circle(img, (c, c), _mm(49.2 / 2), FONDO, -1)
    return img


def dibuja_rueda():
    """Aro 150 con pared radial de 20 -> interior de 110."""
    img = _lienzo()
    c = LADO // 2
    cv2.circle(img, (c, c), _mm(150.0 / 2), COLOR["rueda"], -1)
    cv2.circle(img, (c, c), _mm(110.0 / 2), FONDO, -1)
    return img


def dibuja_poste():
    """Base maciza 110 con el tubo de 20 saliendo del centro."""
    img = _lienzo()
    c = LADO // 2
    cv2.circle(img, (c, c), _mm(110.0 / 2), COLOR["poste"], -1)
    # El tubo NO es un agujero: es material mas claro (le da la luz).
    claro = tuple(min(255, v + 45) for v in COLOR["poste"])
    cv2.circle(img, (c, c), _mm(20.0 / 2), claro, -1)
    return img


def dibuja_estrella():
    """Estrella de 5 puntas, 150 mm punta a punta."""
    img = _lienzo()
    c = LADO // 2
    r_ext, r_int = _mm(150.0 / 2), _mm(150.0 / 2) * 0.42
    puntos = []
    for i in range(10):
        ang = math.pi * i / 5 - math.pi / 2
        r = r_ext if i % 2 == 0 else r_int
        puntos.append([c + r * math.cos(ang), c + r * math.sin(ang)])
    cv2.fillPoly(img, [np.array(puntos, dtype=np.int32)], COLOR["estrella"])
    return img


ESCENAS = {
    "engranaje": dibuja_engranaje,
    "rueda": dibuja_rueda,
    "poste": dibuja_poste,
    "estrella": dibuja_estrella,
}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--guarda", default="", help="directorio para volcar PNGs")
    args = p.parse_args()

    np.random.seed(7)
    reconocedor = ReconocedorLocal()
    aciertos = 0

    print(f"{'esperado':<12} {'obtenido':<12} {'conf':>5}  "
          f"{'sol':>5} {'circ':>5} {'aguj':>6} {'ms':>5}")
    print("-" * 62)

    for esperado, dibuja in ESCENAS.items():
        img = dibuja()
        if args.guarda:
            os.makedirs(args.guarda, exist_ok=True)
            cv2.imwrite(os.path.join(args.guarda, f"{esperado}.png"), img)

        det = reconocedor.detecta(img)
        if det is None:
            print(f"{esperado:<12} {'(nada)':<12}")
            continue
        m = det.metricas
        ok = det.pieza == esperado
        aciertos += ok
        marca = "" if ok else "   <-- FALLO"
        print(
            f"{esperado:<12} {det.pieza:<12} {det.confianza:>5.2f}  "
            f"{m['solidez']:>5.2f} {m['circularidad']:>5.2f} "
            f"{m['frac_agujero']:>6.3f} {reconocedor.ultimo_ms:>5.1f}{marca}"
        )
        if not ok:
            orden = sorted(
                det.puntuaciones.items(), key=lambda kv: -kv[1]
            )
            print("             puntuaciones: " + ", ".join(
                f"{k}={v:.2f}" for k, v in orden
            ))

    print("-" * 62)
    print(f"{aciertos}/{len(ESCENAS)} correctas")
    return 0 if aciertos == len(ESCENAS) else 1


if __name__ == "__main__":
    sys.exit(main())
