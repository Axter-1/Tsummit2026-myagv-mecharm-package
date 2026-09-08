#!/usr/bin/env python3
"""Vision para el agarre inteligente. SOLO PORTATIL.

Dos capas, deliberadamente separadas por coste:

  ReconocedorLocal   OpenCV puro, sin red. MEDIDO: 63 ms de media en la
                     Jetson a 700x700 (peor caso; en el portatil es
                     bastante menos). Clasifica las cuatro
                     piezas por GEOMETRIA (solidez, circularidad, tamaño
                     del agujero central, vertices), no por color: los
                     colores no estan medidos y la geometria sale de los
                     STL. Ademas mide el centroide para corregir la pose.
                     Validado 4/4 por scripts/test_smart_vision.py.

  VerificadorGemini  Una llamada acotada a gemini-3.8-flash. NUNCA en un
                     bucle de control. Se usa solo en los puntos de
                     decision donde lo local no basta, con timeout duro y
                     caida a la decision local si tarda o falla.

Nada de esto toca la Jetson: consume la imagen que el robot YA publica.

Se puede importar sin ROS (para probar la vision con imagenes sueltas).
"""

import base64
import hashlib
import json
import math
import os
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np


# =====================================================================
#  FIRMAS GEOMETRICAS DE LAS CUATRO PIEZAS
# ---------------------------------------------------------------------
#  Derivadas de los STL de Contexto/, vistas DESDE ARRIBA (que es como
#  las ve la camara cuando el brazo va a bajar sobre ellas).
#
#  El discriminante fuerte es la SOLIDEZ (area / area del casco convexo):
#  una estrella tiene concavidades profundas, un engranaje solo dientes
#  someros, y un disco o un aro son casi convexos. La circularidad
#  (4*pi*A / P^2) separa el disco del aro. El agujero central separa el
#  engranaje (bore 49.2) y el aro del disco macizo del poste.
#
#  Los rangos son anchos a proposito: sirven para PUNTUAR, no para
#  aceptar o rechazar. La decision final pondera todas las senales.
# =====================================================================

#  frac_agujero (area del agujero / area de la pieza) es lo que separa
#  limpiamente el engranaje del aro, que en solidez y circularidad se
#  solapan. De los STL: engranaje (49.2/152.8)^2 = 0.10; rueda, con
#  pared radial de 20 sobre un aro de 150, (110/150)^2 = 0.54. Cinco
#  veces de diferencia: no hay ambiguedad posible.

FIRMAS = {
    "estrella": {
        # Star 150 mm punta a punta. Concavidades profundas entre puntas.
        "solidez": (0.40, 0.72),
        "circularidad": (0.25, 0.62),
        "agujero": False,
        "frac_agujero": (0.00, 0.10),
        "vertices": (8, 14),
        "nota": "puntas -> solidez baja, es la firma mas distinguible",
    },
    "engranaje": {
        # Disco 152.8 con dientes; bore 49.2 -> agujero central pequeño.
        # circularidad BAJA pese a ser redondo: los 20 dientes alargan el
        # perimetro y 4*pi*A/P^2 se hunde. Medido 0.18 en la prueba
        # sintetica; el rango 0.62-0.90 que puse a ojo estaba mal.
        "solidez": (0.80, 0.96),
        "circularidad": (0.10, 0.55),
        "agujero": True,
        "frac_agujero": (0.04, 0.26),
        "vertices": (10, 40),
        "nota": "dientes -> solidez alta pero perimetro rugoso",
    },
    "rueda": {
        # Aro 150 de pared fina. El agujero se come media pieza.
        "solidez": (0.85, 1.00),
        "circularidad": (0.72, 1.00),
        "agujero": True,
        "frac_agujero": (0.32, 0.80),
        "vertices": (6, 30),
        "nota": "aro -> agujero central muy grande respecto al exterior",
    },
    "poste": {
        # Base 110 maciza, tubo 20 saliendo del centro (3% del area).
        "solidez": (0.90, 1.00),
        "circularidad": (0.80, 1.00),
        "agujero": False,
        "frac_agujero": (0.00, 0.06),
        "vertices": (6, 30),
        "nota": "disco macizo, el tubo se ve como un circulo pequeño dentro",
    },
}

PIEZAS = tuple(FIRMAS)


def _puntua_rango(valor, rango, margen=0.15):
    """1.0 dentro del rango, cayendo suave fuera. Nunca negativo."""
    bajo, alto = rango
    if bajo <= valor <= alto:
        return 1.0
    ancho = max(alto - bajo, 1e-6)
    fuera = (bajo - valor) if valor < bajo else (valor - alto)
    return max(0.0, 1.0 - fuera / (ancho * margen * 10.0))


class Deteccion:
    """Lo que el reconocedor local encontro en un fotograma."""

    def __init__(self, pieza, confianza, centro_px, area_px,
                 metricas, puntuaciones, contorno=None):
        self.pieza = pieza
        self.confianza = confianza
        self.centro_px = centro_px          # (cx, cy) en pixeles
        self.area_px = area_px
        self.metricas = metricas            # solidez, circularidad, ...
        self.puntuaciones = puntuaciones    # por pieza, para depurar
        self.contorno = contorno

    def __repr__(self):
        m = self.metricas
        return (
            f"<{self.pieza} conf={self.confianza:.2f} "
            f"centro={self.centro_px} sol={m['solidez']:.2f} "
            f"circ={m['circularidad']:.2f} agujero={m['agujero']}>"
        )


class ReconocedorLocal:
    """Clasifica la pieza y mide su centro. OpenCV puro, sin red."""

    def __init__(self, area_min_px=1200, area_max_frac=0.55, debug=False):
        self.area_min_px = int(area_min_px)
        self.area_max_frac = float(area_max_frac)
        self.debug = bool(debug)
        self.ultimo_ms = 0.0

    # -- segmentacion ------------------------------------------------

    def _contornos(self, bgr):
        """Contornos candidatos: la pieza contra la plataforma.

        Se combinan dos vias porque ninguna sola aguanta bien el brillo
        de la pista: bordes (Canny sobre gris ecualizado) y saturacion
        (las piezas impresas son de color, la plataforma no).
        """
        gris = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gris = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gris)
        gris = cv2.GaussianBlur(gris, (5, 5), 0)

        bordes = cv2.Canny(gris, 50, 150)
        bordes = cv2.dilate(bordes, np.ones((3, 3), np.uint8), iterations=2)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        sat = cv2.threshold(
            hsv[:, :, 1], 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )[1]
        sat = cv2.morphologyEx(sat, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

        mascara = cv2.bitwise_or(bordes, sat)
        mascara = cv2.morphologyEx(
            mascara, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8)
        )

        contornos, jerarquia = cv2.findContours(
            mascara, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )
        return contornos, jerarquia

    # -- metricas de forma -------------------------------------------

    def _metricas(self, contorno, jerarquia, indice, contornos, area_img):
        area = cv2.contourArea(contorno)
        perimetro = cv2.arcLength(contorno, True)
        if area <= 0 or perimetro <= 0:
            return None

        casco = cv2.convexHull(contorno)
        area_casco = cv2.contourArea(casco)
        solidez = area / area_casco if area_casco > 0 else 0.0
        circularidad = 4.0 * math.pi * area / (perimetro * perimetro)

        aprox = cv2.approxPolyDP(contorno, 0.02 * perimetro, True)
        vertices = len(aprox)

        # Agujero: un hijo en la jerarquia CCOMP con area apreciable.
        agujero = False
        frac_agujero = 0.0
        if jerarquia is not None:
            hijo = jerarquia[0][indice][2]
            while hijo != -1:
                area_hijo = cv2.contourArea(contornos[hijo])
                if area_hijo > 0.04 * area:
                    agujero = True
                    frac_agujero = max(frac_agujero, area_hijo / area)
                hijo = jerarquia[0][hijo][0]

        momentos = cv2.moments(contorno)
        if momentos["m00"] == 0:
            return None
        cx = momentos["m10"] / momentos["m00"]
        cy = momentos["m01"] / momentos["m00"]

        return {
            "area": area,
            "area_frac": area / area_img,
            "solidez": solidez,
            "circularidad": circularidad,
            "vertices": vertices,
            "agujero": agujero,
            "frac_agujero": frac_agujero,
            "centro": (cx, cy),
        }

    # -- clasificacion -----------------------------------------------

    def _clasifica(self, m):
        puntuaciones = {}
        for pieza, firma in FIRMAS.items():
            s = _puntua_rango(m["solidez"], firma["solidez"])
            c = _puntua_rango(m["circularidad"], firma["circularidad"])
            v = _puntua_rango(float(m["vertices"]), firma["vertices"])
            # El agujero es una senal binaria fuerte: si la firma lo pide
            # y no esta (o al reves), penaliza de verdad.
            a = 1.0 if m["agujero"] == firma["agujero"] else 0.35
            # ...y su TAMAÑO relativo es lo que separa engranaje de aro.
            f = _puntua_rango(m["frac_agujero"], firma["frac_agujero"])
            # Pesos: solidez separa la estrella, frac_agujero separa
            # engranaje de rueda. Son los dos ejes que de verdad deciden.
            puntuaciones[pieza] = (
                2.0 * s + c + 0.5 * v + a + 2.0 * f
            ) / 6.5
        return puntuaciones

    # -- API ---------------------------------------------------------

    def detecta(self, bgr, roi=None):
        """Devuelve la mejor Deteccion del fotograma, o None.

        roi: (x, y, w, h) para acotar la busqueda a donde la aproximacion
        dejo la pieza. Recorta ruido y acelera.
        """
        t0 = time.perf_counter()
        dx = dy = 0
        if roi is not None:
            x, y, w, h = (int(v) for v in roi)
            x = max(0, x); y = max(0, y)
            w = min(w, bgr.shape[1] - x); h = min(h, bgr.shape[0] - y)
            if w > 20 and h > 20:
                bgr = bgr[y:y + h, x:x + w]
                dx, dy = x, y

        area_img = float(bgr.shape[0] * bgr.shape[1])
        contornos, jerarquia = self._contornos(bgr)

        mejor = None
        for indice, contorno in enumerate(contornos):
            # Solo contornos externos en CCOMP (padre == -1).
            if jerarquia is not None and jerarquia[0][indice][3] != -1:
                continue
            area = cv2.contourArea(contorno)
            if area < self.area_min_px or area > self.area_max_frac * area_img:
                continue
            m = self._metricas(contorno, jerarquia, indice, contornos, area_img)
            if m is None:
                continue
            puntuaciones = self._clasifica(m)
            pieza = max(puntuaciones, key=puntuaciones.get)
            confianza = puntuaciones[pieza]
            # Preferimos el candidato mas grande con buena confianza:
            # la pieza llena el encuadre cuando el brazo va a por ella.
            merito = confianza * (0.5 + 0.5 * min(1.0, m["area_frac"] / 0.25))
            if mejor is None or merito > mejor[0]:
                cx, cy = m["centro"]
                mejor = (
                    merito,
                    Deteccion(
                        pieza, confianza, (cx + dx, cy + dy), area,
                        m, puntuaciones, contorno,
                    ),
                )

        self.ultimo_ms = (time.perf_counter() - t0) * 1000.0
        return mejor[1] if mejor else None


# =====================================================================
#  VERIFICADOR GEMINI
# ---------------------------------------------------------------------
#  Reglas que el codigo hace cumplir, no solo recomienda:
#
#    * timeout DURO por llamada (por defecto 2.5 s)
#    * presupuesto de llamadas por agarre (por defecto 2)
#    * cache por hash de imagen: la misma escena no se pregunta dos veces
#    * cualquier fallo -> None, y quien llama sigue con la decision local
#
#  Se usa urllib y no el SDK a proposito: cero dependencias nuevas y
#  control exacto del timeout, que es lo unico que importa aqui.
# =====================================================================

ENDPOINT = (
    "https://generativelanguage.googleapis.com/v1beta/"
    "models/{modelo}:generateContent?key={clave}"
)

ESQUEMA = {
    "type": "OBJECT",
    "properties": {
        "pieza": {"type": "STRING", "enum": list(PIEZAS) + ["ninguna"]},
        "confianza": {"type": "NUMBER"},
        "sujeta": {"type": "BOOLEAN"},
        "motivo": {"type": "STRING"},
    },
    "required": ["pieza", "confianza", "motivo"],
}

DESCRIPCION_PIEZAS = (
    "engranaje: disco dentado de 15 cm con un agujero central de 5 cm.\n"
    "rueda: aro/circunferencia de 15 cm de pared fina, sobre un pie "
    "rectangular.\n"
    "poste: disco base de 11 cm con un tubo vertical de 2 cm saliendo "
    "del centro.\n"
    "estrella: estrella plana azul de unos 15 cm de punta a punta.\n"
)


class VerificadorGemini:

    def __init__(self, clave=None, modelo="gemini-3.8-flash",
                 timeout=8.0, presupuesto=2, debug=False,
                 modelo_reserva="gemini-3.5-flash-lite"):
        self.clave = clave or os.environ.get("GEMINI_API_KEY", "")
        self.modelo = modelo
        # Reserva para cuando el principal devuelve 503. Medido: a
        # gemini-3.8-flash le pasa a menudo.
        self.modelo_reserva = modelo_reserva
        # 8 s y no 2.5: MEDIDO, las respuestas buenas tardan 1.5-8 s. Un
        # timeout de 2.5 mataba casi todas las llamadas correctas. La
        # latencia se esconde lanzando la consulta en segundo plano
        # mientras el brazo hace un movimiento que iba a hacer igual.
        self.timeout = float(timeout)
        self.presupuesto_inicial = int(presupuesto)
        self.presupuesto = int(presupuesto)
        self.debug = bool(debug)
        self.admite_thinking = True
        self.modelo_usado = modelo
        self._cache = {}
        self.llamadas = 0
        self.fallos = 0
        self.ultimo_ms = 0.0

    @property
    def disponible(self):
        return bool(self.clave)

    def _censura(self, texto):
        """La clave viaja en la URL, asi que puede colarse en el texto de
        una excepcion y de ahi a un log. Aqui se tapa siempre."""
        texto = str(texto)
        if self.clave:
            texto = texto.replace(self.clave, "***CLAVE***")
        return texto

    def reinicia_presupuesto(self):
        self.presupuesto = self.presupuesto_inicial

    def _jpeg(self, bgr, lado_max=640, calidad=80):
        """Reescala y comprime. Menos pixeles = menos latencia de subida."""
        h, w = bgr.shape[:2]
        escala = min(1.0, lado_max / float(max(h, w)))
        if escala < 1.0:
            bgr = cv2.resize(
                bgr, (int(w * escala), int(h * escala)),
                interpolation=cv2.INTER_AREA,
            )
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, calidad])
        if not ok:
            raise RuntimeError("no se pudo codificar el JPEG")
        return buf.tobytes()

    def _cuerpo(self, jpeg, prompt, sin_thinking):
        gc = {
            # 256 y no 64: la respuesta es corta, pero si el modelo razona
            # esos tokens salen del MISMO presupuesto. Con 120 el thinking
            # se comia 114 y la respuesta llegaba truncada
            # (finishReason MAX_TOKENS) -> TODAS las llamadas fallaban.
            "maxOutputTokens": 256,
            "temperature": 0.0,
            "responseMimeType": "application/json",
            "responseSchema": ESQUEMA,
        }
        if sin_thinking:
            # Apagar el razonamiento quita 1.5-3 s de latencia y para
            # clasificar una pieza no aporta nada. OJO: no todas las
            # familias lo aceptan -- gemini-3.5-flash-lite devuelve 400
            # con este campo. _pide reintenta sin el.
            gc["thinkingConfig"] = {"thinkingBudget": 0}
        return {
            "contents": [{
                "parts": [
                    {"text": prompt},
                    {"inline_data": {
                        "mime_type": "image/jpeg",
                        "data": base64.b64encode(jpeg).decode("ascii"),
                    }},
                ],
            }],
            "generationConfig": gc,
        }

    def _envia(self, cuerpo, modelo):
        url = ENDPOINT.format(modelo=modelo, clave=self.clave)
        peticion = urllib.request.Request(
            url,
            data=json.dumps(cuerpo).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(peticion, timeout=self.timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def _pide(self, jpeg, prompt):
        """Una consulta, con dos adaptaciones aprendidas midiendo:

        400 INVALID_ARGUMENT -> el modelo no admite thinkingConfig; se
                                repite sin el y se recuerda.
        503 UNAVAILABLE      -> modelo saturado (le pasa a 3.8-flash con
                                frecuencia); se pasa al modelo de reserva.
        """
        t0 = time.perf_counter()
        intentos = [(self.modelo, self.admite_thinking)]
        if self.modelo_reserva and self.modelo_reserva != self.modelo:
            intentos.append((self.modelo_reserva, False))

        ultimo_error = None
        for modelo, sin_thinking in intentos:
            try:
                datos = self._envia(
                    self._cuerpo(jpeg, prompt, sin_thinking), modelo
                )
            except urllib.error.HTTPError as exc:
                ultimo_error = exc
                if exc.code == 400 and sin_thinking:
                    # Este modelo no acepta thinkingConfig: sin el y a
                    # recordarlo para no repetir el error toda la sesion.
                    self.admite_thinking = False
                    try:
                        datos = self._envia(
                            self._cuerpo(jpeg, prompt, False), modelo
                        )
                    except urllib.error.HTTPError as exc2:
                        ultimo_error = exc2
                        continue
                elif exc.code in (429, 500, 502, 503):
                    continue          # saturado: probar el de reserva
                else:
                    raise
            self.ultimo_ms = (time.perf_counter() - t0) * 1000.0
            self.modelo_usado = modelo
            texto = datos["candidates"][0]["content"]["parts"][0]["text"]
            return json.loads(texto)

        self.ultimo_ms = (time.perf_counter() - t0) * 1000.0
        raise ultimo_error if ultimo_error else RuntimeError("sin respuesta")

    def _consulta(self, bgr, prompt, etiqueta):
        if not self.disponible:
            return None
        if self.presupuesto <= 0:
            if self.debug:
                print(f"[gemini] {etiqueta}: presupuesto agotado, se omite")
            return None
        try:
            jpeg = self._jpeg(bgr)
        except Exception as exc:  # noqa: BLE001
            self.fallos += 1
            return None

        clave_cache = hashlib.md5(jpeg + etiqueta.encode()).hexdigest()
        if clave_cache in self._cache:
            return self._cache[clave_cache]

        self.presupuesto -= 1
        self.llamadas += 1
        try:
            respuesta = self._pide(jpeg, prompt)
        except (urllib.error.URLError, OSError, KeyError, ValueError,
                json.JSONDecodeError) as exc:
            self.fallos += 1
            if self.debug:
                print(
                    f"[gemini] {etiqueta} FALLO "
                    f"({type(exc).__name__}: {self._censura(exc)})"
                )
            return None
        self._cache[clave_cache] = respuesta
        if self.debug:
            print(f"[gemini] {etiqueta} {self.ultimo_ms:.0f} ms -> {respuesta}")
        return respuesta

    # -- los dos unicos puntos de decision donde se usa ---------------

    def identifica(self, bgr, candidata_local=None):
        """Que pieza hay. Se llama solo si lo local duda o no ve ArUco."""
        pista = ""
        if candidata_local:
            pista = (
                f"\nUn clasificador geometrico local dice '{candidata_local}' "
                "pero con poca confianza. Confirmalo o corrigelo."
            )
        prompt = (
            "Eres el sistema de vision de un robot de competicion. En la "
            "imagen hay UNA pieza impresa en 3D sobre una plataforma.\n\n"
            f"{DESCRIPCION_PIEZAS}"
            f"{pista}\n\n"
            "Responde que pieza es y con que confianza (0 a 1). Si no ves "
            "ninguna de las cuatro, responde 'ninguna'. Se breve."
        )
        return self._consulta(bgr, prompt, "identifica")

    def verifica_agarre(self, bgr, pieza):
        """Tras cerrar la pinza: esta la pieza sujeta de verdad?"""
        prompt = (
            "Imagen de la pinza de un brazo robotico que acaba de intentar "
            f"agarrar la pieza '{pieza}'.\n\n"
            "Responde en 'sujeta': true si la pieza esta claramente "
            "atrapada entre las mordazas, false si la pinza se cerro al "
            "aire, la pieza se cayo, o quedo solo rozada. En 'motivo', "
            "una frase corta con lo que ves."
        )
        return self._consulta(bgr, prompt, "verifica_agarre")

    # -- version asincrona: la latencia se esconde tras el movimiento --

    def lanza(self, metodo, *args):
        """Arranca una consulta en segundo plano y devuelve un asa.

        Asi es como esta latencia deja de importar: se lanza la pregunta
        y el brazo se pone en marcha hacia una pose que iba a visitar de
        todos modos (home, intermedio). Para cuando hace falta la
        respuesta, ya llego. Si no llego, recoge() devuelve None y se
        sigue con la decision local.

            asa = v.lanza(v.identifica, imagen)
            ... mover el brazo ...
            r = asa.recoge()        # no bloquea mas de lo que quede
        """
        resultado = {}

        def trabajo():
            try:
                resultado["valor"] = metodo(*args)
            except Exception as exc:  # noqa: BLE001
                resultado["valor"] = None
                resultado["error"] = exc

        hilo = threading.Thread(target=trabajo, daemon=True)
        hilo.start()

        class Asa:
            def __init__(self, hilo, resultado, verificador):
                self._hilo = hilo
                self._resultado = resultado
                self._v = verificador

            def listo(self):
                return not self._hilo.is_alive()

            def recoge(self, espera_max=None):
                limite = (
                    self._v.timeout + 1.0 if espera_max is None else espera_max
                )
                self._hilo.join(timeout=limite)
                if self._hilo.is_alive():
                    self._v.fallos += 1
                    return None
                return self._resultado.get("valor")

        return Asa(hilo, resultado, self)

    def resumen(self):
        return (
            f"gemini: {self.llamadas} llamadas ({self.modelo_usado}), "
            f"{self.fallos} fallos, ultima {self.ultimo_ms:.0f} ms"
        )


# =====================================================================
#  FUSION DE SENALES
# =====================================================================

def decide_pieza(det_local, pieza_aruco, verificador=None, imagen=None,
                 umbral_confianza=0.70, debug=False, respuesta_ia=None):
    """Combina ArUco, geometria local y (si hace falta) Gemini.

    Devuelve (pieza, confianza, explicacion). La explicacion se guarda en
    el log: cuando algo salga mal en pista, esto dice POR QUE se eligio.

    Orden deliberado:
      1. ArUco + local coinciden      -> maxima confianza, sin red
      2. Solo ArUco                   -> se acepta (el ID es un dato duro)
      3. Solo local y seguro          -> se acepta
      4. Discrepan, o local dudosa    -> se pregunta a Gemini
      5. Todo falla                   -> None, el que llama aborta
    """
    pieza_local = det_local.pieza if det_local else None
    conf_local = det_local.confianza if det_local else 0.0

    if pieza_aruco and pieza_local and pieza_aruco == pieza_local:
        return pieza_aruco, 0.99, (
            f"ArUco y geometria coinciden en '{pieza_aruco}' "
            f"(conf. local {conf_local:.2f})"
        )

    if pieza_aruco and not pieza_local:
        return pieza_aruco, 0.85, (
            f"solo ArUco -> '{pieza_aruco}'; la geometria no vio la pieza"
        )

    if pieza_local and not pieza_aruco and conf_local >= umbral_confianza:
        return pieza_local, conf_local, (
            f"sin ArUco; geometria segura -> '{pieza_local}' ({conf_local:.2f})"
        )

    # Aqui es donde la IA aporta algo que lo local no puede: discrepancia
    # o duda. Es UNA llamada, acotada, y si falla seguimos igual.
    # respuesta_ia permite pasar una respuesta ya obtenida en segundo
    # plano (ver VerificadorGemini.lanza): asi la latencia se solapa con
    # el movimiento del brazo en vez de sumarse al tiempo de ciclo.
    respuesta = respuesta_ia
    if respuesta is None and verificador is not None and \
            verificador.disponible and imagen is not None:
        respuesta = verificador.identifica(imagen, pieza_local)
    if respuesta is not None:
        if respuesta.get("pieza") in PIEZAS:
            pieza_ia = respuesta["pieza"]
            conf_ia = float(respuesta.get("confianza", 0.0))
            motivo = respuesta.get("motivo", "")
            if pieza_aruco and pieza_ia == pieza_aruco:
                return pieza_aruco, 0.95, (
                    f"discrepancia resuelta: Gemini confirma el ArUco "
                    f"'{pieza_aruco}' frente a la geometria '{pieza_local}'. "
                    f"{motivo}"
                )
            return pieza_ia, max(conf_ia, 0.75), (
                f"decidido por Gemini -> '{pieza_ia}' ({conf_ia:.2f}); "
                f"ArUco={pieza_aruco} geometria={pieza_local}. {motivo}"
            )

    if pieza_aruco:
        return pieza_aruco, 0.60, (
            f"caida a ArUco '{pieza_aruco}' sin confirmar "
            f"(geometria={pieza_local}, IA no disponible o sin respuesta)"
        )
    if pieza_local:
        return pieza_local, conf_local, (
            f"caida a geometria '{pieza_local}' ({conf_local:.2f}), sin ArUco"
        )
    return None, 0.0, "ninguna senal identifico la pieza"


def decodifica(mensaje_comprimido):
    """CompressedImage -> BGR, sin cv_bridge (una dependencia menos)."""
    datos = np.frombuffer(mensaje_comprimido.data, dtype=np.uint8)
    return cv2.imdecode(datos, cv2.IMREAD_COLOR)
