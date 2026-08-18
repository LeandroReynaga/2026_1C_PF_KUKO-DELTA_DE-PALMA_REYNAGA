"""Hilo de vision: camara, deteccion, seguimiento y envio de piezas.

Es el bucle que antes vivia en vision_python/main.py, sin la ventana de
OpenCV: en vez de cv2.imshow() deja el ultimo fotograma anotado en memoria,
codificado a JPEG, para que el servidor lo sirva como MJPEG.

Reutiliza los modulos de vision que ya existen y estan calibrados
(vision_python/): camara, deteccion de color y forma, seguimiento y cruce de
linea. No se copiaron aca a proposito -- son los que andan.
"""

from __future__ import annotations

import statistics
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import cv2

# Los modulos de vision viven en vision_python/, al lado de pc/. La ruta se
# arma desde este archivo y no desde el directorio de trabajo, para que ande
# igual en cualquier maquina y desde cualquier lado que se lo invoque.
RAIZ = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RAIZ / "vision_python"))

import config                                     # noqa: E402
from camera import Camera                         # noqa: E402
from coordinates import pixels_to_robot_cm        # noqa: E402
from detection import detect_objects              # noqa: E402
from line_crossing import LineCrossingDetector, draw_detection_line  # noqa: E402
from tracker import CentroidTracker, TrackedObject                    # noqa: E402

from . import ajustes
from . import protocolo as pr
from .estado import EstadoSistema

COLORES_DIBUJO = {"ROJO": (0, 0, 255), "AZUL": (255, 0, 0), "VERDE": (0, 200, 0)}
COLOR_POR_DEFECTO = (255, 255, 255)

# Muestras de velocidad de cinta que se promedian. Con la mediana de 15
# muestras el ruido de un fotograma suelto no mueve el numero.
MUESTRAS_CINTA = 15


def _dibujar(frame, track: TrackedObject, line_x: int) -> None:
    color = COLORES_DIBUJO.get(track.color, COLOR_POR_DEFECTO)

    # Casco convexo y no contorno crudo: es lo mismo que mira
    # classify_shape(), asi que lo que se ve coincide con lo clasificado.
    cv2.drawContours(frame, [cv2.convexHull(track.contour)], -1, color, 3)

    x, y, _, alto = track.bbox
    cv2.circle(frame, track.center, 6, (0, 255, 0), -1)

    alto_img, ancho_img = frame.shape[:2]
    x_cm, y_cm = pixels_to_robot_cm(track.center, ancho_img, alto_img, line_x)

    cv2.putText(frame, f"ID {track.track_id} | {track.shape} {track.color}",
                (x, max(y - 35, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.putText(frame, f"X:{x_cm:.1f} cm Y:{y_cm:.1f} cm",
                (x, max(y - 12, 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 2)

    if track.crossed_line:
        cv2.putText(frame, "CRUZO LA LINEA", (x, y + alto + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)


class Vision:
    def __init__(self, estado: EstadoSistema, enviar):
        self.estado = estado
        self.enviar = enviar          # callable(str) -> bool, del enlace

        self._parar = threading.Event()
        self._hilo: Optional[threading.Thread] = None
        self._jpeg: Optional[bytes] = None
        self._lock = threading.Lock()

        self.piezas_vistas = 0
        self._camara = None

        # El recorte arranca donde quedo la ultima vez, no en un valor
        # predefinido: es lo que se pidio expresamente para no tener que
        # recentrar la camara en cada arranque.
        self._offset_pedido = int(ajustes.cargar().get("recorte_y_px", 0))

        # Para medir la velocidad de la cinta: ultima posicion conocida de
        # cada pieza seguida, y las ultimas muestras de velocidad.
        self._ultima_pos: dict[int, tuple[float, float]] = {}
        self._muestras: list[float] = []

    def arrancar(self) -> None:
        self._hilo = threading.Thread(target=self._correr, daemon=True, name="vision")
        self._hilo.start()

    def parar(self) -> None:
        self._parar.set()

        if self._hilo:
            self._hilo.join(timeout=3.0)

    def fotograma(self) -> Optional[bytes]:
        with self._lock:
            return self._jpeg

    def mover_recorte(self, delta_px: int) -> int:
        """Corre el area recortada hacia arriba o abajo y lo deja guardado.

        El alto no cambia, solo se desplaza la ventana: la escala pixel->cm
        se mantiene y por lo tanto tambien las coordenadas que se le mandan
        al robot.
        """

        if self._camara is None:
            return self._offset_pedido

        self._offset_pedido = self._camara.mover_recorte(delta_px)
        ajustes.guardar("recorte_y_px", self._offset_pedido)

        return self._offset_pedido

    @property
    def offset_recorte(self) -> int:
        return self._camara.offset_y if self._camara else self._offset_pedido

    # ------------------------------------------------------------------
    def _correr(self) -> None:
        try:
            camara = Camera()
        except Exception as err:                      # noqa: BLE001
            self.estado.consola.append(f"no se pudo abrir la camara: {err}")
            return

        camara.offset_y = self._offset_pedido
        self._camara = camara

        tracker = CentroidTracker()
        cruce = LineCrossingDetector()
        anterior = time.perf_counter()

        try:
            while not self._parar.is_set():
                ok, frame = camara.read()

                if not ok:
                    time.sleep(0.05)
                    continue

                alto, ancho = frame.shape[:2]
                line_x = int(ancho * config.LINE_X_RATIO)

                detecciones, _ = detect_objects(frame)
                seguidas = tracker.update(detecciones)

                self._medir_cinta(seguidas, ancho, alto, line_x)

                for track in cruce.check_crossings(seguidas, line_x):
                    _, y_cm = pixels_to_robot_cm(track.center, ancho, alto, line_x)

                    self.piezas_vistas += 1
                    self.enviar(pr.cmd_pieza(y_cm, track.color, track.shape))

                draw_detection_line(frame, line_x)

                for track in seguidas:
                    _dibujar(frame, track, line_x)

                ahora = time.perf_counter()
                dt = ahora - anterior
                anterior = ahora
                self.estado.fps_camara = 1.0 / dt if dt > 0 else 0.0

                ok, buffer = cv2.imencode(".jpg", frame,
                                          [int(cv2.IMWRITE_JPEG_QUALITY), 80])

                if ok:
                    with self._lock:
                        self._jpeg = buffer.tobytes()
        finally:
            self._camara = None
            camara.release()

    def _medir_cinta(self, seguidas, ancho: int, alto: int, line_x: int) -> None:
        """Velocidad real de la cinta, siguiendo cuanto avanza cada pieza.

        Es el unico chequeo que puede decir si la cinta se trabo: no hay
        sensor en el eje. De paso valida la constante de la que depende toda
        la intercepcion.
        """

        ahora = time.perf_counter()
        vistos = set()

        for track in seguidas:
            x_cm, _ = pixels_to_robot_cm(track.center, ancho, alto, line_x)
            vistos.add(track.track_id)

            previo = self._ultima_pos.get(track.track_id)
            self._ultima_pos[track.track_id] = (ahora, x_cm)

            if previo is None:
                continue

            dt = ahora - previo[0]

            # Ventana chica: por debajo de 0,2 s el ruido del centroide
            # domina sobre el avance real.
            if dt < 0.2:
                self._ultima_pos[track.track_id] = previo
                continue

            self._muestras.append((x_cm - previo[1]) / dt)
            self._muestras[:] = self._muestras[-MUESTRAS_CINTA:]

        for perdido in set(self._ultima_pos) - vistos:
            del self._ultima_pos[perdido]

        self.estado.cinta_medida = (statistics.median(self._muestras)
                                    if len(self._muestras) >= 5 else None)
