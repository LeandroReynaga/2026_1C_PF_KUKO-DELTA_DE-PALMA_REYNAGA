from __future__ import annotations

import time

import cv2

from config import LINE_DIRECTION
from tracker import TrackedObject


class LineCrossingDetector:
    """Detecta el cruce de una línea vertical."""

    def check_crossings(
        self,
        tracks: list[TrackedObject],
        line_x: int,
    ) -> list[TrackedObject]:
        """
        Comprueba si los objetos cruzaron la línea vertical.

        La cinta se ve horizontal en pantalla, así que las piezas
        avanzan en X:
        - X aumenta cuando la pieza va hacia la derecha.
        - X disminuye cuando la pieza va hacia la izquierda.
        """

        crossed_objects: list[TrackedObject] = []

        for track in tracks:
            # La misma pieza no debe generar dos eventos.
            if track.crossed_line:
                continue

            previous_x = track.previous_center[0]
            current_x = track.center[0]

            crossed = False

            # Cinta de izquierda a derecha.
            # La coordenada X aumenta.
            if LINE_DIRECTION == "LEFT_TO_RIGHT":
                crossed = (
                    previous_x < line_x
                    and current_x >= line_x
                )

            # Cinta de derecha a izquierda.
            # La coordenada X disminuye.
            elif LINE_DIRECTION == "RIGHT_TO_LEFT":
                crossed = (
                    previous_x > line_x
                    and current_x <= line_x
                )

            if crossed:
                track.crossed_line = True
                track.crossing_time = time.monotonic()

                crossed_objects.append(track)

        return crossed_objects


def draw_text_right_of_line(
    frame: object,
    text: str,
    line_x: int,
    text_y: int,
    color: tuple[int, int, int],
    font_scale: float = 0.65,
    thickness: int = 2,
) -> None:
    """Escribe un texto en la franja que queda a la derecha de la línea.

    El espacio disponible depende de LINE_X_RATIO, así que no se
    puede fijar un tamaño de letra de antemano: si el texto no
    entra entre la línea y el borde derecho, se achica hasta que
    entre. Así nunca se sale del cuadro ni queda pisando la línea,
    se mueva donde se mueva.
    """

    margin_from_line = 12
    margin_from_edge = 8

    text_x = line_x + margin_from_line

    available_width = (
        frame.shape[1] - text_x - margin_from_edge
    )

    # La línea quedó tan a la derecha que no hay franja donde
    # escribir; se omite el texto antes que dibujar algo cortado.
    if available_width <= 0:
        return

    (text_width, _), _ = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        thickness,
    )

    if text_width > available_width:
        font_scale = font_scale * available_width / text_width

        # Con la letra chica, el grosor 2 empasta los trazos y se
        # vuelve ilegible.
        if font_scale < 0.45:
            thickness = 1

    cv2.putText(
        frame,
        text,
        (text_x, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        color,
        thickness,
    )


def draw_detection_line(
    frame: object,
    line_x: int,
) -> None:
    """Dibuja la línea vertical sobre la zona de la cinta."""

    frame_height = frame.shape[0]

    # El fotograma ya viene recortado a la cinta, así que la línea
    # cruza todo el alto disponible.
    cv2.line(
        frame,
        (line_x, 0),
        (line_x, frame_height),
        (0, 255, 255),
        3,
    )

    draw_text_right_of_line(
        frame,
        "LINEA DE DETECCION",
        line_x,
        frame_height - 15,
        (0, 255, 255),
    )
