from __future__ import annotations

import time

import cv2

from config import (
    LINE_DIRECTION,
    ROI_X_MAX_RATIO,
    ROI_X_MIN_RATIO,
)
from tracker import TrackedObject


class LineCrossingDetector:
    """Detecta el cruce de una línea horizontal."""

    def check_crossings(
        self,
        tracks: list[TrackedObject],
        line_y: int,
    ) -> list[TrackedObject]:
        """
        Comprueba si los objetos cruzaron la línea horizontal.

        En una imagen OpenCV:
        - Y aumenta hacia abajo.
        - Y disminuye cuando una pieza sube.
        """

        crossed_objects: list[TrackedObject] = []

        for track in tracks:
            # La misma pieza no debe generar dos eventos.
            if track.crossed_line:
                continue

            previous_y = track.previous_center[1]
            current_y = track.center[1]

            crossed = False

            # Cinta desde abajo hacia arriba.
            # La coordenada Y disminuye.
            if LINE_DIRECTION == "BOTTOM_TO_TOP":
                crossed = (
                    previous_y > line_y
                    and current_y <= line_y
                )

            # Cinta desde arriba hacia abajo.
            # La coordenada Y aumenta.
            elif LINE_DIRECTION == "TOP_TO_BOTTOM":
                crossed = (
                    previous_y < line_y
                    and current_y >= line_y
                )

            if crossed:
                track.crossed_line = True
                track.crossing_time = time.monotonic()

                crossed_objects.append(track)

        return crossed_objects


def draw_detection_line(
    frame: object,
    line_y: int,
) -> None:
    """Dibuja la línea horizontal sobre la zona de la cinta."""

    frame_height, frame_width = frame.shape[:2]

    # Límites horizontales de la región de interés.
    roi_x_min = int(
        frame_width * ROI_X_MIN_RATIO
    )

    roi_x_max = int(
        frame_width * ROI_X_MAX_RATIO
    )

    # Línea horizontal.
    cv2.line(
        frame,
        (roi_x_min, line_y),
        (roi_x_max, line_y),
        (0, 255, 255),
        3,
    )

    # Texto ubicado encima de la línea.
    text_y = max(line_y - 12, 25)

    cv2.putText(
        frame,
        "LINEA DE DETECCION",
        (roi_x_min + 10, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
    )