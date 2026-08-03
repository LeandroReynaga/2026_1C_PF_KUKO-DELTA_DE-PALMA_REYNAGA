"""
Conversión de píxeles de la imagen a centímetros en el sistema de
referencia del robot.

Es matemática pura, sin cámara ni OpenCV: recibe un punto en píxeles
y devuelve dónde está esa pieza para el robot.

Los dos sistemas no coinciden en nada:

    IMAGEN (OpenCV)                ROBOT
    (0,0) arriba a la izquierda    (0,0) en el centro del robot
    X crece a la derecha           X crece a la derecha
    Y crece hacia ABAJO            Y crece hacia ARRIBA
    unidad: píxeles                unidad: centímetros

Se atan con tres medidas hechas con regla sobre la cinta, que están
en config.py:

  - IMAGE_WIDTH_CM / IMAGE_HEIGHT_CM: cuántos centímetros abarca el
    fotograma ya recortado. Dan la escala cm/píxel.
  - LINE_X_CM: en qué X del robot cae la línea de detección. Es el
    ancla del eje X.
  - IMAGE_BOTTOM_Y_CM: en qué Y del robot cae el borde inferior de
    la imagen. Es el ancla del eje Y.
"""

from __future__ import annotations

from config import (
    IMAGE_BOTTOM_Y_CM,
    IMAGE_HEIGHT_CM,
    IMAGE_WIDTH_CM,
    LINE_X_CM,
)


def pixels_to_robot_cm(
    center: tuple[int, int],
    frame_width: int,
    frame_height: int,
    line_x: int,
) -> tuple[float, float]:
    """Pasa un centro en píxeles a centímetros del robot.

    Se toma como referencia el centro del primer y del último píxel,
    así que los bordes de la imagen caen exactamente sobre las
    medidas cargadas en config.py.
    """

    # Con un solo píxel de ancho no hay escala posible; se evita la
    # división por cero de un recorte degenerado.
    if frame_width < 2 or frame_height < 2:
        return 0.0, 0.0

    x_pixel, y_pixel = center

    cm_per_pixel_x = IMAGE_WIDTH_CM / (frame_width - 1)
    cm_per_pixel_y = IMAGE_HEIGHT_CM / (frame_height - 1)

    # X: se mide cuántos píxeles hay entre la pieza y la línea de
    # detección, y se suma a la posición conocida de esa línea. Una
    # pieza a la izquierda de la línea da diferencia negativa, o sea
    # una X más negativa todavía.
    x_cm = LINE_X_CM + (x_pixel - line_x) * cm_per_pixel_x

    # Y: se mide desde el borde INFERIOR de la imagen hacia arriba.
    # Ahí queda invertido el eje de OpenCV, que crece hacia abajo.
    distance_from_bottom = (frame_height - 1) - y_pixel

    y_cm = IMAGE_BOTTOM_Y_CM + distance_from_bottom * cm_per_pixel_y

    return x_cm, y_cm
