"""
Herramienta de calibración interactiva del recorte de la cinta.

Muestra la imagen COMPLETA (ya rotada, pero sin recortar) con el
recorte dibujado encima, y cuatro sliders para moverlo en vivo. La
ventana "recorte" muestra el resultado, o sea exactamente lo que va
a ver main.py.

Uso:
    python crop_calibrator.py

Controles:
    s        -> imprimir en consola los CROP_*_RATIO actuales,
                listos para copiar y pegar en config.py
    q / ESC  -> salir
"""

from __future__ import annotations

import cv2

from camera import BACKEND_CODES, ROTATION_CODES
from config import (
    CAMERA_AUTO_EXPOSURE,
    CAMERA_BACKEND,
    CAMERA_EXPOSURE,
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_INDEX,
    CAMERA_ROTATION,
    CAMERA_USE_MJPG,
    CAMERA_WIDTH,
    CROP_X_MAX_RATIO,
    CROP_X_MIN_RATIO,
    CROP_Y_MAX_RATIO,
    CROP_Y_MIN_RATIO,
    LINE_X_RATIO,
)

WINDOW_FULL = "Calibracion recorte - imagen completa"
WINDOW_CROP = "Calibracion recorte - resultado"

# Los sliders de OpenCV son enteros, así que se trabaja en milésimas
# (0 a 1000) y se divide al leer.
SCALE = 1000


def nothing(_value: int) -> None:
    pass


def main() -> None:
    # Se abre la cámara a mano en vez de usar Camera: acá hace falta
    # la imagen entera, justamente para elegir dónde recortarla.
    capture = cv2.VideoCapture(
        CAMERA_INDEX,
        BACKEND_CODES.get(CAMERA_BACKEND, cv2.CAP_ANY),
    )

    if not capture.isOpened():
        raise RuntimeError(
            "No se pudo abrir la camara. Revisa CAMERA_INDEX en config.py."
        )

    if CAMERA_USE_MJPG:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    capture.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

    # Misma exposición que main.py: si acá se viera con la exposición
    # automática, la imagen saldría quemada y sería imposible ver
    # dónde termina la cinta.
    if CAMERA_AUTO_EXPOSURE:
        capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
    else:
        capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        capture.set(cv2.CAP_PROP_EXPOSURE, CAMERA_EXPOSURE)

    cv2.namedWindow(WINDOW_FULL, cv2.WINDOW_NORMAL)

    cv2.createTrackbar(
        "X min", WINDOW_FULL, int(CROP_X_MIN_RATIO * SCALE), SCALE, nothing
    )
    cv2.createTrackbar(
        "X max", WINDOW_FULL, int(CROP_X_MAX_RATIO * SCALE), SCALE, nothing
    )
    cv2.createTrackbar(
        "Y min", WINDOW_FULL, int(CROP_Y_MIN_RATIO * SCALE), SCALE, nothing
    )
    cv2.createTrackbar(
        "Y max", WINDOW_FULL, int(CROP_Y_MAX_RATIO * SCALE), SCALE, nothing
    )

    print("Calibrador de recorte iniciado.")
    print("Mové los sliders hasta que solo se vea la cinta.")
    print("Teclas: s=imprimir valores  q/ESC=salir")

    try:
        while True:
            frame_read, frame = capture.read()

            if not frame_read:
                print("No se pudo leer un fotograma de la camara.")
                break

            if CAMERA_ROTATION is not None:
                frame = cv2.rotate(frame, ROTATION_CODES[CAMERA_ROTATION])

            height, width = frame.shape[:2]

            x_min_ratio = cv2.getTrackbarPos("X min", WINDOW_FULL) / SCALE
            x_max_ratio = cv2.getTrackbarPos("X max", WINDOW_FULL) / SCALE
            y_min_ratio = cv2.getTrackbarPos("Y min", WINDOW_FULL) / SCALE
            y_max_ratio = cv2.getTrackbarPos("Y max", WINDOW_FULL) / SCALE

            x_min = int(width * x_min_ratio)
            x_max = int(width * x_max_ratio)
            y_min = int(height * y_min_ratio)
            y_max = int(height * y_max_ratio)

            display_frame = frame.copy()

            cv2.rectangle(
                display_frame,
                (x_min, y_min),
                (x_max, y_max),
                (255, 255, 0),
                2,
            )

            cv2.putText(
                display_frame,
                f"Recorte: {max(x_max - x_min, 0)}x{max(y_max - y_min, 0)} px",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
            )

            cv2.putText(
                display_frame,
                "s=imprimir valores   q=salir",
                (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

            cv2.imshow(WINDOW_FULL, display_frame)

            # Un recorte invertido o vacío no se puede mostrar.
            if x_max - x_min >= 1 and y_max - y_min >= 1:
                cropped = frame[y_min:y_max, x_min:x_max].copy()

                # La línea de detección se dibuja también acá para
                # ver dónde va a caer con el recorte elegido: su
                # posición es relativa al recorte, no a la imagen.
                line_x = int(cropped.shape[1] * LINE_X_RATIO)

                cv2.line(
                    cropped,
                    (line_x, 0),
                    (line_x, cropped.shape[0]),
                    (0, 255, 255),
                    2,
                )

                cv2.imshow(WINDOW_CROP, cropped)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                break

            if key == ord("s"):
                print("\n--- pegar en config.py ---")
                print(f"CROP_X_MIN_RATIO = {x_min_ratio:.3f}")
                print(f"CROP_X_MAX_RATIO = {x_max_ratio:.3f}")
                print(f"CROP_Y_MIN_RATIO = {y_min_ratio:.3f}")
                print(f"CROP_Y_MAX_RATIO = {y_max_ratio:.3f}")
                print(
                    f"(recorte de {x_max - x_min}x{y_max - y_min} px "
                    f"sobre una imagen de {width}x{height})"
                )

    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
