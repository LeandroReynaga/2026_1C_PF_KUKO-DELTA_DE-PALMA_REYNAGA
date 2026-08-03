"""
Herramienta de calibración interactiva de rangos HSV.

Permite mover 6 sliders (H/S/V min y max) y ver la máscara resultante
en vivo, sin tener que editar config.py y reiniciar cada vez.

Uso:
    python hsv_calibrator.py

Controles:
    1  -> cargar preset ROJO (rango 1, cerca de H=0)
    2  -> cargar preset ROJO (rango 2, cerca de H=179)
    3  -> cargar preset AZUL
    s  -> imprimir en consola el rango actual, listo para
          copiar y pegar en config.py
    q / ESC -> salir
"""

from __future__ import annotations

import cv2
import numpy as np

from config import (
    BLUE_LOWER,
    BLUE_UPPER,
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_INDEX,
    CAMERA_WIDTH,
    MORPH_KERNEL_SIZE,
    RED_LOWER_1,
    RED_LOWER_2,
    RED_UPPER_1,
    RED_UPPER_2,
    ROI_X_MAX_RATIO,
    ROI_X_MIN_RATIO,
    ROI_Y_MAX_RATIO,
    ROI_Y_MIN_RATIO,
)

WINDOW_CONTROLS = "Calibracion HSV - controles"
WINDOW_MASK = "Calibracion HSV - mascara"
WINDOW_ORIGINAL = "Calibracion HSV - camara"

PRESETS = {
    ord("1"): ("ROJO rango 1", RED_LOWER_1, RED_UPPER_1),
    ord("2"): ("ROJO rango 2", RED_LOWER_2, RED_UPPER_2),
    ord("3"): ("AZUL", BLUE_LOWER, BLUE_UPPER),
}


def nothing(_value: int) -> None:
    pass


def create_trackbars(
    lower: tuple[int, int, int],
    upper: tuple[int, int, int],
) -> None:
    cv2.namedWindow(WINDOW_CONTROLS, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_CONTROLS, 400, 260)

    cv2.createTrackbar("H min", WINDOW_CONTROLS, lower[0], 179, nothing)
    cv2.createTrackbar("H max", WINDOW_CONTROLS, upper[0], 179, nothing)
    cv2.createTrackbar("S min", WINDOW_CONTROLS, lower[1], 255, nothing)
    cv2.createTrackbar("S max", WINDOW_CONTROLS, upper[1], 255, nothing)
    cv2.createTrackbar("V min", WINDOW_CONTROLS, lower[2], 255, nothing)
    cv2.createTrackbar("V max", WINDOW_CONTROLS, upper[2], 255, nothing)


def read_trackbars() -> tuple[tuple[int, int, int], tuple[int, int, int]]:
    h_min = cv2.getTrackbarPos("H min", WINDOW_CONTROLS)
    h_max = cv2.getTrackbarPos("H max", WINDOW_CONTROLS)
    s_min = cv2.getTrackbarPos("S min", WINDOW_CONTROLS)
    s_max = cv2.getTrackbarPos("S max", WINDOW_CONTROLS)
    v_min = cv2.getTrackbarPos("V min", WINDOW_CONTROLS)
    v_max = cv2.getTrackbarPos("V max", WINDOW_CONTROLS)

    return (h_min, s_min, v_min), (h_max, s_max, v_max)


def roi_mask_for(frame: np.ndarray) -> np.ndarray:
    height, width = frame.shape[:2]

    x_min = int(width * ROI_X_MIN_RATIO)
    x_max = int(width * ROI_X_MAX_RATIO)
    y_min = int(height * ROI_Y_MIN_RATIO)
    y_max = int(height * ROI_Y_MAX_RATIO)

    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.rectangle(mask, (x_min, y_min), (x_max, y_max), 255, -1)

    return mask


def main() -> None:
    capture = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

    if not capture.isOpened():
        capture.release()
        capture = cv2.VideoCapture(CAMERA_INDEX)

    if not capture.isOpened():
        raise RuntimeError(
            "No se pudo abrir la camara. Revisa CAMERA_INDEX en config.py."
        )

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    capture.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

    current_preset = "AZUL"
    lower, upper = BLUE_LOWER, BLUE_UPPER

    create_trackbars(lower, upper)

    kernel = np.ones(
        (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE),
        dtype=np.uint8,
    )

    print("Calibrador HSV iniciado.")
    print(
        "Teclas: 1=rojo rango1  2=rojo rango2  3=azul  "
        "s=imprimir valores  q/ESC=salir"
    )

    try:
        while True:
            frame_read, frame = capture.read()

            if not frame_read:
                print("No se pudo leer un fotograma de la camara.")
                break

            blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            lower_np, upper_np = read_trackbars()

            mask = cv2.inRange(
                hsv,
                np.array(lower_np, dtype=np.uint8),
                np.array(upper_np, dtype=np.uint8),
            )

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            mask = cv2.bitwise_and(mask, roi_mask_for(frame))

            # Color promedio de los pixeles que la máscara está
            # dejando pasar ahora mismo. Sirve para notar cuando
            # se está colando otra cosa (una mano, un reflejo):
            # el recuadro deja de verse como el color de la pieza.
            masked_bgr_pixels = frame[mask == 255]
            masked_hsv_pixels = hsv[mask == 255]

            if masked_bgr_pixels.size > 0:
                average_bgr = masked_bgr_pixels.mean(axis=0)
                average_hsv = masked_hsv_pixels.mean(axis=0)
                swatch_color = tuple(int(c) for c in average_bgr)
                swatch_label = (
                    f"H:{average_hsv[0]:.0f} "
                    f"S:{average_hsv[1]:.0f} "
                    f"V:{average_hsv[2]:.0f}"
                )
            else:
                swatch_color = (40, 40, 40)
                swatch_label = "Sin deteccion"

            display_frame = frame.copy()
            height, width = display_frame.shape[:2]

            x_min = int(width * ROI_X_MIN_RATIO)
            x_max = int(width * ROI_X_MAX_RATIO)
            y_min = int(height * ROI_Y_MIN_RATIO)
            y_max = int(height * ROI_Y_MAX_RATIO)

            cv2.rectangle(
                display_frame,
                (x_min, y_min),
                (x_max, y_max),
                (255, 255, 0),
                2,
            )

            cv2.putText(
                display_frame,
                f"Preset: {current_preset}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                display_frame,
                "Click en esta ventana antes de tocar teclas",
                (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )

            cv2.putText(
                display_frame,
                "1=rojo1  2=rojo2  3=azul  s=imprimir  q=salir",
                (20, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )

            swatch_x0 = width - 180
            swatch_y0 = 20
            swatch_x1 = width - 20
            swatch_y1 = 90

            cv2.putText(
                display_frame,
                "Color detectado",
                (swatch_x0, swatch_y0 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )

            cv2.rectangle(
                display_frame,
                (swatch_x0, swatch_y0),
                (swatch_x1, swatch_y1),
                swatch_color,
                -1,
            )

            cv2.rectangle(
                display_frame,
                (swatch_x0, swatch_y0),
                (swatch_x1, swatch_y1),
                (255, 255, 255),
                2,
            )

            cv2.putText(
                display_frame,
                swatch_label,
                (swatch_x0, swatch_y1 + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )

            cv2.imshow(WINDOW_ORIGINAL, display_frame)
            cv2.imshow(WINDOW_MASK, mask)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                break

            if key in PRESETS:
                current_preset, lower, upper = PRESETS[key]

                cv2.setTrackbarPos("H min", WINDOW_CONTROLS, lower[0])
                cv2.setTrackbarPos("H max", WINDOW_CONTROLS, upper[0])
                cv2.setTrackbarPos("S min", WINDOW_CONTROLS, lower[1])
                cv2.setTrackbarPos("S max", WINDOW_CONTROLS, upper[1])
                cv2.setTrackbarPos("V min", WINDOW_CONTROLS, lower[2])
                cv2.setTrackbarPos("V max", WINDOW_CONTROLS, upper[2])

            if key == ord("s"):
                print(f"\n--- {current_preset} ---")
                print(f"LOWER = {lower_np}")
                print(f"UPPER = {upper_np}")

    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()