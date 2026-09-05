"""
Mide la velocidad REAL de captura de la cámara, sin aplicar
ningún procesamiento, para saber si el techo de FPS está en la
cámara/driver o en el resto del pipeline (detección, dibujo, etc).

Uso:
    python benchmark_camera.py
"""

from __future__ import annotations

import time

import cv2

from config import (
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_INDEX,
    CAMERA_USE_MJPG,
    CAMERA_WIDTH,
)


def main() -> None:
    capture = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

    if not capture.isOpened():
        capture.release()
        capture = cv2.VideoCapture(CAMERA_INDEX)

    if not capture.isOpened():
        raise RuntimeError(
            "No se pudo abrir la camara. Revisa CAMERA_INDEX en config.py."
        )

    # Mismo orden que Camera._configure_resolution(): el formato
    # primero, si no DirectShow ya negocio el modo y lo ignora.
    if CAMERA_USE_MJPG:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    capture.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

    actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    reported_fps = capture.get(cv2.CAP_PROP_FPS)

    print(f"Resolucion real entregada: {actual_width}x{actual_height}")
    print(f"FPS que el driver DICE que va a entregar: {reported_fps:.1f}")

    # Primeros fotogramas descartados: muchas camaras tardan un
    # instante en estabilizar exposicion/enfoque automatico.
    for _ in range(10):
        capture.read()

    num_frames = 60

    print(f"Leyendo {num_frames} fotogramas SIN procesar nada...")

    start = time.perf_counter()

    read_ok = True
    for _ in range(num_frames):
        ok, _frame = capture.read()
        if not ok:
            read_ok = False
            print("Fallo la lectura de un fotograma.")
            break

    elapsed = time.perf_counter() - start

    capture.release()

    if read_ok:
        fps_real = num_frames / elapsed
        ms_per_frame = elapsed / num_frames * 1000

        print(f"FPS real de captura (sin procesamiento): {fps_real:.1f}")
        print(f"Tiempo promedio por fotograma: {ms_per_frame:.1f} ms")
        print()

        if fps_real < 15:
            print(
                "El techo esta en la camara/driver, no en el codigo "
                "de deteccion. Probar con mejor luz (menos tiempo de "
                "exposicion necesario) o revisar si la camara soporta "
                "otro modo/resolucion con mas FPS."
            )
        else:
            print(
                "La camara entrega bastante mas de lo que se ve en "
                "pantalla con la interfaz andando. Ahi el cuello de "
                "botella esta en otra parte del pipeline (deteccion, "
                "dibujo, codificacion a JPEG), no en la captura."
            )


if __name__ == "__main__":
    main()