from __future__ import annotations

import time

import cv2

from camera import Camera
from communication import SerialCommunication
from config import (
    CAMERA_AUTO_EXPOSURE,
    CAMERA_EXPOSURE,
    LINE_X_RATIO,
    SHOW_COLOR_MASKS,
    WINDOW_NAME,
)
from coordinates import pixels_to_robot_cm
from detection import detect_objects
from line_crossing import (
    LineCrossingDetector,
    draw_detection_line,
    draw_text_right_of_line,
)
from tracker import (
    CentroidTracker,
    TrackedObject,
)


# Color con el que se dibuja cada pieza sobre el video, en BGR.
# Es solo presentación: si aparece un color que no está acá se
# dibuja en blanco, no se rompe nada.
DRAWING_COLORS = {
    "ROJO": (0, 0, 255),
    "AZUL": (255, 0, 0),
    "VERDE": (0, 200, 0),
}

DEFAULT_DRAWING_COLOR = (255, 255, 255)


def draw_tracked_object(
    frame: object,
    track: TrackedObject,
    line_x: int,
) -> None:
    """Dibuja los datos de una pieza sobre el video."""

    drawing_color = DRAWING_COLORS.get(
        track.color,
        DEFAULT_DRAWING_COLOR,
    )

    # Se dibuja el casco convexo, no el contorno crudo: es lo
    # mismo que usa classify_shape() para decidir la forma, así
    # que lo que se ve en pantalla coincide con lo que el sistema
    # realmente clasificó (un reflejo fijo puede dejar una muesca
    # en el contorno crudo que el casco convexo ya ignora).
    hull = cv2.convexHull(track.contour)

    cv2.drawContours(
        frame,
        [hull],
        -1,
        drawing_color,
        3,
    )

    # Se usa el bbox solamente para ubicar el texto,
    # ya no se dibuja el rectángulo sobre la pieza.
    x, y, width, height = track.bbox

    center_x, center_y = track.center

    cv2.circle(
        frame,
        (center_x, center_y),
        6,
        (0, 255, 0),
        -1,
    )

    label = (
        f"ID {track.track_id} | "
        f"{track.shape} {track.color}"
    )

    # En pantalla se muestran centímetros del robot, que es lo mismo
    # que viaja por el serial: así lo que se lee en el video es
    # exactamente el dato con el que va a trabajar el ESP32.
    frame_height, frame_width = frame.shape[:2]

    x_cm, y_cm = pixels_to_robot_cm(
        track.center,
        frame_width,
        frame_height,
        line_x,
    )

    position_label = (
        f"X:{x_cm:.1f} cm Y:{y_cm:.1f} cm"
    )

    cv2.putText(
        frame,
        label,
        (x, max(y - 35, 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        drawing_color,
        2,
    )

    cv2.putText(
        frame,
        position_label,
        (x, max(y - 12, 40)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        drawing_color,
        2,
    )

    if track.crossed_line:
        cv2.putText(
            frame,
            "CRUZO LA LINEA",
            (x, y + height + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )


def main() -> None:
    print("Iniciando sistema de visión...")

    camera = Camera()

    capture_width, capture_height = camera.capture_resolution()

    exposure_label = (
        "automática"
        if CAMERA_AUTO_EXPOSURE
        else f"fija en {CAMERA_EXPOSURE}"
    )

    print(
        f"Backend de captura: {camera.backend_name} | "
        "resolución que entrega la cámara: "
        f"{capture_width}x{capture_height} px | "
        f"exposición {exposure_label}"
    )

    # El tamaño final recién se conoce con el primer fotograma:
    # se informa una sola vez para poder verificar el recorte.
    crop_reported = False

    tracker = CentroidTracker()
    line_detector = LineCrossingDetector()

    serial_communication = SerialCommunication()

    total_crossings = 0

    previous_time = time.perf_counter()

    try:
        while True:
            frame_read, frame = camera.read()

            if not frame_read:
                print(
                    "No se pudo obtener una imagen "
                    "de la cámara."
                )
                break

            frame_height, frame_width = frame.shape[:2]

            # El fotograma ya llega recortado a la cinta: no hay
            # área de detección que dibujar, es toda la ventana.
            if not crop_reported:
                print(
                    "Fotograma en pantalla: "
                    f"{frame_width}x{frame_height} px "
                    "(ya rotado y recortado a la cinta). "
                    "Si entra mesa o quedan pedazos de cinta afuera, "
                    "ajustá CROP_Y_MIN_RATIO / CROP_Y_MAX_RATIO "
                    "en config.py."
                )

                crop_reported = True

            line_x = int(
                frame_width * LINE_X_RATIO
            )

            detections, masks = detect_objects(frame)

            tracked_objects = tracker.update(detections)

            crossed_objects = (
                line_detector.check_crossings(
                    tracked_objects,
                    line_x,
                )
            )

            total_crossings += len(crossed_objects)

            for track in crossed_objects:
                x_cm, y_cm = pixels_to_robot_cm(
                    track.center,
                    frame_width,
                    frame_height,
                    line_x,
                )

                # En consola se dejan también los píxeles: sirven
                # para verificar la conversión mientras se calibra.
                print(
                    "PIEZA DETECTADA:",
                    f"ID={track.track_id}",
                    f"FORMA={track.shape}",
                    f"COLOR={track.color}",
                    f"X={x_cm:.2f} cm",
                    f"Y={y_cm:.2f} cm",
                    f"(px {track.center[0]},{track.center[1]})",
                    sep=" | ",
                )

                serial_communication.send_piece(
                    track_id=track.track_id,
                    shape=track.shape,
                    color=track.color,
                    x_cm=x_cm,
                    y_cm=y_cm,
                )

            draw_detection_line(
                frame,
                line_x,
            )

            for track in tracked_objects:
                draw_tracked_object(
                    frame,
                    track,
                    line_x,
                )

            current_time = time.perf_counter()

            elapsed_time = (
                current_time - previous_time
            )

            previous_time = current_time

            fps = (
                1.0 / elapsed_time
                if elapsed_time > 0
                else 0.0
            )

            cv2.putText(
                frame,
                f"FPS: {fps:.1f}",
                (20, frame_height - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )

            cv2.putText(
                frame,
                f"Objetos visibles: {len(tracked_objects)}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )

            # El contador va del lado de la línea por el que las
            # piezas ya salieron, para no taparlas mientras entran.
            draw_text_right_of_line(
                frame,
                f"Piezas contadas: {total_crossings}",
                line_x,
                70,
                (0, 255, 255),
            )

            cv2.imshow(
                WINDOW_NAME,
                frame,
            )

            if SHOW_COLOR_MASKS:
                cv2.imshow(
                    "Mascara roja",
                    masks["ROJO"],
                )

                cv2.imshow(
                    "Mascara azul",
                    masks["AZUL"],
                )

            for message in (
                serial_communication.read_messages()
            ):
                print(
                    "ESP32 → Python:",
                    message,
                )

            key = cv2.waitKey(1) & 0xFF

            # Q o Escape cierran el programa.
            if key == ord("q") or key == 27:
                break

            # R reinicia el contador de piezas.
            if key == ord("r"):
                total_crossings = 0

            # Permitir cerrar mediante la X.
            if (
                cv2.getWindowProperty(
                    WINDOW_NAME,
                    cv2.WND_PROP_VISIBLE,
                )
                < 1
            ):
                break

    finally:
        camera.release()
        serial_communication.close()

        cv2.destroyAllWindows()

        print("Sistema de visión detenido.")


if __name__ == "__main__":
    main()