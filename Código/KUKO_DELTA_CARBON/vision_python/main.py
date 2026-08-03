from __future__ import annotations

import time

import cv2

from camera import Camera
from communication import SerialCommunication
from config import (
    LINE_Y_RATIO,
    ROI_X_MAX_RATIO,
    ROI_X_MIN_RATIO,
    ROI_Y_MAX_RATIO,
    ROI_Y_MIN_RATIO,
    SHOW_COLOR_MASKS,
    WINDOW_NAME,
)
from detection import detect_objects
from line_crossing import (
    LineCrossingDetector,
    draw_detection_line,
)
from tracker import (
    CentroidTracker,
    TrackedObject,
)


def draw_tracked_object(
    frame: object,
    track: TrackedObject,
) -> None:
    """Dibuja los datos de una pieza sobre el video."""

    if track.color == "ROJO":
        drawing_color = (0, 0, 255)
    else:
        drawing_color = (255, 0, 0)

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

    position_label = (
        f"X:{center_x} px Y:{center_y} px"
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

            roi_x_min = int(frame_width * ROI_X_MIN_RATIO)
            roi_x_max = int(frame_width * ROI_X_MAX_RATIO)

            roi_y_min = int(frame_height * ROI_Y_MIN_RATIO)
            roi_y_max = int(frame_height * ROI_Y_MAX_RATIO)

            cv2.rectangle(
                frame,
                (roi_x_min, roi_y_min),
                (roi_x_max, roi_y_max),
                (255, 255, 0),
                2,
            )

            cv2.putText(
                frame,
                ".", #AREA DE DETECCION",
                (roi_x_min + 10, roi_y_min + 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
            )

            line_y = int(
                frame_height * LINE_Y_RATIO
            )

            detections, masks = detect_objects(frame)

            tracked_objects = tracker.update(detections)

            crossed_objects = (
                line_detector.check_crossings(
                    tracked_objects,
                    line_y,
                )
            )

            total_crossings += len(crossed_objects)

            for track in crossed_objects:
                print(
                    "PIEZA DETECTADA:",
                    f"ID={track.track_id}",
                    f"FORMA={track.shape}",
                    f"COLOR={track.color}",
                    f"X={track.center[0]}",
                    f"Y={track.center[1]}",
                    sep=" | ",
                )

                serial_communication.send_piece(
                    track_id=track.track_id,
                    shape=track.shape,
                    color=track.color,
                    x_pixel=track.center[0],
                    y_pixel=track.center[1],
                )

            draw_detection_line(
                frame,
                line_y,
            )

            for track in tracked_objects:
                draw_tracked_object(
                    frame,
                    track,
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

            cv2.putText(
                frame,
                f"Piezas contadas: {total_crossings}",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
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