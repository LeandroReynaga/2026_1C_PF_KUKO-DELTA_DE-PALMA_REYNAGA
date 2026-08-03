from __future__ import annotations

import cv2

from config import (
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_INDEX,
    CAMERA_ROTATION,
    CAMERA_WIDTH,
)

_ROTATION_CODES = {
    "90_CLOCKWISE": cv2.ROTATE_90_CLOCKWISE,
    "90_COUNTERCLOCKWISE": cv2.ROTATE_90_COUNTERCLOCKWISE,
    "180": cv2.ROTATE_180,
}


class Camera:
    """Control y configuración de la cámara de OpenCV."""

    def __init__(self) -> None:
        # En Windows se utiliza DirectShow para tener
        # mayor acceso a propiedades de la cámara.
        self._capture = cv2.VideoCapture(
            CAMERA_INDEX,
            cv2.CAP_DSHOW,
        )

        # Si DirectShow no funciona, se intenta abrir
        # la cámara con el backend predeterminado.
        if not self._capture.isOpened():
            self._capture.release()
            self._capture = cv2.VideoCapture(CAMERA_INDEX)

        if not self._capture.isOpened():
            raise RuntimeError(
                "No se pudo abrir la cámara. "
                "Revisá CAMERA_INDEX en config.py."
            )

        self._configure_resolution()

    def _configure_resolution(self) -> None:
        """Configura resolución y FPS."""

        self._capture.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            CAMERA_WIDTH,
        )

        self._capture.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            CAMERA_HEIGHT,
        )

        self._capture.set(
            cv2.CAP_PROP_FPS,
            CAMERA_FPS,
        )


    def read(self) -> tuple[bool, object]:
        """Obtiene un fotograma de la cámara, ya rotado si corresponde."""

        frame_read, frame = self._capture.read()

        if frame_read and CAMERA_ROTATION is not None:
            frame = cv2.rotate(
                frame,
                _ROTATION_CODES[CAMERA_ROTATION],
            )

        return frame_read, frame

    def release(self) -> None:
        """Libera la cámara."""

        if self._capture.isOpened():
            self._capture.release()