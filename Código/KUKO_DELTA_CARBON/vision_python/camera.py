from __future__ import annotations

import cv2

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
    CROP_ENABLED,
    CROP_X_MAX_RATIO,
    CROP_X_MIN_RATIO,
    CROP_Y_MAX_RATIO,
    CROP_Y_MIN_RATIO,
)

ROTATION_CODES = {
    "90_CLOCKWISE": cv2.ROTATE_90_CLOCKWISE,
    "90_COUNTERCLOCKWISE": cv2.ROTATE_90_COUNTERCLOCKWISE,
    "180": cv2.ROTATE_180,
}

BACKEND_CODES = {
    "MSMF": cv2.CAP_MSMF,
    "DSHOW": cv2.CAP_DSHOW,
    "AUTO": cv2.CAP_ANY,
}


class Camera:
    """Control y configuración de la cámara de OpenCV.

    Entrega los fotogramas listos para procesar: ya rotados (para
    que la cinta se vea horizontal) y ya recortados a la zona de la
    cinta. Todo lo que está aguas abajo trabaja sobre ese fotograma,
    así que las coordenadas en píxeles son relativas al recorte.
    """

    def __init__(self) -> None:
        self._capture, self.backend_name = self._open_capture()

        self._configure_resolution()
        self._configure_exposure()

        # Los límites del recorte dependen del tamaño real que
        # entrega la cámara (que puede no ser el pedido), así que se
        # calculan con el primer fotograma.
        self._crop_bounds: tuple[int, int, int, int] | None = None

    @staticmethod
    def _open_capture() -> tuple[cv2.VideoCapture, str]:
        """Abre la cámara con el backend configurado.

        Si ese backend no la puede abrir se prueban los otros, así
        el programa arranca igual aunque el backend elegido falle en
        otra máquina.
        """

        preferred = (
            CAMERA_BACKEND
            if CAMERA_BACKEND in BACKEND_CODES
            else "MSMF"
        )

        fallbacks = [
            name
            for name in ("MSMF", "DSHOW", "AUTO")
            if name != preferred
        ]

        for name in [preferred] + fallbacks:
            capture = cv2.VideoCapture(
                CAMERA_INDEX,
                BACKEND_CODES[name],
            )

            if capture.isOpened():
                if name != preferred:
                    print(
                        f"El backend {preferred} no pudo abrir la "
                        f"cámara; se usa {name}. Ojo que los FPS "
                        "dependen mucho del backend."
                    )

                return capture, name

            capture.release()

        raise RuntimeError(
            "No se pudo abrir la cámara con ningún backend. "
            "Revisá CAMERA_INDEX en config.py."
        )

    def _configure_resolution(self) -> None:
        """Configura formato, resolución y FPS."""

        # El formato se pide ANTES que la resolución: si se cambia
        # después, DirectShow ya negoció el modo y el pedido de MJPG
        # se ignora.
        if CAMERA_USE_MJPG:
            self._capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*"MJPG"),
            )

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

    def _configure_exposure(self) -> None:
        """Fija la exposición, o la deja en automático."""

        if CAMERA_AUTO_EXPOSURE:
            # 0.75 es el valor que usan los backends de Windows para
            # "automático"; MSMF además acepta 1.
            self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
            return

        # 0.25 es "manual" en DirectShow y 0 en MSMF. Se mandan los
        # dos porque no cuesta nada y así no depende del backend que
        # haya terminado abriendo la cámara.
        self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

        self._capture.set(cv2.CAP_PROP_EXPOSURE, CAMERA_EXPOSURE)

        # No se lee CAP_PROP_EXPOSURE para verificar: MSMF devuelve
        # siempre -6.0 sin importar lo que se haya fijado. El valor
        # sí se aplica (se confirmó midiendo el brillo del fotograma,
        # que cambia como corresponde), pero el readback miente.

    def capture_resolution(self) -> tuple[int, int]:
        """Resolución que la cámara entrega realmente, sin rotar."""

        width = int(
            self._capture.get(cv2.CAP_PROP_FRAME_WIDTH)
        )

        height = int(
            self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
        )

        return width, height

    def _compute_crop_bounds(
        self,
        frame: object,
    ) -> tuple[int, int, int, int]:
        """Convierte las fracciones de recorte en píxeles."""

        frame_height, frame_width = frame.shape[:2]

        x_min = int(frame_width * CROP_X_MIN_RATIO)
        x_max = int(frame_width * CROP_X_MAX_RATIO)

        y_min = int(frame_height * CROP_Y_MIN_RATIO)
        y_max = int(frame_height * CROP_Y_MAX_RATIO)

        # Un recorte vacío o invertido dejaría un fotograma de 0
        # píxeles y el error recién aparecería mucho más abajo, en
        # medio de la detección. Mejor avisar acá.
        if x_max - x_min < 1 or y_max - y_min < 1:
            raise ValueError(
                "El recorte de la cinta quedó vacío. "
                "Revisá los CROP_*_RATIO en config.py: "
                "el MIN tiene que ser menor que el MAX."
            )

        return x_min, y_min, x_max, y_max

    def read(self) -> tuple[bool, object]:
        """Obtiene un fotograma ya rotado y recortado a la cinta."""

        frame_read, frame = self._capture.read()

        if not frame_read:
            return False, frame

        if CAMERA_ROTATION is not None:
            frame = cv2.rotate(
                frame,
                ROTATION_CODES[CAMERA_ROTATION],
            )

        if CROP_ENABLED:
            if self._crop_bounds is None:
                self._crop_bounds = self._compute_crop_bounds(
                    frame,
                )

            x_min, y_min, x_max, y_max = self._crop_bounds

            # .copy() para devolver un fotograma contiguo en memoria
            # y no una vista del original: así el recorte es
            # independiente y se puede dibujar encima sin sorpresas.
            frame = frame[y_min:y_max, x_min:x_max].copy()

        return True, frame

    def release(self) -> None:
        """Libera la cámara."""

        if self._capture.isOpened():
            self._capture.release()
