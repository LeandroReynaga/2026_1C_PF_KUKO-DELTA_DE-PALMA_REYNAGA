from __future__ import annotations

import cv2

import config

# El MODULO y no los valores, por el mismo motivo que en detection.py: la
# pestana de Vision cambia la exposicion con la camara andando, y
# `from config import CAMERA_EXPOSURE` deja una copia congelada del numero
# que estaba escrito al importar. Lo unico que sigue leyendose una sola vez
# es lo que se le pide al sensor al abrirlo (resolucion, backend, FPS):
# cambiarlo en marcha no lo aplica ningun backend de Windows sin cerrar y
# volver a abrir el dispositivo.

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
            config.CAMERA_BACKEND
            if config.CAMERA_BACKEND in BACKEND_CODES
            else "MSMF"
        )

        fallbacks = [
            name
            for name in ("MSMF", "DSHOW", "AUTO")
            if name != preferred
        ]

        for name in [preferred] + fallbacks:
            capture = cv2.VideoCapture(
                config.CAMERA_INDEX,
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
        if config.CAMERA_USE_MJPG:
            self._capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*"MJPG"),
            )

        self._capture.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            config.CAMERA_WIDTH,
        )

        self._capture.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            config.CAMERA_HEIGHT,
        )

        self._capture.set(
            cv2.CAP_PROP_FPS,
            config.CAMERA_FPS,
        )

    def _configure_exposure(self) -> None:
        """Fija la exposición, o la deja en automático.

        Lee de config.py y se lo pasa a `aplicar_exposicion()`, que es la
        misma puerta que usa la pestaña de Visión con la cámara ya andando:
        una sola implementación, así el modo que se elige al arrancar y el
        que se elige a mano no se pueden separar.
        """

        self.aplicar_exposicion(
            config.CAMERA_AUTO_EXPOSURE,
            config.CAMERA_EXPOSURE,
        )

    def aplicar_exposicion(
        self,
        automatica: bool,
        valor: float,
    ) -> None:
        """Cambia el modo de exposición sobre la cámara ya abierta.

        Se puede llamar en marcha: los dos backends de Windows aceptan
        CAP_PROP_AUTO_EXPOSURE y CAP_PROP_EXPOSURE con el dispositivo
        abierto, a diferencia de la resolución o el FOURCC, que sólo se
        negocian al abrirlo.

        OJO con dónde se llama desde: `set()` y `read()` sobre el mismo
        `VideoCapture` desde dos hilos a la vez cuelgan MSMF. El hilo de
        visión lo aplica entre dos fotogramas (ver `Vision._aplicar_camara`),
        nunca la interfaz por su cuenta.
        """

        if automatica:
            # 0.75 es el valor que usan los backends de Windows para
            # "automático"; MSMF además acepta 1.
            self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
            return

        # 0.25 es "manual" en DirectShow y 0 en MSMF. Se mandan los
        # dos porque no cuesta nada y así no depende del backend que
        # haya terminado abriendo la cámara.
        self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self._capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

        self._capture.set(cv2.CAP_PROP_EXPOSURE, float(valor))

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

    # Píxeles que se corrió el recorte respecto de los CROP_*_RATIO de
    # config.py. Lo mueve la interfaz; positivo = la ventana baja.
    offset_y = 0

    # Corrección de exposición POR SOFTWARE, en porcentaje: -10 deja el
    # fotograma al 90 % del brillo que entregó la cámara. Es el complemento
    # del modo automático, que es el que se quiere para poder mudar el robot
    # sin recalibrar, pero que sobreexpone porque promedia una cinta clara
    # que ocupa casi todo el cuadro (ver CAMERA_AUTO_EXPOSURE en config.py).
    #
    # Es una ganancia lineal sobre el BGR, o sea que en HSV escala V y deja
    # H casi intacto: justo el eje que se quiere mover. Lo que NO puede es
    # recuperar un píxel quemado -- un pixel que llegó a 255 perdió su color
    # en el sensor y bajarle el brillo lo deja gris, no del color de la
    # pieza. Contra el quemado sirve la exposición manual (o menos luz), no
    # esto.
    correccion_pct = 0.0

    def mover_recorte(self, delta_px: int) -> int:
        """Corre el recorte y devuelve el desplazamiento efectivo."""

        self.offset_y += int(delta_px)

        # El recorte se recalcula con el próximo fotograma: ahí es donde se
        # satura contra el borde de la imagen.
        self._crop_bounds = None

        return self.offset_y

    def _compute_crop_bounds(
        self,
        frame: object,
    ) -> tuple[int, int, int, int]:
        """Convierte las fracciones de recorte en píxeles."""

        frame_height, frame_width = frame.shape[:2]

        x_min = int(frame_width * config.CROP_X_MIN_RATIO)
        x_max = int(frame_width * config.CROP_X_MAX_RATIO)

        y_min = int(frame_height * config.CROP_Y_MIN_RATIO)
        y_max = int(frame_height * config.CROP_Y_MAX_RATIO)

        # Desplazamiento vertical que ajusta el operador desde la interfaz.
        # Los dos límites se mueven JUNTOS: el alto del recorte no cambia,
        # sólo se corre la ventana. Si cambiara el alto, cambiaría también
        # la escala píxel->cm y con ella todas las coordenadas que se le
        # mandan al robot.
        #
        # Existe porque la cámara se corre un poco cuando se mueve el robot,
        # y porque la cinta se desliza sobre el rodillo hacia un extremo.
        # Recentrar eso a mano en config.py obligaba a reiniciar el programa.
        alto_recorte = y_max - y_min

        desplazamiento = max(-y_min, min(self.offset_y, frame_height - y_max))

        y_min += desplazamiento
        y_max = y_min + alto_recorte

        # Se guarda el efectivo: si el pedido se salió del fotograma, la
        # interfaz tiene que mostrar dónde quedó de verdad y no seguir
        # contando clicks que no mueven nada.
        self.offset_y = desplazamiento

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

        if config.CAMERA_ROTATION is not None:
            frame = cv2.rotate(
                frame,
                ROTATION_CODES[config.CAMERA_ROTATION],
            )

        if config.CROP_ENABLED:
            if self._crop_bounds is None:
                self._crop_bounds = self._compute_crop_bounds(
                    frame,
                )

            x_min, y_min, x_max, y_max = self._crop_bounds

            # .copy() para devolver un fotograma contiguo en memoria
            # y no una vista del original: así el recorte es
            # independiente y se puede dibujar encima sin sorpresas.
            frame = frame[y_min:y_max, x_min:x_max].copy()

        # Va acá y no en el hilo de visión a propósito: así el fotograma que
        # se detecta y el que se muestra en pantalla son EL MISMO. Calibrar
        # mirando una imagen que no es sobre la que se decide sería mirar
        # otra cosa.
        if self.correccion_pct:
            frame = cv2.convertScaleAbs(
                frame,
                alpha=1.0 + self.correccion_pct / 100.0,
                beta=0.0,
            )

        return True, frame

    def release(self) -> None:
        """Libera la cámara."""

        if self._capture.isOpened():
            self._capture.release()
