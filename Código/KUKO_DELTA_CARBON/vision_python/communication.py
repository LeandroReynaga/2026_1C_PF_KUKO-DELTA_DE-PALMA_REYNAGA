from __future__ import annotations

import time
from typing import Optional

import serial
import serial.tools.list_ports

from config import (
    SERIAL_BAUDRATE,
    SERIAL_ENABLED,
    SERIAL_PORT,
)

# ------------------------------------------------------------------
#  Protocolo hacia el ESP32
# ------------------------------------------------------------------
#
# El firmware espera una línea por pieza con exactamente tres campos:
#
#     Y,color,forma\n          por ejemplo:  3.50,B,S
#
# La X no se manda: la pieza se informa justo cuando cruza la línea
# de detección, así que su X es SIEMPRE la de la línea. El firmware
# ya la tiene hardcodeada como DETECTION_LINE_X = -23.0 cm, el mismo
# valor que LINE_X_CM en config.py. Si se mueve la línea hay que
# cambiar los dos lados.
#
# Traducción de los nombres que usa la visión a los códigos de un
# solo carácter que valida Robot::procesarComando().
COLOR_CODES = {
    "ROJO": "R",
    "VERDE": "G",
    "AZUL": "B",
}

SHAPE_CODES = {
    "CUADRADO": "S",
    "HEXAGONO": "H",
    "CIRCULO": "C",
}

# Fabricantes de los chips USB-serie que montan las placas ESP32.
# Sirve para reconocer la placa entre todos los COM del sistema
# (los de Bluetooth, por ejemplo, no tienen VID de USB).
USB_SERIAL_VENDOR_IDS = {
    0x10C4,  # Silicon Labs CP210x, el típico de las NodeMCU-32S
    0x1A86,  # QinHeng CH340 / CH9102
    0x0403,  # FTDI
    0x303A,  # Espressif (USB nativo del ESP32-S2/S3/C3)
}


def find_esp32_port() -> Optional[str]:
    """Busca el puerto COM de la placa entre los del sistema."""

    candidates = [
        port
        for port in serial.tools.list_ports.comports()
        if port.vid in USB_SERIAL_VENDOR_IDS
    ]

    if not candidates:
        available = ", ".join(
            f"{port.device} ({port.description})"
            for port in serial.tools.list_ports.comports()
        )

        print(
            "No se encontró ninguna placa ESP32 conectada. "
            f"Puertos visibles: {available or 'ninguno'}. "
            "Revisá el cable USB, o poné el COM a mano en "
            "SERIAL_PORT (config.py)."
        )

        return None

    if len(candidates) > 1:
        print(
            "Hay más de una placa USB-serie conectada: "
            + ", ".join(port.device for port in candidates)
            + f". Se usa {candidates[0].device}. Si no es la "
            "correcta, poné el COM a mano en SERIAL_PORT."
        )

    return candidates[0].device


class SerialCommunication:
    """Comunicación serial entre Python y el ESP32."""

    def __init__(self) -> None:
        self._serial: Optional[serial.Serial] = None

        if SERIAL_ENABLED:
            self.connect()

    @property
    def connected(self) -> bool:
        return (
            self._serial is not None
            and self._serial.is_open
        )

    def connect(self) -> None:
        if SERIAL_PORT.upper() == "AUTO":
            port = find_esp32_port()

            if port is None:
                return
        else:
            port = SERIAL_PORT

        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=SERIAL_BAUDRATE,
                timeout=0.01,
            )

            # Abrir el puerto activa DTR y eso REINICIA al ESP32: al
            # conectarse, el robot vuelve a arrancar y rehomea. Es el
            # comportamiento deseado (se arranca desde un estado
            # conocido), pero hay que esperarlo, porque los primeros
            # bytes que mande se perderían durante el reset.
            time.sleep(2)

            # Lo que haya quedado en el buffer durante el reinicio es
            # basura previa al arranque del firmware.
            self._serial.reset_input_buffer()

            print(f"Conectado al ESP32 en {port}")

        except serial.SerialException as error:
            self._serial = None

            # Un puerto que existe pero da "acceso denegado" está
            # abierto por otro programa. En la práctica siempre es el
            # monitor serie de PlatformIO: un puerto COM lo toma un
            # solo proceso a la vez, así que no se puede tener el
            # monitor abierto y main.py andando al mismo tiempo.
            #
            # Se busca en el texto porque pyserial NO encadena la
            # excepción original: mete el PermissionError adentro del
            # mensaje y deja __cause__ y __context__ en None. El
            # nombre de la clase no se traduce, el texto del sistema
            # sí, así que se compara contra el nombre.
            if "PermissionError" in str(error):
                print(
                    f"El puerto {port} está ocupado por otro "
                    "programa. Cerrá el monitor serie de PlatformIO "
                    "(o el Serial Monitor del IDE) y volvé a "
                    "arrancar main.py. La visión sigue funcionando, "
                    "pero no se le manda nada al robot."
                )
            else:
                print(
                    f"No se pudo abrir el puerto serial {port}:",
                    error,
                )

    def send_piece(
        self,
        track_id: int,
        shape: str,
        color: str,
        y_cm: float,
    ) -> None:
        """Envía al ESP32 una pieza que cruzó la línea.

        Formato: "Y,color,forma" (ej: "3.50,B,S"), en centímetros y
        en el sistema de referencia del robot. Ver el comentario del
        protocolo arriba: la X no viaja porque siempre es la de la
        línea de detección.

        El track_id se usa solo para el mensaje de consola; al
        firmware no le sirve porque él lleva su propia cola.
        """

        if not self.connected:
            return

        color_code = COLOR_CODES.get(color)
        shape_code = SHAPE_CODES.get(shape)

        # Un color o forma que el firmware no conoce se descarta acá
        # y no se manda: mandarlo solo lograría que el ESP32 conteste
        # "invalido" y ensucie la consola.
        if color_code is None or shape_code is None:
            print(
                f"No se envía la pieza ID={track_id}: el firmware no "
                f"reconoce {shape} {color}."
            )
            return

        message = (
            f"{y_cm:.2f},"
            f"{color_code},"
            f"{shape_code}\n"
        )

        try:
            self._serial.write(
                message.encode("utf-8")
            )

            print(
                "Python -> ESP32:",
                message.strip(),
            )

        except serial.SerialException as error:
            print(
                "Error enviando datos al ESP32:",
                error,
            )

    def read_messages(self) -> list[str]:
        """Lee mensajes disponibles enviados por el ESP32."""

        messages: list[str] = []

        if not self.connected:
            return messages

        try:
            while self._serial.in_waiting > 0:
                message = (
                    self._serial.readline()
                    .decode("utf-8", errors="ignore")
                    .strip()
                )

                if message:
                    messages.append(message)

        except serial.SerialException as error:
            print(
                "Error leyendo el puerto serial:",
                error,
            )

        return messages

    def close(self) -> None:
        if self.connected:
            self._serial.close()