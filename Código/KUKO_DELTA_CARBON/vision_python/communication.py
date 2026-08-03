from __future__ import annotations

import time
from typing import Optional

import serial

from config import (
    SERIAL_BAUDRATE,
    SERIAL_ENABLED,
    SERIAL_PORT,
)


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
        try:
            self._serial = serial.Serial(
                port=SERIAL_PORT,
                baudrate=SERIAL_BAUDRATE,
                timeout=0.01,
            )

            # Algunos ESP32 se reinician al abrir el puerto.
            time.sleep(2)

            print(
                f"Conectado al ESP32 en {SERIAL_PORT}"
            )

        except serial.SerialException as error:
            self._serial = None

            print(
                "No se pudo abrir el puerto serial:",
                error,
            )

    def send_piece(
        self,
        track_id: int,
        shape: str,
        color: str,
        x_pixel: int,
        y_pixel: int,
    ) -> None:
        """Envía al ESP32 una pieza que cruzó la línea."""

        if not self.connected:
            return

        message = (
            f"PIEZA,"
            f"{track_id},"
            f"{shape},"
            f"{color},"
            f"{x_pixel},"
            f"{y_pixel}\n"
        )

        try:
            self._serial.write(
                message.encode("utf-8")
            )

            print(
                "Python → ESP32:",
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