"""Hilo dueño del puerto serie. Es el UNICO que lo abre.

En Windows un COM lo toma un solo proceso, y abrirlo resetea el ESP32: por
eso la conexion se hace una vez, al arrancar, y no se reintenta por cada
accion del operador. Si se cae, se reintenta cada RETARDO_RECONEXION_S.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import serial
import serial.tools.list_ports

from . import protocolo as pr
from .estado import EstadoSistema

VID_USB_SERIE = {0x10C4, 0x1A86, 0x0403, 0x303A}
RETARDO_RECONEXION_S = 3.0
MAX_LINEAS_CONSOLA = 300


def buscar_puerto() -> Optional[str]:
    for p in serial.tools.list_ports.comports():
        if p.vid in VID_USB_SERIE:
            return p.device

    return None


class Enlace:
    def __init__(self, estado: EstadoSistema, puerto: str = "AUTO", baudios: int = 115200):
        self.estado = estado
        self.puerto_pedido = puerto
        self.baudios = baudios

        self._ser: Optional[serial.Serial] = None
        self._parar = threading.Event()
        self._lock = threading.Lock()
        self._hilo: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    def arrancar(self) -> None:
        self._hilo = threading.Thread(target=self._correr, daemon=True, name="enlace")
        self._hilo.start()

    def parar(self) -> None:
        self._parar.set()

        if self._hilo:
            self._hilo.join(timeout=2.0)

    def enviar(self, linea: str) -> bool:
        """Manda un comando. False si no hay enlace (la UI deshabilita el boton)."""

        with self._lock:
            if not self._ser or not self._ser.is_open:
                return False

            try:
                self._ser.write(linea.encode("ascii"))
                self._ser.flush()
                return True
            except serial.SerialException as err:
                self.estado.error_enlace = str(err)
                return False

    # ------------------------------------------------------------------
    def _correr(self) -> None:
        while not self._parar.is_set():
            if not self._conectar():
                time.sleep(RETARDO_RECONEXION_S)
                continue

            self._leer()

            with self._lock:
                if self._ser:
                    try:
                        self._ser.close()
                    except serial.SerialException:
                        pass

                self._ser = None

            self.estado.conectado = False

    def _conectar(self) -> bool:
        puerto = buscar_puerto() if self.puerto_pedido.upper() == "AUTO" else self.puerto_pedido

        if puerto is None:
            self.estado.error_enlace = "no se encontro ninguna placa ESP32 conectada"
            return False

        try:
            ser = serial.Serial(puerto, self.baudios, timeout=0.2)
        except serial.SerialException as err:
            # Un puerto que existe pero da acceso denegado esta abierto por
            # otro programa: casi siempre el monitor serie de PlatformIO.
            if "PermissionError" in str(err):
                self.estado.error_enlace = (
                    f"{puerto} esta ocupado por otro programa "
                    "(cerra el monitor serie de PlatformIO)")
            else:
                self.estado.error_enlace = f"no se pudo abrir {puerto}: {err}"

            return False

        # Abrir el puerto reinicia la placa: los primeros bytes son basura
        # previa al arranque del firmware.
        time.sleep(2.0)
        ser.reset_input_buffer()

        with self._lock:
            self._ser = ser

        self.estado.puerto = puerto
        self.estado.conectado = True
        self.estado.error_enlace = ""

        # Foto del estado, tabla de parametros y stream encendido, en ese
        # orden: la interfaz arma sus controles con lo que devuelve 'P?'.
        self.enviar(pr.cmd_foto_estado())
        self.enviar(pr.cmd_listar_parametros())
        self.enviar(pr.cmd_telemetria(True))

        return True

    def _leer(self) -> None:
        while not self._parar.is_set():
            try:
                cruda = self._ser.readline()
            except serial.SerialException as err:
                self.estado.error_enlace = f"enlace perdido: {err}"
                return

            if not cruda:
                continue

            texto = cruda.decode("utf-8", errors="replace").strip()

            if texto:
                self._despachar(pr.parsear(texto))

    def _despachar(self, m: pr.Mensaje) -> None:
        est = self.estado

        if isinstance(m, pr.Telemetria):
            est.t = m
            est.ultimo_t = time.monotonic()
            est.suavizar()

        elif isinstance(m, pr.Proceso):
            est.e = m

        elif isinstance(m, pr.Salud):
            est.h = m

        elif isinstance(m, pr.Parametro):
            est.parametros[m.nombre] = m

        elif isinstance(m, pr.ParametroCambiado):
            if m.ok and m.nombre in est.parametros and m.valor is not None:
                est.parametros[m.nombre].valor = m.valor
            elif not m.ok:
                self._consola(f"parametro rechazado: {m.nombre} ({m.error})")

        elif isinstance(m, pr.Boot):
            # El firmware se reinicio: todo lo que sabiamos quedo viejo.
            est.boot = m
            est.e = None
            est.h = None
            est.parametros.clear()
            self._consola(f"el firmware arranco (proto={m.proto} fw={m.fw})")

            if not m.compatible:
                self._consola(f"PROTOCOLO INCOMPATIBLE: firmware={m.proto} "
                              f"interfaz={pr.VERSION_PROTOCOLO}")

            self.enviar(pr.cmd_listar_parametros())
            self.enviar(pr.cmd_telemetria(True))

        elif isinstance(m, pr.Fallo):
            est.fallos.append(m)
            est.fallos[:] = est.fallos[-32:]
            self._consola(f"FALLO {m.tipo} eje {m.eje} err={m.error_deg}")

        elif isinstance(m, pr.Texto):
            if m.crudo:
                self._consola(m.crudo)

    def _consola(self, linea: str) -> None:
        self.estado.consola.append(f"{time.strftime('%H:%M:%S')}  {linea}")
        self.estado.consola[:] = self.estado.consola[-MAX_LINEAS_CONSOLA:]
