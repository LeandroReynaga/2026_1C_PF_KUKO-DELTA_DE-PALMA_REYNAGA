"""Desenchufar la cámara sin desenchufarla: el hilo de visión con una falsa.

Lo que hay que verificar acá es justamente lo que no se puede probar a mano
sin pasarse la tarde enchufando y desenchufando un USB, y lo que peor falla
si está mal, porque falla en silencio: una cámara USB desconectada **no da
error**. `VideoCapture.read()` devuelve `False` para siempre, el hilo sigue
girando, la última imagen queda congelada en la pantalla y los FPS quedan
clavados en el último valor bueno. Todo parece andar.

Así que se le pone al hilo una `Camera` de mentira y se la desconecta desde
la prueba: se verifica que la caída se detecte, que se avise, que la cámara
se vuelva a abrir sola y que el contador de reconexiones suba — que es el
número que delata un cable flojo.

Los vencimientos se achican mientras dura la prueba: son segundos de reloj
de verdad y no hay nada que ganar esperándolos.

Se corre con:  python -m pytest pc/tests    (o  python pc/tests/test_camara.py)
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from kuko import estado as est_mod
from kuko import vision as vis
from kuko.estado import EstadoSistema

# Vencimientos de la prueba. Los de produccion son 2 s y 3 s; con estos, una
# caida completa con su reconexion entra en menos de un segundo.
TIMEOUT_PRUEBA_S = 0.25
REAPERTURA_PRUEBA_S = 0.10

# Tope de espera de cualquier condicion. Si algo no pasa en este tiempo, no
# es que la maquina esté lenta: es que no va a pasar.
PACIENCIA_S = 6.0


class Guion:
    """Lo que la cámara va a hacer. La prueba lo cambia en marcha."""

    def __init__(self) -> None:
        self.abre = True          # ¿se puede construir? (cámara presente)
        self.entrega = True       # ¿read() devuelve fotogramas?
        self.aperturas = 0
        self.liberadas = 0


class CamaraFalsa:
    """Reemplaza a `Camera` dentro de `kuko.vision` mientras dura la prueba."""

    guion = Guion()
    backend_name = "FALSA"
    offset_y = 0

    def __init__(self) -> None:
        CamaraFalsa.guion.aperturas += 1

        if not CamaraFalsa.guion.abre:
            raise RuntimeError("no se pudo abrir la camara de mentira")

    def read(self):
        # Un respiro por fotograma: sin esto el bucle gira a millones de
        # vueltas por segundo y la prueba se come un núcleo entero.
        time.sleep(0.004)

        if not CamaraFalsa.guion.entrega:
            return False, None

        # Un fotograma negro chico: `detect_objects` no encuentra nada en él,
        # que es lo que se quiere -- lo que se prueba es el manejo de la
        # cámara, no la detección.
        return True, np.zeros((60, 120, 3), dtype=np.uint8)

    def mover_recorte(self, delta_px: int) -> int:
        CamaraFalsa.offset_y += int(delta_px)
        return CamaraFalsa.offset_y

    def release(self) -> None:
        CamaraFalsa.guion.liberadas += 1


class Banco:
    """Monta el hilo de visión con la cámara falsa y lo desarma al terminar."""

    def __init__(self) -> None:
        self.guion = Guion()
        CamaraFalsa.guion = self.guion

        self._reales = (vis.Camera, vis.RETARDO_REAPERTURA_S,
                        vis.RETARDO_REAPERTURA_MAX_S, est_mod.CAMARA_TIMEOUT_S)

        vis.Camera = CamaraFalsa                     # type: ignore[assignment]
        vis.RETARDO_REAPERTURA_S = REAPERTURA_PRUEBA_S
        vis.RETARDO_REAPERTURA_MAX_S = REAPERTURA_PRUEBA_S
        est_mod.CAMARA_TIMEOUT_S = TIMEOUT_PRUEBA_S

        self.estado = EstadoSistema()
        self.enviados: list[str] = []
        self.vision = vis.Vision(self.estado, self.enviados.append)

        # `Vision` no le pide nada al puerto serie, pero el guardado del
        # recorte sí escribe en disco. Acá no interesa y no se toca.
        self._hilo: threading.Thread | None = None

    def arrancar(self) -> None:
        self.vision.arrancar()

    def esperar(self, condicion, que: str) -> None:
        limite = time.monotonic() + PACIENCIA_S

        while time.monotonic() < limite:
            if condicion():
                return

            time.sleep(0.01)

        raise AssertionError(f"nunca paso: {que}")

    def soltar(self) -> None:
        self.vision.parar()
        (vis.Camera, vis.RETARDO_REAPERTURA_S, vis.RETARDO_REAPERTURA_MAX_S,
         est_mod.CAMARA_TIMEOUT_S) = self._reales


# ==================================================================

def test_la_camara_desenchufada_se_nota():
    """El caso que motivó todo esto: `read()` devuelve False para siempre."""

    banco = Banco()

    try:
        banco.arrancar()
        banco.esperar(lambda: banco.estado.fotogramas > 3, "llegan fotogramas")

        assert banco.estado.camara_viva()
        assert banco.estado.chequeos()["camara"].estado == est_mod.VERDE

        # Se desenchufa. Nadie da error: simplemente dejan de venir.
        banco.guion.entrega = False

        banco.esperar(lambda: not banco.estado.camara_viva(),
                      "se detecta que no hay imagen")

        assert banco.estado.chequeos()["camara"].estado == est_mod.ROJO
        assert "sin imagen" in banco.estado.chequeos()["camara"].detalle

        # Y queda anotado en el historial, con su hora.
        banco.esperar(lambda: any(e.clase == "camara"
                                  for e in banco.estado.rendimiento.lista_eventos()),
                      "el corte queda en la cronologia de eventos")

        assert banco.estado.rendimiento.resumen().camara_viva is False
    finally:
        banco.soltar()


def test_la_camara_se_vuelve_a_abrir_sola():
    """Volver a enchufarla tiene que alcanzar.

    Un `VideoCapture` cuyo USB se desenchufó queda muerto: vuelve a andar
    recién con uno nuevo. Sin reabrirla, enchufar la cámara de nuevo no
    arreglaría nada y habría que reiniciar el programa entero.
    """

    banco = Banco()

    try:
        banco.arrancar()
        banco.esperar(lambda: banco.estado.fotogramas > 3, "llegan fotogramas")

        aperturas = banco.guion.aperturas
        banco.guion.entrega = False

        banco.esperar(lambda: not banco.estado.camara_viva(), "se cae")

        # Hay que esperar a que SUELTE el dispositivo muerto, no solo a que
        # el punto se ponga rojo: son dos instantes distintos, y volver a
        # enchufarla en el medio haria que el hilo nunca llegue a decidir
        # que se cayo -- que es justo lo que esta prueba tiene que ver.
        banco.esperar(lambda: banco.guion.liberadas >= 1,
                      "suelta el dispositivo muerto")

        # Se vuelve a enchufar.
        banco.guion.entrega = True

        banco.esperar(lambda: banco.estado.reconexiones_camara >= 1, "vuelve sola")

        assert banco.estado.camara_viva()
        assert banco.guion.aperturas > aperturas, "no se reabrio el dispositivo"
        assert banco.estado.rendimiento.resumen().camara_caidas >= 1

        # Y el punto queda en ambar: anda, pero se reenganchó.
        assert banco.estado.chequeos()["camara"].estado == est_mod.AMBAR
    finally:
        banco.soltar()


def test_si_no_abre_al_arrancar_se_sigue_intentando():
    """Antes el hilo se moría y no había forma de recuperarlo sin reiniciar."""

    banco = Banco()

    try:
        banco.guion.abre = False
        banco.arrancar()

        banco.esperar(lambda: banco.guion.aperturas >= 3, "reintenta abrirla")

        chequeo = banco.estado.chequeos()["camara"]

        assert chequeo.estado == est_mod.ROJO
        assert "no abre" in chequeo.detalle

        # El historial la ve como caída aunque nunca haya entregado nada.
        resumen = banco.estado.rendimiento.resumen()

        assert resumen.camara_presente and resumen.camara_viva is False

        # A la consola una sola vez, no un mensaje cada vez que reintenta.
        avisos = [l for l in banco.estado.consola if "no se pudo abrir" in l]

        assert len(avisos) == 1, f"{len(avisos)} avisos repetidos en la consola"

        # Y cuando aparece, arranca sin que nadie la toque.
        banco.guion.abre = True

        banco.esperar(lambda: banco.estado.camara_viva(), "arranca al aparecer")
    finally:
        banco.soltar()


def test_sin_vision_no_hay_camara_que_falle():
    """Con --sin-vision no se construye ningún `Vision`, y no falta nada."""

    estado = EstadoSistema()

    assert not estado.camara_presente
    assert not estado.camara_viva()
    assert estado.chequeos()["camara"].estado == est_mod.GRIS
    assert estado.rendimiento.resumen().camara_presente is False


def test_el_promedio_de_fps_se_alimenta_del_hilo_de_verdad():
    """El historial recibe fotogramas del hilo real, no sólo de las pruebas.

    Es lo que engancha las dos mitades: `Vision._avisar` contra
    `Rendimiento.observar_camara`. Un nombre de argumento cambiado de un
    lado se ve acá y no delante del robot.
    """

    banco = Banco()

    try:
        banco.arrancar()

        # Seis muestras: el promedio pide una ventana de mas de tres
        # segundos para existir, y el historial muestrea a 1 Hz.
        banco.esperar(
            lambda: len(banco.estado.rendimiento.camara) >= 6,
            "el historial junta muestras de camara")

        resumen = banco.estado.rendimiento.resumen()

        assert resumen.fps_5min is not None and resumen.fps_5min > 0.0

        # El contador del historial va un poco atras del de la vision: el
        # hilo avisa dos veces por segundo, no una vez por fotograma.
        assert 0 < resumen.fotogramas <= banco.estado.fotogramas
        assert banco.estado.rendimiento.fps_serie()
    finally:
        banco.soltar()


if __name__ == "__main__":
    fallidos = 0

    for nombre, prueba in sorted(globals().items()):
        if not nombre.startswith("test_") or not callable(prueba):
            continue

        try:
            prueba()
            print(f"  ok    {nombre}")
        except Exception as error:                      # noqa: BLE001
            fallidos += 1
            print(f"  FALLA {nombre}: {error!r}")

    print()
    print("todo bien" if not fallidos else f"{fallidos} prueba(s) fallidas")
    sys.exit(1 if fallidos else 0)
