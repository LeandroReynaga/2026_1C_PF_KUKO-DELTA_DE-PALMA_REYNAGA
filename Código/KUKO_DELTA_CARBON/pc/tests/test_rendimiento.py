"""El historial de rendimiento, alimentado con telemetría inventada.

Todo lo que mide este módulo es tiempo, y el tiempo es exactamente lo que no
se puede probar esperando: una corrida de veinte minutos con dos colisiones
tarda veinte minutos en pasar. Así que las pruebas le mienten el reloj —
`Rendimiento` sólo lee `time.time()`— y le meten la telemetría a mano.

La consecuencia práctica: cualquier cambio en el reparto de tiempos, en la
detección de atascos o en la cuenta de la disponibilidad se verifica acá en
milisegundos, sin robot y sin esperar.

Se corre con:  python -m pytest pc/tests    (o  python pc/tests/test_rendimiento.py)
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kuko import estado as est
from kuko import protocolo as pr
from kuko import rendimiento as rd


# ==================================================================
#  Un reloj que avanza cuando la prueba lo dice
# ==================================================================

class Reloj:
    """Reemplaza a `time.time()` dentro del módulo mientras dura la prueba."""

    def __init__(self, inicio: float = 1_700_000_000.0):
        self.ahora = inicio
        self._real = rd.time.time
        rd.time.time = lambda: self.ahora            # type: ignore[assignment]

    def avanzar(self, s: float) -> None:
        self.ahora += s

    def soltar(self) -> None:
        rd.time.time = self._real                    # type: ignore[assignment]


class Robot:
    """Un robot de mentira: se le dice en qué estado está y cuánto dura."""

    def __init__(self):
        self.reloj = Reloj()
        self.r = rd.Rendimiento()
        self.t_ms = 1000
        self.contadores = dict(detectadas=0, depositadas=0, descartadas=0, fallos=0)

    # Mantiene el estado `estado` durante `segundos`, muestreando a 10 Hz
    # como lo hace la línea [T] de verdad.
    def estar(self, estado: pr.EstadoRobot, segundos: float, paso: float = 0.1) -> None:
        pasos = max(1, int(round(segundos / paso)))

        for _ in range(pasos):
            self.reloj.avanzar(paso)
            self.t_ms += int(paso * 1000)
            self.r.observar_telemetria(
                pr.Telemetria(crudo="", t_ms=self.t_ms, estado=estado))

    def proceso(self, estado: pr.EstadoRobot, **cambios) -> None:
        self.contadores.update(cambios)
        self.r.observar_proceso(pr.Proceso(
            crudo="", t_ms=self.t_ms, estado=estado, **self.contadores))

    def ciclo(self, segundos: float = 3.0) -> None:
        """Una maniobra completa: buscar la pieza, agarrarla y depositarla."""

        for e in (pr.EstadoRobot.PICK_APPROACH, pr.EstadoRobot.PICK_DESCEND,
                  pr.EstadoRobot.PICK_LIFT, pr.EstadoRobot.GO_BIN,
                  pr.EstadoRobot.RELEASE_WAIT):
            self.estar(e, segundos / 5.0)

    def soltar(self) -> None:
        self.reloj.soltar()


def _fallo(t_ms: int, numero: int, tipo: str = "COLISION",
           dcmd: float = 40.0, denc: float = 2.0, **extra) -> pr.Fallo:
    return pr.Fallo(crudo="", numero=numero, t_ms=t_ms, tipo=tipo, eje=2,
                    error_deg=13.2, cmd_delta=dcmd, enc_delta=denc,
                    estado_nombre="GO_BIN", **extra)


# ==================================================================
#  Reparto del tiempo
# ==================================================================

def test_cada_estado_cae_en_un_solo_cajon():
    """Un estado nuevo del firmware no puede desaparecer de la contabilidad.

    Los conjuntos de `rendimiento.py` están escritos a mano, así que un
    estado agregado al enum se caería en el `else` final —contado como
    arranque— sin ningún síntoma: la disponibilidad quedaría bien y el
    tiempo estaría en el cajón equivocado. Esta prueba obliga a decidir.
    """

    sueltos = [e.name for e in pr.EstadoRobot
               if e not in rd._TRABAJANDO
               and e not in rd._ESPERANDO
               and e not in rd._PARADO
               and e not in (pr.EstadoRobot.TEACH, pr.EstadoRobot.HOMING,
                             pr.EstadoRobot.IDLE)]

    assert not sueltos, f"estados sin cajon asignado en rendimiento.py: {sueltos}"

    # Y ninguno en dos cajones a la vez.
    assert not (rd._TRABAJANDO & rd._ESPERANDO)
    assert not (rd._TRABAJANDO & rd._PARADO)
    assert not (rd._ESPERANDO & rd._PARADO)


def test_las_severidades_son_las_mismas_que_las_de_estado():
    """Los eventos se pintan con la tabla de colores de los chequeos.

    Están escritos en los dos módulos porque `estado.py` importa a
    `rendimiento.py` y al revés sería circular; si se separan, los eventos
    de la cronología salen todos grises.
    """

    assert (rd.INFO, rd.AVISO, rd.FALLA) == (est.VERDE, est.AMBAR, est.ROJO)


def test_esperar_pieza_no_baja_la_disponibilidad():
    """Sin piezas el robot no está caído: está disponible y ocioso.

    Es la confusión que arruina el indicador. Si esperar contara como
    parada, dejar la cinta vacía cinco minutos daría 0 % de disponibilidad
    con el robot perfecto.
    """

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 60.0)
        resumen = r.r.resumen()

        assert resumen.disponibilidad == 1.0
        assert resumen.utilizacion == 0.0        # disponible, pero sin hacer nada
    finally:
        r.soltar()


def test_una_colision_se_descuenta_de_la_disponibilidad():
    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 80.0)
        r.estar(pr.EstadoRobot.COLLISION_STOP, 5.0)
        r.estar(pr.EstadoRobot.HOMING, 15.0)     # rehoming: sigue siendo tiempo perdido
        r.estar(pr.EstadoRobot.WAIT_PIECE, 100.0)

        resumen = r.r.resumen()

        # 20 s parados de 200 s en servicio.
        assert abs(resumen.tiempos[rd.RECUPERANDO] - 20.0) < 0.5
        assert abs(resumen.disponibilidad - 0.90) < 0.01
    finally:
        r.soltar()


def test_el_homing_del_arranque_no_es_una_parada():
    """Prender el robot no es una falla, y no puede hundir el indicador."""

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.IDLE, 5.0)
        r.estar(pr.EstadoRobot.HOMING, 20.0)
        r.estar(pr.EstadoRobot.WAIT_PIECE, 60.0)

        resumen = r.r.resumen()

        assert resumen.tiempos[rd.RECUPERANDO] == 0.0
        assert resumen.disponibilidad == 1.0
        assert abs(resumen.tiempos[rd.ARRANQUE] - 25.0) < 0.5
    finally:
        r.soltar()


def test_el_modo_teach_queda_afuera_de_la_cuenta():
    """Enseñar un movimiento no es producir ni es estar caído."""

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 30.0)
        r.estar(pr.EstadoRobot.TEACH, 120.0)
        r.estar(pr.EstadoRobot.WAIT_PIECE, 30.0)

        resumen = r.r.resumen()

        assert abs(resumen.tiempos[rd.TEACH] - 120.0) < 0.5
        assert abs(resumen.en_servicio_s - 60.0) < 0.5
        assert resumen.disponibilidad == 1.0
    finally:
        r.soltar()


def test_un_hueco_de_enlace_no_se_le_cuenta_al_robot():
    """Veinte minutos sin cable no son veinte minutos agarrando la pieza."""

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.GO_BIN, 2.0)

        # Se corta el enlace: nadie manda nada durante 20 minutos.
        r.reloj.avanzar(1200.0)
        r.t_ms += 1_200_000
        r.estar(pr.EstadoRobot.WAIT_PIECE, 10.0)

        resumen = r.r.resumen()

        assert abs(resumen.tiempos[rd.SIN_ENLACE] - 1200.0) < 1.0
        assert resumen.tiempos[rd.TRABAJANDO] < 5.0
        assert resumen.en_servicio_s < 20.0

        # Y el hueco aparece dibujado en la cronologia, no como un tramo de
        # trabajo de veinte minutos.
        huecos = [s for s in r.r.cronologia() if s.cajon == rd.SIN_ENLACE]

        assert len(huecos) == 1 and abs(huecos[0].duracion - 1200.0) < 1.0
    finally:
        r.soltar()


# ==================================================================
#  Maniobras y atascos
# ==================================================================

def test_la_maniobra_es_toda_la_tanda_de_trabajo():
    """Los cinco estados de agarrar y depositar son UN ciclo, no cinco."""

    r = Robot()

    try:
        for _ in range(4):
            r.estar(pr.EstadoRobot.WAIT_PIECE, 2.0)
            r.ciclo(3.0)

        r.estar(pr.EstadoRobot.WAIT_PIECE, 2.0)
        resumen = r.r.resumen()

        assert resumen.maniobras == 4
        assert abs(resumen.maniobra_mediana_s - 3.0) < 0.3
        assert resumen.maniobras_interrumpidas == 0
    finally:
        r.soltar()


def test_una_maniobra_cortada_por_una_colision_queda_marcada():
    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 2.0)
        r.estar(pr.EstadoRobot.PICK_DESCEND, 1.5)
        r.estar(pr.EstadoRobot.COLLISION_STOP, 3.0)

        resumen = r.r.resumen()

        assert resumen.maniobras == 1
        assert resumen.maniobras_interrumpidas == 1
    finally:
        r.soltar()


def test_el_brazo_trabado_se_ve_como_atasco_en_curso():
    """La pregunta del enunciado: ¿se trabó? Se contesta sin esperar el fallo.

    El guard tarda su ventana de confirmación en declarar la colisión. Un
    brazo que lleva medio minuto en el mismo movimiento ya está trabado,
    haya o no un `[FALLO]` todavía.
    """

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 10.0)
        r.estar(pr.EstadoRobot.PICK_DESCEND, 30.0)

        resumen = r.r.resumen()

        assert resumen.atasco_s is not None and resumen.atasco_s > rd.MANIOBRA_LARGA_S
        assert resumen.atasco_estado == "PICK_DESCEND"

        veredicto = r.r.veredicto(resumen)

        assert veredicto.severidad == rd.FALLA
        assert "trabado" in veredicto.texto.lower()
    finally:
        r.soltar()


# ==================================================================
#  Fallos
# ==================================================================

def test_un_fallo_repetido_por_el_volcado_no_se_cuenta_dos_veces():
    """`D` vuelca fallos que ya se vieron en vivo. Se deduplica por número."""

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 5.0)

        assert r.r.observar_fallo(_fallo(r.t_ms, numero=1)) is True
        assert r.r.observar_fallo(_fallo(r.t_ms, numero=1)) is False
        assert r.r.observar_fallo(_fallo(r.t_ms, numero=2)) is True

        assert r.r.resumen().fallos_vistos == 2
    finally:
        r.soltar()


def test_el_fallo_viejo_del_volcado_cae_donde_ocurrio():
    """Un fallo de hace tres minutos no puede aparecer fechado ahora.

    Es lo que hace que la cronología sirva: si todo lo que vuelca `D` se
    amontonara en el instante de conectarse, la línea de tiempo diría que
    el robot falló seis veces en el mismo segundo.
    """

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 200.0)
        ahora = r.reloj.ahora

        r.r.observar_fallo(_fallo(r.t_ms - 180_000, numero=1))

        cuando = r.r.lista_eventos()[-1].t

        assert abs((ahora - cuando) - 180.0) < 1.0
    finally:
        r.soltar()


def test_se_distingue_el_brazo_trabado_del_encoder_que_miente():
    """`dcmd` contra `denc`: la pregunta que decide qué se va a arreglar."""

    trabado = _fallo(0, 1, dcmd=40.0, denc=1.5)
    encoder = _fallo(0, 2, dcmd=40.0, denc=39.0)
    chico = _fallo(0, 3, dcmd=0.5, denc=0.1)      # giro demasiado chico: no dice nada
    otro = _fallo(0, 4, tipo="HOMING", dcmd=40.0, denc=0.0)

    assert trabado.brazo_frenado is True
    assert encoder.brazo_frenado is False
    assert chico.brazo_frenado is None
    assert otro.brazo_frenado is None


def test_las_trabas_mandan_en_el_veredicto():
    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 60.0)
        r.r.observar_fallo(_fallo(r.t_ms, 1, dcmd=40.0, denc=1.0))
        r.estar(pr.EstadoRobot.WAIT_PIECE, 60.0)

        resumen = r.r.resumen()

        assert resumen.trabas == 1
        assert "mecanico" in r.r.veredicto(resumen).texto
    finally:
        r.soltar()


def test_el_total_por_tipo_sale_del_firmware_cuando_lo_informa():
    """El registro guarda 16 fallos; el contador del firmware, todos."""

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 60.0)
        r.r.observar_fallo(_fallo(r.t_ms, 1))
        r.r.observar_resumen(pr.parsear(
            "[FALLOS] total=41 COLISION=38 ENCODER=2 HOMING=0 MANUAL=1 "
            "DESCALIBRACION=0 guardados=16"))

        resumen = r.r.resumen()

        assert resumen.fallos_total == 41
        assert resumen.por_tipo["COLISION"] == 38
        assert resumen.fallos_vistos == 1        # lo que vio ESTA corrida
        assert resumen.mtbf_s is not None
    finally:
        r.soltar()


# ==================================================================
#  Producción
# ==================================================================

def test_las_piezas_que_se_pasaron_quedan_en_la_cronologia():
    """Las descartadas son las que el robot NO llegó a agarrar.

    Las que se dejan pasar a propósito en modo caja no las cuenta el
    firmware como descartadas (ver Robot.h), así que este evento no puede
    aparecer por una pieza que no hacía falta.
    """

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 3.0)
        r.proceso(pr.EstadoRobot.WAIT_PIECE, detectadas=10, depositadas=9, descartadas=1)
        r.estar(pr.EstadoRobot.WAIT_PIECE, 3.0)
        r.proceso(pr.EstadoRobot.WAIT_PIECE, detectadas=13, depositadas=10, descartadas=3)

        perdidas = [e for e in r.r.lista_eventos() if e.clase == "pieza"]

        resumen = r.r.resumen()

        assert len(perdidas) == 1
        assert "2 piezas" in perdidas[0].titulo

        # Dos EN ESTA MEDICION; la tercera ya estaba contada cuando empezo.
        # Los tiempos de la pantalla son de esta ventana, asi que las piezas
        # tienen que estar en la misma o los dos numeros mienten juntos.
        assert resumen.descartadas == 2
        assert resumen.descartadas_total == 3
    finally:
        r.soltar()


def test_el_reinicio_del_esp32_corta_la_cronologia():
    """Un `[BOOT]` pone los contadores en cero: no se resta a través de él."""

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.GO_BIN, 5.0)
        r.proceso(pr.EstadoRobot.GO_BIN, detectadas=40, depositadas=38, descartadas=2)

        r.r.observar_boot(pr.Boot(crudo="", proto=pr.VERSION_PROTOCOLO, fw="hoy"))
        r.t_ms = 500
        r.contadores = dict(detectadas=0, depositadas=0, descartadas=0, fallos=0)
        r.estar(pr.EstadoRobot.HOMING, 10.0)
        r.proceso(pr.EstadoRobot.HOMING, detectadas=1)

        resumen = r.r.resumen()

        assert resumen.arranques == 1

        # El `[BOOT]` deja la referencia en cero, asi que la primera pieza de
        # la sesion nueva se cuenta entera.
        assert resumen.detectadas == 1

        # Ninguna pieza "perdida" inventada por la resta contra los 2
        # descartes de antes del reinicio.
        assert not [e for e in r.r.lista_eventos() if e.clase == "pieza"]
    finally:
        r.soltar()


def test_la_curva_acumulada_no_se_desploma_con_un_reinicio():
    """El ESP32 se reinicia y sus contadores vuelven a cero, la curva no.

    Sin rebasar, el grafico mostraria al robot desproduciendo treinta piezas
    de golpe en el medio de la corrida. El reinicio se cuenta en la lista de
    eventos, que es donde corresponde.
    """

    r = Robot()

    try:
        for i in range(6):
            r.estar(pr.EstadoRobot.WAIT_PIECE, 2.0)
            r.proceso(pr.EstadoRobot.WAIT_PIECE,
                      detectadas=30 + i, depositadas=28 + i, descartadas=2)

        r.r.observar_boot(pr.Boot(crudo="", proto=pr.VERSION_PROTOCOLO, fw="hoy"))
        r.t_ms = 400
        r.contadores = dict(detectadas=0, depositadas=0, descartadas=0, fallos=0)

        for i in range(6):
            r.estar(pr.EstadoRobot.WAIT_PIECE, 2.0)
            r.proceso(pr.EstadoRobot.WAIT_PIECE,
                      detectadas=i + 1, depositadas=i + 1, descartadas=0)

        curva = [m.depositadas for m in r.r.serie_acumulada()]

        assert curva == sorted(curva), f"la curva retrocede: {curva}"
        assert curva[0] == 0                     # arranca en cero, no en 28

        # 5 piezas antes del reinicio (de 28 a 33) y 6 despues. La curva y la
        # tarjeta tienen que dar lo mismo: cuentan la misma cosa.
        assert curva[-1] == 11
        assert r.r.resumen().depositadas == 11
    finally:
        r.soltar()


def test_el_ritmo_se_mide_contra_el_tiempo_en_servicio():
    r = Robot()

    try:
        r.proceso(pr.EstadoRobot.WAIT_PIECE, depositadas=0)

        for i in range(20):
            r.estar(pr.EstadoRobot.WAIT_PIECE, 3.0)
            r.ciclo(3.0)
            r.proceso(pr.EstadoRobot.WAIT_PIECE, detectadas=i + 1, depositadas=i + 1)

        resumen = r.r.resumen()

        # 20 piezas en 120 s -> 10 por minuto.
        assert resumen.piezas_por_min is not None
        assert 8.0 < resumen.piezas_por_min < 12.0
        assert resumen.utilizacion is not None and 0.4 < resumen.utilizacion < 0.6
    finally:
        r.soltar()


def test_reiniciar_deja_todo_en_cero_sin_tocar_al_firmware():
    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 30.0)
        r.r.observar_fallo(_fallo(r.t_ms, 1))
        r.r.reiniciar()

        resumen = r.r.resumen()

        assert resumen.fallos_vistos == 0
        assert resumen.en_servicio_s == 0.0
        assert resumen.disponibilidad is None
        assert r.r.cronologia() == []
    finally:
        r.soltar()


# ==================================================================
#  Camara
# ==================================================================

def _camara(r: Robot, fps: float, segundos: float, viva: bool = True,
            detalle: str = "") -> None:
    """Le hace llegar `segundos` de camara al historial, a `fps`.

    Avisa dos veces por segundo, que es lo que hace el hilo de vision de
    verdad cuando la camara no anda (`PERIODO_AVISO_S`); con la camara
    andando avisa mas seguido, pero el historial muestrea a 1 Hz igual.
    """

    paso = 0.5
    fotogramas = r.r.camara_fotogramas

    for _ in range(max(1, int(segundos / paso))):
        r.reloj.avanzar(paso)

        if viva:
            fotogramas += int(round(fps * paso))

        r.r.observar_camara(viva=viva, fotogramas=fotogramas, detalle=detalle)


def test_el_promedio_de_fps_sale_de_contar_fotogramas():
    """Cinco minutos a 30 fps dan 30, y no hay que esperarlos para saberlo."""

    r = Robot()

    try:
        _camara(r, fps=30.0, segundos=300.0)
        resumen = r.r.resumen()

        assert resumen.camara_presente
        assert resumen.camara_viva is True
        assert abs(resumen.fps_5min - 30.0) < 0.5
        assert resumen.camara_disponibilidad == 1.0
    finally:
        r.soltar()


def test_un_rato_sin_camara_le_baja_el_promedio():
    """El punto de promediar contando fotogramas y no promediando tasas.

    Un minuto muerta dentro de la ventana de cinco tiene que bajar el
    numero: es la unica forma de que el promedio diga algo. Promediando FPS
    instantaneos ese minuto no existiria -- no hay muestras que promediar
    con la camara apagada -- y el numero diria que todo anduvo perfecto.
    """

    r = Robot()

    try:
        _camara(r, fps=30.0, segundos=120.0)
        _camara(r, fps=0.0, segundos=60.0, viva=False, detalle="sin imagen")
        _camara(r, fps=30.0, segundos=120.0)

        resumen = r.r.resumen()

        # 240 s a 30 fps repartidos en 300 s -> 24.
        assert abs(resumen.fps_5min - 24.0) < 1.0

        # Y el corte se ve como tal, no diluido en el promedio.
        assert resumen.camara_caidas == 1
        assert abs(resumen.camara_sin_imagen_s - 60.0) < 2.0
        assert abs(resumen.camara_disponibilidad - 0.80) < 0.02
    finally:
        r.soltar()


def test_la_ventana_de_fps_se_mide_sobre_lo_observado():
    """A los treinta segundos de arrancar no se divide por trescientos."""

    r = Robot()

    try:
        _camara(r, fps=25.0, segundos=30.0)

        assert abs(r.r.resumen().fps_5min - 25.0) < 1.5
    finally:
        r.soltar()


def test_la_camara_caida_manda_en_el_veredicto():
    """Es la falla que mejor se disfraza de "hoy no vinieron piezas".

    Sin camara el firmware no recibe una sola pieza, el robot se queda en
    WAIT_PIECE y la disponibilidad da 100 % sin un solo fallo. Ningun otro
    numero de la pantalla la delata, asi que va primero.
    """

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 300.0)
        _camara(r, fps=0.0, segundos=20.0, viva=False, detalle="sin imagen")

        resumen = r.r.resumen()

        assert resumen.disponibilidad == 1.0     # el robot, impecable
        assert resumen.fallos_total == 0

        veredicto = r.r.veredicto(resumen)

        assert veredicto.severidad == rd.FALLA
        assert "camara" in veredicto.texto.lower()
    finally:
        r.soltar()


def test_un_hilo_de_vision_mudo_no_pasa_por_camara_sana():
    """Que deje de informar no es lo mismo que informar que anda mal.

    Si el hilo de vision se cae, nadie manda nada y el ultimo aviso queda
    diciendo "todo bien". Sin este vencimiento, una vision muerta se
    dibujaria como una camara perfecta congelada en el ultimo dato.
    """

    r = Robot()

    try:
        _camara(r, fps=30.0, segundos=30.0)
        assert r.r.resumen().camara_viva is True

        r.reloj.avanzar(rd.CAMARA_MUDA_S + 2.0)
        resumen = r.r.resumen()

        assert resumen.camara_muda is True
        assert resumen.camara_viva is False
        assert "vision" in r.r.veredicto(resumen).texto.lower()
    finally:
        r.soltar()


def test_sin_vision_no_se_inventa_una_camara():
    """Con --sin-vision no falta nada: se pidio que no estuviera."""

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 60.0)
        resumen = r.r.resumen()

        assert resumen.camara_presente is False
        assert resumen.camara_viva is None
        assert resumen.fps_5min is None
        assert r.r.veredicto(resumen).severidad != rd.FALLA
    finally:
        r.soltar()


def test_la_serie_de_fps_marca_el_corte_con_ceros_y_no_con_un_hueco():
    """Un cero dibujado es una caida; un hueco parece el grafico cargando."""

    r = Robot()

    try:
        _camara(r, fps=30.0, segundos=20.0)
        _camara(r, fps=0.0, segundos=10.0, viva=False)
        _camara(r, fps=30.0, segundos=20.0)

        serie = r.r.fps_serie()
        ceros = [v for _, v in serie if v < 1.0]

        assert ceros, "el corte no quedo dibujado"
        assert len(serie) > 40, "se perdieron muestras"
        assert max(v for _, v in serie) > 25.0
    finally:
        r.soltar()


def test_una_camara_lenta_se_avisa_sin_estar_caida():
    """Por debajo de FPS_MINIMOS el tracker deja de emparejar las piezas."""

    r = Robot()

    try:
        r.estar(pr.EstadoRobot.WAIT_PIECE, 300.0)
        _camara(r, fps=4.0, segundos=120.0)

        resumen = r.r.resumen()
        veredicto = r.r.veredicto(resumen)

        assert resumen.camara_viva is True
        assert resumen.fps_5min < rd.FPS_MINIMOS
        assert veredicto.severidad == rd.AVISO
        assert "fps" in veredicto.texto.lower()
    finally:
        r.soltar()


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
