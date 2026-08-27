"""Historial de rendimiento: cuánto trabajó el robot y cuánto estuvo parado.

El firmware lleva CONTADORES (piezas detectadas, depositadas, descartadas,
fallos) pero no lleva historia: no sabe cuándo pasó cada cosa ni cuánto
tiempo estuvo en cada estado, y no debería — el ESP32 tiene 320 kB de RAM y
todo lo que gaste en recordar el pasado se lo saca a la generación de pasos.
Guardar la historia es exactamente el trabajo de la PC, que ya está leyendo
todas las líneas igual.

Este módulo es esa memoria. Entra telemetría, salen tres cosas:

  * **cuánto tiempo** pasó el robot en cada situación (produciendo,
    esperando pieza, o parado por una falla), que es de donde sale la
    disponibilidad;
  * **cuándo** ocurrió cada evento, que es de donde salen la cronología y
    los gráficos contra el tiempo;
  * **por qué**, cuando se puede: un fallo de colisión trae los grados que
    se le ordenaron al eje y los que giró de verdad, y con eso se distingue
    un brazo trabado de un encoder que mide mal.

No toca la interfaz ni el puerto: entra `pr.Mensaje`, sale estado. Eso lo
hace verificable sin robot, que es lo mismo que se buscó en `protocolo.py`.

Tres decisiones que conviene entender antes de tocarlo:

**El reloj es el del ESP32, no el de la PC.** Cada línea trae su `millis()`
y de ahí sale la posición de todo en la cronología. La hora de pared se
obtiene anclando: se guarda qué hora era en la PC cuando llegó el último
`millis()`, y con esa pareja se traduce cualquier otro. Hace falta porque el
volcado del comando `D` trae fallos VIEJOS —hasta 16, de minutos atrás— y
fecharlos con la hora en que llegó el mensaje los amontonaría todos juntos
en el instante de conectarse, que es justo lo contrario de una cronología.

**Un hueco entre dos muestras no es tiempo del robot.** Si el cable se
desenchufa veinte minutos, la próxima muestra llega con un salto de veinte
minutos, y sumárselos al estado que estaba corriendo diría que el robot
estuvo media hora agarrando la misma pieza. Todo salto mayor a
`HUECO_MAX_S` se contabiliza aparte, como tiempo sin enlace, y no entra en
la cuenta de la disponibilidad: no se afirma nada sobre lo que no se vio.

**La disponibilidad se mide contra el tiempo en que el robot ESTABA para
trabajar.** El homing del arranque, el modo teach y el rato apagado no son
paradas: no van ni al numerador ni al denominador. Contarlos hundiría el
número por motivos que no tienen nada que ver con la confiabilidad del
robot, y un indicador que baja cuando uno enseña un movimiento es un
indicador que nadie va a mirar.
"""

from __future__ import annotations

import statistics
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from typing import Optional

from . import protocolo as pr

# Severidad de un evento. Son a propósito los mismos tres textos que
# `estado.VERDE` / `AMBAR` / `ROJO`, para que la interfaz pinte los eventos
# con la misma tabla de colores que los puntitos de componentes. Están
# escritos acá en vez de importados porque `estado.py` importa este módulo y
# al revés sería circular; `test_rendimiento.py` verifica que coincidan.
INFO = "ok"
AVISO = "aviso"
FALLA = "falla"

# ------------------------------------------------------------------
#  Reparto del tiempo
# ------------------------------------------------------------------
# Cada estado del robot cae en exactamente uno de estos cajones, y de cómo
# se reparten sale la disponibilidad. Los tres primeros son "el robot estaba
# encendido para producir"; los tres últimos, no.
TRABAJANDO = "trabajando"      # maniobrando con una pieza
ESPERANDO = "esperando"        # listo, sin pieza que agarrar
RECUPERANDO = "recuperando"    # parado por una falla, o rehomeando por una
ARRANQUE = "arranque"          # encendido y homing inicial
TEACH = "teach"                # el operador manejando el brazo a mano
SIN_ENLACE = "sin_enlace"      # no se vio nada; no se afirma nada

CAJONES = (TRABAJANDO, ESPERANDO, RECUPERANDO, ARRANQUE, TEACH, SIN_ENLACE)

# Los que cuentan para la disponibilidad: tiempo en que el robot estaba
# puesto a producir. `TRABAJANDO` y `ESPERANDO` son tiempo bueno,
# `RECUPERANDO` es tiempo perdido.
PRODUCTIVOS = (TRABAJANDO, ESPERANDO)
EN_SERVICIO = (TRABAJANDO, ESPERANDO, RECUPERANDO)

_TRABAJANDO = frozenset({
    pr.EstadoRobot.PICK_APPROACH,
    pr.EstadoRobot.PICK_DESCEND,
    pr.EstadoRobot.PICK_LIFT,
    pr.EstadoRobot.GO_BIN,
    pr.EstadoRobot.BIN_SETTLE,
    pr.EstadoRobot.RELEASE_WAIT,
    pr.EstadoRobot.BOX_TRANSIT,
    pr.EstadoRobot.BOX_APPROACH,
    pr.EstadoRobot.BOX_DESCEND,
    pr.EstadoRobot.BOX_LIFT,
})

_ESPERANDO = frozenset({
    pr.EstadoRobot.WAIT_PIECE,
    pr.EstadoRobot.GO_HOME_IDLE,
})

_PARADO = frozenset({
    pr.EstadoRobot.COLLISION_STOP,
    pr.EstadoRobot.ERROR,
})

# ------------------------------------------------------------------
#  Topes
# ------------------------------------------------------------------
# Salto entre dos muestras por encima del cual se da por perdido el enlace
# en vez de estirar el último estado conocido. La telemetría rápida viene a
# 10 Hz y la de proceso a 1 Hz, así que 2,5 s ya es un enlace que no está.
HUECO_MAX_S = 2.5

# Cuánto se guarda. Son colas de largo fijo: la interfaz puede quedar
# abierta días entre demostraciones y la memoria no puede crecer sin freno.
MAX_SEGMENTOS = 1200           # tramos de la cronología (uno por transición)
MAX_MUESTRAS = 7200            # foto de los contadores, 1 Hz -> 2 horas
MAX_EVENTOS = 400
MAX_MANIOBRAS = 800
MAX_FALLOS = 200               # el firmware guarda 16; acá entran todos los vistos

# Cada cuánto se guarda una foto de los contadores para los gráficos contra
# el tiempo. `[E]` viene a 1 Hz de fábrica pero su período es un parámetro
# ajustable, así que el ritmo del muestreo se decide acá y no allá.
PERIODO_MUESTRA_S = 1.0

# Ventana de la que sale el "ritmo" instantáneo, en segundos. Un minuto es
# suficiente para que el número no salte con cada pieza y suficientemente
# corto para que se note cuando la cinta se vacía.
VENTANA_RITMO_S = 60.0

# Una maniobra que pasa de esto es un robot trabado, no un robot lento: el
# ciclo completo de agarrar y depositar está en el orden de los 3 s.
MANIOBRA_LARGA_S = 12.0

# Por debajo de estas vueltas por segundo hay algo bloqueando el loop del
# ESP32. En marcha normal ronda las 1000.
LOOP_HZ_MINIMO = 200

# Los contadores de produccion del firmware que esta clase lleva rebasados a
# la ventana de medicion. Estan escritos una sola vez porque los tres se
# tratan igual: se restan contra la referencia y se pliegan en un acumulado
# cuando el ESP32 se reinicia.
CONTADORES = ("detectadas", "depositadas", "descartadas")

# ------------------------------------------------------------------
#  Camara
# ------------------------------------------------------------------
# Sobre cuanto tiempo se promedian los FPS. Cinco minutos: lo suficiente
# para que un fotograma perdido no mueva el numero, y lo suficientemente
# corto para que se note en el acto cuando la camara empieza a atrasarse.
VENTANA_FPS_S = 300.0

# Sin noticias de la vision por mas de esto, se da por muerto el hilo. Es
# distinto de "la camara no entrega imagen": eso lo informa la propia vision
# y se ve como tal. Esto es que la vision dejo de informar, o sea que el hilo
# se cayo, y no tenerlo haria que un hilo muerto se dibujara como una camara
# perfecta congelada en el ultimo dato.
CAMARA_MUDA_S = 3.0

# Por debajo de estos FPS la vision no llega a seguir las piezas: el tracker
# empareja por cercania de centroides entre fotogramas, y con la cinta a
# 7 cm/s una pieza avanza casi un centimetro entre uno y otro.
FPS_MINIMOS = 8.0


# ==================================================================
#  Piezas de la historia
# ==================================================================

@dataclass
class Segmento:
    """Un tramo continuo en el mismo estado. Es un renglón de la cronología."""

    cajon: str
    estado: Optional[pr.EstadoRobot]
    inicio: float                  # hora de pared (epoch)
    fin: float

    @property
    def duracion(self) -> float:
        return max(0.0, self.fin - self.inicio)


@dataclass
class Muestra:
    """Foto de los contadores en un instante. Alimenta los gráficos."""

    t: float
    detectadas: int = 0
    depositadas: int = 0
    descartadas: int = 0
    fallos: int = 0
    cola: int = 0
    loop_hz: Optional[int] = None


@dataclass
class MuestraCamara:
    """Foto del contador de fotogramas. De la resta salen los FPS."""

    t: float
    fotogramas: int
    viva: bool


@dataclass
class Maniobra:
    """Un intento completo de agarrar y depositar una pieza.

    Es la tanda continua de estados de trabajo, del primer movimiento hacia
    la pieza hasta que el brazo vuelve a quedar libre. Su duración es el
    tiempo de ciclo REAL del robot: no incluye la espera de la pieza
    siguiente, que depende de la cinta y no de la mecánica.
    """

    inicio: float
    fin: float
    interrumpida: bool = False     # la cortó una colisión o un error

    @property
    def duracion(self) -> float:
        return max(0.0, self.fin - self.inicio)


@dataclass
class Evento:
    """Algo que pasó, con su hora. Es un renglón de la lista de eventos."""

    t: float
    clase: str                     # "fallo", "parada", "pieza", "enlace", "modo"
    titulo: str
    detalle: str = ""
    severidad: str = INFO


@dataclass
class Veredicto:
    """La frase que contesta '¿qué está pasando?' sin leer ningún gráfico."""

    severidad: str = INFO
    texto: str = "sin datos todavia"


@dataclass
class Resumen:
    """Todo lo que la interfaz dibuja, calculado de una sola pasada."""

    desde: Optional[float] = None
    tiempos: dict[str, float] = field(default_factory=dict)
    en_servicio_s: float = 0.0
    disponibilidad: Optional[float] = None
    utilizacion: Optional[float] = None

    # Piezas DESDE QUE EMPEZO ESTA MEDICION, no desde que se encendio el
    # ESP32. Tienen que estar en la misma ventana que los tiempos: si la
    # disponibilidad dice "12 min parado de 20 min" al lado de un contador de
    # piezas de hace tres horas, los dos numeros mienten juntos y no hay
    # forma de darse cuenta. Los totales del firmware van aparte.
    detectadas: Optional[int] = None
    depositadas: Optional[int] = None
    descartadas: Optional[int] = None
    detectadas_total: Optional[int] = None
    depositadas_total: Optional[int] = None
    descartadas_total: Optional[int] = None
    efectividad: Optional[float] = None
    piezas_por_min: Optional[float] = None
    ritmo_reciente: Optional[float] = None

    maniobras: int = 0
    maniobra_mediana_s: Optional[float] = None
    maniobra_peor_s: Optional[float] = None
    maniobras_interrumpidas: int = 0

    fallos_total: int = 0
    fallos_vistos: int = 0
    por_tipo: dict[str, int] = field(default_factory=dict)
    por_estado: dict[str, int] = field(default_factory=dict)
    por_eje: dict[int, int] = field(default_factory=dict)
    mtbf_s: Optional[float] = None
    trabas: int = 0
    piezas_caidas: int = 0

    atasco_s: Optional[float] = None
    atasco_estado: str = ""
    loop_hz_min: Optional[int] = None
    arranques: int = 0

    # ---------------- Camara ----------------
    #: Nunca informo nada: o se arranco con --sin-vision, o el hilo de vision
    #: no llego a arrancar. Es distinto de una camara caida.
    camara_presente: bool = False

    #: None mientras no se sepa; False es "esta y no entrega imagen".
    camara_viva: Optional[bool] = None
    camara_detalle: str = ""
    camara_muda: bool = False          # el hilo de vision dejo de informar

    fps_5min: Optional[float] = None
    fps_ahora: Optional[float] = None
    fps_peor: Optional[float] = None
    camara_caidas: int = 0
    camara_con_imagen_s: float = 0.0
    camara_sin_imagen_s: float = 0.0
    camara_disponibilidad: Optional[float] = None
    fotogramas: int = 0


# ==================================================================
#  El historiador
# ==================================================================

class Rendimiento:
    """Junta la historia de una corrida. Lo escribe el enlace, lo lee la UI.

    Todos los `observar_*` corren en el hilo del enlace y todos los lectores
    en el del servidor web, así que hay un candado. Es barato —unas pocas
    operaciones por mensaje— y evita que la interfaz agarre la mitad de una
    actualización y dibuje un reparto de tiempos que no suma.
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.reiniciar()

    # ------------------------------------------------------------------
    def reiniciar(self) -> None:
        """Empieza a medir de nuevo desde ahora.

        Los contadores del firmware NO se tocan: siguen contando desde que
        se encendió el ESP32. Lo que se reinicia es la medición de la PC,
        que es lo que uno quiere cuando arranca una corrida nueva y no le
        interesa arrastrar los fallos de la prueba anterior.
        """

        with self._lock:
            self.desde: Optional[float] = None
            self.tiempos: dict[str, float] = {c: 0.0 for c in CAJONES}

            self.segmentos: deque[Segmento] = deque(maxlen=MAX_SEGMENTOS)
            self.muestras: deque[Muestra] = deque(maxlen=MAX_MUESTRAS)
            self.eventos: deque[Evento] = deque(maxlen=MAX_EVENTOS)
            self.maniobras: deque[Maniobra] = deque(maxlen=MAX_MANIOBRAS)
            self.fallos: deque[pr.Fallo] = deque(maxlen=MAX_FALLOS)

            # Totales por tipo que informa `[FALLOS]`. Son los del firmware
            # desde el encendido, así que pueden ser MAYORES que los fallos
            # que esta medición vio: el registro guarda 16 y el contador no
            # se pierde aunque el buffer dé la vuelta.
            self.total_firmware: Optional[int] = None
            self.por_tipo_firmware: dict[str, int] = {}

            self._vistos: set[tuple[int, int]] = set()   # (sesion, numero de fallo)
            self._sesion = 0
            self._arranques = 0

            # Ancla del reloj: qué hora de la PC era cuando llegó este
            # `millis()` del ESP32.
            self._ancla_ms: Optional[int] = None
            self._ancla_wall: float = 0.0

            self._prev_wall: Optional[float] = None
            self._abierto: Optional[Segmento] = None

            # El estado de la muestra ANTERIOR. Se lleva aparte del tramo
            # abierto porque el tramo se cierra antes de abrir el siguiente,
            # y para entonces ya no queda de donde leerlo. Es lo unico que
            # distingue un rehoming despues de un choque del homing de
            # puesta en marcha.
            self._estado_previo: Optional[pr.EstadoRobot] = None
            self._homing_recupera = False

            self._maniobra_inicio: Optional[float] = None
            self._ultima_muestra: float = 0.0

            self._base: Optional[pr.Proceso] = None

            # Los contadores del firmware valian esto cuando empezo la
            # sesion actual, y esto es lo que se produjo en las sesiones
            # anteriores de la misma medicion. Hacen falta los dos porque el
            # ESP32 puede reiniciarse a mitad de una corrida: sus contadores
            # vuelven a cero y sin el acumulado lo producido antes del
            # reinicio se perderia de la pantalla.
            self._referencia: Optional[dict[str, int]] = None
            self._acumulado: dict[str, int] = {c: 0 for c in CONTADORES}

            self._loop_hz_min: Optional[int] = None
            self._rsy: list[Optional[int]] = [None] * pr.NUM_EJES

            # ---------------- Camara ----------------
            self.camara: deque[MuestraCamara] = deque(maxlen=MAX_MUESTRAS)
            self.camara_tiempos = {"con_imagen": 0.0, "sin_imagen": 0.0}
            self.camara_caidas = 0
            self.camara_detalle = ""
            self.camara_fotogramas = 0

            self._camara_viva: Optional[bool] = None
            self._camara_ultimo_aviso: Optional[float] = None
            self._camara_muestra: float = 0.0

    # ------------------------------------------------------------------
    #  Reloj
    # ------------------------------------------------------------------
    def _anclar(self, t_ms: Optional[int]) -> float:
        """Fija la pareja (millis del ESP32, hora de la PC) y devuelve la hora.

        Se reancla con cada mensaje: la deriva entre los dos relojes es de
        partes por millón y no importa, pero reanclar hace que un `millis()`
        que volvió a empezar —un reinicio del ESP32— se note en el acto en
        vez de mandar todo lo nuevo cuarenta minutos al pasado.
        """

        ahora = time.time()

        if t_ms is None:
            return ahora

        if self._ancla_ms is None or t_ms < self._ancla_ms:
            self._ancla_ms, self._ancla_wall = t_ms, ahora
            return ahora

        self._ancla_ms, self._ancla_wall = t_ms, ahora
        return ahora

    def instante(self, t_ms: Optional[int]) -> float:
        """Traduce un `millis()` del ESP32 a hora de pared.

        Para los mensajes que llegan en vivo da lo mismo que `time.time()`.
        Sirve para los que NO llegan en vivo: los fallos que vuelca el
        comando `D`, que ocurrieron hace minutos y tienen que caer en la
        cronología donde ocurrieron, no donde se los leyó.
        """

        with self._lock:
            if t_ms is None or self._ancla_ms is None:
                return time.time()

            return self._ancla_wall - (self._ancla_ms - t_ms) / 1000.0

    # ------------------------------------------------------------------
    #  Entradas
    # ------------------------------------------------------------------
    def observar_telemetria(self, t: pr.Telemetria) -> None:
        """`[T]`, 10 Hz. Es lo que le da resolución a la cronología."""

        with self._lock:
            self._muestrear(t.t_ms, t.estado)

    def observar_proceso(self, e: pr.Proceso) -> None:
        """`[E]`, 1 Hz. Trae los contadores de producción."""

        with self._lock:
            ahora = self._muestrear(e.t_ms, e.estado)
            self._contadores(e, ahora)

    def observar_salud(self, h: pr.Salud) -> None:
        """`[H]`, cada 2 s. De acá salen las vueltas de loop y las resincronizaciones."""

        with self._lock:
            ahora = self.instante(h.t_ms)

            if h.loop_hz:
                if self._loop_hz_min is None or h.loop_hz < self._loop_hz_min:
                    self._loop_hz_min = h.loop_hz

                    if h.loop_hz < LOOP_HZ_MINIMO:
                        self._anotar(ahora, "atasco", "El loop del ESP32 se frenó",
                                     f"{h.loop_hz} vueltas/s (lo normal es ~1000)",
                                     FALLA)

            # Un canal de encoder que se reengancha deja al guard sin
            # supervisar ese eje hasta el próximo homing. No es un fallo
            # —no para el robot— pero explica por qué dejó de avisar.
            for i, eje in enumerate(h.ejes):
                if eje.resincronizaciones is None:
                    continue

                previo = self._rsy[i]
                self._rsy[i] = eje.resincronizaciones

                if previo is not None and eje.resincronizaciones > previo:
                    self._anotar(ahora, "encoder",
                                 f"Encoder {i + 1} se resincronizó",
                                 "el guard deja de supervisar ese eje hasta el proximo homing",
                                 AVISO)

    def observar_camara(self, viva: bool, fotogramas: int,
                        detalle: str = "") -> None:
        """Lo llama el hilo de vision, con o sin camara. Ver `Vision._avisar`.

        Llega el CONTADOR de fotogramas, no una tasa: el promedio de los
        ultimos cinco minutos es una resta entre dos lecturas dividida por el
        tiempo entre ellas, y asi los ratos sin imagen bajan el promedio
        solos. Con una tasa ya calculada habria que acordarse de mandar ceros
        mientras la camara no esta, que es justo cuando nadie manda nada.
        """

        with self._lock:
            ahora = time.time()
            self.camara_detalle = detalle
            self.camara_fotogramas = fotogramas

            # Reparto del tiempo de la camara, con el mismo criterio que el
            # del robot: un hueco grande entre dos avisos no se le cuenta a
            # ninguno de los dos lados, porque no se vio.
            if self._camara_ultimo_aviso is not None:
                dt = ahora - self._camara_ultimo_aviso

                if 0.0 < dt <= HUECO_MAX_S and self._camara_viva is not None:
                    cajon = "con_imagen" if self._camara_viva else "sin_imagen"
                    self.camara_tiempos[cajon] += dt

            self._camara_ultimo_aviso = ahora

            if viva != self._camara_viva:
                if self._camara_viva is None:
                    if not viva:
                        self._anotar(ahora, "camara", "La camara no arranco",
                                     detalle or "no entrega imagen", FALLA)
                elif viva:
                    self._anotar(ahora, "camara", "La camara volvio",
                                 "vuelve a entregar imagen", AVISO)
                else:
                    self.camara_caidas += 1
                    self._anotar(
                        ahora, "camara", "La camara dejo de entregar imagen",
                        detalle or "sin ella el robot no ve una sola pieza",
                        FALLA)

                self._camara_viva = viva

            if ahora - self._camara_muestra >= PERIODO_MUESTRA_S:
                self._camara_muestra = ahora
                self.camara.append(MuestraCamara(ahora, fotogramas, viva))

    def observar_fallo(self, f: pr.Fallo) -> bool:
        """Un `[FALLO]`. Devuelve False si ya se había visto.

        La misma línea llega dos veces por diseño: una en vivo cuando el
        fallo ocurre, y otra cuando el volcado de `D` repasa el registro.
        Quien llama usa el valor de retorno para no repetir el aviso en la
        consola por algo que pasó hace media hora.
        """

        with self._lock:
            clave = (self._sesion, f.numero if f.numero is not None else -1)

            if f.numero is not None and clave in self._vistos:
                return False

            self._vistos.add(clave)

            cuando = self.instante(f.t_ms)
            self.fallos.append(f)

            # Se ordena por hora porque el volcado de `D` intercala fallos
            # viejos entre los que ya se habían visto en vivo.
            ordenados = sorted(self.fallos, key=lambda x: self.instante(x.t_ms))
            self.fallos.clear()
            self.fallos.extend(ordenados)

            donde = f.estado_nombre or (f.estado.name if f.estado is not None else "?")
            detalle = f"eje {f.eje or '-'} · {donde}"

            if f.error_deg is not None:
                detalle += f" · error {f.error_deg:+.1f}°"

            if f.en_mano:
                detalle += " · con la pieza en la mano"

            self._anotar(cuando, "fallo", f"Fallo {f.tipo or '?'}", detalle, FALLA)

            return True

    def observar_resumen(self, r: pr.ResumenFallos) -> None:
        """`[FALLOS]`: los totales por tipo que lleva el firmware."""

        with self._lock:
            if r.total is not None:
                self.total_firmware = r.total

            if r.por_tipo:
                self.por_tipo_firmware = dict(r.por_tipo)

    def observar_boot(self, b: pr.Boot) -> None:
        """`[BOOT]`: el ESP32 arrancó. Todos sus contadores volvieron a cero."""

        with self._lock:
            ahora = time.time()
            self._sesion += 1
            self._arranques += 1
            self._plegar()                  # antes de soltar `_base`: lo necesita
            self._base = None

            # Un `[BOOT]` dice que los contadores del firmware valen cero AHORA,
            # asi que esa es la referencia. Sin esto la referencia la fijaria
            # la primera `[E]` que llegue y se perderian las piezas contadas
            # entre el arranque y ella.
            self._referencia = {c: 0 for c in CONTADORES}
            self._ancla_ms = None
            self._rsy = [None] * pr.NUM_EJES

            # Se corta el tramo abierto: lo que venga después es de otra
            # sesión y estirar el estado anterior a través del reinicio
            # dibujaría un robot que nunca se apagó.
            self._cerrar(ahora)
            self._prev_wall = None
            self._estado_previo = None

            self._anotar(ahora, "arranque", "Arrancó el firmware",
                         f"proto={b.proto} fw={b.fw}", AVISO)

    def enlace_caido(self) -> None:
        """Se perdió el puerto. Cierra el tramo abierto para no estirarlo."""

        with self._lock:
            if self._prev_wall is None:
                return

            self._cerrar(time.time())
            self._prev_wall = None
            self._estado_previo = None
            self._anotar(time.time(), "enlace", "Se perdió el enlace",
                         "no se sabe qué hizo el robot mientras tanto", AVISO)

    # ------------------------------------------------------------------
    #  Reparto del tiempo
    # ------------------------------------------------------------------
    def _cajon(self, estado: Optional[pr.EstadoRobot]) -> str:
        if estado is None:
            return SIN_ENLACE

        if estado in _TRABAJANDO:
            return TRABAJANDO

        if estado in _ESPERANDO:
            return ESPERANDO

        if estado in _PARADO:
            return RECUPERANDO

        if estado is pr.EstadoRobot.TEACH:
            return TEACH

        if estado is pr.EstadoRobot.HOMING:
            # El homing del arranque y el que sigue a un choque son el mismo
            # estado y no significan lo mismo: uno es puesta en marcha y el
            # otro es tiempo perdido por una falla. Se distinguen por lo que
            # venía antes, que es lo único que los diferencia.
            return RECUPERANDO if self._homing_recupera else ARRANQUE

        return ARRANQUE       # IDLE, y cualquier estado que este modulo no conozca

    def _muestrear(self, t_ms: Optional[int], estado: Optional[pr.EstadoRobot]) -> float:
        """Suma el tiempo transcurrido al cajón que corresponde y sigue la cronología."""

        ahora = self._anclar(t_ms)

        if self.desde is None:
            self.desde = ahora

        if self._prev_wall is None:
            self._abrir(estado, ahora)
            self._prev_wall = ahora
            self._estado_previo = estado
            return ahora

        dt = ahora - self._prev_wall

        if dt < 0.0:
            # Mensajes fuera de orden (sólo pasa con un reinicio a mitad de
            # línea). No se inventa tiempo: se reancla y se sigue.
            self._prev_wall = ahora
            return ahora

        if dt > HUECO_MAX_S:
            self._cerrar(self._prev_wall)
            self.tiempos[SIN_ENLACE] += dt
            self.segmentos.append(
                Segmento(SIN_ENLACE, None, self._prev_wall, ahora))
            self._cerrar_maniobra(self._prev_wall, interrumpida=False)

            # Del otro lado del hueco no se sabe de dónde viene el robot,
            # así que el estado anterior se olvida: un homing después de un
            # corte es puesta en marcha, no recuperación de un choque que
            # nadie vio.
            self._estado_previo = None
            self._abrir(estado, ahora)
            self._prev_wall = ahora
            self._estado_previo = estado
            return ahora

        # El tiempo transcurrido pertenece al estado en que el robot ESTABA,
        # no al que acaba de informar: entre las dos muestras estuvo en el
        # anterior.
        if self._abierto is not None:
            self.tiempos[self._abierto.cajon] += dt

        if self._abierto is None or estado is not self._abierto.estado:
            self._cerrar(ahora)
            self._abrir(estado, ahora)
        else:
            self._abierto.fin = ahora

        self._prev_wall = ahora
        self._estado_previo = estado
        return ahora

    def _abrir(self, estado: Optional[pr.EstadoRobot], cuando: float) -> None:
        anterior = self._estado_previo

        if estado is pr.EstadoRobot.HOMING:
            # Pegajoso mientras dure el homing: la decisión se toma al
            # entrar, porque una vez adentro ya no queda rastro de por qué
            # se está rehomeando.
            if anterior is not pr.EstadoRobot.HOMING:
                self._homing_recupera = anterior in _PARADO
        else:
            self._homing_recupera = False

        segmento = Segmento(self._cajon(estado), estado, cuando, cuando)
        self._abierto = segmento

        # Una maniobra es la tanda continua de estados de trabajo. Empieza
        # cuando el brazo sale a buscar una pieza y termina cuando vuelve a
        # quedar libre, sin importar por cuántos estados haya pasado.
        if segmento.cajon == TRABAJANDO:
            if self._maniobra_inicio is None:
                self._maniobra_inicio = cuando
        else:
            self._cerrar_maniobra(cuando, interrumpida=segmento.cajon == RECUPERANDO)

        if segmento.cajon == RECUPERANDO and anterior not in _PARADO:
            self._anotar(cuando, "parada",
                         "El robot se detuvo",
                         f"estado {estado.name if estado is not None else '?'}", FALLA)

    def _cerrar(self, cuando: float) -> None:
        if self._abierto is None:
            return

        self._abierto.fin = max(self._abierto.fin, cuando)
        self.segmentos.append(self._abierto)
        self._abierto = None

    def _cerrar_maniobra(self, cuando: float, interrumpida: bool) -> None:
        if self._maniobra_inicio is None:
            return

        self.maniobras.append(Maniobra(self._maniobra_inicio, cuando, interrumpida))
        self._maniobra_inicio = None

    # ------------------------------------------------------------------
    #  Contadores de producción
    # ------------------------------------------------------------------
    def _contadores(self, e: pr.Proceso, ahora: float) -> None:
        previo = self._base
        self._base = e
        self._seguir_referencia(e)

        # Un contador que baja es el ESP32 que se reinició. No se anota
        # nada: `observar_boot` ya puso el evento y restar daría negativo.
        if previo is not None and e.descartadas is not None \
                and previo.descartadas is not None \
                and e.descartadas > previo.descartadas:
            perdidas = e.descartadas - previo.descartadas
            self._anotar(
                ahora, "pieza",
                f"{perdidas} pieza{'s' if perdidas > 1 else ''} sin agarrar",
                "el brazo no llegaba a tiempo; se dejo pasar", AVISO)

        if previo is not None and e.modo is not None and previo.modo is not None \
                and e.modo is not previo.modo:
            self._anotar(ahora, "modo", f"Modo {e.modo.name}", "", INFO)

        if ahora - self._ultima_muestra < PERIODO_MUESTRA_S:
            return

        self._ultima_muestra = ahora
        self.muestras.append(Muestra(
            t=ahora,
            detectadas=e.detectadas or 0,
            depositadas=e.depositadas or 0,
            descartadas=e.descartadas or 0,
            fallos=e.fallos or 0,
            cola=e.cola or 0,
            loop_hz=self._loop_hz_min,
        ))

    def _anotar(self, cuando: float, clase: str, titulo: str,
                detalle: str = "", severidad: str = INFO) -> None:
        self.eventos.append(Evento(cuando, clase, titulo, detalle, severidad))

    # ------------------------------------------------------------------
    #  Salidas
    # ------------------------------------------------------------------
    def cronologia(self) -> list[Segmento]:
        """Los tramos, incluido el que está abierto ahora mismo."""

        with self._lock:
            tramos = list(self.segmentos)

            if self._abierto is not None:
                tramos.append(Segmento(self._abierto.cajon, self._abierto.estado,
                                       self._abierto.inicio, self._abierto.fin))

            return tramos

    def serie(self) -> list[Muestra]:
        with self._lock:
            return list(self.muestras)

    def serie_acumulada(self) -> list[Muestra]:
        """La serie con los contadores rebasados al comienzo de la medición.

        Es lo que dibujan las curvas acumuladas, y no las muestras crudas,
        por dos motivos que van juntos: los contadores crudos arrancan en lo
        que el firmware llevara cuando se abrió la interfaz —una curva que
        empieza en 143 no se lee—, y si el ESP32 se reinicia a mitad de la
        corrida vuelven a cero y la curva se desploma en el medio como si el
        robot hubiera desproducido ciento cuarenta piezas.

        Acá el reinicio no se ve: lo producido antes se guarda y la curva
        sigue de largo. El reinicio en sí queda en la lista de eventos, que
        es donde corresponde contarlo.
        """

        with self._lock:
            muestras = list(self.muestras)

        if not muestras:
            return []

        base = {c: getattr(muestras[0], c) for c in CONTADORES}
        llevado = {c: 0 for c in CONTADORES}
        previo = muestras[0]
        salida: list[Muestra] = []

        for m in muestras:
            # Mismo criterio que `_seguir_referencia`, y tiene que serlo: si
            # la curva y el numero de la tarjeta contaran distinto, uno de
            # los dos estaria mintiendo y no habria forma de saber cual.
            if any(getattr(m, c) < getattr(previo, c) for c in CONTADORES):
                for c in CONTADORES:
                    llevado[c] += max(0, getattr(previo, c) - base[c])
                    base[c] = 0

            salida.append(Muestra(
                t=m.t,
                cola=m.cola,
                fallos=m.fallos,
                loop_hz=m.loop_hz,
                **{c: llevado[c] + max(0, getattr(m, c) - base[c])
                   for c in CONTADORES}))

            previo = m

        return salida

    def ritmo_serie(self) -> list[tuple[float, float]]:
        """Piezas por minuto a lo largo de la corrida, para el gráfico.

        Es una ventana móvil de `VENTANA_RITMO_S` y no la derivada punto a
        punto: los contadores suben de a una pieza entera, así que la
        derivada cruda es un tren de picos de 60 piezas/min separados por
        ceros, que no se parece en nada al ritmo real.
        """

        muestras = self.serie_acumulada()
        salida: list[tuple[float, float]] = []
        atras = 0

        for m in muestras:
            while muestras[atras].t < m.t - VENTANA_RITMO_S:
                atras += 1

            lapso = m.t - muestras[atras].t

            # Al principio de la corrida todavía no hay ventana suficiente;
            # un ritmo calculado sobre tres segundos no dice nada.
            if lapso < VENTANA_RITMO_S / 4.0:
                continue

            salida.append(
                (m.t, (m.depositadas - muestras[atras].depositadas) * 60.0 / lapso))

        return salida

    # ------------------------------------------------------------------
    #  Camara
    # ------------------------------------------------------------------
    def fps_promedio(self, ventana_s: float = VENTANA_FPS_S) -> Optional[float]:
        """Fotogramas por segundo promediados sobre los últimos `ventana_s`.

        Es una resta de contadores sobre el tiempo transcurrido, no el
        promedio de los FPS instantáneos, y la diferencia es justamente lo
        que se quiere ver: si la cámara estuvo un minuto muerta dentro de la
        ventana, el contador no avanzó durante ese minuto y el promedio baja.
        Promediando tasas, ese minuto no existiría —no habría muestras que
        promediar— y el número diría que todo anduvo perfecto.

        Se divide por el tiempo REALMENTE observado y no por `ventana_s`: a
        los treinta segundos de arrancar, dividir por trescientos daría una
        décima parte de los FPS que hay.
        """

        with self._lock:
            muestras = list(self.camara)

        if len(muestras) < 2:
            return None

        ultima = muestras[-1]
        corte = ultima.t - ventana_s
        dentro = [m for m in muestras if m.t >= corte]

        if len(dentro) < 2:
            return None

        lapso = ultima.t - dentro[0].t

        # Menos de esto es una ventana demasiado corta para promediar nada:
        # el numero saltaria con cada fotograma.
        if lapso < 3.0:
            return None

        return (ultima.fotogramas - dentro[0].fotogramas) / lapso

    def fps_serie(self) -> list[tuple[float, float]]:
        """FPS segundo a segundo, para el gráfico.

        Los tramos sin cámara salen en cero y no como un hueco: un cero
        dibujado es una caída que se ve, y un hueco en la curva se confunde
        con el gráfico todavía cargando.
        """

        with self._lock:
            muestras = list(self.camara)

        salida: list[tuple[float, float]] = []

        for previa, m in zip(muestras, muestras[1:]):
            dt = m.t - previa.t

            if dt <= 0.0 or dt > HUECO_MAX_S * 2:
                continue

            salida.append((m.t, max(0.0, (m.fotogramas - previa.fotogramas) / dt)))

        return salida

    def lista_eventos(self) -> list[Evento]:
        with self._lock:
            return list(self.eventos)

    def lista_fallos(self) -> list[pr.Fallo]:
        with self._lock:
            return list(self.fallos)

    def lista_maniobras(self) -> list[Maniobra]:
        with self._lock:
            return list(self.maniobras)

    # ------------------------------------------------------------------
    def resumen(self) -> Resumen:
        """Todos los números de la pantalla, en una sola pasada bajo candado."""

        with self._lock:
            r = Resumen(desde=self.desde, tiempos=dict(self.tiempos))

            # El tramo abierto todavía no sumó su tiempo (se suma en la
            # muestra siguiente). Sin esto el reloj de la pantalla se
            # quedaría clavado cada vez que el robot se queda quieto, que es
            # justo cuando uno lo está mirando.
            if self._abierto is not None and self._prev_wall is not None:
                corriendo = max(0.0, time.time() - self._prev_wall)

                if corriendo <= HUECO_MAX_S:
                    r.tiempos[self._abierto.cajon] += corriendo

            r.en_servicio_s = sum(r.tiempos[c] for c in EN_SERVICIO)

            if r.en_servicio_s > 0.0:
                r.disponibilidad = sum(r.tiempos[c] for c in PRODUCTIVOS) / r.en_servicio_s

            ocupacion = r.tiempos[TRABAJANDO] + r.tiempos[ESPERANDO]

            if ocupacion > 0.0:
                r.utilizacion = r.tiempos[TRABAJANDO] / ocupacion

            self._resumir_produccion(r)
            self._resumir_maniobras(r)
            self._resumir_fallos(r)
            self._resumir_camara(r)

            r.arranques = self._arranques
            r.loop_hz_min = self._loop_hz_min

            # Atasco en curso: el brazo lleva demasiado en la misma maniobra.
            if self._maniobra_inicio is not None:
                corriendo = time.time() - self._maniobra_inicio

                if corriendo > MANIOBRA_LARGA_S:
                    r.atasco_s = corriendo
                    r.atasco_estado = (self._abierto.estado.name
                                       if self._abierto and self._abierto.estado is not None
                                       else "?")

            return r

    def _resumir_produccion(self, r: Resumen) -> None:
        e = self._base

        if e is None:
            return

        r.detectadas_total = e.detectadas
        r.depositadas_total = e.depositadas
        r.descartadas_total = e.descartadas

        r.detectadas = self._desde_el_arranque("detectadas")
        r.depositadas = self._desde_el_arranque("depositadas")
        r.descartadas = self._desde_el_arranque("descartadas")

        if r.detectadas and r.depositadas is not None:
            r.efectividad = r.depositadas / r.detectadas

        # Ritmo medio de la corrida: se mide contra el tiempo en servicio y
        # no contra el reloj de pared, para que dejar la interfaz abierta
        # toda la noche no diluya el número.
        if r.depositadas is not None and r.en_servicio_s > 30.0:
            r.piezas_por_min = r.depositadas * 60.0 / r.en_servicio_s

        r.ritmo_reciente = self._ritmo_reciente()

    def _seguir_referencia(self, e: pr.Proceso) -> None:
        """Fija contra qué valores se restan los contadores del firmware."""

        valores = {c: getattr(e, c) for c in CONTADORES}

        if any(v is None for v in valores.values()):
            return

        if self._referencia is None:
            self._referencia = valores
            return

        # Un contador que BAJÓ es el ESP32 que se reinició (con o sin que
        # llegara su `[BOOT]`: puede haberse reiniciado con el cable
        # desenchufado). Lo producido antes se guarda y la referencia vuelve
        # a cero, que es de donde arrancó el firmware nuevo. Cero y no lo que
        # diga esta línea: entre el reinicio y ella el robot pudo depositar
        # una pieza, y tomar su valor como referencia la descontaría.
        if any(valores[c] < self._referencia[c] for c in CONTADORES):
            self._plegar()
            self._referencia = {c: 0 for c in CONTADORES}

    def _plegar(self) -> None:
        """Guarda lo producido en la sesión que termina y suelta la referencia."""

        if self._referencia is None or self._base is None:
            self._referencia = None
            return

        for c in CONTADORES:
            valor = getattr(self._base, c)

            if valor is not None:
                self._acumulado[c] += max(0, valor - self._referencia[c])

        self._referencia = None

    def _desde_el_arranque(self, campo: str) -> Optional[int]:
        """Un contador del firmware, rebasado al comienzo de esta medición."""

        guardado = self._acumulado[campo]

        if self._referencia is None:
            return guardado or None

        actual = getattr(self._base, campo, None) if self._base else None

        if actual is None:
            return guardado or None

        return guardado + max(0, actual - self._referencia[campo])

    def _ritmo_reciente(self) -> Optional[float]:
        """Piezas por minuto en el último minuto, para el gráfico y el tablero."""

        if len(self.muestras) < 2:
            return None

        ultima = self.muestras[-1]
        corte = ultima.t - VENTANA_RITMO_S
        previas = [m for m in self.muestras if m.t >= corte]

        if len(previas) < 2:
            return None

        lapso = ultima.t - previas[0].t

        if lapso < 5.0:
            return None

        return (ultima.depositadas - previas[0].depositadas) * 60.0 / lapso

    def _resumir_maniobras(self, r: Resumen) -> None:
        duraciones = [m.duracion for m in self.maniobras if m.duracion > 0.0]

        r.maniobras = len(self.maniobras)
        r.maniobras_interrumpidas = sum(1 for m in self.maniobras if m.interrumpida)

        if duraciones:
            r.maniobra_mediana_s = statistics.median(duraciones)
            r.maniobra_peor_s = max(duraciones)

    def _resumir_fallos(self, r: Resumen) -> None:
        r.fallos_vistos = len(self.fallos)

        # El total sale del firmware si lo informó: sus contadores no se
        # pierden aunque el registro de 16 dé la vuelta. Si nunca contestó
        # un `D`, se usa lo que se vio en vivo, que es un piso.
        if self.total_firmware is not None:
            r.fallos_total = self.total_firmware
        elif self._base is not None and self._base.fallos is not None:
            r.fallos_total = self._base.fallos
        else:
            r.fallos_total = len(self.fallos)

        por_tipo = Counter(f.tipo or "?" for f in self.fallos)

        # Los totales del firmware mandan donde existan: cubren los fallos
        # que ocurrieron antes de que la interfaz estuviera abierta.
        r.por_tipo = {t: self.por_tipo_firmware.get(t, por_tipo.get(t, 0))
                      for t in pr.TIPOS_FALLO}

        for t, n in por_tipo.items():
            if t not in r.por_tipo:
                r.por_tipo[t] = n

        r.por_estado = dict(Counter(
            (f.estado_nombre or (f.estado.name if f.estado is not None else "?"))
            for f in self.fallos))

        r.por_eje = dict(Counter(f.eje for f in self.fallos
                                 if f.eje is not None and f.eje > 0))

        r.trabas = sum(1 for f in self.fallos if f.brazo_frenado)
        r.piezas_caidas = sum(1 for f in self.fallos if f.en_mano)

        if r.fallos_total and r.en_servicio_s > 0.0:
            r.mtbf_s = r.en_servicio_s / r.fallos_total

    # ------------------------------------------------------------------
    def _resumir_camara(self, r: Resumen) -> None:
        r.camara_presente = self._camara_ultimo_aviso is not None

        if not r.camara_presente:
            return

        # El hilo de vision dejo de informar. No es lo mismo que una camara
        # caida -- eso lo informa la propia vision -- y hay que distinguirlo:
        # aca lo que se murio es el programa, no el dispositivo.
        r.camara_muda = (time.time() - self._camara_ultimo_aviso) > CAMARA_MUDA_S
        r.camara_viva = False if r.camara_muda else self._camara_viva

        r.camara_detalle = self.camara_detalle
        r.camara_caidas = self.camara_caidas
        r.fotogramas = self.camara_fotogramas
        r.camara_con_imagen_s = self.camara_tiempos["con_imagen"]
        r.camara_sin_imagen_s = self.camara_tiempos["sin_imagen"]

        total = r.camara_con_imagen_s + r.camara_sin_imagen_s

        if total > 0.0:
            r.camara_disponibilidad = r.camara_con_imagen_s / total

        r.fps_5min = self.fps_promedio()
        serie = self.fps_serie()

        if serie:
            r.fps_ahora = serie[-1][1]

            # El peor segundo DENTRO de la ventana, recortada por tiempo y no
            # por cantidad de muestras: si la vision estuvo callada un rato
            # hay menos muestras que segundos, y contar muestras haria que la
            # ventana se estirara hacia atras sin que nadie lo pida.
            corte = serie[-1][0] - VENTANA_FPS_S
            r.fps_peor = min(v for t, v in serie if t >= corte)

    def veredicto(self, r: Optional[Resumen] = None) -> Veredicto:
        """La frase de arriba de todo: qué está pasando, sin leer gráficos.

        El orden de los casos es el orden en que hay que atenderlos, no el
        orden en que quedan lindos: primero lo que está pasando AHORA, y
        recién después lo que se acumuló en la corrida.
        """

        r = r or self.resumen()

        # La camara va PRIMERO, y antes incluso del corte por falta de datos.
        # Sin ella el firmware no recibe una sola pieza y el robot se queda
        # esperando en WAIT_PIECE: o sea con 100 % de disponibilidad, sin un
        # solo fallo y sin producir nada. Es la falla que mejor se disfraza
        # de "hoy no vinieron piezas", y ningun otro numero de esta pantalla
        # la delata.
        if r.camara_presente and r.camara_viva is False:
            if r.camara_muda:
                return Veredicto(
                    FALLA, "El hilo de vision dejo de responder. El robot no "
                           "va a ver ninguna pieza.")

            return Veredicto(
                FALLA,
                f"La camara no esta entregando imagen"
                f"{' (' + r.camara_detalle[:60] + ')' if r.camara_detalle else ''}. "
                "El robot no va a ver ninguna pieza.")

        if r.en_servicio_s < 5.0:
            return Veredicto(INFO, "Midiendo. Todavia no hay tiempo suficiente.")

        if r.atasco_s is not None:
            return Veredicto(
                FALLA,
                f"El brazo lleva {r.atasco_s:.0f} s en la misma maniobra "
                f"({r.atasco_estado}). Esta trabado.")

        if r.loop_hz_min is not None and r.loop_hz_min < LOOP_HZ_MINIMO:
            return Veredicto(
                FALLA,
                f"El loop del ESP32 bajo a {r.loop_hz_min} vueltas/s: "
                "algo lo esta bloqueando.")

        if r.trabas:
            return Veredicto(
                FALLA,
                f"{r.trabas} colision{'es' if r.trabas > 1 else ''} con el brazo "
                "frenado de verdad: el problema es mecanico, no del umbral.")

        if r.camara_caidas:
            return Veredicto(
                AVISO,
                f"La camara se corto {r.camara_caidas} "
                f"{'vez' if r.camara_caidas == 1 else 'veces'} en la corrida: "
                "revisa el cable USB.")

        if r.fps_5min is not None and r.fps_5min < FPS_MINIMOS:
            return Veredicto(
                AVISO,
                f"La camara viene a {r.fps_5min:.0f} fps de promedio: el "
                "seguimiento de piezas no es confiable por debajo de "
                f"{FPS_MINIMOS:.0f}.")

        if r.disponibilidad is not None and r.disponibilidad < 0.90:
            return Veredicto(
                AVISO,
                f"Disponibilidad {r.disponibilidad * 100:.0f} %: "
                f"{r.tiempos[RECUPERANDO] / 60.0:.1f} min parado por fallas.")

        if r.descartadas and r.detectadas and r.descartadas / r.detectadas > 0.10:
            return Veredicto(
                AVISO,
                f"{r.descartadas} piezas se pasaron sin agarrar "
                f"({r.descartadas / r.detectadas * 100:.0f} % de las detectadas): "
                "el robot no llega al ritmo de la cinta.")

        if r.fallos_total:
            return Veredicto(
                AVISO,
                f"{r.fallos_total} fallo{'s' if r.fallos_total > 1 else ''} en la "
                "corrida, pero el robot se recupero y esta produciendo.")

        return Veredicto(INFO, "Sin fallas. El robot esta produciendo normalmente.")
