"""Contrato serie con el firmware: parseo de mensajes y armado de comandos.

Este módulo es la mitad en Python de lo que especifica PROTOCOLO.md. No
abre puertos, no tiene hilos y no sabe nada de la interfaz: entra texto,
salen objetos, y al revés. Eso lo hace verificable sin robot y sin cámara,
que es justamente lo que se necesita de la capa que traduce.

Dos reglas gobiernan todo el parseo, y las dos vienen de la misma
observación: el ESP32 puede reiniciarse en medio de una línea.

  1. Una línea que no se entiende NO es un error. Se devuelve como Texto y
     la interfaz la muestra en la consola. Nada de excepciones: una
     excepción acá mata el hilo del enlace y con él la comunicación entera
     por culpa de un mensaje cortado.

  2. Un campo ausente o ilegible vale None, no cero. Cero es un valor
     válido para casi todos los campos (un error de 0 grados es un eje
     perfecto), así que confundir "no vino" con "vino cero" haría que la
     interfaz muestre datos inventados con total seguridad.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional

# Versión del contrato. El firmware la anuncia en su línea [BOOT] y el
# núcleo compara: si no coinciden, la interfaz no habilita los controles.
# Un protocolo desparejo hay que verlo ANTES de mover el brazo.
VERSION_PROTOCOLO = 3


# ==================================================================
#  Tablas de códigos (ver PROTOCOLO.md §3)
# ==================================================================

class EstadoRobot(IntEnum):
    """Espejo del enum RobotState de src/robot/Robot.h.

    El índice es el orden de declaración en el .h. Estados nuevos van
    SIEMPRE al final; meter uno en el medio corre la numeración de todos
    los siguientes y la interfaz pasa a mostrar el estado equivocado sin
    ningún síntoma. La línea [E] manda además el nombre (`sn`) para poder
    detectar exactamente esa discrepancia.
    """

    IDLE = 0
    HOMING = 1
    WAIT_PIECE = 2
    GO_HOME_IDLE = 3
    PICK_APPROACH = 4
    PICK_DESCEND = 5
    PICK_LIFT = 6
    GO_BIN = 7
    BIN_SETTLE = 8
    RELEASE_WAIT = 9
    BOX_TRANSIT = 10
    BOX_APPROACH = 11
    BOX_DESCEND = 12
    BOX_LIFT = 13
    COLLISION_STOP = 14
    ERROR = 15

    # Modo aprendizaje. Va al final -- después de ERROR -- porque el índice
    # viaja tal cual en `st` y meter uno en el medio corre la numeración de
    # todos los siguientes sin ningún síntoma visible.
    TEACH = 16


class EstadoGuard(IntEnum):
    """Espejo de CollisionGuard::Estado."""

    DESARMADO = 0
    PROMEDIANDO = 1
    ARMADO = 2


class Modo(IntEnum):
    """Espejo de Robot::SortMode. Por serie viaja como letra, no como número."""

    COLOR = 0
    FORMA = 1
    BOX = 2


# Letra que viaja por serie <-> modo. Es el mismo comando para pedirlo
# ('C', 'F', 'A') que el que informa el firmware en `md`/`mp`.
#
# La 'A' del modo Box es histórica: el modo se llamaba ALFAJORES y se
# renombró, pero la letra quedó. NO se cambió a 'B' a propósito, porque en
# este mismo protocolo 'B' ya significa AZUL en la disposición de la caja
# (`XBRGRBG`) y en los contadores por color; darle un segundo significado
# sería peor que la letra heredada. Si algún día se cambia, hay que mover
# firmware, este diccionario y PROTOCOLO.md a la vez, y subir
# VERSION_PROTOCOLO para que un par desparejo se note.
LETRA_A_MODO = {"C": Modo.COLOR, "F": Modo.FORMA, "A": Modo.BOX}
MODO_A_LETRA = {v: k for k, v in LETRA_A_MODO.items()}

COLORES = {"R": "ROJO", "G": "VERDE", "B": "AZUL"}
FORMAS = {"S": "CUADRADO", "H": "HEXAGONO", "C": "CIRCULO"}

# Tipos de fallo, en el ORDEN del `enum TipoFallo` de src/robot/FaultLog.h.
# El orden importa: es el que usa `[FALLOS]` para volcar los contadores por
# tipo, y el que la interfaz respeta al dibujarlos para que la barra de cada
# tipo no cambie de lugar entre una corrida y la siguiente.
TIPOS_FALLO = ("COLISION", "ENCODER", "HOMING", "MANUAL", "DESCALIBRACION")

# Nombres de la visión -> códigos de un carácter que valida el firmware.
COLOR_A_CODIGO = {v: k for k, v in COLORES.items()}
FORMA_A_CODIGO = {v: k for k, v in FORMAS.items()}

NUM_EJES = 3
CELDAS_CAJA = 6

# Nivel de cada parámetro, que decide en qué pestaña lo dibuja la interfaz.
NIVEL_OPERACION = 1
NIVEL_PROCESO = 2
NIVEL_SERVICIO = 3


# ==================================================================
#  Mensajes del ESP32 hacia la PC
# ==================================================================

@dataclass
class Mensaje:
    """Base de todo lo que llega. `crudo` es la línea tal cual vino.

    Se conserva siempre, incluso en los mensajes que se parsearon bien:
    cuando algo no cierra, poder mirar el texto original al lado del
    objeto interpretado es la diferencia entre encontrar el problema en
    un minuto o en una tarde.
    """

    crudo: str


@dataclass
class Texto(Mensaje):
    """Línea que no es de máquina: prosa del firmware o basura de un reset.

    No es una falla. `[GUARD]`, `[MODO]`, `[SERIAL]`, `[EMERGENCIA]` y
    compañía caen acá a propósito: se muestran en la consola de la
    interfaz sin que nadie tenga que escribir un parser por cada aviso
    que el firmware quiera agregar.
    """

    etiqueta: str = ""


@dataclass
class Boot(Mensaje):
    """[BOOT]: el firmware arrancó. Reinicia toda la vista de estado."""

    proto: Optional[int] = None
    fw: str = ""
    estados: Optional[int] = None
    params: Optional[int] = None

    @property
    def compatible(self) -> bool:
        return self.proto == VERSION_PROTOCOLO


@dataclass
class Telemetria(Mensaje):
    """[T]: la línea rápida, 10 Hz. Lo que se mueve."""

    t_ms: Optional[int] = None
    estado: Optional[EstadoRobot] = None
    bomba: Optional[bool] = None

    # Finales de carrera, uno por eje. Lista vacía si el campo no vino:
    # una lista de tres False diría "ninguno pisado", que es una
    # afirmación sobre el hardware que no estaríamos en posición de hacer.
    finales: list[bool] = field(default_factory=list)

    # Un valor por eje. Se guardan como listas de 3 y no como campos
    # sueltos porque todo lo que la interfaz hace con ellos es por eje.
    angulo: list[Optional[float]] = field(default_factory=lambda: [None] * NUM_EJES)
    comandado: list[Optional[float]] = field(default_factory=lambda: [None] * NUM_EJES)
    error: list[Optional[float]] = field(default_factory=lambda: [None] * NUM_EJES)
    umbral: list[Optional[float]] = field(default_factory=lambda: [None] * NUM_EJES)
    velocidad: list[Optional[float]] = field(default_factory=lambda: [None] * NUM_EJES)

    def margen(self, eje: int) -> Optional[float]:
        """Cuánto le falta a ese eje para declarar colisión, de 0 a 1.

        Es el número que dibuja la barra del tablero de diagnóstico: 0 es
        el eje perfecto, 1 es el umbral justo. Por encima de 1 el guard ya
        está contando la ventana de confirmación.
        """

        err = self.error[eje]
        umb = self.umbral[eje]

        if err is None or umb is None or umb <= 0.0:
            return None

        return abs(err) / umb


@dataclass
class Proceso(Mensaje):
    """[E]: qué está haciendo el robot y con qué pieza. 1 Hz."""

    t_ms: Optional[int] = None
    estado: Optional[EstadoRobot] = None
    estado_nombre: str = ""
    modo: Optional[Modo] = None
    modo_pendiente: Optional[Modo] = None
    esperando_tapa: bool = False
    tapa_restante_ms: Optional[int] = None
    cola: Optional[int] = None
    cola_antiguedad_ms: Optional[int] = None
    homed: Optional[bool] = None
    guard: Optional[EstadoGuard] = None
    observando: Optional[bool] = None
    paradas_activas: Optional[bool] = None
    cinta: Optional[bool] = None
    cinta_pwm: Optional[int] = None

    # Caja del modo Box. `layout` son los 6 colores objetivo y `llenas` las
    # celdas ya ocupadas; ambos vacíos fuera del modo Box.
    layout: str = ""
    llenas: list[bool] = field(default_factory=list)
    celda_reservada: Optional[int] = None

    pieza_color: str = ""
    pieza_forma: str = ""
    pieza_y: Optional[float] = None
    pieza_tacho: Optional[int] = None

    detectadas: Optional[int] = None
    depositadas: Optional[int] = None
    descartadas: Optional[int] = None
    fallos: Optional[int] = None

    # Modo teach: puntos de la ruta cargada en el firmware y cuál se está
    # ejecutando (0 = ninguno). Llegan en la línea periódica y no sólo en el
    # evento `[TEACH]` a propósito: un evento suelto se puede perder en un
    # reinicio y dejaría a la pantalla creyendo que todavía reproduce.
    teach_puntos: Optional[int] = None
    teach_indice: Optional[int] = None

    # Depositadas discriminadas, para los contadores que van al lado de cada
    # color y de cada forma en el panel de modo. Se indexan con el código de
    # un carácter ('R'/'G'/'B', 'S'/'H'/'C') para poder recorrerlos junto con
    # las tablas COLORES y FORMAS sin una segunda lista de nombres.
    por_color: dict[str, int] = field(default_factory=dict)
    por_forma: dict[str, int] = field(default_factory=dict)

    @property
    def tasa_exito(self) -> Optional[float]:
        """Depositadas sobre detectadas. El indicador del sistema completo.

        No se calcula con `descartadas` en el denominador ni se le resta
        nada: lo que importa es qué fracción de las piezas que la cámara
        vio terminó donde tenía que terminar, sin importar por qué se
        perdieron las otras.
        """

        if not self.detectadas or self.depositadas is None:
            return None

        return self.depositadas / self.detectadas

    @property
    def discrepancia_estado(self) -> bool:
        """El nombre que manda el firmware no es el que dice nuestra tabla.

        Significa que el enum del firmware y el de este módulo se
        desincronizaron (típicamente, alguien agregó un estado en el
        medio). La interfaz lo muestra como falla de configuración: todo
        lo que dependa del estado está mostrando cualquier cosa.
        """

        if self.estado is None or not self.estado_nombre:
            return False

        return self.estado.name != self.estado_nombre


@dataclass
class SaludEje:
    """Diagnóstico de un eje. Ver PROTOCOLO.md §2.4 para qué mira cada uno."""

    encoder: str = ""          # "ok", "caido" o "rango"
    ganancia: Optional[float] = None
    atraso_ms: Optional[float] = None
    pico: Optional[float] = None
    pico_reposo: Optional[float] = None
    fuga: Optional[float] = None
    raw_min: Optional[int] = None
    raw_max: Optional[int] = None
    resincronizaciones: Optional[int] = None

    # Límites de lectura cruda dentro de los que el AS5600 analógico es
    # confiable (GuardConfig::RAW_MIN_FIABLE / RAW_MAX_FIABLE). Fuera de
    # ahí el ADC ya no mide y el encoder se "clava" sin avisar.
    RAW_UTIL_MIN = 60
    RAW_UTIL_MAX = 3950

    # Por debajo de esto la ganancia deja de ser ruido de medición y pasa a
    # ser pérdida de cuentas real. Ningún umbral del guard lo arregla: el
    # problema está en el sensor o en la mecánica.
    GANANCIA_MINIMA = 0.97

    @property
    def encoder_ok(self) -> bool:
        return self.encoder == "ok"

    @property
    def pierde_cuentas(self) -> bool:
        return self.ganancia is not None and 0.0 < self.ganancia < self.GANANCIA_MINIMA

    @property
    def margen_raw(self) -> Optional[int]:
        """Cuántas cuentas quedan hasta la zona donde el ADC ya no mide.

        Es el número que dice si el recorrido del eje está pegado al borde
        del rango útil del encoder: si se acerca a cero, cualquier
        corrimiento mecánico mete al eje en la zona ciega.
        """

        if self.raw_min is None or self.raw_max is None:
            return None

        return min(self.raw_min - self.RAW_UTIL_MIN, self.RAW_UTIL_MAX - self.raw_max)


@dataclass
class Salud(Mensaje):
    """[H]: salud del ESP32 y de los tres canales de encoder. Cada 2 s."""

    t_ms: Optional[int] = None
    uptime_s: Optional[int] = None
    loop_hz: Optional[int] = None
    heap: Optional[int] = None
    ejes: list[SaludEje] = field(default_factory=lambda: [SaludEje() for _ in range(NUM_EJES)])


@dataclass
class Parametro(Mensaje):
    """[P] n=... : una fila de la tabla de parámetros del firmware.

    La interfaz NO lleva una lista propia de parámetros: pide `P?` y arma
    los controles con lo que recibe, incluidos el rango y el nivel. Así,
    agregar un parámetro es tocar una sola tabla en C++ y aparece solo en
    la pantalla que le corresponde.
    """

    nombre: str = ""
    valor: Optional[float] = None
    defecto: Optional[float] = None
    minimo: Optional[float] = None
    maximo: Optional[float] = None
    unidad: str = ""
    nivel: int = NIVEL_SERVICIO
    tipo: str = "f"            # "f" real, "i" entero, "b" booleano

    @property
    def modificado(self) -> bool:
        """El valor actual no es el de fábrica. La interfaz lo marca."""

        if self.valor is None or self.defecto is None:
            return False

        return abs(self.valor - self.defecto) > 1e-9


@dataclass
class ParametroCambiado(Mensaje):
    """[P] set ... : respuesta a un cambio, haya salido o no.

    Llega también cuando el cambio se pidió con los comandos históricos
    ('U', 'T', 'K', 'L', 'Q'), así que la interfaz se mantiene en sincronía
    aunque alguien toque el robot desde un monitor serie.
    """

    nombre: str = ""
    valor: Optional[float] = None
    ok: bool = False
    error: str = ""            # "rango", "desconocido", "bloqueado"


@dataclass
class FinParametros(Mensaje):
    """[P] fin : terminó el volcado de la tabla."""

    cantidad: Optional[int] = None


@dataclass
class PiezaEncolada(Mensaje):
    """[PIEZA]: el firmware aceptó una pieza y la puso en la cola."""

    y: Optional[float] = None
    color: str = ""
    forma: str = ""
    cola: Optional[int] = None


@dataclass
class Teach(Mensaje):
    """[TEACH]: eventos del modo aprendizaje.

    Una sola clase para todas las formas que toma la etiqueta (`on`, `off`,
    `pedido`, `buf`, `run`, `fin`, `abort`, `err=...` y el volcado de `J?`)
    porque todas dicen lo mismo desde ángulos distintos y la interfaz las
    consume en el mismo lugar. `evento` es la primera palabra suelta de la
    línea, o "estado" cuando es el volcado completo.

    Los límites del volumen vienen acá **y** en la tabla de parámetros. No es
    redundancia inútil: la tabla los da como valores ajustables y esta línea
    los da ya resueltos, incluido el piso en Z, que el firmware calcula a
    partir de `grab_z` y no existe como parámetro propio.
    """

    #: Primera palabra suelta de la línea. Los que importan:
    #:   "p"       una posición del volcado a 20 Hz (x, y, z, bomba)
    #:   "on"/"off"/"pedido"   entradas y salidas del modo
    #:   "buf"     cambió la cantidad de puntos cargados
    #:   "run"     arrancó una reproducción
    #:   "fin"     terminó bien
    #:   "ir"      arrancó un "ir a una coordenada" (x, y, z del destino)
    #:   "irfin"   el "ir a" llegó
    #:   "abort"   se cortó (ver `motivo`)
    #:   "err"     un pedido rechazado (ver `error`)
    #:   "estado"  el volcado completo de `J?`
    evento: str = ""
    modo: str = ""             # "on", "pedido" u "off", sólo en el volcado
    puntos: Optional[int] = None
    indice: Optional[int] = None
    pct: Optional[float] = None
    capacidad: Optional[int] = None
    error: str = ""
    motivo: str = ""

    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    bomba: Optional[bool] = None

    # (min, max) de cada eje. None si la línea no los traía.
    limite_x: Optional[tuple[float, float]] = None
    limite_y: Optional[tuple[float, float]] = None
    limite_z: Optional[tuple[float, float]] = None


@dataclass
class Fallo(Mensaje):
    """[FALLO]: una entrada del registro de fallos, con su contexto."""

    numero: Optional[int] = None
    t_ms: Optional[int] = None
    tipo: str = ""
    eje: Optional[int] = None
    error_deg: Optional[float] = None
    cmd_delta: Optional[float] = None
    enc_delta: Optional[float] = None
    estado: Optional[EstadoRobot] = None

    # El campo tal cual vino. Se conserva porque `estado` es None ante un
    # estado que este modulo todavia no conoce, y en ese caso el nombre
    # crudo sigue diciendo en que parte del ciclo fallo el robot.
    estado_nombre: str = ""

    con_pieza: bool = False
    en_mano: bool = False
    color: str = ""
    forma: str = ""
    pieza_y: Optional[float] = None
    pieza_x: Optional[float] = None
    tacho: Optional[int] = None

    @property
    def brazo_frenado(self) -> Optional[bool]:
        """En una COLISION: ¿el brazo se trabó, o el encoder está midiendo mal?

        Es la pregunta que decide qué se va a arreglar, y la contesta el par
        `dcmd`/`denc` (ver FaultLog.h): los micropasos dicen cuánto se le
        ordenó girar al eje y el encoder cuánto giró de verdad. Si el eje
        casi no se movió teniendo orden de moverse, hay algo que lo frena;
        si se movió lo que se le pidió, el que se equivocó fue el sensor —o
        el umbral es demasiado angosto— y la mecánica está bien.

        None cuando la pregunta no aplica: los otros tipos de fallo no
        traen un movimiento contra el cual comparar.
        """

        if self.tipo != "COLISION":
            return None

        if self.cmd_delta is None or self.enc_delta is None:
            return None

        # Con una orden de giro chica el cociente es puro ruido: un eje al
        # que se le pidieron 0,5 grados no permite distinguir nada.
        if abs(self.cmd_delta) < self.CMD_MINIMO_DEG:
            return None

        return abs(self.enc_delta) < self.FRACCION_TRABADO * abs(self.cmd_delta)

    #: Por debajo de esta orden de giro el cociente denc/dcmd no distingue
    #: nada (grados).
    CMD_MINIMO_DEG = 2.0

    #: Fracción del giro ordenado por debajo de la cual se considera que el
    #: eje directamente no se movió. Deliberadamente holgada: un brazo
    #: trabado da denc ~ 0, no un 30 % prolijo.
    FRACCION_TRABADO = 0.35


@dataclass
class ResumenFallos(Mensaje):
    """[FALLOS]: el encabezado del volcado de 'D' y su línea de cierre.

    El firmware ya lo imprimía; lo que no había era quien lo leyera, así que
    los contadores POR TIPO —que el firmware lleva desde el encendido y que
    sobreviven a que el buffer de 16 dé la vuelta— se iban a la consola
    como texto. Son el único lugar donde está el total histórico de cada
    tipo de fallo: el registro guarda los últimos 16 y nada más.

        [FALLOS] total=7 COLISION=5 ENCODER=1 HOMING=0 MANUAL=1 DESCALIBRACION=0 guardados=7
        [FALLOS] fin
    """

    total: Optional[int] = None
    guardados: Optional[int] = None
    fin: bool = False
    por_tipo: dict[str, int] = field(default_factory=dict)


# ==================================================================
#  Parseo
# ==================================================================

_ETIQUETA = re.compile(r"^\s*\[([A-Za-z]+)\]\s*(.*)$")
_PAR = re.compile(r"([A-Za-z_][A-Za-z_0-9]*)=(\S*)")
_EN_COLA = re.compile(r"en cola:\s*(\d+)")


def _pares(cuerpo: str) -> dict[str, str]:
    """Extrae los `clave=valor` de una línea, ignorando lo que no lo sea.

    Se usa una expresión regular y no un `split()` porque las líneas del
    firmware mezclan pares con prosa ("[PIEZA] Y=4.20 ... en cola: 3") y
    con tokens sueltos ("[P] set n=x v=1 ok"). Recorrer buscando el patrón
    deja pasar todo eso sin tener que preverlo.
    """

    return {m.group(1): m.group(2) for m in _PAR.finditer(cuerpo)}


def _f(pares: dict[str, str], clave: str) -> Optional[float]:
    """Real, o None si el campo no vino o no es un número.

    None y no 0.0: cero es un valor perfectamente válido para casi todos
    estos campos, así que devolverlo ante un dato ausente equivaldría a
    inventar una medición.
    """

    texto = pares.get(clave)

    if texto is None:
        return None

    try:
        return float(texto)
    except ValueError:
        return None


def _i(pares: dict[str, str], clave: str) -> Optional[int]:
    valor = _f(pares, clave)
    return None if valor is None else int(valor)


def _b(pares: dict[str, str], clave: str) -> Optional[bool]:
    valor = _i(pares, clave)
    return None if valor is None else valor != 0


def _por_eje(pares: dict[str, str], prefijo: str) -> list[Optional[float]]:
    """Los tres valores de un campo por eje: a1, a2, a3 -> [.., .., ..]."""

    return [_f(pares, f"{prefijo}{i + 1}") for i in range(NUM_EJES)]


def _bits(texto: str) -> list[bool]:
    """Campo de dígitos 0/1 ("010", "000100") a lista de booleanos.

    Devuelve lista vacía ante cualquier cosa que no sean sólo dígitos: una
    lista de False significaría "ningún final pisado", que es una
    afirmación sobre el hardware, y no se afirma nada sobre un dato que no
    llegó o que llegó roto.
    """

    if not texto or not texto.isdigit():
        return []

    return [c == "1" for c in texto]


def _contadores(pares: dict[str, str], prefijo: str, codigos: str) -> dict[str, int]:
    """Contadores por código: kr, kg, kb -> {'R': 12, 'G': 14, 'B': 12}.

    Sólo entran los que realmente vinieron. Un contador ausente se omite
    del diccionario en vez de valer 0, así la interfaz puede mostrar "—"
    y no un cero que parecería decir "todavía no depositó ninguna".
    """

    salida = {}

    for codigo in codigos:
        valor = _i(pares, f"{prefijo}{codigo.lower()}")

        if valor is not None:
            salida[codigo] = valor

    return salida


def _estado(valor: Optional[int]) -> Optional[EstadoRobot]:
    """Índice a estado, tolerando un firmware más nuevo que este módulo.

    Un estado que todavía no existe acá no puede tirar abajo el parseo: se
    devuelve None y la interfaz muestra el número crudo. La línea [E]
    manda además el nombre, así que igual se ve qué estaba pasando.
    """

    if valor is None:
        return None

    try:
        return EstadoRobot(valor)
    except ValueError:
        return None


def _estado_o_nombre(texto: str) -> Optional[EstadoRobot]:
    """El campo `estado` de [FALLO], que puede venir como nombre o como índice.

    El firmware manda el NOMBRE: `FaultLog` guarda el literal que devuelve
    `Robot::nombreEstado()` y lo imprime tal cual. El parser, en cambio,
    leía un índice —el ejemplo de PROTOCOLO.md mostraba `estado=6`—, así que
    el campo valía None en todos los fallos reales y la columna que dice en
    qué parte del ciclo falló el robot venía siempre vacía.

    Se aceptan las dos formas en vez de elegir una: cuesta cuatro líneas y
    deja de importar con qué firmware esté flasheada la placa.
    """

    if not texto:
        return None

    try:
        return EstadoRobot[texto]
    except KeyError:
        pass

    try:
        return EstadoRobot(int(texto))
    except (ValueError, KeyError):
        return None


def parsear(linea: str) -> Mensaje:
    """Convierte una línea del ESP32 en un mensaje tipado.

    Nunca levanta excepciones y nunca devuelve None: lo que no se entiende
    vuelve como Texto. El hilo del enlace no puede permitirse morir por una
    línea cortada a la mitad por un reset.
    """

    linea = linea.rstrip("\r\n")
    m = _ETIQUETA.match(linea)

    if m is None:
        return Texto(crudo=linea)

    etiqueta = m.group(1).upper()
    cuerpo = m.group(2)
    p = _pares(cuerpo)

    if etiqueta == "T":
        return Telemetria(
            crudo=linea,
            t_ms=_i(p, "t"),
            estado=_estado(_i(p, "st")),
            bomba=_b(p, "pm"),
            finales=_bits(p.get("fc", "")),
            angulo=_por_eje(p, "a"),
            comandado=_por_eje(p, "c"),
            error=_por_eje(p, "e"),
            umbral=_por_eje(p, "u"),
            velocidad=_por_eje(p, "v"),
        )

    if etiqueta == "E":
        return Proceso(
            crudo=linea,
            t_ms=_i(p, "t"),
            estado=_estado(_i(p, "st")),
            estado_nombre=p.get("sn", ""),
            modo=LETRA_A_MODO.get(p.get("md", "")),
            modo_pendiente=LETRA_A_MODO.get(p.get("mp", "")),
            esperando_tapa=bool(_b(p, "cf")),
            tapa_restante_ms=_i(p, "cr"),
            cola=_i(p, "q"),
            cola_antiguedad_ms=_i(p, "qa"),
            homed=_b(p, "hm"),
            guard=_guard(_i(p, "gd")),
            observando=_b(p, "ob"),
            paradas_activas=_b(p, "sup"),
            cinta=_b(p, "cv"),
            cinta_pwm=_i(p, "cvp"),
            layout="" if p.get("bx", "-") == "-" else p.get("bx", ""),
            llenas=_bits(p.get("bf", "")),
            celda_reservada=_i(p, "bc"),
            pieza_color="" if p.get("pc", "-") == "-" else p.get("pc", ""),
            pieza_forma="" if p.get("pf", "-") == "-" else p.get("pf", ""),
            pieza_y=_f(p, "py"),
            pieza_tacho=_i(p, "pb"),
            teach_puntos=_i(p, "tw"),
            teach_indice=_i(p, "ti"),
            detectadas=_i(p, "nd"),
            depositadas=_i(p, "nk"),
            descartadas=_i(p, "nx"),
            fallos=_i(p, "nf"),
            por_color=_contadores(p, "k", "RGB"),
            por_forma=_contadores(p, "k", "SHC"),
        )

    if etiqueta == "H":
        ejes = []

        for i in range(NUM_EJES):
            n = i + 1
            ejes.append(
                SaludEje(
                    encoder=p.get(f"enc{n}", ""),
                    ganancia=_f(p, f"gan{n}"),
                    atraso_ms=_f(p, f"atr{n}"),
                    pico=_f(p, f"pic{n}"),
                    pico_reposo=_f(p, f"rep{n}"),
                    fuga=_f(p, f"fug{n}"),
                    raw_min=_i(p, f"rmn{n}"),
                    raw_max=_i(p, f"rmx{n}"),
                    resincronizaciones=_i(p, f"rsy{n}"),
                )
            )

        return Salud(
            crudo=linea,
            t_ms=_i(p, "t"),
            uptime_s=_i(p, "up"),
            loop_hz=_i(p, "loop"),
            heap=_i(p, "heap"),
            ejes=ejes,
        )

    if etiqueta == "P":
        return _parsear_parametro(linea, cuerpo, p)

    if etiqueta == "BOOT":
        return Boot(
            crudo=linea,
            proto=_i(p, "proto"),
            fw=p.get("fw", ""),
            estados=_i(p, "estados"),
            params=_i(p, "params"),
        )

    if etiqueta == "PIEZA":
        cola = _EN_COLA.search(cuerpo)

        return PiezaEncolada(
            crudo=linea,
            y=_f(p, "Y"),
            color=p.get("color", ""),
            forma=p.get("forma", ""),
            cola=int(cola.group(1)) if cola else None,
        )

    if etiqueta == "TEACH":
        palabras = cuerpo.split()
        suelta = palabras[0] if palabras and "=" not in palabras[0] else ""

        def rango(a: str, b: str):
            lo, hi = _f(p, a), _f(p, b)
            return None if lo is None or hi is None else (lo, hi)

        return Teach(
            crudo=linea,
            evento=suelta or ("err" if p.get("err") else "estado"),
            modo=p.get("est", ""),
            puntos=_i(p, "n"),
            indice=_i(p, "i"),
            pct=_f(p, "pct"),
            capacidad=_i(p, "cap"),
            error=p.get("err", ""),
            motivo=p.get("motivo", ""),
            x=_f(p, "x"),
            y=_f(p, "y"),
            z=_f(p, "z"),
            bomba=_b(p, "b"),
            limite_x=rango("xmin", "xmax"),
            limite_y=rango("ymin", "ymax"),
            limite_z=rango("zmin", "zmax"),
        )

    if etiqueta == "FALLOS":
        palabras = cuerpo.split()

        return ResumenFallos(
            crudo=linea,
            total=_i(p, "total"),
            guardados=_i(p, "guardados"),
            fin="fin" in palabras,
            por_tipo={t: _i(p, t) for t in TIPOS_FALLO if _i(p, t) is not None},
        )

    if etiqueta == "FALLO":
        return Fallo(
            crudo=linea,
            numero=_i(p, "n"),
            t_ms=_i(p, "t"),
            tipo=p.get("tipo", ""),
            eje=_i(p, "eje"),
            error_deg=_f(p, "err"),
            cmd_delta=_f(p, "dcmd"),
            enc_delta=_f(p, "denc"),
            estado=_estado_o_nombre(p.get("estado", "")),
            estado_nombre=p.get("estado", ""),
            con_pieza=bool(_b(p, "pieza")),
            en_mano=bool(_b(p, "enmano")),
            color=p.get("color", ""),
            forma=p.get("forma", ""),
            pieza_y=_f(p, "py"),
            pieza_x=_f(p, "px"),
            tacho=_i(p, "tacho"),
        )

    # Etiqueta conocida por el firmware pero no por nosotros ([GUARD],
    # [MODO], [CAJA], [SERIAL]...). Va a la consola tal cual: que el
    # firmware pueda agregar avisos sin tocar Python es deliberado.
    return Texto(crudo=linea, etiqueta=etiqueta)


def _guard(valor: Optional[int]) -> Optional[EstadoGuard]:
    if valor is None:
        return None

    try:
        return EstadoGuard(valor)
    except ValueError:
        return None


def _parsear_parametro(linea: str, cuerpo: str, p: dict[str, str]) -> Mensaje:
    """Las tres formas de la etiqueta [P]: fila, cierre y confirmación."""

    palabras = cuerpo.split()

    if palabras and palabras[0] == "fin":
        return FinParametros(crudo=linea, cantidad=_i(p, "n"))

    if palabras and palabras[0] == "set":
        return ParametroCambiado(
            crudo=linea,
            nombre=p.get("n", ""),
            valor=_f(p, "v"),
            ok="ok" in palabras,
            error=p.get("err", ""),
        )

    nivel = _i(p, "l")

    return Parametro(
        crudo=linea,
        nombre=p.get("n", ""),
        valor=_f(p, "v"),
        defecto=_f(p, "d"),
        minimo=_f(p, "min"),
        maximo=_f(p, "max"),
        unidad=p.get("u", ""),
        nivel=NIVEL_SERVICIO if nivel is None else nivel,
        tipo=p.get("t", "f"),
    )


# ==================================================================
#  Comandos de la PC hacia el ESP32
# ==================================================================
#
#  Todas las funciones devuelven la línea COMPLETA, con su '\n'. Quien las
#  llama no tiene que acordarse de agregarlo, que es la clase de olvido que
#  deja al firmware esperando el fin de línea de un comando que ya se mandó.

def cmd_pieza(y_cm: float, color: str, forma: str) -> str:
    """Pieza que cruzó la línea de detección.

    `color` y `forma` se aceptan como código de un carácter ('B', 'S') o
    como nombre de la visión ('AZUL', 'CUADRADO'), porque los dos existen
    en el proyecto y hacer que el llamador se acuerde de cuál va acá es
    pedir un bug.
    """

    c = COLOR_A_CODIGO.get(color.upper(), color.upper())
    f = FORMA_A_CODIGO.get(forma.upper(), forma.upper())

    if c not in COLORES:
        raise ValueError(f"color desconocido: {color!r}")

    if f not in FORMAS:
        raise ValueError(f"forma desconocida: {forma!r}")

    return f"{y_cm:.2f},{c},{f}\n"


def cmd_modo(modo: Modo) -> str:
    return MODO_A_LETRA[modo] + "\n"


def cmd_caja_nueva() -> str:
    return "N\n"


def cmd_paro() -> str:
    """Paro manual. Desde ERROR, el mismo comando rehomea.

    Ojo: es una conveniencia de la interfaz, NO un paro de emergencia. El
    de verdad corta la alimentación de los drivers por hardware.
    """

    return "R\n"


def cmd_historial_fallos() -> str:
    return "D\n"


def cmd_estado_supervision() -> str:
    return "S\n"


def cmd_alternar_paradas() -> str:
    return "G\n"


def cmd_alternar_traza() -> str:
    return "M\n"


def cmd_layout_caja(colores: str) -> str:
    """Disposición de la caja del modo Box: 6 colores, de la celda 1 a la 6.

    Se valida acá lo mismo que valida el firmware (6 celdas, colores R/G/B,
    máximo 3 de cada uno) para poder deshabilitar el botón antes de mandar
    algo que va a ser rechazado. La validación del firmware sigue siendo la
    que manda: ésta es sólo para que la interfaz no ofrezca lo imposible.
    """

    colores = colores.upper().strip()

    if len(colores) != CELDAS_CAJA:
        raise ValueError(f"la caja tiene {CELDAS_CAJA} celdas, vinieron {len(colores)}")

    if any(c not in COLORES for c in colores):
        raise ValueError(f"colores válidos: {', '.join(COLORES)}")

    # Con más de 3 de un color la caja no se puede terminar nunca y el
    # robot esperaría para siempre una pieza que no tiene dónde ir.
    for c in COLORES:
        if colores.count(c) > CELDAS_CAJA // 2:
            raise ValueError(f"como máximo {CELDAS_CAJA // 2} celdas de color {c}")

    return f"X{colores}\n"


def cmd_listar_parametros() -> str:
    return "P?\n"


def cmd_parametro(nombre: str, valor: float) -> str:
    """Fija un parámetro. El firmware satura contra su propio mín/máx.

    El valor va con 4 decimales y sin notación científica: el `strtof` del
    firmware no interpreta '1e-3', y un parámetro que se manda como '0' por
    redondeo puede ser la diferencia entre agarrar la pieza y clavar la
    ventosa contra la cinta.
    """

    nombre = nombre.strip()

    if not nombre or " " in nombre:
        raise ValueError(f"nombre de parámetro inválido: {nombre!r}")

    return f"P{nombre}={valor:.4f}\n"


def cmd_guardar_parametros() -> str:
    """Persiste en NVS: los valores sobreviven al reinicio y al reflasheo."""

    return "P*\n"


def cmd_parametros_de_fabrica() -> str:
    return "P0\n"


# ------------------------------------------------------------------
#  Modo teach
# ------------------------------------------------------------------
#
#  Todos empiezan con 'J'. El firmware los consume ANTES del parser de
#  piezas justamente porque varios llevan comas y ese parser toma cualquier
#  línea con dos comas por un mensaje de la visión.
#
#  El reparto de trabajo: el firmware recorta al volumen, resuelve la
#  cinemática y encadena los puntos; acá viven el dibujo, la grabación, los
#  nombres y los porcentajes ya verificados de cada secuencia.

def cmd_teach(activo: bool) -> str:
    """Entra o sale del modo teach.

    Entrar no es inmediato: el firmware termina la pieza que tenga en vuelo,
    vuelve a home y recién ahí entra. Se sabe que entró cuando `st` pasa a
    `EstadoRobot.TEACH`, no cuando contesta.
    """

    return "J1\n" if activo else "J0\n"


def cmd_teach_estado() -> str:
    """Pide el volcado con el estado y los límites del volumen de trabajo."""

    return "J?\n"


def cmd_teach_mover(x: float, y: float, z: float) -> str:
    """Destino absoluto de la punta, en cm. El firmware recorta al volumen."""

    return f"JM{x:.2f},{y:.2f},{z:.2f}\n"


def cmd_teach_lineal(x: float, y: float, z: float) -> str:
    """Va al punto en LÍNEA RECTA (movL), no por el camino de los motores.

    `cmd_teach_mover` y `cmd_teach_ir` mueven los tres ejes en proporción, que
    es una recta en el espacio de los ángulos y una curva en el de la punta:
    de un tacho al otro el brazo pasa un centímetro fuera de la recta. Esto
    parte el camino en tramos y lo recorre derecho.

    Es más lento y puede fallar donde `JM` no falla (la recta puede salirse
    del alcance aunque las dos puntas estén adentro). Ver PROTOCOLO.md §6.3.2.
    """

    return f"JL{x:.2f},{y:.2f},{z:.2f}\n"


def cmd_teach_ir(x: float, y: float, z: float) -> str:
    """Va a un punto PASANDO POR HOME, a máxima velocidad y aceleración.

    La diferencia con `cmd_teach_mover` no es la velocidad sino el camino:
    quien escribe una coordenada no ve por dónde va a pasar el brazo, y la
    recta entre dos puntos bajos del volumen va raspando la cinta. Home es
    brazos horizontales -- lo más arriba que llega el robot --, así que subir
    primero y bajar después no puede tocar nada. Si el brazo ya está en home,
    el rodeo se saltea.

    El firmware contesta `[TEACH] ir ...` al arrancar y `[TEACH] irfin` al
    llegar; `JX` lo corta igual que a una reproducción.
    """

    return f"JI{x:.2f},{y:.2f},{z:.2f}\n"


def cmd_teach_jog(vx: float, vy: float, vz: float) -> str:
    """Dirección del jog, cada componente en [-1, 1]. El vector nulo frena.

    Es una DIRECCIÓN y no una posición porque la dirección vence sola en el
    firmware: si la interfaz deja de refrescarla (se cerró el navegador, se
    cortó el enlace) el brazo se para en el tramo que esté haciendo. Un
    destino, en cambio, se seguiría cumpliendo con nadie mirando.
    """

    def recortar(v: float) -> float:
        return max(-1.0, min(1.0, float(v)))

    return f"JD{recortar(vx):.2f},{recortar(vy):.2f},{recortar(vz):.2f}\n"


def cmd_teach_bomba(encendida: bool) -> str:
    return "JP1\n" if encendida else "JP0\n"


def cmd_teach_limpiar() -> str:
    return "JC\n"


def cmd_teach_punto(x: float, y: float, z: float,
                    bomba: bool, espera_ms: int = 0) -> str:
    """Agrega un punto a la ruta que el firmware va a reproducir."""

    espera = max(0, min(60000, int(espera_ms)))

    return f"JA{x:.2f},{y:.2f},{z:.2f},{1 if bomba else 0},{espera}\n"


def cmd_teach_reproducir(pct: float) -> str:
    """Corre la ruta cargada al `pct` % de la velocidad y aceleración tope.

    Los tres valores que usa la verificación por etapas son 15, 50 y 100: se
    graba jogueando al 15 %, así que la primera pasada reproduce a la misma
    velocidad a la que se enseñó el movimiento.
    """

    return f"JR{max(1.0, min(100.0, float(pct))):.0f}\n"


def cmd_teach_volcado(activo: bool) -> str:
    """Enciende el volcado de la posición comandada a 20 Hz.

    Es el dato que se graba: la posición MEDIDA ya viaja en `[T]`, pero
    arrastra el ruido del encoder, y ese ruido reproducido después es
    temblor. Sólo se enciende con la pestaña de teach a la vista —
    760 B/s del enlace no se gastan con la pantalla en otra cosa.
    """

    return "JG1\n" if activo else "JG0\n"


def cmd_teach_abortar() -> str:
    return "JX\n"


def cmd_telemetria(encendida: bool) -> str:
    return "V1\n" if encendida else "V0\n"


def cmd_foto_estado() -> str:
    """Pide una vez [E] y [H] sin encender el stream. Se usa al conectar."""

    return "V?\n"
