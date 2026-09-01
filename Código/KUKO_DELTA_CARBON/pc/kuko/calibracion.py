"""Calibracion de la VISION: umbrales de color, forma, exposicion y luces.

Es la mitad de la pestana de Vision que no dibuja nada. Aca viven cuatro
cosas que se apoyan una en la otra:

  * la TABLA de lo que se puede mover (`FICHAS`), con su rango, su unidad y
    la explicacion de que ataca cada umbral;
  * la lectura y escritura de esos valores contra `vision_python/config.py`,
    que sigue siendo el unico lugar donde estan escritos los numeros de
    fabrica junto con la medicion que los eligio;
  * las HABITACIONES, o sea el conjunto completo de ajustes con un nombre
    ("Pieza", "Aula facultad"), guardadas en disco;
  * la MEDICION sobre un fotograma: que color tienen de verdad las piezas
    que hay en la cinta ahora mismo, y de ahi el ajuste automatico.

Por que esto no vive en `vision_python/`: ese paquete es el que anda y esta
calibrado, y lo unico que se le cambio es leer `config.X` en el punto de uso
en vez de copiarlo al importar. Todo lo que es "la interfaz mueve un umbral
y lo guarda con un nombre" es de este lado.

El problema que resuelve, escrito por si alguna vez alguien duda de si vale
la pena: el robot se muda de habitacion, cambia la luz, y los rangos HSV
--que se eligieron midiendo bajo OTRA luz-- dejan de contener a las piezas.
El verde es el peor caso porque su rango es el mas angosto de los tres
(H 50-74 contra los 34 grados de ancho del rojo), y porque su modo de falla
es total y silencioso: no detecta peor, no detecta NADA (ver el comentario
del VERDE en config.py, donde ya paso una vez con 0/120 circulos).
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

# Mismo camino que usa `vision.py` para llegar a los modulos de vision: la
# ruta se arma desde este archivo y no desde el directorio de trabajo.
RAIZ = Path(__file__).resolve().parents[2]

if str(RAIZ / "vision_python") not in sys.path:
    sys.path.insert(0, str(RAIZ / "vision_python"))

import config                                        # noqa: E402

ARCHIVO = Path(__file__).resolve().parents[1] / "config" / "vision.json"

# Los tres colores, en el orden en que se muestran. No se leen de
# COLOR_HSV_RANGES para que el orden de la pantalla no dependa del orden de
# un diccionario, pero se verifica contra el (`_colores_conocidos`).
COLORES = ("ROJO", "VERDE", "AZUL")

# Como se llama cada uno en pantalla y con que se lo dibuja cuando hace
# falta un color de referencia (no es el color detectado: es la etiqueta).
NOMBRE_COLOR = {"ROJO": "Rojo", "VERDE": "Verde", "AZUL": "Azul"}

# Tono de referencia de cada color en la rueda de OpenCV (H de 0 a 179).
# Sirve para dos cosas: decidir a que color se parece una pieza medida
# cuando los rangos configurados no la agarran --que es justo el caso que
# hay que arreglar-- y ordenar los grupos del ajuste automatico.
#
# El rojo esta partido: se compara contra los DOS y se toma el mas cercano,
# porque la rueda de Hue da la vuelta y el rojo vive en las dos puntas.
TONO_REFERENCIA = {"ROJO": (0, 179), "VERDE": (60,), "AZUL": (105,)}


# ======================================================================
#  1. QUE SE PUEDE MOVER
# ======================================================================

@dataclass(frozen=True)
class Ficha:
    """Un valor ajustable de la vision: donde vive y hasta donde llega."""

    nombre: str                 # atributo de config.py
    etiqueta: str               # como se llama en pantalla
    minimo: float
    maximo: float
    paso: float = 1.0
    unidad: str = ""
    entero: bool = False
    ayuda: str = ""

    def saturar(self, valor: float) -> float:
        valor = max(self.minimo, min(self.maximo, float(valor)))

        return round(valor) if self.entero else valor


# Los umbrales de forma. Estan en su propia seccion de la pantalla y no
# mezclados con el color porque no se tocan por el mismo motivo: el color se
# mueve con la LUZ (o sea cada vez que se muda el robot) y la forma con la
# GEOMETRIA de las piezas (o sea casi nunca). Ponerlos juntos invitaria a
# mover la forma cuando lo que cambio fue la lampara.
#
# Los rangos de los sliders son deliberadamente mas anchos que la zona util:
# la gracia de un slider es poder pasarse y ver que empeora. Los que tienen
# un valor bien medido lo dicen en la ayuda, que es lo que evita que alguien
# "mejore" un numero que costo una tarde de mediciones.
FICHAS_GEOMETRIA: tuple[Ficha, ...] = (
    Ficha("MIN_CONTOUR_AREA", "Area minima", 500, 12000, 100, "px2", True,
          "Por debajo de esto un contorno se descarta. Depende de la "
          "resolucion: viene calibrado para 720p."),
    Ficha("MAX_CONTOUR_AREA", "Area maxima", 10000, 90000, 500, "px2", True,
          "Por encima de esto se descarta. Ataja la mancha que aparece "
          "cuando fuga medio fotograma dentro de la mascara."),
    Ficha("MORPH_KERNEL_SIZE", "Filtro morfologico", 3, 11, 1, "px", True,
          "Tapa huecos DENTRO de la pieza, pero tambien el hueco ENTRE dos "
          "piezas que se tocan. Medido, el minimo de fallos de separacion "
          "esta en 5 y no es monotono: bajar mas tampoco ayuda."),
    Ficha("MASK_SMOOTHING_KERNEL_SIZE", "Suavizado de borde", 3, 15, 2, "px", True,
          "Promedia el festoneado del borde de la mascara. Tiene que ser "
          "impar."),
    Ficha("SHAPE_APPROX_EPSILON_RATIO", "Tolerancia de vertices", 0.015, 0.06,
          0.001, "", False,
          "NO BAJARLO: medido, con 0,025 el cuadrado azul se reconoce en "
          "1 de 60 fotogramas contra 60 de 60 con 0,035."),
    Ficha("SQUARE_ASPECT_RATIO_MIN", "Cuadrado: relacion minima", 0.5, 1.0,
          0.01, "", False, "Relacion ancho/alto aceptada para un cuadrado."),
    Ficha("SQUARE_ASPECT_RATIO_MAX", "Cuadrado: relacion maxima", 1.0, 1.8,
          0.01, "", False, "Relacion ancho/alto aceptada para un cuadrado."),
    Ficha("HEXAGON_FILL_RATIO_MIN", "Hexagono: llenado minimo", 0.55, 0.85,
          0.005, "", False,
          "Separa el hexagono del CUADRADO por abajo. Medido: cuadrado real "
          "hasta 0,720, hexagono desde 0,651. Estan pegados, asi que bajarlo "
          "para recuperar hexagonos comidos suma cuadrados al tacho "
          "equivocado sin ganar hexagonos."),
    Ficha("HEXAGON_FILL_RATIO_MAX", "Hexagono: llenado maximo", 0.80, 0.98,
          0.005, "", False,
          "Por arriba lo separa del circulo. El hexagono mas lleno medido "
          "dio 0,899."),
    Ficha("HEXAGON_CIRCULARITY_MAX", "Hexagono: circularidad maxima", 0.90,
          1.0, 0.001, "", False,
          "Lo unico que separa un hexagono de un CIRCULO MORDIDO (el que "
          "quedo cortado porque tocaba a otra pieza): en llenado se solapan "
          "por completo, en circularidad no. 1,0 desactiva la prueba."),
    Ficha("CIRCLE_CIRCULARITY_MIN", "Circulo: circularidad minima", 0.80, 0.99,
          0.005, "", False,
          "Es la ULTIMA prueba, o sea el cajon donde cae todo lo que no "
          "clasifico antes. Por eso conviene que sea exigente."),
    Ficha("WATERSHED_MIN_PEAK_DISTANCE", "Separacion de picos", 6, 40, 1, "px",
          True,
          "Aproximadamente el radio de la pieza mas chica. Muy alto funde "
          "dos piezas chicas; muy bajo parte una pieza sana en dos."),
    Ficha("WATERSHED_MIN_PEAK_HEIGHT", "Altura minima de pico", 1, 20, 1, "px",
          True, "Filtra picos falsos cerca del borde de la mascara."),
    Ficha("SPLIT_CLEANUP_KERNEL_SIZE", "Limpieza del corte", 0, 25, 1, "px",
          True,
          "Borra la pua que queda sobre la linea del corte al separar dos "
          "piezas. Sin ella dos circulos pegados salen los dos HEXAGONO. "
          "0 la desactiva."),
)

# La camara. La exposicion no es un umbral de deteccion pero es lo primero
# que hay que mirar cuando cambia la luz, asi que va en la misma pantalla.
FICHA_EXPOSICION = Ficha(
    "CAMERA_EXPOSURE", "Exposicion manual", -13, -4, 1, "2^n s", True,
    "En Windows el valor es un exponente: el tiempo de exposicion es 2^n "
    "segundos. Mas negativo = mas oscuro. Medido con las piezas sobre la "
    "cinta, la ventana util era angosta y -9 caia en el medio.")

FICHA_CORRECCION = Ficha(
    "correccion_pct", "Correccion", -60, 60, 1, "%", True,
    "Ganancia por software sobre el fotograma ya capturado: -10 lo deja al "
    "90 % del brillo que entrego la camara. Se usa junto con el automatico, "
    "para dejar que la camara se acomode sola pero corriendole el punto de "
    "trabajo. No recupera un pixel quemado.")

FICHAS: dict[str, Ficha] = {f.nombre: f for f in
                            FICHAS_GEOMETRIA + (FICHA_EXPOSICION,)}

# Topes de los sliders de color. H llega a 179 y no a 255 porque OpenCV
# comprime la rueda de tono a 0-179 para que entre en un byte.
H_MAX, SV_MAX = 179, 255

# Modos de exposicion. "auto" es el de fabrica y el que se quiere para poder
# mudar el robot; "manual" fija el tiempo; los dos aceptan ademas la
# correccion por software, que es la unica forma de dejar que la camara se
# acomode sola pero mas oscura de lo que ella elegiria.
MODO_AUTO = "auto"
MODO_MANUAL = "manual"


# ======================================================================
#  2. EL CONJUNTO DE AJUSTES
# ======================================================================

@dataclass
class Rango:
    """El color: UN arco de tono, con su caja de saturacion y de brillo.

    Un color es UN rango y no una lista de sectores, aunque OpenCV necesite
    dos para el rojo. Esa diferencia es la que hacia que la pantalla
    mostrara "dos rojos", que no significa nada: el rojo no son dos colores,
    es un solo color que queda partido al escribirlo, porque `cv2.inRange()`
    no sabe que H=179 y H=0 son vecinos.

    Asi que la vuelta a la rueda se representa aca, donde se entiende --
    `h0 > h1` quiere decir "de 145 para arriba, dar la vuelta por el 0 y
    seguir hasta 15"-- y se traduce a los dos sectores recien al escribir en
    `config`, que es el unico lugar donde hace falta.

    S y V son uno solo para todo el arco. Antes cada sector tenia el suyo, y
    los dos del rojo diferian (S>=50 en un lado, S>=35 en el otro) sin que
    hubiera ninguna razon medida para eso: el sector de abajo era historico
    y, segun el propio comentario de config.py, "hoy no aporta pixeles
    nuevos".
    """

    h0: int
    h1: int
    s0: int
    s1: int
    v0: int
    v1: int

    # ------------------------------------------------------------------
    def da_la_vuelta(self) -> bool:
        """El arco cruza el 0 de la rueda (el caso del rojo)."""

        return self.h1 < self.h0

    def tonos(self) -> list[int]:
        """Todos los tonos que el arco acepta, en orden de recorrido."""

        if not self.da_la_vuelta():
            return list(range(self.h0, self.h1 + 1))

        return list(range(self.h0, H_MAX + 1)) + list(range(0, self.h1 + 1))

    def ancho_h(self) -> int:
        return len(self.tonos())

    # ------------------------------------------------------------------
    def a_tramos(self) -> tuple:
        """Lo que consume `create_color_masks()`: uno o dos sectores.

        Dos solo cuando el arco da la vuelta, y ahi los dos llevan la MISMA
        caja de S y V: son el mismo color escrito en dos pedazos, no dos
        colores.
        """

        bajo = (self.s0, self.v0)
        alto = (self.s1, self.v1)

        if not self.da_la_vuelta():
            return (((self.h0, *bajo), (self.h1, *alto)),)

        return (((0, *bajo), (self.h1, *alto)),
                ((self.h0, *bajo), (H_MAX, *alto)))

    def contiene(self, h: float, s: float, v: float) -> bool:
        if not (self.s0 <= s <= self.s1 and self.v0 <= v <= self.v1):
            return False

        if not self.da_la_vuelta():
            return self.h0 <= h <= self.h1

        return h >= self.h0 or h <= self.h1

    def margen_h(self, h: float) -> float:
        """Cuanto le falta a este tono para caerse del arco.

        Negativo si ya esta afuera: cuanto se paso, por el lado mas cerca.
        Se mide sobre la RUEDA y no restando numeros, porque con un arco que
        da la vuelta la resta cruda da cualquier cosa.
        """

        def vuelta(desde: float, hasta: float) -> float:
            return (hasta - desde) % (H_MAX + 1)

        if not self.contiene(h, self.s0, self.v0):
            # Lo que falta para entrar por el borde mas cercano.
            return -min(vuelta(h, self.h0), vuelta(self.h1, h))

        return min(vuelta(self.h0, h), vuelta(h, self.h1))

    def centro(self) -> tuple[int, int, int]:
        """El color que MEJOR se detectaria con este rango.

        Es el medio del arco (dando la vuelta si hace falta) y el medio de
        la caja de S y V, o sea el punto mas lejos de todos los bordes.
        Sirve de referencia visual: si la pieza que se ve en la camara no se
        parece a este color, el rango esta corrido.
        """

        tonos = self.tonos()

        return (tonos[len(tonos) // 2] if tonos else 0,
                (self.s0 + self.s1) // 2, (self.v0 + self.v1) // 2)

    def copia(self) -> "Rango":
        return Rango(self.h0, self.h1, self.s0, self.s1, self.v0, self.v1)

    def a_lista(self) -> list[int]:
        return [self.h0, self.h1, self.s0, self.s1, self.v0, self.v1]

    # ------------------------------------------------------------------
    @classmethod
    def desde_tramos(cls, tramos) -> "Rango":
        """Fusiona los sectores de `config.py` en un solo arco.

        Dos reglas, y las dos tienen su motivo:

          * el ARCO es el mas chico que cubre todos los sectores. Se
            encuentra buscando el hueco mas grande entre los tonos aceptados
            --dando la vuelta-- y quedandose con el complemento. Para el
            rojo de fabrica (0-15 y 145-179) el hueco grande es 16-144, asi
            que el arco es 145 -> 15, que es exactamente lo que uno diria
            mirando la rueda.
          * la caja de S y V sale del sector MAS ANCHO, no del promedio ni
            del minimo. El sector ancho es el que contiene la pieza de
            verdad; el otro, cuando existe, es un resto historico. Promediar
            los dos moveria un umbral medido en base a uno que no lo esta.
        """

        tramos = list(tramos)

        if not tramos:
            return cls(0, H_MAX, 0, SV_MAX, 0, SV_MAX)

        aceptados = set()

        for bajo, alto in tramos:
            h0, h1 = int(bajo[0]), int(alto[0])
            aceptados.update(range(h0, h1 + 1) if h0 <= h1 else
                             list(range(h0, H_MAX + 1)) + list(range(0, h1 + 1)))

        ordenados = sorted(aceptados)

        if not ordenados:
            ordenados = [0]

        # El hueco mas grande entre dos tonos aceptados consecutivos, la
        # vuelta incluida. El arco arranca despues del hueco y termina antes.
        peor, corte = -1, 0

        for indice, tono in enumerate(ordenados):
            siguiente = ordenados[(indice + 1) % len(ordenados)]
            hueco = (siguiente - tono) % (H_MAX + 1)

            if hueco > peor:
                peor, corte = hueco, indice

        h0 = ordenados[(corte + 1) % len(ordenados)]
        h1 = ordenados[corte]

        dominante = max(tramos, key=lambda t: (int(t[1][0]) - int(t[0][0])) % (H_MAX + 1))
        bajo, alto = dominante

        return cls(int(h0), int(h1), int(bajo[1]), int(alto[1]),
                   int(bajo[2]), int(alto[2]))


@dataclass
class Ajustes:
    """Todo lo que la pestana de Vision puede cambiar, junto.

    Es lo que se guarda con el nombre de una habitacion y lo que se aplica
    al volver a ella. Se copia entero a proposito: media calibracion de una
    habitacion mezclada con media de otra no es la calibracion de ninguna.
    """

    colores: dict[str, Rango] = field(default_factory=dict)
    geometria: dict[str, float] = field(default_factory=dict)

    exposicion_modo: str = MODO_AUTO
    exposicion_valor: int = -9
    correccion_pct: int = 0

    #: Cual de los tres botones de temperatura de luz se aplico por ultima
    #: vez, o "" si se toco algo a mano despues. Es informativo: no cambia
    #: como se detecta, solo permite que la pantalla diga desde donde salio
    #: esta calibracion.
    temperatura: str = ""

    def copia(self) -> "Ajustes":
        return Ajustes(
            colores={c: r.copia() for c, r in self.colores.items()},
            geometria=dict(self.geometria),
            exposicion_modo=self.exposicion_modo,
            exposicion_valor=self.exposicion_valor,
            correccion_pct=self.correccion_pct,
            temperatura=self.temperatura)

    # ------------------------------------------------------------------
    def a_json(self) -> dict:
        return {
            "colores": {c: r.a_lista() for c, r in self.colores.items()},
            "geometria": self.geometria,
            "exposicion_modo": self.exposicion_modo,
            "exposicion_valor": self.exposicion_valor,
            "correccion_pct": self.correccion_pct,
            "temperatura": self.temperatura,
        }

    @classmethod
    def desde_json(cls, datos: dict) -> "Ajustes":
        """Reconstruye unos ajustes guardados, tolerando lo que falte.

        Un archivo escrito por una version anterior --o editado a mano-- no
        tiene por que tener todos los campos. Lo que falta se completa con
        lo que dice config.py, que es preferible a negarse a cargar la
        habitacion entera por un campo nuevo.
        """

        base = de_fabrica()

        colores = {}

        for color, guardado in (datos.get("colores") or {}).items():
            if color not in COLORES:
                continue

            rango = _rango_guardado(guardado)

            if rango is not None:
                colores[color] = rango

        geometria = {n: v for n, v in (datos.get("geometria") or {}).items()
                     if n in FICHAS}

        return cls(
            colores=colores or base.colores,
            geometria={**base.geometria, **geometria},
            exposicion_modo=(datos.get("exposicion_modo")
                             if datos.get("exposicion_modo") in (MODO_AUTO, MODO_MANUAL)
                             else base.exposicion_modo),
            exposicion_valor=int(datos.get("exposicion_valor",
                                           base.exposicion_valor)),
            correccion_pct=int(datos.get("correccion_pct", base.correccion_pct)),
            temperatura=str(datos.get("temperatura", "")))


def _rango_guardado(guardado) -> Optional[Rango]:
    """Lee un color de un `vision.json`, en cualquiera de las dos formas.

    La nueva son seis numeros sueltos (un arco). La vieja era una lista de
    sectores, de cuando un color se guardaba partido; se fusiona al leerla.
    Se aceptan las dos porque un archivo guardado ayer sigue siendo una
    calibracion medida, y perderla por un cambio de formato seria perder una
    tarde de trabajo de alguien.
    """

    if not isinstance(guardado, (list, tuple)) or not guardado:
        return None

    if all(isinstance(x, (int, float)) for x in guardado):
        return Rango(*[int(x) for x in guardado]) if len(guardado) == 6 else None

    tramos = [((int(t[0]), int(t[2]), int(t[4])), (int(t[1]), int(t[3]), int(t[5])))
              for t in guardado
              if isinstance(t, (list, tuple)) and len(t) == 6]

    return Rango.desde_tramos(tramos) if tramos else None


def de_fabrica() -> Ajustes:
    """Los valores tal como estan escritos en `vision_python/config.py`.

    No es una copia de los numeros: se leen del modulo. Ese archivo tiene al
    lado de cada umbral la medicion que lo eligio, y duplicar los valores
    aca los dejaria separandose en silencio del dia que se recalibre.
    """

    return Ajustes(
        colores={color: Rango.desde_tramos(_RANGOS_ORIGINALES[color])
                 for color in _colores_conocidos()},
        geometria={f.nombre: _ORIGINAL_GEOMETRIA[f.nombre] for f in FICHAS_GEOMETRIA},
        exposicion_modo=MODO_AUTO if _ORIGINAL_AUTO else MODO_MANUAL,
        exposicion_valor=int(_ORIGINAL_EXPOSICION),
        correccion_pct=0,
        temperatura="")


def _colores_conocidos() -> tuple[str, ...]:
    """Los de `COLORES` que existen de verdad en la tabla de config.py.

    Si alguna vez se agrega un color alla (que es todo lo que hace falta:
    `create_color_masks()` recorre el diccionario), esto lo deja aparecer
    en la pantalla con solo sumarlo a `COLORES`, y mientras tanto no rompe
    si uno de los tres se saca.
    """

    return tuple(c for c in COLORES if c in _RANGOS_ORIGINALES)


# Foto de config.py tomada al importar, ANTES de que nadie escriba encima.
# Es lo que hace que "volver a fabrica" siga existiendo despues de aplicar
# tres habitaciones seguidas: una vez que se pisa `config.COLOR_HSV_RANGES`,
# el valor original ya no esta en ningun lado.
_RANGOS_ORIGINALES: dict[str, tuple] = {
    color: tuple(rangos) for color, rangos in config.COLOR_HSV_RANGES.items()}

_ORIGINAL_GEOMETRIA: dict[str, float] = {
    f.nombre: getattr(config, f.nombre) for f in FICHAS_GEOMETRIA}

_ORIGINAL_AUTO = config.CAMERA_AUTO_EXPOSURE
_ORIGINAL_EXPOSICION = config.CAMERA_EXPOSURE


# ======================================================================
#  3. APLICAR Y LEER
# ======================================================================

def aplicar(ajustes: Ajustes) -> None:
    """Escribe los ajustes en el modulo `config`, o sea en la deteccion.

    Se ve en el fotograma siguiente: `detection.py` y `camera.py` leen
    `config.X` en el punto de uso justamente para esto. Lo unico que no se
    aplica aca es la exposicion sobre el dispositivo, que la tiene que
    pedir el hilo de vision entre dos fotogramas (ver `Vision`).
    """

    # `a_tramos()` es donde el arco se parte en los uno o dos sectores que
    # `cv2.inRange()` necesita. Es el UNICO lugar del programa donde el rojo
    # se ve como dos cosas, y es el correcto: la limitacion es de inRange,
    # no del color.
    config.COLOR_HSV_RANGES = {
        color: rango.a_tramos() for color, rango in ajustes.colores.items()}

    for nombre, valor in ajustes.geometria.items():
        ficha = FICHAS.get(nombre)

        if ficha is None:
            continue

        setattr(config, nombre, ficha.saturar(valor))

    config.CAMERA_AUTO_EXPOSURE = ajustes.exposicion_modo == MODO_AUTO
    config.CAMERA_EXPOSURE = int(ajustes.exposicion_valor)


def leer() -> Ajustes:
    """Lo que hay puesto AHORA en `config`, venga de donde venga.

    Se usa para arrancar la pantalla mostrando lo que de verdad esta
    corriendo, en vez de lo que dice el archivo guardado. Son la misma cosa
    mientras nadie edite config.py a mano, y cuando no lo son, la que vale
    es la que esta andando.
    """

    return Ajustes(
        colores={color: Rango.desde_tramos(config.COLOR_HSV_RANGES[color])
                 for color in _colores_conocidos()
                 if color in config.COLOR_HSV_RANGES},
        geometria={f.nombre: getattr(config, f.nombre) for f in FICHAS_GEOMETRIA},
        exposicion_modo=MODO_AUTO if config.CAMERA_AUTO_EXPOSURE else MODO_MANUAL,
        exposicion_valor=int(config.CAMERA_EXPOSURE),
        correccion_pct=0,
        temperatura="")


# ======================================================================
#  4. TEMPERATURA DE LUZ
# ======================================================================
#
#  Los tres botones de ajuste rapido. No son una calibracion: son un PUNTO
#  DE PARTIDA para cuando el robot llega a una sala nueva y no hay tiempo de
#  hacer el ajuste automatico, y despues se afina a mano o se corre el
#  automatico igual.
#
#  Que hace cada lampara con las piezas, que es de donde salen los numeros:
#
#    CALIDA (incandescente, ~2700 K) tiene poco azul. Un verde se ve mas
#        amarillento --su tono BAJA hacia el 30-- y un azul pierde
#        saturacion porque le falta la componente que lo hace azul. El rojo
#        es el que menos sufre: sobra justamente rojo.
#    FRIA (LED frio o luz de dia, ~6500 K) es al reves: sobra azul, los
#        tonos SUBEN y el rojo es el que pierde saturacion.
#
#  Asi que cada preset corre el tono de fabrica y afloja el piso de
#  saturacion del color que la lampara castiga, ensanchando el rango en la
#  direccion en que sabemos que se va a mover la pieza. El ancho del corrimiento
#  (4 grados de Hue) es del orden de lo que separa los rangos calibrados del
#  fondo, asi que es un empujon y no un salto al vacio.
#
#  IMPORTANTE, y esta escrito para que nadie lo tome por mas de lo que es:
#  estos numeros NO estan medidos contra las lamparas de la facultad -- son
#  la direccion correcta con una magnitud razonable. El que da el numero
#  bueno es el ajuste automatico con las piezas delante. Si algun dia se
#  miden, este es el comentario a reescribir.

TEMPERATURAS = ("calida", "neutra", "fria")

NOMBRE_TEMPERATURA = {"calida": "Luz calida", "neutra": "Luz neutra",
                      "fria": "Luz fria"}

# Por temperatura y color: (corrimiento de H, cuanto se afloja el piso de S,
# cuanto se afloja el piso de V).
_CORRIMIENTOS: dict[str, dict[str, tuple[int, int, int]]] = {
    "calida": {"ROJO": (0, -5, -10), "VERDE": (-4, -8, -15), "AZUL": (-3, -12, -15)},
    "neutra": {"ROJO": (0, 0, 0), "VERDE": (0, 0, 0), "AZUL": (0, 0, 0)},
    "fria":   {"ROJO": (+2, -12, -10), "VERDE": (+4, -6, -10), "AZUL": (+3, -4, -5)},
}

# Correccion de exposicion que acompana a cada preset. Una sala con luz
# calida suele ser mas oscura que un aula con tubos, y el automatico de la
# camara la levanta de mas.
_CORRECCION_TEMPERATURA = {"calida": 0, "neutra": 0, "fria": -8}


def aplicar_temperatura(temperatura: str) -> Ajustes:
    """Ajustes de fabrica corridos hacia la luz que se eligio.

    Parte SIEMPRE de fabrica y no de lo que hay puesto: si partiera de lo
    actual, apretar dos veces el mismo boton correria el tono dos veces, y
    alternar entre calida y fria iria acumulando corrimientos hasta dejar
    los rangos en cualquier lado. Un preset tiene que ser un lugar, no un
    empujon.
    """

    if temperatura not in _CORRIMIENTOS:
        raise ValueError(f"temperatura desconocida: {temperatura!r}")

    ajustes = de_fabrica()
    corrimientos = _CORRIMIENTOS[temperatura]

    for color, rango in ajustes.colores.items():
        dh, ds, dv = corrimientos.get(color, (0, 0, 0))

        # El tono se corre entero (las dos puntas del arco), asi que el
        # ancho no cambia: se mueve la ventana, no se agranda. Y se corre
        # DANDO LA VUELTA -- con el modulo y no saturando en 0 y 179 --,
        # porque el arco del rojo ya esta pasando por ahi: saturarlo lo
        # aplastaria contra el borde en vez de rotarlo.
        rango.h0 = (rango.h0 + dh) % (H_MAX + 1)
        rango.h1 = (rango.h1 + dh) % (H_MAX + 1)

        # Los pisos de S y V si se aflojan, que es agrandar el rango hacia
        # abajo. Estos si saturan: no hay nada por debajo de 0.
        rango.s0 = max(0, min(SV_MAX, rango.s0 + ds))
        rango.v0 = max(0, min(SV_MAX, rango.v0 + dv))

    ajustes.correccion_pct = _CORRECCION_TEMPERATURA[temperatura]
    ajustes.temperatura = temperatura

    return ajustes


# ======================================================================
#  5. HABITACIONES GUARDADAS
# ======================================================================

@dataclass
class Habitacion:
    nombre: str
    ajustes: Ajustes
    guardada: float = 0.0       # time.time() de la ultima vez

    def a_json(self) -> dict:
        return {"guardada": self.guardada, **self.ajustes.a_json()}


class Memoria:
    """Las habitaciones guardadas, en disco.

    "La config actual anda en mi pieza": se le pone nombre y queda. Despues
    el robot va al aula, se recalibra, se guarda con otro nombre, y volver
    de una a la otra es un click en vez de una tarde.

    El archivo es `pc/config/vision.json` y SI va al repositorio, igual que
    las secuencias de teach: una calibracion medida es trabajo hecho, y la
    del aula de la facultad la va a necesitar cualquier PC que se lleve.
    Lo que no va es el puerto COM ni el encuadre de la camara, que viven en
    local.json porque son distintos en cada maquina a proposito.
    """

    def __init__(self, archivo: Optional[Path] = None) -> None:
        self.archivo = Path(archivo) if archivo else ARCHIVO
        self.habitaciones: dict[str, Habitacion] = {}
        self.activa: str = ""

        self.cargar()

    # ------------------------------------------------------------------
    def cargar(self) -> None:
        if not self.archivo.exists():
            return

        try:
            datos = json.loads(self.archivo.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError) as err:
            print(f"[vision] no se pudo leer {self.archivo.name}: {err}")
            return

        for nombre, guardada in (datos.get("habitaciones") or {}).items():
            self.habitaciones[nombre] = Habitacion(
                nombre=nombre,
                ajustes=Ajustes.desde_json(guardada),
                guardada=float(guardada.get("guardada", 0.0)))

        activa = datos.get("activa", "")
        self.activa = activa if activa in self.habitaciones else ""

    def guardar_archivo(self) -> None:
        datos = {
            "activa": self.activa,
            "habitaciones": {n: h.a_json() for n, h in self.habitaciones.items()},
        }

        try:
            self.archivo.parent.mkdir(parents=True, exist_ok=True)
            self.archivo.write_text(
                json.dumps(datos, indent=2, ensure_ascii=False), encoding="utf-8")
        except OSError as err:
            print(f"[vision] no se pudo guardar {self.archivo.name}: {err}")

    # ------------------------------------------------------------------
    def nombres(self) -> list[str]:
        """Ordenadas por nombre y no por fecha: es una lista para BUSCAR.

        Con cuatro o cinco habitaciones, "donde estaba Aula facultad" se
        contesta mucho mas rapido si el orden no cambia cada vez que se
        guarda una.
        """

        return sorted(self.habitaciones, key=str.casefold)

    def guardar(self, nombre: str, ajustes: Ajustes) -> Habitacion:
        nombre = nombre.strip()

        if not nombre:
            raise ValueError("la habitacion necesita un nombre")

        habitacion = Habitacion(nombre=nombre, ajustes=ajustes.copia(),
                                guardada=time.time())

        self.habitaciones[nombre] = habitacion
        self.activa = nombre
        self.guardar_archivo()

        return habitacion

    def cargar_habitacion(self, nombre: str) -> Optional[Ajustes]:
        habitacion = self.habitaciones.get(nombre)

        if habitacion is None:
            return None

        self.activa = nombre
        self.guardar_archivo()

        # Una COPIA: si se devolviera la guardada, mover un slider editaria
        # la habitacion en memoria sin que nadie haya apretado Guardar, y al
        # cerrar el programa se perderia la version buena sin aviso.
        return habitacion.ajustes.copia()

    def borrar(self, nombre: str) -> bool:
        if nombre not in self.habitaciones:
            return False

        del self.habitaciones[nombre]

        if self.activa == nombre:
            self.activa = ""

        self.guardar_archivo()

        return True


# ======================================================================
#  6. MEDIR LO QUE HAY EN LA CINTA
# ======================================================================
#
#  Esta es la parte que hace util a toda la pantalla, y conviene entender
#  por que no alcanza con mirar lo que detecta `detect_objects()`.
#
#  Cuando el verde deja de detectarse, la deteccion no da un verde malo: da
#  CERO verdes. O sea que justo cuando hace falta saber que color tiene la
#  pieza, el camino normal no informa nada. Preguntarle a la deteccion "de
#  que color viste la pieza" solo contesta cuando ya esta bien calibrada,
#  que es cuando la respuesta no hace falta.
#
#  Asi que se mide de una forma que NO depende de los rangos configurados:
#  todo lo que sobresale del fondo por saturacion es candidato a pieza. La
#  cinta es gris (S 17-50 de mediana medido, ver el comentario del VERDE en
#  config.py) y las piezas son de plastico de color, asi que la saturacion
#  separa las dos poblaciones sin saber de que color es ninguna.

# Piso de saturacion para dar un pixel por "de una pieza" al medir. Es
# generoso a proposito --50 esta por encima del maximo medido de la cinta
# (S hasta 50) y muy por debajo del minimo de las piezas (61)-- y no es un
# umbral de deteccion: solo sirve para encontrar DONDE hay una pieza que
# despues se mide entera.
S_PIEZA = 55
V_PIEZA = 60

# Cuanto se erosiona la mascara de cada pieza antes de medir su color. El
# borde de una pieza es una franja mezclada con la cinta --ni el color de
# la pieza ni el del fondo-- y meterla en la cuenta corre todas las medianas
# hacia el gris. Las mediciones de config.py se hicieron sobre el nucleo
# erosionado por este mismo motivo.
EROSION_NUCLEO = 7


@dataclass
class Muestra:
    """Una pieza vista en el fotograma, medida sin mirar los rangos."""

    color: str                  # a que color se PARECE (por tono)
    detectado: Optional[str]    # que color dicen los rangos de hoy, o None

    h: int
    s: int
    v: int

    #: Percentiles 5 y 95 de cada canal sobre el nucleo de la pieza. Es lo
    #: que dice si el rango la contiene entera o le corta una parte -- la
    #: mediana sola no lo puede decir.
    h5: int
    h95: int
    s5: int
    s95: int
    v5: int
    v95: int

    area: float
    centro: tuple[int, int]

    def hex(self) -> str:
        return hsv_a_hex(self.h, self.s, self.v)


def _mascara_piezas(hsv: np.ndarray) -> np.ndarray:
    """Todo lo que sobresale del fondo, sin mirar de que color es."""

    mascara = cv2.inRange(
        hsv,
        np.array((0, S_PIEZA, V_PIEZA), dtype=np.uint8),
        np.array((H_MAX, SV_MAX, SV_MAX), dtype=np.uint8))

    kernel = np.ones((5, 5), dtype=np.uint8)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)

    return cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)


def _color_mas_parecido(tono: float) -> str:
    """A cual de los tres se parece este tono, dando la vuelta a la rueda."""

    def distancia(color: str) -> float:
        return min(min(abs(tono - ref), H_MAX + 1 - abs(tono - ref))
                   for ref in TONO_REFERENCIA[color])

    return min(_colores_conocidos(), key=distancia)


def _color_detectado(h: float, s: float, v: float) -> Optional[str]:
    """Que color le asignarian los rangos que estan puestos ahora."""

    for color, rangos in config.COLOR_HSV_RANGES.items():
        for bajo, alto in rangos:
            if (bajo[0] <= h <= alto[0] and bajo[1] <= s <= alto[1]
                    and bajo[2] <= v <= alto[2]):
                return color

    return None


def muestrear(frame: np.ndarray) -> list[Muestra]:
    """Mide el color real de cada pieza del fotograma.

    Devuelve una muestra por pieza encontrada, ordenadas de mayor a menor
    area. No usa los rangos configurados para ENCONTRARLAS --solo para
    informar si hoy las agarra o no--, que es todo el punto: una pieza que
    la deteccion no ve tiene que aparecer igual, con el numero que explica
    por que no la ve.
    """

    if frame is None or frame.size == 0:
        return []

    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    mascara = _mascara_piezas(hsv)

    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)

    muestras: list[Muestra] = []

    for contorno in contornos:
        area = cv2.contourArea(contorno)

        # El mismo filtro de area que la deteccion: lo que no llega al
        # minimo no es una pieza, es ruido, y promediarlo ensucia la
        # medicion justo con los pixeles menos confiables.
        if area < config.MIN_CONTOUR_AREA or area > config.MAX_CONTOUR_AREA:
            continue

        relleno = np.zeros(mascara.shape, dtype=np.uint8)
        cv2.drawContours(relleno, [contorno], -1, 255, -1)

        nucleo = cv2.erode(
            relleno, np.ones((EROSION_NUCLEO, EROSION_NUCLEO), dtype=np.uint8))

        # Una pieza chica se puede quedar sin nucleo despues de erosionar.
        # Ahi se mide el contorno entero: peor medicion que con nucleo, pero
        # muchisimo mejor que no informar la pieza.
        if not nucleo.any():
            nucleo = relleno

        pixeles = hsv[nucleo > 0]

        if len(pixeles) < 20:
            continue

        # El tono se promedia CIRCULARMENTE: el rojo vive en las dos puntas
        # de la rueda, y una pieza con pixeles en 178 y en 2 tiene mediana
        # aritmetica 90, que es cian. Se rota la rueda para que el grueso de
        # los pixeles quede junto, se saca la mediana ahi y se desrota.
        tonos = pixeles[:, 0].astype(np.float64)
        rotacion = _rotacion_para(tonos)
        tonos_rot = (tonos + rotacion) % (H_MAX + 1)

        def p(canal, percentil: float) -> float:
            return float(np.percentile(canal, percentil))

        h_mediana = (p(tonos_rot, 50) - rotacion) % (H_MAX + 1)
        h_bajo = (p(tonos_rot, 5) - rotacion) % (H_MAX + 1)
        h_alto = (p(tonos_rot, 95) - rotacion) % (H_MAX + 1)

        s, v = pixeles[:, 1], pixeles[:, 2]

        momentos = cv2.moments(contorno)
        centro = ((int(momentos["m10"] / momentos["m00"]),
                   int(momentos["m01"] / momentos["m00"]))
                  if momentos["m00"] else (0, 0))

        muestras.append(Muestra(
            color=_color_mas_parecido(h_mediana),
            detectado=_color_detectado(h_mediana, p(s, 50), p(v, 50)),
            h=int(round(h_mediana)), s=int(round(p(s, 50))), v=int(round(p(v, 50))),
            h5=int(round(h_bajo)), h95=int(round(h_alto)),
            s5=int(round(p(s, 5))), s95=int(round(p(s, 95))),
            v5=int(round(p(v, 5))), v95=int(round(p(v, 95))),
            area=area, centro=centro))

    return sorted(muestras, key=lambda m: m.area, reverse=True)


def _rotacion_para(tonos: np.ndarray) -> float:
    """Cuanto rotar la rueda de tono para que estos pixeles queden juntos.

    Se rota al hueco mas grande: se ordenan los tonos, se busca el mayor
    salto entre dos consecutivos (dando la vuelta) y se corta ahi. Si la
    pieza no cruza el 0 esto no cambia nada; si lo cruza, lo arregla.
    """

    ordenados = np.unique(np.round(tonos))

    if len(ordenados) < 2:
        return 0.0

    huecos = np.diff(ordenados)
    vuelta = (ordenados[0] + H_MAX + 1) - ordenados[-1]

    if vuelta <= huecos.max():
        return 0.0

    # El hueco mas grande es el de la vuelta: hay que cortar ahi, o sea
    # llevar el primer tono despues del hueco al cero.
    return float(H_MAX + 1 - ordenados[-1])


def hsv_a_hex(h: int, s: int, v: int) -> str:
    """El color HSV de OpenCV como '#rrggbb', para pintar una muestra."""

    pixel = np.array([[[int(h) % (H_MAX + 1),
                        max(0, min(SV_MAX, int(s))),
                        max(0, min(SV_MAX, int(v)))]]], dtype=np.uint8)

    b, g, r = cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0][0]

    return f"#{r:02x}{g:02x}{b:02x}"


# ======================================================================
#  7. AJUSTE AUTOMATICO
# ======================================================================

@dataclass
class Resultado:
    """Que salio del ajuste automatico, ande o no."""

    ok: bool
    mensaje: str
    ajustes: Optional[Ajustes] = None
    muestras: list[Muestra] = field(default_factory=list)


# Margenes con los que se abre el rango alrededor de lo medido. Cada uno
# ataca una cosa distinta y por eso son tres numeros y no uno:
#
#   El de TONO cubre la variacion entre piezas del mismo color y la sombra
#       de un borde. Se suma a los percentiles 5-95, que ya excluyen las
#       colas.
#   El de SATURACION es un piso, no una ventana: se baja por debajo del
#       minimo medido, porque una pieza mas lejos de la lampara satura
#       menos. El techo queda siempre en 255 (una pieza no satura "de mas").
#   El de BRILLO es el mas generoso de los tres, y es deliberado: el piso de
#       V mato una vez la deteccion entera del verde por estar pegado al
#       valor de la pieza (0/120 circulos, ver config.py). V ya casi no
#       separa nada -- es solo un piso contra la sombra profunda -- asi que
#       conviene lejos.
#
# Los tres numeros salen de comparar los rangos que se eligieron a mano en
# config.py contra la medicion de la pieza que los origino:
#
#     verde   pieza H 63-69, S 82-98, V 176-193
#     rango elegido a mano   H 50-74   S>=56    V>=100
#     o sea margen           13 / 5     26        76
#
# El de tono quedo en 7 y no en 13 porque el 50 de abajo no es margen: el
# comentario del verde dice que ese piso es plano de 36 a 60 (no hay nada
# mas en esa franja de Hue), asi que esta lejos porque no costaba nada, no
# porque hiciera falta. Por arriba, 74 y 76 dieron los dos 120/120 medidos.
MARGEN_H = 7
MARGEN_S = 25
MARGEN_V = 70

# Cuanto tiene que separarse el rango propuesto del fondo para darlo por
# bueno. Si el tono de una pieza queda a menos de esto del tono de la cinta,
# el rango va a fugar fondo y no hay margen que lo arregle: lo que falta es
# luz o una pieza de otro color, y hay que decirlo en vez de guardar una
# calibracion que no va a andar.
SEPARACION_MINIMA_H = 5


def calibrar(frame: np.ndarray, base: Optional[Ajustes] = None) -> Resultado:
    """Propone rangos de color a partir de un fotograma con las tres piezas.

    La receta es la misma que se uso a mano para elegir los rangos que estan
    en config.py, y esta escrita ahi con las mediciones al lado: se mide el
    NUCLEO de cada pieza (no el borde, que es una franja mezclada con la
    cinta), se toman percentiles en vez de minimos y maximos (una cola de
    tres pixeles no puede decidir un umbral), y se abre el rango con margen
    -- distinto por canal, porque cada canal falla distinto.

    No aplica nada: devuelve la propuesta. Aplicarla o no es de la interfaz,
    que es la que puede mostrar antes/despues y dejar deshacer.
    """

    muestras = muestrear(frame)

    if not muestras:
        return Resultado(False, "No se ve ninguna pieza. Pone una de cada "
                                "color sobre la cinta, con la cinta parada.")

    # Una por color: si hay dos del mismo, se queda la mas grande (que es la
    # que esta mas entera dentro del cuadro). Mezclar dos piezas del mismo
    # color en una sola medicion ensancharia el rango con la diferencia
    # entre ellas, que es justo lo que no se quiere medir.
    por_color: dict[str, Muestra] = {}

    for muestra in muestras:
        if muestra.color not in por_color:
            por_color[muestra.color] = muestra

    faltan = [NOMBRE_COLOR[c] for c in _colores_conocidos() if c not in por_color]

    if faltan:
        vistos = ", ".join(f"{NOMBRE_COLOR[m.color]} (H={m.h})"
                           for m in muestras[:4])

        return Resultado(
            False,
            f"Falta ver: {', '.join(faltan)}. Se encontraron {len(muestras)} "
            f"pieza(s): {vistos}.",
            muestras=muestras)

    # El fondo: todo lo que no es ninguna de las piezas. Es la mitad que no
    # se puede saltear -- un rango se elige por donde NO tiene que llegar, y
    # eso es la cinta.
    fondo = _medir_fondo(frame)

    propuesta = (base.copia() if base else de_fabrica())
    avisos: list[str] = []

    for color, muestra in por_color.items():
        rango, aviso = _rango_para(muestra, fondo)
        propuesta.colores[color] = rango

        if aviso:
            avisos.append(f"{NOMBRE_COLOR[color]}: {aviso}")

    # La calibracion la decidieron las piezas, no un preset: dejar puesto el
    # nombre de la temperatura anterior haria creer que este resultado sale
    # de ese boton.
    propuesta.temperatura = ""

    mensaje = "Calibrado con las 3 piezas."

    if avisos:
        mensaje += " Ojo: " + "; ".join(avisos)

    return Resultado(True, mensaje, ajustes=propuesta, muestras=muestras)


def _medir_fondo(frame: np.ndarray) -> dict[str, float]:
    """Percentiles de la cinta, o sea de lo que NO tiene que entrar."""

    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    piezas = _mascara_piezas(hsv)

    # Se dilata la mascara de piezas antes de invertirla: el halo del borde
    # no es cinta y meterlo en la estadistica del fondo la corre hacia el
    # color de las piezas, que es exactamente al reves de lo que se quiere.
    halo = cv2.dilate(piezas, np.ones((15, 15), dtype=np.uint8))
    pixeles = hsv[halo == 0]

    if len(pixeles) < 100:
        return {}

    return {
        "h5": float(np.percentile(pixeles[:, 0], 5)),
        "h95": float(np.percentile(pixeles[:, 0], 95)),
        "s95": float(np.percentile(pixeles[:, 1], 95)),
        "s99": float(np.percentile(pixeles[:, 1], 99)),
        "v95": float(np.percentile(pixeles[:, 2], 95)),
    }


def _rango_para(muestra: Muestra, fondo: dict[str, float]) -> tuple[Rango, str]:
    """El rango HSV que contiene a esta pieza y deja afuera a la cinta."""

    aviso = ""

    h0 = muestra.h5 - MARGEN_H
    h1 = muestra.h95 + MARGEN_H

    # Piso de saturacion: por debajo de la pieza, pero por encima de la
    # cinta. Cuando los dos requisitos se pelean gana la pieza --un rango
    # que no contiene la pieza no detecta nada, uno que fuga un poco de
    # fondo todavia tiene el filtro de area detras-- y se avisa.
    piso_s = muestra.s5 - MARGEN_S
    techo_fondo = fondo.get("s99")

    if techo_fondo is not None and piso_s <= techo_fondo:
        holgado = (muestra.s5 + techo_fondo) / 2.0

        if holgado > techo_fondo:
            piso_s = holgado
        else:
            aviso = ("la pieza satura casi como la cinta; conviene mas luz "
                     "o bajar la exposicion")

    piso_v = max(0, muestra.v5 - MARGEN_V)

    # Separacion contra el tono del fondo. No se corrige sola --recortar el
    # rango dejaria afuera parte de la pieza-- pero se avisa, que es lo que
    # permite entender por que despues aparecen falsas.
    if fondo:
        if fondo["h5"] - SEPARACION_MINIMA_H <= muestra.h <= fondo["h95"] + SEPARACION_MINIMA_H:
            aviso = (aviso + "; " if aviso else "") + \
                f"el tono de la pieza (H={muestra.h}) cae sobre el de la cinta"

    piso_s = int(max(0, min(SV_MAX, round(piso_s))))

    # El arco puede dar la vuelta a la rueda --el caso del rojo, cuya pieza
    # mide H cerca de 175 y el margen la lleva por encima de 179-- y eso se
    # escribe con el modulo, sin ningun caso especial: un arco que da la
    # vuelta es simplemente uno con h0 > h1. Partirlo en dos es problema de
    # `Rango.a_tramos()`, y solo al momento de escribir en `config`.
    return (Rango(int(h0) % (H_MAX + 1), int(h1) % (H_MAX + 1),
                  piso_s, SV_MAX, piso_v, SV_MAX),
            aviso)
