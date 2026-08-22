"""Secuencias del modo teach: grabarlas, simplificarlas y guardarlas.

El firmware ejecuta; acá se decide **qué** ejecutar. Ese corte no es
arbitrario: una secuencia con nombre, que se renombra, se borra y se
verifica por etapas es un archivo, y un archivo vive mucho mejor en una PC
que en la NVS de un ESP32.

DE QUÉ SE GRABA UNA POSICIÓN
----------------------------
De la posición **comandada**, no de la medida por los encoders. La medida
trae ±1° de ruido del AS5600 analógico, y ese ruido convertido a cartesiano
son milímetros que después se reproducen como temblor. Lo que el operador
enseñó es a dónde llevó el brazo, no cómo vibró el sensor mientras tanto.

POR QUÉ SE SIMPLIFICA
---------------------
Se muestrea a 20 Hz, así que un movimiento de 20 s son 400 muestras. El
buffer del firmware entra 150 puntos, pero el problema real no es la
memoria: `Stepper::moveTo()` reinicia la rampa en cada destino, así que cada
punto es un arranque y una frenada. Reproducir 400 puntos sería un movimiento
a los tirones y lentísimo. Con Ramer–Douglas–Peucker a unos milímetros de
tolerancia, una trayectoria hecha a mano baja a unas decenas de puntos y se
reproduce como lo que es: una sucesión de tramos rectos.

LO QUE NO SE SIMPLIFICA
-----------------------
Los cambios de bomba y las pausas. Un `E` apretado en el aire y medio segundo
quieto esperando que el vacío agarre es exactamente lo que hay que conservar,
y es lo primero que borraría un simplificador que sólo mire geometría.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

ARCHIVO = Path(__file__).resolve().parents[1] / "config" / "movimientos.json"


def es_local(nombre: str) -> bool:
    """Un movimiento que todavía tiene el nombre de fábrica.

    Los que conservan el «Movimiento N» con el que nacieron son la prueba de
    la tarde: se graban de a diez, se miran una vez y se tiran. Los que valen
    la pena se renombran. Esa es toda la regla, y de ahí sale en qué archivo
    se guarda cada uno (ver `Biblioteca`).
    """

    return "movimiento" in nombre.lower()

# Cadencia de muestreo de la grabación. 20 Hz alcanza de sobra para una mano
# humana moviendo un joystick y deja las muestras lo bastante juntas como
# para que la simplificación tenga de dónde elegir.
PERIODO_MUESTREO_S = 0.05

# Tolerancia de partida del simplificador, en cm. 0,25 cm es del orden del
# tramo con el que avanza el jog, o sea: se conserva todo lo que el operador
# realmente pudo apuntar, y se tira lo que es cuantización del propio jog.
TOLERANCIA_CM = 0.25

# Tope del buffer del firmware (Robot::TEACH_MAX_PUNTOS). Si la secuencia no
# entra con la tolerancia de arriba, se afloja la tolerancia hasta que entre:
# es preferible una versión un poco más gruesa del movimiento que una
# recortada por la mitad.
MAX_PUNTOS = 150

# Largo mínimo de un tramo, en cm. Dos puntos más juntos que esto se unen.
#
# No es por ahorrar memoria: el redondeo de esquinas del firmware sólo puede
# encadenar un tramo si lo que queda alcanza para frenar desde la velocidad
# que trae. Con tramos de milímetros nunca alcanza, así que cae al camino de
# siempre —frenar en cada punto— y se pierde justo lo que se quería ganar.
TRAMO_MINIMO_CM = 0.6

# Separación a la que se reparten los puntos de la ruta final, en cm.
#
# Los puntos quedan repartidos parejo (ver `_repartir_parejo`), pero JUNTOS:
# a esta distancia la trayectoria se sigue de cerca y se puede grabar con
# detalle, que es como tiene que sentirse al enseñar un movimiento.
#
# Subirlo hace tramos más largos, y un tramo largo alcanza más velocidad
# (el pico va con `sqrt(a·d)`). O sea que es el canje entre seguir el camino
# con detalle y reproducirlo rápido. Si un movimiento sale demasiado lento,
# es más barato subir `t_acel` desde la interfaz que engordar esto.
PASO_PAREJO_CM = 0.6

# Los cuatro estados de verificación. Una secuencia recién grabada arranca
# sin verificar y sólo sube de escalón cuando alguien confirma que la pasada
# anterior salió bien.
SIN_VERIFICAR = 0
ESCALONES = (15, 50, 100)

_SIGUIENTE = {0: 15, 15: 50, 50: 100, 100: 100}


@dataclass
class Muestra:
    """Una posición comandada durante la grabación."""

    t: float          # segundos desde que arrancó la grabación
    x: float
    y: float
    z: float
    bomba: bool


@dataclass
class Punto:
    """Un punto de la ruta que se sube al firmware."""

    x: float
    y: float
    z: float
    bomba: bool = False
    espera_ms: int = 0   # cuánto se queda quieto AL LLEGAR

    def como_dict(self) -> dict:
        return {"x": round(self.x, 3), "y": round(self.y, 3), "z": round(self.z, 3),
                "bomba": self.bomba, "espera_ms": self.espera_ms}

    @staticmethod
    def desde_dict(d: dict) -> "Punto":
        return Punto(
            x=float(d.get("x", 0.0)),
            y=float(d.get("y", 0.0)),
            z=float(d.get("z", 0.0)),
            bomba=bool(d.get("bomba", False)),
            espera_ms=int(d.get("espera_ms", 0)),
        )


@dataclass
class Movimiento:
    nombre: str
    puntos: list[Punto] = field(default_factory=list)

    # A qué porcentaje se verificó: 0, 15, 50 o 100. Volver a grabar encima
    # lo devuelve a 0 -- es otro movimiento, aunque conserve el nombre.
    verificado: int = SIN_VERIFICAR

    creado: str = ""
    duracion_s: float = 0.0   # lo que duró la grabación original

    @property
    def siguiente_escalon(self) -> int:
        """El porcentaje al que toca reproducirlo ahora.

        Una secuencia ya verificada al 100 % se reproduce al 100 % siempre:
        el escalonado es para estrenarla, no un peaje permanente.
        """

        return _SIGUIENTE.get(self.verificado, 15)

    @property
    def falta_verificar(self) -> bool:
        return self.verificado < 100

    def aprobar(self, pct: int) -> None:
        # Sólo sube, nunca baja: aprobar el 50 % después de haber llegado al
        # 100 % no puede degradar lo que ya estaba verificado.
        self.verificado = max(self.verificado, int(pct))

    def como_dict(self) -> dict:
        return {
            "nombre": self.nombre,
            "verificado": self.verificado,
            "creado": self.creado,
            "duracion_s": round(self.duracion_s, 2),
            "puntos": [p.como_dict() for p in self.puntos],
        }

    @staticmethod
    def desde_dict(d: dict) -> "Movimiento":
        verificado = int(d.get("verificado", 0))

        return Movimiento(
            nombre=str(d.get("nombre", "sin nombre")),
            puntos=[Punto.desde_dict(p) for p in d.get("puntos", [])],
            verificado=verificado if verificado in (0, 15, 50, 100) else 0,
            creado=str(d.get("creado", "")),
            duracion_s=float(d.get("duracion_s", 0.0)),
        )


# ==================================================================
#  Simplificación
# ==================================================================
def _distancia_a_recta(p: Muestra, a: Muestra, b: Muestra) -> float:
    """Distancia de p al segmento a-b, en 3D."""

    ax, ay, az = b.x - a.x, b.y - a.y, b.z - a.z
    largo2 = ax * ax + ay * ay + az * az

    if largo2 < 1e-12:
        return math.dist((p.x, p.y, p.z), (a.x, a.y, a.z))

    t = ((p.x - a.x) * ax + (p.y - a.y) * ay + (p.z - a.z) * az) / largo2
    t = max(0.0, min(1.0, t))

    return math.dist((p.x, p.y, p.z),
                     (a.x + t * ax, a.y + t * ay, a.z + t * az))


def _rdp(muestras: list[Muestra], desde: int, hasta: int, tol: float, guardar: set[int]) -> None:
    """Ramer-Douglas-Peucker sobre [desde, hasta], marcando los índices que quedan.

    Iterativo y no recursivo: una grabación larga puede tener miles de
    muestras y la recursión de Python se agota antes que la paciencia del
    operador.
    """

    pila = [(desde, hasta)]

    while pila:
        i, j = pila.pop()

        if j <= i + 1:
            continue

        peor, peor_i = 0.0, -1

        for k in range(i + 1, j):
            d = _distancia_a_recta(muestras[k], muestras[i], muestras[j])

            if d > peor:
                peor, peor_i = d, k

        if peor > tol and peor_i > 0:
            guardar.add(peor_i)
            pila.append((i, peor_i))
            pila.append((peor_i, j))


def _espera_ms(muestras: list[Muestra], i: int, tol: float) -> int:
    """Cuánto se quedó quieto el operador en la muestra `i`.

    Se mide como la racha de muestras siguientes que no se alejan más de la
    tolerancia. Es lo que convierte "apreté E y esperé a que agarrara" en una
    espera explícita del punto, en vez de en nada.
    """

    j = i

    while j + 1 < len(muestras):
        s = muestras[j + 1]

        if math.dist((s.x, s.y, s.z),
                     (muestras[i].x, muestras[i].y, muestras[i].z)) > tol:
            break

        j += 1

    return int(round((muestras[j].t - muestras[i].t) * 1000.0))


def simplificar(muestras: list[Muestra],
                tolerancia_cm: float = TOLERANCIA_CM,
                max_puntos: int = MAX_PUNTOS) -> list[Punto]:
    """Convierte la grabación cruda en la ruta que se sube al firmware.

    Se conservan siempre el primero, el último y **las dos muestras de cada
    cambio de bomba** (la de antes y la de después). Lo demás lo decide el
    RDP; si con la tolerancia pedida no entra en el buffer, se afloja la
    tolerancia y se rehace, hasta un tope razonable.
    """

    if not muestras:
        return []

    if len(muestras) == 1:
        m = muestras[0]
        return [Punto(m.x, m.y, m.z, m.bomba, 0)]

    tol = max(0.01, float(tolerancia_cm))

    for _ in range(12):
        obligados = {0, len(muestras) - 1}

        for k in range(1, len(muestras)):
            if muestras[k].bomba != muestras[k - 1].bomba:
                obligados.add(k - 1)
                obligados.add(k)

        guardar = set(obligados)

        # El RDP corre por tramos entre puntos obligados: así un cambio de
        # bomba nunca queda "cortado" por una recta que lo atraviesa.
        anclas = sorted(obligados)

        for a, b in zip(anclas, anclas[1:]):
            _rdp(muestras, a, b, tol, guardar)

        indices = sorted(guardar)

        if len(indices) <= max_puntos:
            break

        tol *= 1.6
    else:
        indices = sorted(guardar)[:max_puntos]

    puntos = []

    for k in indices:
        m = muestras[k]
        nuevo = Punto(m.x, m.y, m.z, m.bomba, _espera_ms(muestras, k, tol))

        # Se descartan los puntos demasiado pegados al anterior, salvo el
        # último y los que significan algo (bomba o espera): un tramo de
        # milímetros no se puede redondear y obliga a frenar.
        if (puntos and k != indices[-1] and nuevo.espera_ms == 0
                and nuevo.bomba == puntos[-1].bomba
                and math.dist((nuevo.x, nuevo.y, nuevo.z),
                              (puntos[-1].x, puntos[-1].y, puntos[-1].z)) < TRAMO_MINIMO_CM):
            continue

        puntos.append(nuevo)

    puntos = _repartir_parejo(puntos, max_puntos)

    # La espera del último punto se recorta: es el tiempo que pasó entre que
    # el operador dejó de mover y que apretó "parar", y no es parte del
    # movimiento enseñado.
    if puntos:
        puntos[-1].espera_ms = 0

    return puntos


def _repartir_parejo(puntos: list[Punto], max_puntos: int) -> list[Punto]:
    """Redistribuye los puntos a distancia pareja sobre el mismo camino.

    POR QUÉ HACE FALTA
    ------------------
    El simplificador deja los puntos donde está la curvatura: muchos en las
    curvas y pocos en las rectas. Eso parece razonable y en movimiento es
    horrible, porque **el pico de velocidad de un tramo va con la raíz de su
    largo** (perfil triangular: `v = sqrt(a·d)`). Con puntos apretados en las
    curvas, ahí el brazo se arrastra; en las rectas largas sale disparado. La
    secuencia se reproduce a dos velocidades distintas sin que nadie se lo
    haya pedido.

    Repartiéndolos parejo, todos los tramos miden lo mismo, todos alcanzan el
    mismo pico y el movimiento sale a velocidad constante — que es lo que uno
    esperaría de una trayectoria enseñada.

    LO QUE NO SE MUEVE
    ------------------
    Los puntos que significan algo: el primero, el último, y los que llevan
    espera o cambio de bomba. Ésos anclan el reparto y el camino entre dos
    anclas se rehace por separado, así un agarre sigue ocurriendo exactamente
    donde se enseñó.
    """

    if len(puntos) < 3:
        return puntos

    def anclado(i: int) -> bool:
        return (i == 0 or i == len(puntos) - 1
                or puntos[i].espera_ms > 0
                or puntos[i].bomba != puntos[i - 1].bomba)

    anclas = [i for i in range(len(puntos)) if anclado(i)]

    largo = sum(math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
                for a, b in zip(puntos, puntos[1:]))

    if largo <= 0.0:
        return puntos

    # El paso más chico que entre en el buffer. Si el camino es corto, manda
    # el tramo mínimo: no tiene sentido picarlo en pedazos de milímetros que
    # después no se pueden redondear.
    # Si el camino no entra a esa separación, se estira el paso hasta que
    # entre; nunca al revés.
    libres = max(1, max_puntos - len(anclas))
    paso = max(PASO_PAREJO_CM, largo / libres)

    salida = [puntos[0]]

    for a, b in zip(anclas, anclas[1:]):
        tramo = puntos[a:b + 1]

        # Longitud acumulada del tramo, para poder interpolar por distancia.
        acum = [0.0]

        for q, r in zip(tramo, tramo[1:]):
            acum.append(acum[-1] + math.dist((q.x, q.y, q.z), (r.x, r.y, r.z)))

        total = acum[-1]
        cortes = max(1, int(round(total / paso)))
        bomba = tramo[0].bomba

        for k in range(1, cortes + 1):
            objetivo = total * k / cortes

            if k == cortes:
                salida.append(puntos[b])   # el ancla se conserva tal cual
                break

            # Punto sobre la poligonal a esa distancia del arranque.
            j = 1

            while j < len(acum) - 1 and acum[j] < objetivo:
                j += 1

            franja = acum[j] - acum[j - 1]
            f = 0.0 if franja <= 0 else (objetivo - acum[j - 1]) / franja

            q, r = tramo[j - 1], tramo[j]

            salida.append(Punto(q.x + (r.x - q.x) * f,
                                q.y + (r.y - q.y) * f,
                                q.z + (r.z - q.z) * f,
                                bomba, 0))

    return salida


# ==================================================================
#  Biblioteca en disco
# ==================================================================
class Biblioteca:
    """Las secuencias guardadas, en `pc/config/`.

    Se guarda entero en cada cambio (son pocos kB) y se tolera un archivo
    roto devolviendo una biblioteca vacía: perder las secuencias es malo,
    pero no arrancar la interfaz por un JSON mal cerrado es peor.

    Son DOS archivos, y la diferencia no es técnica sino de qué merece
    viajar con el proyecto:

        movimientos.json           va al repositorio
        movimientos_locales.json   no (está en .gitignore)

    El reparto lo decide el nombre (`es_local`): lo que todavía se llama
    «Movimiento N» se quedó con el nombre de fábrica, o sea que nadie lo
    consideró digno de un nombre, o sea que es descarte de una tarde de
    pruebas. Renombrarlo es lo que lo salva, y no hay ningún botón de
    «guardar en el repo» que aprender.

    Por qué dos archivos y no un filtro al commitear: el programa reescribe
    esto en cada cambio. Un filtro hecho a mano vuelve atrás la primera vez
    que alguien graba algo, y a partir de ahí el repositorio queda sucio con
    diffs de miles de líneas que nadie lee. Adentro, en cambio, la regla se
    cumple sola.
    """

    def __init__(self, archivo: Path = ARCHIVO):
        self.archivo = archivo
        self.archivo_local = archivo.with_name(f"{archivo.stem}_locales{archivo.suffix}")
        self.movimientos: list[Movimiento] = []
        self.cargar()

    def _leer(self, archivo: Path) -> list:
        if not archivo.exists():
            return []

        try:
            datos = json.loads(archivo.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError) as err:
            print(f"[teach] no se pudo leer {archivo.name}: {err}")
            return []

        return [Movimiento.desde_dict(d) for d in datos.get("movimientos", [])]

    def cargar(self) -> None:
        # Los dos archivos se juntan en una sola lista y se ordenan por fecha
        # de grabación: para el operador es una biblioteca sola, y de dónde
        # salió cada secuencia no es asunto suyo.
        self.movimientos = self._leer(self.archivo) + self._leer(self.archivo_local)
        self.movimientos.sort(key=lambda m: (m.creado, m.nombre))

    def _escribir(self, archivo: Path, movimientos: list) -> None:
        try:
            archivo.parent.mkdir(parents=True, exist_ok=True)
            archivo.write_text(
                json.dumps({"movimientos": [m.como_dict() for m in movimientos]},
                           indent=2, ensure_ascii=False),
                encoding="utf-8")
        except OSError as err:
            print(f"[teach] no se pudo guardar {archivo.name}: {err}")

    def guardar(self) -> None:
        # El reparto se resuelve en cada guardado y no al crear cada
        # movimiento: así renombrar uno lo muda de archivo sin que renombrar
        # tenga que saber que los archivos son dos.
        self._escribir(self.archivo,
                       [m for m in self.movimientos if not es_local(m.nombre)])
        self._escribir(self.archivo_local,
                       [m for m in self.movimientos if es_local(m.nombre)])

    # ------------------------------------------------------------------
    def nombre_libre(self, base: str = "Movimiento") -> str:
        usados = {m.nombre for m in self.movimientos}

        for i in range(1, 1000):
            candidato = f"{base} {i}"

            if candidato not in usados:
                return candidato

        return base

    def agregar(self, movimiento: Movimiento) -> Movimiento:
        self.movimientos.append(movimiento)
        self.guardar()
        return movimiento

    def borrar(self, indice: int) -> None:
        if 0 <= indice < len(self.movimientos):
            del self.movimientos[indice]
            self.guardar()

    def renombrar(self, indice: int, nombre: str) -> None:
        nombre = nombre.strip()

        if 0 <= indice < len(self.movimientos) and nombre:
            self.movimientos[indice].nombre = nombre
            self.guardar()

    def obtener(self, indice: int) -> Optional[Movimiento]:
        if 0 <= indice < len(self.movimientos):
            return self.movimientos[indice]

        return None


def ahora_texto() -> str:
    return time.strftime("%Y-%m-%d %H:%M")
