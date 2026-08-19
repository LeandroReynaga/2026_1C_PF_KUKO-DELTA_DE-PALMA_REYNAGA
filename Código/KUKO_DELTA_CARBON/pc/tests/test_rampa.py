"""La rampa del stepper, simulada paso a paso, sin robot y sin placa.

POR QUÉ EXISTE ESTA PRUEBA
--------------------------
`Stepper::redirigir()` cambia el destino de un eje **sin soltar la rampa**,
que es lo que permite encadenar los tramos de una trayectoria enseñada sin
frenar en cada punto. Si se equivoca en cuánto cuesta frenar, el eje llega al
destino todavía andando, los pulsos se cortan de golpe y se pierden pasos —
sin que el robot choque contra nada ni se entere.

Eso no se puede verificar mirando el código y no se puede probar a mano sobre
el robot sin arriesgar el brazo. Acá se reproduce la aritmética **entera** de
la ISR, paso por paso, y se comprueban las tres propiedades que importan:

  1. el eje nunca llega a un destino a velocidad alta (pasos perdidos);
  2. la velocidad nunca pega un salto hacia arriba mayor que el de la propia
     rampa (un paso a paso no sigue un escalón de velocidad);
  3. encadenando, el eje efectivamente NO se detiene en los puntos
     intermedios — que es todo el punto del cambio.

La primera versión de `redirigir()` calculaba el frenado con `v²/(2a)`, la
fórmula del movimiento continuo. Esta prueba la rechazó: en el peor caso el
eje tocaba el destino a 7.600 pasos/s. La rampa de Austin en punto fijo se
aparta de esa fórmula hasta un 190 % en rampas largas, porque la división
entera de la recurrencia trunca cada vez más a medida que `n` crece.

OJO: ESTO ES UN ESPEJO, NO EL CÓDIGO
------------------------------------
`_Eje` reproduce `computeNextInterval()`, `moveTo()` y `redirigir()` de
`src/robot/Stepper.cpp`. **Si se toca la rampa allá, hay que tocarla acá.**
No hay forma de que el espejo se entere solo; lo único que sí se lee del
firmware son las constantes, así que al menos ésas no pueden desincronizarse
en silencio.

Se corre con:  python -m pytest pc/tests   (o  python pc/tests/test_rampa.py)
"""

from __future__ import annotations

import math
import random
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

RAIZ = Path(__file__).resolve().parents[2]
FUENTE_H = (RAIZ / "src" / "robot" / "Stepper.h").read_text(encoding="utf-8")


def _constante(patron: str, defecto):
    m = re.search(patron, FUENTE_H)
    return type(defecto)(m.group(1)) if m else defecto


# Leídas del firmware para que no se desincronicen sin avisar.
RAMP_SCALE = _constante(r"RAMP_SCALE\s*=\s*(\d+)", 256)
MARGEN_FRENADO = _constante(r"MARGEN_FRENADO\s*=\s*([\d.]+)f", 1.10)
FRENADO_EXTRA = _constante(r"FRENADO_EXTRA\s*=\s*(\d+)", 4)

IDLE, POSITION = 0, 2


def _cdiv(a: int, b: int) -> int:
    """División entera de C: trunca hacia cero. Python redondea hacia abajo."""

    return int(a / b)


class _Eje:
    """Espejo de Stepper: sólo la rampa, sin GPIO ni timers."""

    def __init__(self, vel: float, acel: float):
        self.pos = 0
        self.target = 0
        self.dirn = True
        self.modo = IDLE
        self.cn = 0
        self.n = 0
        self.stepIndex = 0
        self.accelSteps = 0
        self.decelStart = 0
        self.maxSpeed = vel
        self.acel = acel
        self.cmin = int((1000000.0 / vel) * RAMP_SCALE)
        self.c0 = int(0.676 * math.sqrt(2.0 / acel) * 1000000.0 * RAMP_SCALE)

    # -- computeNextInterval() --
    def _siguiente(self) -> None:
        if self.n > 0 and self.stepIndex >= self.decelStart:
            self.n = -self.n

        if self.n == 0:
            self.cn = self.c0
            self.n = 1
            return

        if self.n > 0 and self.cn <= self.cmin:
            self.cn = self.cmin

            if self.modo == POSITION and self.accelSteps != self.n:
                self.decelStart = (self.decelStart + self.accelSteps) - self.n
                self.accelSteps = self.n
            return

        self.cn = self.cn - _cdiv(2 * self.cn, 4 * self.n + 1)

        if self.cn < self.cmin:
            self.cn = self.cmin

        self.n += 1

    # -- moveTo() --
    def moveTo(self, destino: int) -> None:
        self.target = destino

        if destino == self.pos:
            self.modo = IDLE
            return

        self.dirn = destino > self.pos

        D = abs(destino - self.pos)
        steps = int((self.maxSpeed * self.maxSpeed) / (2.0 * self.acel))

        if 2 * steps > D:
            steps = D // 2

        self.accelSteps = steps
        self.decelStart = D - steps
        self.n = 0
        self.cn = 0
        self.stepIndex = 0
        self.modo = POSITION
        self._siguiente()

    # -- pasosDeFrenado() --
    def pasos_de_frenado(self, acel_nueva: float) -> int:
        n_rampa = max(1, abs(self.n))

        if acel_nueva <= 0 or self.acel <= 0:
            return n_rampa

        return int(n_rampa * (self.acel / acel_nueva) * MARGEN_FRENADO) + FRENADO_EXTRA

    # -- redirigir() --
    def redirigir(self, destino: int, vel_max: float, acel: float) -> bool:
        if vel_max <= 0 or acel <= 0 or self.modo != POSITION or self.cn <= 0:
            return False

        delta = destino - self.pos

        if delta == 0 or (delta > 0) != self.dirn:
            return False

        n0 = self.pasos_de_frenado(acel)
        dist = abs(delta)

        if dist < n0:
            return False

        self.maxSpeed = vel_max
        self.acel = acel
        self.cmin = int((1000000.0 / vel_max) * RAMP_SCALE)
        self.c0 = int(0.676 * math.sqrt(2.0 / acel) * 1000000.0 * RAMP_SCALE)

        self.decelStart = max(0, (dist - n0) // 2)
        self.accelSteps = dist - self.decelStart
        self.target = destino
        self.n = n0
        self.stepIndex = 0

        return True

    # -- helpers --
    def restantes(self) -> int:
        return abs(self.target - self.pos) if self.modo == POSITION else 0

    def velocidad(self) -> float:
        return (1000000.0 * RAMP_SCALE) / self.cn if self.cn > 0 else 0.0

    # -- onTimerTick(), un paso --
    def paso(self) -> bool:
        if self.modo != POSITION or self.pos == self.target:
            self.modo = IDLE
            return False

        self.pos += 1 if self.dirn else -1
        self.stepIndex += 1

        if self.pos == self.target:
            self.modo = IDLE
            self.n = 0
            self.cn = 0
            self.stepIndex = 0
        else:
            self._siguiente()

        return True


def _recorrer(destinos, vel, acel, encadenar, fraccion_mezcla=0.15):
    """Recorre una lista de destinos absolutos y devuelve lo que hizo el eje.

    `fraccion_mezcla` es a qué altura del tramo se intenta redirigir, como
    fracción de lo que mide. Es el equivalente del radio de mezcla en cm que
    usa `Robot::updateTeachPlayback()`.
    """

    eje = _Eje(vel, acel)

    velocidades = []
    llegadas = []      # velocidad con la que se tocó cada destino
    esquinas = []      # velocidad al pasar por cada punto INTERMEDIO
    detenciones = 0
    tiempo_us = 0.0

    def avanzar(hasta_restantes: int) -> float:
        ultima = 0.0

        nonlocal tiempo_us

        while eje.modo == POSITION and eje.restantes() > hasta_restantes:
            ultima = eje.velocidad()
            velocidades.append(ultima)
            tiempo_us += eje.cn / RAMP_SCALE

            if not eje.paso():
                break

        return ultima

    for i, destino in enumerate(destinos):
        largo = abs(destino - eje.pos)

        if largo == 0:
            continue

        if eje.modo == POSITION:
            antes = eje.velocidad()

            if encadenar and eje.redirigir(destino, vel, acel):
                esquinas.append(antes)   # pasó por el punto sin frenar
            else:
                llegada = avanzar(0)
                esquinas.append(llegada) # tuvo que frenar hasta detenerse
                llegadas.append(llegada)
                detenciones += 1
                eje.moveTo(destino)
        else:
            eje.moveTo(destino)

        ultimo = (i == len(destinos) - 1)
        umbral = 0 if ultimo else max(1, int(largo * fraccion_mezcla))

        v = avanzar(umbral)

        if ultimo:
            llegadas.append(v)

    return {
        "velocidades": velocidades,
        "llegadas": llegadas,
        "esquinas": esquinas,
        "detenciones": detenciones,
        "tiempo_us": tiempo_us,
    }


def _trayectoria(rnd, tramos=None):
    pos = 0
    destinos = []

    for _ in range(tramos or rnd.randint(2, 25)):
        pos += rnd.randint(40, 2500)
        destinos.append(pos)

    return destinos


# ==================================================================
#  El espejo se parece al original
# ==================================================================
def test_las_constantes_salen_del_firmware():
    """Si esto falla, el resto de la prueba está midiendo otra cosa."""

    assert "RAMP_SCALE" in FUENTE_H and "MARGEN_FRENADO" in FUENTE_H
    assert RAMP_SCALE == 256
    assert 1.0 <= MARGEN_FRENADO <= 2.0
    assert 0 <= FRENADO_EXTRA <= 100


def test_un_movimiento_normal_llega_frenado():
    """La referencia contra la que se compara todo lo demás.

    Un `moveTo` desde parado llega al destino a un ~3 % de la velocidad de
    crucero. Eso es lo que hace hoy el robot y lo que el encadenado no puede
    empeorar.
    """

    for acel in (5000.0, 40000.0, 97000.0):
        r = _recorrer([3000], 12000.0, acel, encadenar=False)

        assert r["llegadas"][-1] < 600, f"acel={acel}: llega a {r['llegadas'][-1]:.0f} pasos/s"


# ==================================================================
#  Lo que tiene que lograr el encadenado
# ==================================================================
def test_encadenar_saca_las_frenadas_intermedias():
    destinos = [400, 900, 1200, 1750, 2100, 2700, 3000]

    suelto = _recorrer(destinos, 12000.0, 40000.0, encadenar=False)
    unido = _recorrer(destinos, 12000.0, 40000.0, encadenar=True)

    assert suelto["detenciones"] >= 6, "la prueba no está midiendo lo que cree"
    assert unido["detenciones"] == 0, "el encadenado igual frenó en los puntos"

    # Y no es sólo que no frene: por cada punto intermedio pasa a una
    # velocidad de trabajo, no arrastrándose. Sin encadenar los toca todos a
    # ~200 pasos/s, que es el final de la rampa de frenado.
    assert max(suelto["esquinas"]) < 400
    assert min(unido["esquinas"]) > 1500, unido["esquinas"]


def test_el_encadenado_no_es_mas_lento():
    """Redondear la esquina no puede costar tiempo: ése sería el peor canje."""

    destinos = [400, 900, 1200, 1750, 2100, 2700, 3000]

    suelto = _recorrer(destinos, 12000.0, 40000.0, encadenar=False)
    unido = _recorrer(destinos, 12000.0, 40000.0, encadenar=True)

    # El mismo recorrido en menos tiempo: no frenar seis veces se nota.
    assert unido["tiempo_us"] < suelto["tiempo_us"] * 0.9, (
        f"{unido['tiempo_us'] / 1000:.0f} ms contra "
        f"{suelto['tiempo_us'] / 1000:.0f} ms")


# ==================================================================
#  Lo que NO puede pasar nunca
# ==================================================================
def test_nunca_se_llega_a_un_destino_a_velocidad_alta():
    """La prueba que rechazó la primera versión de `redirigir()`.

    Llegar rápido significa que los pulsos se cortan de golpe: pasos
    perdidos, sin choque y sin aviso. Se barren trayectorias al azar con
    todos los extremos de velocidad, aceleración y radio de mezcla.
    """

    rnd = random.Random(20260819)
    peor = 0.0
    peor_caso = None

    for _ in range(2000):
        destinos = _trayectoria(rnd)
        vel = rnd.choice([6000.0, 12000.0, 20000.0])
        acel = rnd.choice([5000.0, 40000.0, 97000.0])
        frac = rnd.choice([0.05, 0.15, 0.3, 0.5])

        r = _recorrer(destinos, vel, acel, encadenar=True, fraccion_mezcla=frac)
        llegada = max(r["llegadas"])

        if llegada > peor:
            peor, peor_caso = llegada, (vel, acel, frac, len(destinos))

    assert peor < 1200, (
        f"un eje toca el destino a {peor:.0f} pasos/s (caso {peor_caso}): "
        "los pulsos se cortan de golpe y se pierden pasos")


def test_la_velocidad_nunca_pega_un_salto_hacia_arriba():
    """Un paso a paso no sigue un escalón de velocidad: se desincroniza.

    Bajar de golpe sí se puede (el motor siempre puede ir más lento), así que
    sólo se vigila el sentido peligroso.
    """

    rnd = random.Random(4242)
    peor = 0.0

    for _ in range(400):
        destinos = _trayectoria(rnd)
        vel = rnd.choice([6000.0, 12000.0, 20000.0])
        acel = rnd.choice([5000.0, 40000.0, 97000.0])

        r = _recorrer(destinos, vel, acel, encadenar=True, fraccion_mezcla=0.3)
        v = r["velocidades"]

        peor = max(peor, max(v[i + 1] - v[i] for i in range(len(v) - 1)))

    # El primer paso de una rampa es el escalón más grande que da la propia
    # rampa; el encadenado no puede introducir ninguno mayor.
    assert peor < 700, f"salto de +{peor:.0f} pasos/s entre dos pasos seguidos"


def test_se_rechaza_lo_que_no_se_puede_encadenar():
    """Redirigir tiene que decir que no cuando corresponde, sin tocar nada."""

    eje = _Eje(12000.0, 40000.0)
    eje.moveTo(3000)

    for _ in range(1500):
        eje.paso()

    estado = (eje.target, eje.n, eje.cn, eje.decelStart, eje.accelSteps)

    # Hacia atrás: habría que invertir el sentido con el rotor girando.
    assert not eje.redirigir(eje.pos - 500, 12000.0, 40000.0)

    # Tan cerca que no llega a frenar.
    assert not eje.redirigir(eje.pos + 2, 12000.0, 40000.0)

    # Y el eje quedó exactamente como estaba.
    assert (eje.target, eje.n, eje.cn, eje.decelStart, eje.accelSteps) == estado

    # Con distancia suficiente, sí.
    assert eje.redirigir(eje.pos + 3000, 12000.0, 40000.0)


def test_un_eje_parado_no_se_encadena():
    eje = _Eje(12000.0, 40000.0)

    assert not eje.redirigir(1000, 12000.0, 40000.0)

    eje.moveTo(100)

    while eje.paso():
        pass

    assert not eje.redirigir(1000, 12000.0, 40000.0)


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
