"""Cinemática del delta, del lado de Python. Espejo de `DeltaKinematics`.

Existe por dos motivos, y ninguno es "recalcular lo que ya hace el robot":

  * **Directa (FK).** La telemetría manda los tres ángulos MEDIDOS por los
    encoders; para dibujar dónde está la punta hay que resolver la directa,
    y el firmware no la tiene ni le hace falta (`PROTOCOLO.md` §2.2: la FK
    se calcula acá, no se gastan ciclos del ESP32 en algo que no necesita
    para moverse).

  * **Inversa (IK).** El volumen de trabajo del modo teach es un cajón, y un
    cajón tiene esquinas a las que el brazo no llega. Con la inversa acá, la
    pantalla puede pintar la zona alcanzable y no ofrecerle al operador un
    punto que el firmware va a rechazar. El firmware la sigue resolviendo
    igual: ésta es para no ofrecer lo imposible, la de allá es la que manda.

**Las constantes son las mismas de `src/kinematics/DeltaKinematics.h` y
tienen que valer lo mismo.** Si se vuelve a medir el robot se tocan los dos
lados; `test_cinematica.py` verifica la ida y vuelta, no la coincidencia con
el firmware, así que un cambio de un solo lado no lo delata nadie.
"""

from __future__ import annotations

import math
from typing import Optional

# ==================================================================
#  Geometría (cm) — espejo de DeltaKinematics.h
# ==================================================================
BICEP = 18.0            # L1: brazo superior, motor -> codo
ANTEBRAZO = 35.2        # L2: varilla libre codo -> efector
RADIO_BASE = 8.275      # centro de la base al eje del motor
RADIO_EFECTOR = 3.83    # centro del plato al eje de articulación

# La punta del gripper cuelga 2,8 cm por debajo del plato.
OFFSET_HERRAMIENTA_Z = -2.8

LIMITE_THETA_MIN = -50.0
LIMITE_THETA_MAX = 50.0

# Ganancias medidas sobre el robot real. Se aplican DESPUÉS de descontar el
# offset de herramienta, igual que en el firmware.
GANANCIA_X = 1.02
GANANCIA_Y = 0.98

_PIVOTE_Y = -RADIO_BASE
_SQRT3_2 = 0.8660254037844386
_EPSILON = 1e-6


# ==================================================================
#  Inversa
# ==================================================================
def _pierna(x0: float, y0: float, z0: float) -> Optional[float]:
    """Ángulo del motor de una pierna, con el punto ya en su sistema rotado.

    Intersección de dos esferas: uno de radio L1 centrado en el motor y otro
    de radio L2 centrado en el punto del efector. Devuelve None cuando no se
    cortan (el punto está fuera de alcance de esa pierna).
    """

    if abs(z0) < _EPSILON:
        return None

    y0p = y0 - RADIO_EFECTOR  # desplaza el OBJETIVO por el radio del efector

    a = (x0 * x0 + y0p * y0p + z0 * z0
         + BICEP * BICEP - ANTEBRAZO * ANTEBRAZO - _PIVOTE_Y * _PIVOTE_Y) / (2.0 * z0)
    b = (_PIVOTE_Y - y0p) / z0

    discriminante = -(a + b * _PIVOTE_Y) ** 2 + BICEP * BICEP * (b * b + 1.0)

    if discriminante < 0.0:
        return None

    # Raíz de la configuración física del brazo (codo hacia afuera).
    yj = (_PIVOTE_Y - a * b - math.sqrt(discriminante)) / (b * b + 1.0)
    zj = a + b * yj

    return math.degrees(math.atan2(-zj, _PIVOTE_Y - yj))


def inversa(x: float, y: float, z: float) -> Optional[tuple[float, float, float]]:
    """Ángulos de los tres motores para una punta en (x, y, z), en grados.

    None si el punto no tiene solución o si algún ángulo se sale de los
    límites articulares. No se devuelve "la mejor aproximación" a propósito:
    un punto inalcanzable tiene que verse como inalcanzable, no como uno
    parecido que sí lo es.
    """

    # punta -> plato, y después las ganancias de calibración.
    z = z - OFFSET_HERRAMIENTA_Z
    x = x * GANANCIA_X
    y = y * GANANCIA_Y

    # M1 a 150°, M2 a 30°, M3 a 270° respecto de X+.
    piernas = (
        _pierna(-0.5 * x - _SQRT3_2 * y, _SQRT3_2 * x - 0.5 * y, z),
        _pierna(-0.5 * x + _SQRT3_2 * y, -_SQRT3_2 * x - 0.5 * y, z),
        _pierna(x, y, z),
    )

    if any(t is None for t in piernas):
        return None

    if any(t < LIMITE_THETA_MIN or t > LIMITE_THETA_MAX for t in piernas):
        return None

    return piernas  # type: ignore[return-value]


def alcanzable(x: float, y: float, z: float) -> bool:
    return inversa(x, y, z) is not None


# ==================================================================
#  Directa
# ==================================================================
def _centro(theta_deg: float, giro: float) -> tuple[float, float, float]:
    """Centro de la esfera de radio L2 que impone una pierna, en global.

    El codo queda a L1 del pivote, en el plano de su pierna. La esfera que
    define el antebrazo está centrada ahí, corrida por el radio del efector
    -- se corre el CENTRO en vez del punto, que es la misma cuenta que hace
    la inversa al revés.
    """

    t = math.radians(theta_deg)

    yj = _PIVOTE_Y - BICEP * math.cos(t)
    zj = -BICEP * math.sin(t)

    q = yj + RADIO_EFECTOR

    # El punto (0, q) del sistema de la pierna, devuelto a coordenadas
    # globales rotando -giro.
    return (-math.sin(giro) * q, math.cos(giro) * q, zj)


def directa(theta1: float, theta2: float, theta3: float) -> Optional[tuple[float, float, float]]:
    """Posición de la PUNTA del gripper (cm) para los tres ángulos, en grados.

    Trilateración: tres esferas de radio L2 centradas en los codos. Se restan
    de a pares para bajar a dos ecuaciones lineales, se despejan x e y en
    función de z y se cierra con la cuadrática que queda. De las dos raíces
    se toma la de z menor, que es el brazo colgando -- la otra es el codo
    invertido, una postura que este robot no puede adoptar.

    None si los ángulos no corresponden a ninguna postura real (típicamente,
    tres lecturas de encoder tomadas en instantes distintos durante un
    movimiento rápido).
    """

    c1 = _centro(theta1, math.radians(-120.0))
    c2 = _centro(theta2, math.radians(120.0))
    c3 = _centro(theta3, 0.0)

    def norma2(c):
        return c[0] * c[0] + c[1] * c[1] + c[2] * c[2]

    a1, b1, k1 = 2 * (c1[0] - c3[0]), 2 * (c1[1] - c3[1]), 2 * (c1[2] - c3[2])
    a2, b2, k2 = 2 * (c2[0] - c3[0]), 2 * (c2[1] - c3[1]), 2 * (c2[2] - c3[2])

    d1 = norma2(c1) - norma2(c3)
    d2 = norma2(c2) - norma2(c3)

    det = a1 * b2 - a2 * b1

    if abs(det) < _EPSILON:
        return None  # postura singular: los tres codos alineados en planta

    # x = px + qx*z ,  y = py + qy*z
    px = (d1 * b2 - d2 * b1) / det
    qx = (k2 * b1 - k1 * b2) / det
    py = (a1 * d2 - a2 * d1) / det
    qy = (a2 * k1 - a1 * k2) / det

    ex, ey = px - c3[0], py - c3[1]

    aa = qx * qx + qy * qy + 1.0
    bb = 2.0 * (qx * ex + qy * ey - c3[2])
    cc = ex * ex + ey * ey + c3[2] * c3[2] - ANTEBRAZO * ANTEBRAZO

    discriminante = bb * bb - 4.0 * aa * cc

    if discriminante < 0.0:
        return None

    z = (-bb - math.sqrt(discriminante)) / (2.0 * aa)  # la raíz de abajo

    x = px + qx * z
    y = py + qy * z

    # plato -> punta, deshaciendo las ganancias.
    return (x / GANANCIA_X, y / GANANCIA_Y, z + OFFSET_HERRAMIENTA_Z)


def directa_desde(angulos) -> Optional[tuple[float, float, float]]:
    """Igual, pero tolerando la lista de tres Optional que trae `[T]`.

    Es la forma en que llega el dato: `Telemetria.angulo` es una lista de
    tres valores que pueden ser None si el campo no vino. Devolver None ante
    un ángulo faltante -- en vez de tomarlo como cero -- es lo que evita
    dibujar la punta en un lugar inventado.
    """

    if angulos is None or len(angulos) < 3 or any(a is None for a in angulos):
        return None

    return directa(float(angulos[0]), float(angulos[1]), float(angulos[2]))
