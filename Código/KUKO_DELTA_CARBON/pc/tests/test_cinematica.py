"""La cinemática de Python, verificada contra sí misma y contra el firmware.

Dos cosas distintas se comprueban acá:

  * **Ida y vuelta.** `directa(inversa(p)) == p` para todo el volumen útil.
    Es lo que garantiza que el punto que dibuja la pantalla es el mismo que
    el robot iba a alcanzar, y no una versión parecida.

  * **Las constantes no se desincronizaron.** La geometría está escrita dos
    veces —`DeltaKinematics.h` y `cinematica.py`— y tocar una sola es el
    tipo de error que no da ningún síntoma: la pantalla dibuja el brazo unos
    milímetros corrido y nadie se entera. Acá se leen las dos y se comparan.

Se corre con:  python -m pytest pc/tests   (o  python pc/tests/test_cinematica.py)
"""

from __future__ import annotations

import random
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kuko import cinematica as cin

RAIZ = Path(__file__).resolve().parents[2]

# Nombre en C++ -> nombre acá. Sólo las que se usan para resolver la
# geometría; los pasos por vuelta y los flags de inversión no entran en esto
# porque Python no comanda motores.
EQUIVALENCIAS = {
    "BICEP_LENGTH": "BICEP",
    "FOREARM_LENGTH": "ANTEBRAZO",
    "BASE_RADIUS": "RADIO_BASE",
    "EFFECTOR_RADIUS": "RADIO_EFECTOR",
    "EFFECTOR_OFFSET_Z": "OFFSET_HERRAMIENTA_Z",
    "THETA_MIN": "LIMITE_THETA_MIN",
    "THETA_MAX": "LIMITE_THETA_MAX",
}


def _constantes_del_firmware() -> dict[str, float]:
    fuente = (RAIZ / "src" / "kinematics" / "DeltaKinematics.h").read_text(encoding="utf-8")
    patron = re.compile(r"constexpr\s+float\s+(\w+)\s*=\s*(-?[\d.]+)f")

    return {n: float(v) for n, v in patron.findall(fuente)}


def test_las_constantes_coinciden_con_el_firmware():
    firmware = _constantes_del_firmware()

    assert firmware, "no se pudo leer DeltaKinematics.h"

    for en_cpp, en_python in EQUIVALENCIAS.items():
        assert en_cpp in firmware, f"{en_cpp} ya no está en DeltaKinematics.h"

        esperado = firmware[en_cpp]
        actual = getattr(cin, en_python)

        assert abs(esperado - actual) < 1e-6, \
            f"{en_cpp}={esperado} en el firmware pero {en_python}={actual} en Python"


def test_las_ganancias_coinciden_con_el_firmware():
    """Las dos correcciones medidas sobre el robot real.

    No son `constexpr` sino dos multiplicaciones sueltas dentro de
    `solveIK()`, así que se buscan tal cual están escritas.
    """

    fuente = (RAIZ / "src" / "kinematics" / "DeltaKinematics.cpp").read_text(encoding="utf-8")

    assert f"x = x * {cin.GANANCIA_X:.2f}f" in fuente, "cambió la ganancia en X"
    assert f"y = y * {cin.GANANCIA_Y:.2f}f" in fuente, "cambió la ganancia en Y"


def test_ida_y_vuelta():
    """La directa deshace exactamente lo que hizo la inversa."""

    random.seed(20260819)

    peor = 0.0
    resueltos = 0

    for _ in range(3000):
        x = random.uniform(-14.0, 14.0)
        y = random.uniform(-12.0, 14.0)
        z = random.uniform(-34.0, -24.0)

        angulos = cin.inversa(x, y, z)

        if angulos is None:
            continue

        punto = cin.directa(*angulos)

        assert punto is not None, f"la directa no resolvió {angulos}"

        peor = max(peor, max(abs(punto[0] - x), abs(punto[1] - y), abs(punto[2] - z)))
        resueltos += 1

    assert resueltos > 1000, f"sólo se resolvieron {resueltos} puntos"
    assert peor < 1e-6, f"la ida y vuelta se desvía {peor:.2e} cm"


def test_el_home_da_los_brazos_horizontales():
    """Con los tres ángulos en cero el brazo cuelga sobre el eje.

    Es el único punto que se puede verificar sin el robot delante: en home
    los tres brazos están horizontales, así que la punta tiene que quedar en
    X = 0, Y = 0 y a la profundidad que impone la geometría.
    """

    punto = cin.directa(0.0, 0.0, 0.0)

    assert punto is not None
    assert abs(punto[0]) < 1e-6
    assert abs(punto[1]) < 1e-6
    assert -31.0 < punto[2] < -29.0


def test_lo_inalcanzable_se_informa_como_inalcanzable():
    """Nunca se devuelve "el punto más parecido que sí se puede"."""

    assert not cin.alcanzable(0.0, 0.0, -60.0)     # más abajo que el alcance
    assert not cin.alcanzable(60.0, 0.0, -30.0)    # fuera de rango en X
    assert cin.alcanzable(0.0, 0.0, -30.0)


def test_un_angulo_que_falta_no_inventa_una_posicion():
    assert cin.directa_desde([0.0, 0.0, None]) is None
    assert cin.directa_desde(None) is None
    assert cin.directa_desde([0.0, 0.0, 0.0]) is not None


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
