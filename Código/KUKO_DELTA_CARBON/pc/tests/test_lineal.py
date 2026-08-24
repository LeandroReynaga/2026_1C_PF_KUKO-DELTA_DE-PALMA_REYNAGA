"""Espejo en Python del movimiento lineal (src/motion/Trajectory.*).

No prueba el firmware: prueba los NÚMEROS con los que está construido. La
cinemática de `kuko/cinematica.py` es la misma que la de
`DeltaKinematics.h` (eso lo verifica `test_cinematica.py`), así que acá se
puede medir sin robot cuánto se aparta de la recta el movimiento articular
y cuánto lo corrige partirla en tramos.

Es un espejo, igual que `test_rampa.py` lo es de la aritmética de la ISR: si
se toca la regla que elige el largo del tramo, hay que tocarla también acá.
Y si un día se vuelve a medir el robot y la geometría cambia, estas pruebas
fallan — que es exactamente lo que se quiere, porque los números que están
escritos en los comentarios de Trajectory.h habrían dejado de ser ciertos.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kuko import cinematica as cin

PASOS_POR_GRADO = 10000 / 360.0


# ==================================================================
#  Herramientas de medición
# ==================================================================
def desvio_mm(p0, p1, muestras: int = 120) -> float:
    """Cuánto se aparta de la recta el camino que hace movJ, en mm.

    movJ interpola LOS ÁNGULOS, así que el camino se obtiene interpolando
    los tres ángulos y resolviendo la directa en cada paso.
    """

    a0, a1 = cin.inversa(*p0), cin.inversa(*p1)

    assert a0 is not None and a1 is not None, "el caso de prueba no es alcanzable"

    largo = math.dist(p0, p1)
    peor = 0.0

    for i in range(muestras + 1):
        s = i / muestras
        angulos = tuple(a0[k] + s * (a1[k] - a0[k]) for k in range(3))
        p = cin.directa(*angulos)

        if p is None:
            continue

        # Distancia del punto a la recta, por proyección.
        d = [p1[k] - p0[k] for k in range(3)]
        w = [p[k] - p0[k] for k in range(3)]
        t = sum(w[k] * d[k] for k in range(3)) / (largo * largo)
        proy = [p0[k] + t * d[k] for k in range(3)]

        peor = max(peor, math.dist(p, proy))

    return peor * 10.0


def desvio_partido_mm(p0, p1, paso_cm: float) -> float:
    """Lo mismo, pero partiendo la recta en tramos de `paso_cm`."""

    n = max(1, math.ceil(math.dist(p0, p1) / paso_cm))
    peor = 0.0

    for k in range(n):
        a = tuple(p0[j] + (k / n) * (p1[j] - p0[j]) for j in range(3))
        b = tuple(p0[j] + ((k + 1) / n) * (p1[j] - p0[j]) for j in range(3))
        peor = max(peor, desvio_mm(a, b, 30))

    return peor


def pasos_por_cm(p0, p1) -> float:
    """Pasos del eje que más recorre por cm de recta."""

    a0, a1 = cin.inversa(*p0), cin.inversa(*p1)
    d = max(abs(a1[k] - a0[k]) for k in range(3)) * PASOS_POR_GRADO

    return d / math.dist(p0, p1)


# Los mismos casos que están escritos en el encabezado de Trajectory.h.
CASOS = {
    "cruzar la cinta en X": ((-12, 3, -29), (12, 3, -29), 27.3),
    "de un tacho al otro": ((-8, -9.5, -30), (8, -9.5, -30), 10.2),
    "bajada vertical en el centro": ((0, 0, -26.6), (0, 0, -32.6), 0.0),
    "bajada vertical en una esquina": ((10, 10, -26.6), (10, 10, -32.6), 1.2),
    "diagonal larga": ((-10, 10, -27), (10, -8, -32), 33.4),
}


# ==================================================================
#  Lo que justifica que movL exista
# ==================================================================
def test_movj_se_aparta_de_la_recta():
    """El movimiento articular no va derecho, y no por poco.

    Es el número que decide si movL vale la pena: si esto diera décimas de
    milímetro, partir la recta sería complejidad sin beneficio.
    """

    for nombre, (p0, p1, esperado) in CASOS.items():
        medido = desvio_mm(p0, p1)

        assert abs(medido - esperado) <= max(0.15 * esperado, 0.1), (
            f"{nombre}: Trajectory.h dice {esperado} mm y ahora mide "
            f"{medido:.2f} mm")


def test_una_bajada_vertical_en_el_centro_ya_es_recta():
    """Por simetría, y es la razón por la que hoy el robot funciona.

    Las bajadas del ciclo normal son verticales y bastante centradas, así
    que movJ alcanza. Lo que se desvía es todo lo que cruza en X o en Y.
    """

    p0, p1, _ = CASOS["bajada vertical en el centro"]

    assert desvio_mm(p0, p1) < 0.01

    # Pero la misma bajada en una esquina ya se corre un milímetro, que es
    # del orden del juego que tiene una celda de la caja del modo Box.
    p0, p1, _ = CASOS["bajada vertical en una esquina"]

    assert desvio_mm(p0, p1) > 0.5


# ==================================================================
#  Que partirla funciona, y cuánto hay que partirla
# ==================================================================
def test_el_error_cae_con_el_cuadrado_del_paso():
    p0, p1, _ = CASOS["diagonal larga"]

    medidos = {paso: desvio_partido_mm(p0, p1, paso) for paso in (1.0, 2.0, 4.0)}

    # Los valores que están escritos en Trajectory.h.
    assert medidos[1.0] < 0.10, medidos
    assert medidos[2.0] < 0.30, medidos

    # Duplicar el paso multiplica el error por ~4. Se verifica flojo (entre
    # 3 y 5) porque la relación es asintótica, no exacta.
    for chico, grande in ((1.0, 2.0), (2.0, 4.0)):
        razon = medidos[grande] / medidos[chico]

        assert 3.0 <= razon <= 5.0, f"{chico}->{grande} cm: x{razon:.1f}"


def test_un_paso_de_un_centimetro_es_invisible_en_todos_los_casos():
    """El valor de fábrica tiene que servir en toda la mesa, no en un caso."""

    for nombre, (p0, p1, _) in CASOS.items():
        if math.dist(p0, p1) < 1.0:
            continue

        medido = desvio_partido_mm(p0, p1, 1.0)

        assert medido < 0.10, f"{nombre}: {medido:.3f} mm"


# ==================================================================
#  La regla que ata el paso con la velocidad
# ==================================================================
def paso_efectivo_cm(p0, p1, paso_pedido: float, vel_cms: float,
                     acel: float, margen: float = 3.0) -> float:
    """Espejo de MovimientoLineal::comenzar (src/motion/Trajectory.cpp).

    El tramo tiene que medir `margen` veces la distancia de frenado. No es
    holgura: con el tramo justo, el eje acelera hasta que frenar le cuesta el
    tramo entero, y ahí redirigir pide 1,1 veces eso más 4 pasos -- más de lo
    que hay -- y se niega. El equilibrio se estabiliza JUSTO en el borde en
    que falla, y eso es el movimiento a los tirones que se vio en el robot.
    """

    k = pasos_por_cm(p0, p1)
    vel_pasos = vel_cms * k
    frenado_cm = (vel_pasos * vel_pasos) / (2.0 * acel) / k

    paso = max(paso_pedido, frenado_cm * margen)

    n = max(1, math.ceil(math.dist(p0, p1) / paso))

    return math.dist(p0, p1) / n


def test_el_paso_nunca_baja_de_la_distancia_de_frenado():
    p0, p1, _ = CASOS["diagonal larga"]
    acel = 40000.0

    for vel in (10.0, 20.0, 35.0, 50.0):
        paso = paso_efectivo_cm(p0, p1, 0.2, vel, acel)
        k = pasos_por_cm(p0, p1)
        frenado_cm = (vel * k) ** 2 / (2.0 * acel) / k

        assert paso >= frenado_cm, (
            f"a {vel} cm/s el tramo ({paso:.2f} cm) es más corto que la "
            f"frenada ({frenado_cm:.2f} cm): no se podría encadenar")


def test_ir_mas_rapido_se_paga_en_precision():
    """El resumen del compromiso, escrito como prueba.

    A la velocidad de fábrica la recta es exacta; pidiendo el triple, el
    tramo mínimo crece tanto que movL se empieza a parecer a movJ. Está acá
    para que quede medido y no como opinión: es la razón por la que
    `movl_vel` viene en 20 cm/s y no en el tope.
    """

    p0, p1, desvio_movj = CASOS["diagonal larga"]
    acel = 40000.0

    lento = desvio_partido_mm(p0, p1, paso_efectivo_cm(p0, p1, 1.0, 20.0, acel))
    rapido = desvio_partido_mm(p0, p1, paso_efectivo_cm(p0, p1, 1.0, 60.0, acel))

    assert lento < 0.10, lento
    assert rapido > 5 * lento, (lento, rapido)

    # Aun así, lo peor que puede hacer movL sigue siendo mejor que movJ.
    assert rapido < desvio_movj


# ==================================================================
#  Por qué se valida la recta entera antes de moverse
# ==================================================================
def test_el_alcance_no_es_convexo():
    """Dos puntos alcanzables NO garantizan que la recta lo sea.

    Por eso `comenzar` recorre la recta entera resolviendo la inversa antes
    de mover un paso: descubrirlo a mitad de camino sería frenar el brazo en
    cualquier lado. movJ no tiene este problema, y es de las pocas cosas en
    las que movL es peor.

    Buscando al azar sobre todo el alcance, 342 de 60.000 rectas entre dos
    puntos alcanzables se salen. Adentro del cajón de teach no se encontró
    ninguna en 40.000, así que en el uso de hoy el chequeo casi nunca salta
    -- pero cuesta unos pocos cientos de microsegundos y el día que movL se
    use en el ciclo normal, con la caja puesta, es la diferencia entre un
    mensaje y un brazo trabado.
    """

    a = (12.08, -4.76, -24.41)
    b = (-0.22, 17.56, -20.48)

    assert cin.alcanzable(*a) and cin.alcanzable(*b)

    medio = [not cin.alcanzable(*(a[k] + s / 50 * (b[k] - a[k]) for k in range(3)))
             for s in range(1, 50)]

    assert any(medio), "la recta de ejemplo dejó de salirse del alcance"


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
