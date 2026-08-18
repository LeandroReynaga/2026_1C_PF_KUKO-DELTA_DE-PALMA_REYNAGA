"""Verificación del contrato serie, sin robot y sin cámara.

Las líneas de ejemplo son las de PROTOCOLO.md. Si el firmware cambia el
formato, estos tests fallan antes de que el error llegue a la interfaz — que
es todo el punto de tener el parseo separado del hilo del enlace.

Se corre con:  python -m pytest pc/tests    (o  python pc/tests/test_protocolo.py)
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kuko import protocolo as pr


def test_telemetria():
    linea = (
        "[T] t=125430 st=4 pm=1 fc=010 a1=-45.10 a2=-44.28 a3=-44.51 "
        "c1=-45.00 c2=-44.30 c3=-44.50 e1=0.31 e2=-0.12 e3=0.44 "
        "u1=8.2 u2=8.1 u3=8.3 v1=1200 v2=-300 v3=0"
    )

    m = pr.parsear(linea)

    assert isinstance(m, pr.Telemetria)
    assert m.t_ms == 125430
    assert m.estado is pr.EstadoRobot.PICK_APPROACH
    assert m.bomba is True
    assert m.finales == [False, True, False]
    assert m.angulo == [-45.10, -44.28, -44.51]
    assert m.error[2] == 0.44
    assert m.velocidad == [1200.0, -300.0, 0.0]

    # El margen es lo que dibuja la barra de "cuánto le falta para
    # declarar colisión": 0,31 grados de error sobre un umbral de 8,2.
    assert abs(m.margen(0) - 0.31 / 8.2) < 1e-9
    assert m.margen(1) is not None and m.margen(1) > 0  # el signo no importa


def test_telemetria_incompleta_no_inventa_datos():
    """Un campo que no vino vale None, nunca cero."""

    m = pr.parsear("[T] t=100 st=2")

    assert isinstance(m, pr.Telemetria)
    assert m.angulo == [None, None, None]
    assert m.bomba is None
    assert m.margen(0) is None

    # Y los finales quedan como lista vacía, no como tres False: decir
    # "ninguno pisado" sería afirmar algo sobre el hardware que nadie midió.
    assert m.finales == []


def test_proceso():
    linea = (
        "[E] t=125400 st=4 sn=PICK_APPROACH md=C mp=- cf=0 cr=0 q=3 qa=2100 "
        "hm=1 gd=2 ob=0 sup=1 cv=1 cvp=60 bx=BRGRBG bf=000100 bc=4 "
        "pc=B pf=C py=4.20 pb=3 nd=41 nk=38 nx=3 nf=2 "
        "kr=12 kg=14 kb=12 ks=15 kh=11 kc=12"
    )

    m = pr.parsear(linea)

    assert isinstance(m, pr.Proceso)
    assert m.estado is pr.EstadoRobot.PICK_APPROACH
    assert m.discrepancia_estado is False
    assert m.modo is pr.Modo.COLOR
    assert m.modo_pendiente is None          # '-' es "no hay", no un modo
    assert m.cola == 3
    assert m.guard is pr.EstadoGuard.ARMADO
    assert m.layout == "BRGRBG"
    assert m.llenas == [False, False, False, True, False, False]
    assert m.pieza_color == "B"
    assert m.cinta_pwm == 60
    assert abs(m.tasa_exito - 38 / 41) < 1e-9

    # Los contadores de los paneles de modo. Suman las depositadas, que es
    # justamente lo que tienen que informar.
    assert m.por_color == {"R": 12, "G": 14, "B": 12}
    assert m.por_forma == {"S": 15, "H": 11, "C": 12}
    assert sum(m.por_color.values()) == m.depositadas
    assert sum(m.por_forma.values()) == m.depositadas

    # Un contador que no vino se omite: la interfaz muestra "—" y no un
    # cero que se leería como "todavía no depositó ninguna".
    assert "R" not in pr.parsear("[E] t=1 kg=3").por_color


def test_proceso_detecta_enum_desincronizado():
    """El nombre no coincide con el índice: las tablas se desincronizaron."""

    m = pr.parsear("[E] t=1 st=4 sn=GO_BIN")

    assert isinstance(m, pr.Proceso)
    assert m.discrepancia_estado is True


def test_salud():
    linea = (
        "[H] t=125000 up=125 loop=980 heap=182340 "
        "enc1=ok gan1=1.000 atr1=70 pic1=2.10 rep1=0.80 fug1=0.30 rmn1=210 rmx1=3800 rsy1=0 "
        "enc2=ok gan2=0.940 atr2=68 pic2=1.80 rep2=0.70 fug2=0.10 rmn2=80 rmx2=3900 rsy2=1 "
        "enc3=caido gan3=0.000 atr3=0 pic3=0.00 rep3=0.00 fug3=0.00 rmn3=0 rmx3=4095 rsy3=3"
    )

    m = pr.parsear(linea)

    assert isinstance(m, pr.Salud)
    assert m.loop_hz == 980
    assert m.heap == 182340

    assert m.ejes[0].encoder_ok is True
    assert m.ejes[0].pierde_cuentas is False
    assert m.ejes[0].margen_raw == 150       # min(210-60, 3950-3800)

    # 0,94 de ganancia no es ruido: ese eje está perdiendo cuentas y
    # ningún umbral del guard lo arregla.
    assert m.ejes[1].pierde_cuentas is True

    assert m.ejes[2].encoder_ok is False
    assert m.ejes[2].resincronizaciones == 3


def test_parametros():
    fila = pr.parsear("[P] n=vis_lat v=0.180 d=0.150 min=0.000 max=0.500 u=s l=2 t=f")

    assert isinstance(fila, pr.Parametro)
    assert fila.nombre == "vis_lat"
    assert fila.valor == 0.18
    assert fila.nivel == pr.NIVEL_PROCESO
    assert fila.modificado is True           # 0,180 no es el 0,150 de fábrica

    fin = pr.parsear("[P] fin n=24")
    assert isinstance(fin, pr.FinParametros) and fin.cantidad == 24

    ok = pr.parsear("[P] set n=vis_lat v=0.180 ok")
    assert isinstance(ok, pr.ParametroCambiado) and ok.ok is True and not ok.error

    mal = pr.parsear("[P] set n=vis_lat v=9.900 err=rango")
    assert isinstance(mal, pr.ParametroCambiado) and mal.ok is False
    assert mal.error == "rango"


def test_boot():
    m = pr.parsear("[BOOT] proto=1 fw=2026-08-16 estados=16 params=24")

    assert isinstance(m, pr.Boot)
    assert m.compatible is True
    assert pr.parsear("[BOOT] proto=99").compatible is False


def test_pieza_y_fallo():
    """Los dos formatos que el firmware YA emite hoy, sin tocar nada."""

    m = pr.parsear("[PIEZA] Y=4.20 color=B forma=C  en cola: 3")

    assert isinstance(m, pr.PiezaEncolada)
    assert m.y == 4.20 and m.color == "B" and m.cola == 3

    f = pr.parsear(
        "[FALLO] n=2 t=125430 tipo=COLISION eje=1 err=13.20 dcmd=45.10 denc=31.90 "
        "estado=6 pieza=1 enmano=1 color=B forma=C py=4.20 px=-1.30 tacho=3"
    )

    assert isinstance(f, pr.Fallo)
    assert f.tipo == "COLISION"
    assert f.estado is pr.EstadoRobot.PICK_LIFT
    assert f.en_mano is True
    assert f.pieza_x == -1.30


def test_lineas_que_no_son_de_maquina():
    """Prosa, avisos y basura de un reset: todo sale como Texto, sin romper."""

    for linea in (
        "[GUARD] paradas ACTIVAS",
        "[EMERGENCIA] Parada manual. Presiona 'R' para rehomear.",
        "arrancando...",
        "",
        "\x00\xff basura de un reset a mitad de linea",
        "[T",
    ):
        assert isinstance(pr.parsear(linea), pr.Texto)

    # La etiqueta se conserva para poder colorearla en la consola.
    assert pr.parsear("[GUARD] hola").etiqueta == "GUARD"


def test_estado_futuro_no_rompe_el_parseo():
    """Un firmware más nuevo puede tener estados que este módulo no conoce."""

    m = pr.parsear("[T] t=1 st=99")

    assert isinstance(m, pr.Telemetria)
    assert m.estado is None


def test_comandos():
    assert pr.cmd_pieza(3.5, "B", "S") == "3.50,B,S\n"
    assert pr.cmd_pieza(3.5, "AZUL", "CUADRADO") == "3.50,B,S\n"
    assert pr.cmd_modo(pr.Modo.ALFAJORES) == "A\n"
    assert pr.cmd_paro() == "R\n"
    assert pr.cmd_layout_caja("brgrbg") == "XBRGRBG\n"
    assert pr.cmd_parametro("vis_lat", 0.18) == "Pvis_lat=0.1800\n"
    assert pr.cmd_telemetria(True) == "V1\n"

    # Un valor chico no puede irse a notación científica: el strtof del
    # firmware no la interpreta y leería 1 en vez de 0,0001. Se mira sólo
    # el valor, que el nombre del parámetro bien puede tener una 'e'.
    valor = pr.cmd_parametro("press_dz", 0.0001).split("=")[1]

    assert "e" not in valor and valor.strip() == "0.0001"


def test_comandos_rechazan_lo_invalido():
    for llamada in (
        lambda: pr.cmd_pieza(3.5, "NARANJA", "S"),
        lambda: pr.cmd_layout_caja("RGB"),          # faltan celdas
        lambda: pr.cmd_layout_caja("RRRRGB"),       # 4 rojos: no se llena nunca
        lambda: pr.cmd_layout_caja("RGBXYZ"),       # colores inventados
        lambda: pr.cmd_parametro("con espacio", 1),
    ):
        try:
            llamada()
        except ValueError:
            continue

        raise AssertionError("tendría que haber sido rechazado")


def test_ida_y_vuelta_de_los_codigos():
    """Las tablas de traducción no se contradicen entre sí."""

    for nombre, codigo in pr.COLOR_A_CODIGO.items():
        assert pr.COLORES[codigo] == nombre

    for modo, letra in pr.MODO_A_LETRA.items():
        assert pr.LETRA_A_MODO[letra] is modo


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
