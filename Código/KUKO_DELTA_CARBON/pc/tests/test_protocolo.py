"""Verificación del contrato serie, sin robot y sin cámara.

Las líneas de ejemplo son las de PROTOCOLO.md. Si el firmware cambia el
formato, estos tests fallan antes de que el error llegue a la interfaz — que
es todo el punto de tener el parseo separado del hilo del enlace.

Se corre con:  python -m pytest pc/tests    (o  python pc/tests/test_protocolo.py)
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kuko import protocolo as pr

RAIZ = Path(__file__).resolve().parents[2]


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
        "pc=B pf=C py=4.20 pb=3 tw=0 ti=0 nd=41 nk=38 nx=3 nf=2 "
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
    assert m.teach_puntos == 0 and m.teach_indice == 0
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
    """La version que anuncia el firmware es la que espera este modulo.

    Se arma la linea con `pr.VERSION_PROTOCOLO` en vez de con un numero
    escrito: asi subir la version del contrato no obliga a acordarse de tocar
    esta prueba, que es exactamente el descuido que dejaria pasar una
    incompatibilidad sin que nada avise.
    """

    m = pr.parsear(f"[BOOT] proto={pr.VERSION_PROTOCOLO} "
                   "fw=2026-08-19 estados=17 params=55")

    assert isinstance(m, pr.Boot)
    assert m.compatible is True
    assert m.estados == 17
    assert pr.parsear("[BOOT] proto=99").compatible is False


def test_teach():
    """Las formas que toma `[TEACH]`, y los comandos que las provocan."""

    # El volcado de `J?`: es de donde salen los límites del volumen.
    m = pr.parsear(
        "[TEACH] est=on n=12 i=0 pct=15 x=1.50 y=-2.00 z=-30.10 "
        "xmin=-12.00 xmax=12.00 ymin=-9.55 ymax=12.05 "
        "zmin=-32.60 zmax=-26.60 cap=150")

    assert isinstance(m, pr.Teach)
    assert m.evento == "estado" and m.modo == "on"
    assert m.puntos == 12 and m.indice == 0 and m.capacidad == 150
    assert m.limite_x == (-12.0, 12.0)
    assert m.limite_z == (-32.6, -26.6)

    # El volcado de posición a 20 Hz: es lo que se graba.
    p = pr.parsear("[TEACH] p x=-3.20 y=4.50 z=-30.10 b=1")

    assert p.evento == "p" and p.bomba is True
    assert (p.x, p.y, p.z) == (-3.2, 4.5, -30.1)

    # Los eventos sueltos.
    assert pr.parsear("[TEACH] fin").evento == "fin"
    assert pr.parsear("[TEACH] buf n=42").puntos == 42
    assert pr.parsear("[TEACH] run pct=50 n=42").pct == 50.0
    assert pr.parsear("[TEACH] abort motivo=ik").motivo == "ik"
    assert pr.parsear("[TEACH] err=nomodo").error == "nomodo"

    # Ninguno de estos lleva coma... salvo los que sí, y por eso el firmware
    # los consume antes que el parser de piezas.
    assert pr.cmd_teach(True) == "J1\n"
    assert pr.cmd_teach(False) == "J0\n"
    assert pr.cmd_teach_estado() == "J?\n"
    assert pr.cmd_teach_mover(-3.2, 4.5, -30.1) == "JM-3.20,4.50,-30.10\n"
    assert pr.cmd_teach_bomba(True) == "JP1\n"
    assert pr.cmd_teach_limpiar() == "JC\n"
    assert pr.cmd_teach_punto(1.0, 2.0, -30.0, True, 250) == "JA1.00,2.00,-30.00,1,250\n"
    assert pr.cmd_teach_reproducir(50) == "JR50\n"
    assert pr.cmd_teach_abortar() == "JX\n"
    assert pr.cmd_teach_volcado(True) == "JG1\n"

    # La dirección se satura acá también: el firmware la recorta igual, pero
    # mandar un 3,7 sería mandar algo que no significa nada.
    assert pr.cmd_teach_jog(-4.0, 0.5, 0.0) == "JD-1.00,0.50,0.00\n"

    # Una espera absurda no se manda: el campo del firmware es de 16 bits.
    assert pr.cmd_teach_punto(0, 0, -30, False, 10**9).endswith(",0,60000\n")


def test_los_estados_coinciden_con_el_firmware():
    """El enum de `Robot.h` y el de este módulo, uno al lado del otro.

    El índice de estado viaja crudo en `st`, así que agregar uno en el medio
    corre la numeración de todos los siguientes y la interfaz pasa a mostrar
    el estado equivocado sin ningún síntoma. El campo `sn` está para detectar
    eso en marcha; esta prueba lo detecta antes de compilar.
    """

    fuente = (RAIZ / "src" / "robot" / "Robot.h").read_text(encoding="utf-8")

    cuerpo = re.search(r"enum RobotState\s*\{(.*?)\};", fuente, re.S)

    assert cuerpo, "no se encontró el enum RobotState en Robot.h"

    limpio = re.sub(r"//[^\n]*", "", cuerpo.group(1))
    nombres = [n.strip() for n in limpio.split(",") if n.strip()]

    assert nombres, "el enum se leyó vacío"

    esperados = [e.name for e in pr.EstadoRobot]

    assert nombres == esperados, (
        f"el firmware dice {nombres}\ny protocolo.py dice {esperados}")


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


def test_el_estado_de_un_fallo_viene_por_nombre():
    """Es la forma que emite el firmware de verdad.

    `FaultLog` guarda el literal que devuelve `Robot::nombreEstado()` y lo
    imprime tal cual, pero el parser leia un indice -- el ejemplo del
    documento decia `estado=6` -- y el campo valia None en todos los fallos
    reales. Se aceptan las dos formas, y esta prueba cubre justamente la que
    no estaba cubierta.
    """

    f = pr.parsear(
        "[FALLO] n=7 t=900000 tipo=COLISION eje=3 err=-12.40 dcmd=18.30 "
        "denc=5.90 estado=GO_BIN pieza=1 enmano=1 color=R forma=S "
        "py=3.50 px=-4.20 tacho=1"
    )

    assert f.estado is pr.EstadoRobot.GO_BIN
    assert f.estado_nombre == "GO_BIN"

    # Un estado que este modulo todavia no conoce no rompe nada: el nombre
    # crudo sigue diciendo donde fallo.
    g = pr.parsear("[FALLO] n=8 t=1 tipo=COLISION eje=1 estado=INVENTADO")

    assert g.estado is None and g.estado_nombre == "INVENTADO"


def test_una_colision_dice_si_el_brazo_se_freno():
    """`dcmd` contra `denc` es lo que separa la mecanica del sensor."""

    def fallo(dcmd: float, denc: float) -> pr.Fallo:
        return pr.parsear(f"[FALLO] n=1 t=1 tipo=COLISION eje=2 err=13.2 "
                          f"dcmd={dcmd} denc={denc} estado=GO_BIN")

    assert fallo(44.1, 1.9).brazo_frenado is True     # tenia orden y no giro
    assert fallo(44.1, 42.0).brazo_frenado is False   # giro lo que se le pidio
    assert fallo(0.4, 0.1).brazo_frenado is None      # giro muy chico: no dice nada

    # Los otros tipos de fallo no traen un movimiento contra el cual comparar.
    assert pr.parsear("[FALLO] n=1 t=1 tipo=HOMING eje=0").brazo_frenado is None


def test_el_resumen_de_fallos():
    """[FALLOS]: el encabezado del volcado de 'D' y su linea de cierre.

    El firmware ya lo imprimia; hasta ahora se iba a la consola como texto y
    con el se perdian los totales POR TIPO, que son los unicos que sobreviven
    a que el registro de 16 de la vuelta.
    """

    r = pr.parsear("[FALLOS] total=41 COLISION=38 ENCODER=2 HOMING=0 "
                   "MANUAL=1 DESCALIBRACION=0 guardados=16")

    assert isinstance(r, pr.ResumenFallos)
    assert r.total == 41 and r.guardados == 16 and not r.fin
    assert r.por_tipo == {"COLISION": 38, "ENCODER": 2, "HOMING": 0,
                          "MANUAL": 1, "DESCALIBRACION": 0}

    cierre = pr.parsear("[FALLOS] fin")

    assert isinstance(cierre, pr.ResumenFallos) and cierre.fin
    assert cierre.total is None                        # ausente no es cero


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
    assert pr.cmd_modo(pr.Modo.BOX) == "A\n"
    assert pr.cmd_paro() == "R\n"
    assert pr.cmd_layout_caja("brgrbg") == "XBRGRBG\n"
    assert pr.cmd_parametro("vis_lat", 0.18) == "Pvis_lat=0.1800\n"
    assert pr.cmd_telemetria(True) == "V1\n"
    assert pr.cmd_calibracion(True) == "CAL1\n"
    assert pr.cmd_calibracion(False) == "CAL0\n"

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
