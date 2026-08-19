"""Las pestañas de ajustes se arman de verdad, sin robot y sin cámara.

Los parámetros NO están escritos acá: se leen de `Robot::registrarParametros()`
en el propio firmware. Eso hace que estas pruebas comparen las dos mitades del
sistema, que es lo único que importa verificar de esta parte:

  * que cada parámetro registrado en C++ tenga su ficha en `parametros.py`
    (si no, aparece en pantalla con el nombre corto y sin explicación);
  * que ninguno pase el límite de 12 caracteres de la NVS;
  * que la página completa se renderice con los tres paneles, alimentada con
    telemetría falsa — que es lo que hace que un nombre de campo mal escrito
    en un panel lateral falle acá y no delante del jurado.

Se corre con:  python -m pytest pc/tests    (o  python pc/tests/test_interfaz.py)
"""

from __future__ import annotations

import re
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kuko import parametros as par
from kuko import protocolo as pr
from kuko.estado import EstadoSistema

RAIZ = Path(__file__).resolve().parents[2]

# Los argumentos de params.registrar(): nombre, puntero, mínimo, máximo,
# unidad, nivel y (opcional) tipo. El puntero se saltea; puede ser un
# miembro (&telemetria.periodoRapida_ms) o un elemento de un arreglo
# (&BIN_X[0]), y eso no cambia nada de lo que se verifica.
REGISTRO = re.compile(
    r'params\.registrar\(\s*"([^"]+)"\s*,\s*&[\w:.\[\]]+\s*,\s*'
    r'([-\d.ef]+)\s*,\s*([-\d.ef]+)\s*,\s*"([^"]*)"\s*,\s*NIVEL_(\w+)'
    r"(?:\s*,\s*'(\w)')?")

NIVELES = {"OPERACION": pr.NIVEL_OPERACION,
           "PROCESO": pr.NIVEL_PROCESO,
           "SERVICIO": pr.NIVEL_SERVICIO}


def _tabla_del_firmware() -> dict[str, pr.Parametro]:
    fuente = (RAIZ / "src" / "robot" / "Robot.cpp").read_text(encoding="utf-8")
    tabla: dict[str, pr.Parametro] = {}

    for nombre, mn, mx, unidad, nivel, tipo in REGISTRO.findall(fuente):
        minimo, maximo = float(mn.rstrip("f")), float(mx.rstrip("f"))

        # Un valor cualquiera dentro del rango, distinto de los extremos:
        # los extremos esconden errores de saturación en los controles.
        valor = minimo + (maximo - minimo) * 0.37

        tabla[nombre] = pr.Parametro(
            crudo="", nombre=nombre, valor=valor, defecto=valor,
            minimo=minimo, maximo=maximo, unidad=unidad,
            nivel=NIVELES[nivel], tipo=tipo or "f")

    return tabla


def _estado_completo() -> EstadoSistema:
    estado = EstadoSistema()
    estado.conectado = True
    estado.ultimo_t = time.monotonic()
    estado.puerto = "COM-de-mentira"
    estado.parametros = _tabla_del_firmware()

    # Uno fuera de fábrica, para que se dibuje el punto de "modificado".
    estado.parametros["press_dz"].valor = estado.parametros["press_dz"].maximo

    estado.t = pr.Telemetria(
        crudo="", t_ms=1000, estado=pr.EstadoRobot.WAIT_PIECE, bomba=True,
        finales=[False, True, False],
        angulo=[-44.0, -12.0, 3.0], comandado=[-45.0, -12.5, 3.2],
        error=[1.0, 9.5, 22.0], umbral=[14.0, 14.0, 14.0],
        velocidad=[0.0, 120.0, 300.0])

    estado.e = pr.Proceso(
        crudo="", estado=pr.EstadoRobot.WAIT_PIECE, estado_nombre="WAIT_PIECE",
        modo=pr.Modo.COLOR, cola=2, cinta=True, cinta_pwm=60,
        paradas_activas=False, detectadas=48, depositadas=45, descartadas=2,
        fallos=1, por_color={"R": 12, "G": 20, "B": 13},
        por_forma={"S": 15, "H": 15, "C": 15})

    estado.cinta_medida = 7.05
    estado.consola.append("12:00:00  [GUARD] pico 2.1 / 9.5 / 22.0")

    return estado


def test_la_tabla_del_firmware_se_pudo_leer():
    """Si esto falla, el resto de las pruebas no está verificando nada."""

    tabla = _tabla_del_firmware()

    assert len(tabla) >= 40, f"solo se leyeron {len(tabla)} parametros"
    assert "vis_lat" in tabla and "grab_z" in tabla


def test_todos_los_parametros_tienen_ficha():
    faltan = [n for n in _tabla_del_firmware() if n not in par.FICHAS]

    assert not faltan, f"sin ficha en parametros.py: {faltan}"


def test_los_nombres_entran_en_la_nvs():
    """La NVS del ESP32 limita las claves a 15 caracteres y el firmware a 12.

    Un nombre más largo se rechaza al registrarlo, o sea que el parámetro
    directamente no existiría en marcha.
    """

    largos = [n for n in _tabla_del_firmware() if len(n) > 12]

    assert not largos, f"nombres demasiado largos: {largos}"


def test_cada_nivel_tiene_contenido():
    tabla = _tabla_del_firmware().values()

    for nivel in (pr.NIVEL_PROCESO, pr.NIVEL_SERVICIO):
        grupos = par.agrupar(tabla, nivel)

        assert grupos, f"el nivel {nivel} quedo sin ajustes"
        assert all(ps for _, ps in grupos), "hay un grupo vacio"

        # Nada debería caer en "Otros": ese grupo es la red de seguridad
        # para un parámetro recién agregado, no un lugar donde vivir.
        assert "Otros" not in dict(grupos), "hay parametros sin grupo asignado"


def test_la_pagina_entera_se_renderiza():
    """Arma la página completa y la pide por HTTP, sin navegador.

    Cubre las tres pestañas de una: cualquier excepción al construirlas o al
    refrescarlas sale como error del pedido, no como un panel en blanco.
    """

    from fastapi import FastAPI
    from fastapi.testclient import TestClient
    from nicegui import ui

    from kuko import ui as interfaz_ui

    estado = _estado_completo()
    interfaz = interfaz_ui.Interfaz(estado, lambda linea: True, None)
    servidor = FastAPI()

    @ui.page("/prueba")
    def pagina():
        interfaz.construir()

        # Lo que normalmente harían los temporizadores de 0,1 s y 0,5 s. El
        # segundo refresco es a propósito: el primero arma las listas y el
        # segundo pasa por el camino de "ya estaban armadas".
        interfaz._refrescar_rapido()
        interfaz._refrescar_lento()
        interfaz._refrescar_lento()

    ui.run_with(servidor)

    with TestClient(servidor) as cliente:
        respuesta = cliente.get("/prueba")

    assert respuesta.status_code == 200

    for texto in ("Presion sobre la pieza", "Supervision de colisiones",
                  "Corrimiento de la caja", "Consola del robot",
                  "medida por la vision", "Guardar en la placa"):
        assert texto in respuesta.text, f"no se renderizo {texto!r}"

    # Una fila de control por parámetro de proceso y de servicio (los de
    # operación viven en la otra pestaña).
    esperadas = sum(1 for p in estado.parametros.values()
                    if p.nivel in (pr.NIVEL_PROCESO, pr.NIVEL_SERVICIO))

    assert len(interfaz._filas) == esperadas

    # El slider de latencia de la pestaña de operación se construye con el
    # rango que declaró el firmware, no con uno escrito en Python. Esto ya
    # se desincronizó una vez (el mínimo quedó en -0,10 s cuando el firmware
    # pasó a -0,20 s) y el síntoma es un tope que el robot rechaza.
    vis_lat = estado.parametros["vis_lat"]

    assert interfaz.slider_latencia is not None, "el slider no se creo"
    assert interfaz.slider_latencia._props["min"] == vis_lat.minimo
    assert interfaz.slider_latencia._props["max"] == vis_lat.maximo


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
