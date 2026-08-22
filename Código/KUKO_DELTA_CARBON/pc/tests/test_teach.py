"""El modo Teach entero, sin robot y sin navegador.

Se verifica la cadena completa: teclas y joystick -> comandos de jog ->
grabación -> simplificación -> carga de la ruta -> reproducción por etapas ->
el cartel de "¿salió bien?" que sube el movimiento de escalón.

Lo que hace útiles a estas pruebas es que el enlace es una lista de líneas:
se comprueba **lo que sale por el puerto**, que es el único contrato real con
el firmware. Un cambio de nombre de método no las rompe; un comando mal
armado, sí.

Se corre con:  python -m pytest pc/tests   (o  python pc/tests/test_teach.py)
"""

from __future__ import annotations

import json
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from kuko import protocolo as pr
from kuko import ui as ui_mod
from kuko import teach as tch
from kuko.estado import EstadoSistema


# ==================================================================
#  Andamios
# ==================================================================
def _tecla(codigo: str, abajo: bool = True, repeticion: bool = False):
    return SimpleNamespace(
        key=SimpleNamespace(code=codigo, name=codigo),
        action=SimpleNamespace(keydown=abajo, keyup=not abajo, repeat=repeticion),
        modifiers=SimpleNamespace())


def _raton(x: float, y: float, botones: int = 1):
    return SimpleNamespace(args={"offsetX": x, "offsetY": y, "buttons": botones})


def _estado_en_teach() -> EstadoSistema:
    est = EstadoSistema()
    est.conectado = True
    est.ultimo_t = time.monotonic()
    est.puerto = "COM-de-mentira"

    est.t = pr.Telemetria(
        crudo="", t_ms=1000, estado=pr.EstadoRobot.TEACH, bomba=False,
        finales=[False, False, False],
        angulo=[0.0, 0.0, 0.0], comandado=[0.0, 0.0, 0.0],
        error=[0.0, 0.0, 0.0], umbral=[14.0, 14.0, 14.0],
        velocidad=[0.0, 0.0, 0.0])

    est.e = pr.Proceso(
        crudo="", estado=pr.EstadoRobot.TEACH, estado_nombre="TEACH",
        modo=pr.Modo.COLOR, homed=True, cola=0,
        teach_puntos=0, teach_indice=0)

    est.angulo_suave = [0.0, 0.0, 0.0]
    est.teach = pr.parsear(
        "[TEACH] est=on n=0 i=0 pct=15 x=0.00 y=0.00 z=-26.60 "
        "xmin=-12.00 xmax=12.00 ymin=-9.55 ymax=12.05 "
        "zmin=-32.60 zmax=-26.60 cap=150")
    est.teach_pos = (0.0, 0.0, -26.6)
    est.teach_bomba = False

    return est


class Banco:
    """La interfaz montada sobre un enlace de mentira que anota las líneas."""

    def __init__(self, estado: EstadoSistema, carpeta: Path):
        from kuko import ui as interfaz_ui

        self.lineas: list[str] = []
        self.estado = estado
        self.interfaz = interfaz_ui.Interfaz(estado, self._enviar, None)

        # La biblioteca real vive en pc/config; las pruebas no la tocan.
        self.interfaz.biblioteca = tch.Biblioteca(carpeta / "movimientos.json")

    def _enviar(self, linea: str) -> bool:
        self.lineas.append(linea.strip())
        return True

    def ultimas(self, prefijo: str) -> list[str]:
        return [l for l in self.lineas if l.startswith(prefijo)]

    def limpiar(self) -> None:
        self.lineas.clear()


def _correr_en_pagina(prueba) -> None:
    """Corre `prueba(banco)` dentro de una página real de NiceGUI.

    Hace falta un cliente de verdad porque el código de teach usa `ui.notify`
    y `ui.dialog`, y los dos necesitan un contexto de página. Renderizar la
    página también verifica de paso que la pestaña se construye.
    """

    import pagina

    resultado = {}

    with tempfile.TemporaryDirectory() as carpeta:
        banco = Banco(_estado_en_teach(), Path(carpeta))

        def cuerpo():
            banco.interfaz.construir()
            banco.interfaz._refrescar_lento()

            try:
                prueba(banco)
            except Exception as error:                  # noqa: BLE001
                resultado["error"] = error

        respuesta = pagina.pedir(cuerpo)

        assert respuesta.status_code == 200, respuesta.text[:400]

        if "error" in resultado:
            raise resultado["error"]


# ==================================================================
#  Simplificación de la grabación
# ==================================================================
def test_una_recta_se_reduce_a_sus_extremos():
    muestras = [tch.Muestra(i * 0.05, i * 0.1, 0.0, -30.0, False) for i in range(60)]

    puntos = tch.simplificar(muestras)

    assert len(puntos) == 2
    assert abs(puntos[0].x - 0.0) < 1e-9
    assert abs(puntos[-1].x - 5.9) < 1e-9


def test_los_cambios_de_bomba_no_se_pierden():
    """Es lo primero que borraría un simplificador que sólo mire geometría."""

    muestras = []

    for i in range(40):
        muestras.append(tch.Muestra(i * 0.05, i * 0.1, 0.0, -30.0, False))

    # Quieto medio segundo con la bomba prendida, en el medio de la recta.
    for i in range(10):
        muestras.append(tch.Muestra(2.0 + i * 0.05, 3.9, 0.0, -30.0, True))

    for i in range(40):
        muestras.append(tch.Muestra(2.5 + i * 0.05, 3.9 + i * 0.1, 0.0, -30.0, True))

    puntos = tch.simplificar(muestras)

    assert any(p.bomba for p in puntos), "se perdió el prendido de la bomba"
    assert not puntos[0].bomba, "el arranque no era con vacío"

    # La pausa se conserva como espera del punto donde ocurrió.
    esperas = [p.espera_ms for p in puntos if p.bomba]

    assert max(esperas) >= 400, f"la pausa se perdió: {esperas}"


def test_nunca_se_pasa_de_la_capacidad_del_firmware():
    """Con una trayectoria enredada, se afloja la tolerancia hasta que entra."""

    import math

    muestras = [
        tch.Muestra(i * 0.05,
                    6.0 * math.cos(i * 0.21),
                    6.0 * math.sin(i * 0.13),
                    -30.0 + 2.0 * math.sin(i * 0.07),
                    False)
        for i in range(1500)
    ]

    puntos = tch.simplificar(muestras)

    assert 2 <= len(puntos) <= tch.MAX_PUNTOS


def test_los_escalones_de_verificacion_van_en_orden():
    m = tch.Movimiento(nombre="x")

    assert m.verificado == tch.SIN_VERIFICAR
    assert m.siguiente_escalon == 15

    m.aprobar(15)
    assert m.siguiente_escalon == 50

    m.aprobar(50)
    assert m.siguiente_escalon == 100

    m.aprobar(100)
    assert m.siguiente_escalon == 100
    assert not m.falta_verificar

    # Aprobar un escalón viejo no degrada lo ya verificado.
    m.aprobar(15)
    assert m.verificado == 100


def test_la_biblioteca_sobrevive_al_disco():
    with tempfile.TemporaryDirectory() as carpeta:
        archivo = Path(carpeta) / "movimientos.json"

        biblioteca = tch.Biblioteca(archivo)
        biblioteca.agregar(tch.Movimiento(
            nombre="Tomar de la cinta",
            puntos=[tch.Punto(0, 0, -30), tch.Punto(1, 2, -31, True, 300)],
            verificado=50, creado="2026-08-19 10:00", duracion_s=4.2))

        recargada = tch.Biblioteca(archivo)

        assert len(recargada.movimientos) == 1

        m = recargada.movimientos[0]

        assert m.nombre == "Tomar de la cinta"
        assert m.verificado == 50
        assert len(m.puntos) == 2
        assert m.puntos[1].bomba and m.puntos[1].espera_ms == 300


def test_lo_que_conserva_el_nombre_de_fabrica_no_va_al_repositorio():
    """El reparto entre los dos archivos lo decide el nombre, y nada más.

    Es la regla que hace que el repositorio no se llene con el descarte de
    cada tarde de pruebas sin que nadie tenga que acordarse de limpiarlo.
    """

    with tempfile.TemporaryDirectory() as carpeta:
        archivo = Path(carpeta) / "movimientos.json"
        biblioteca = tch.Biblioteca(archivo)

        biblioteca.agregar(tch.Movimiento(nombre="Movimiento 7",
                                          puntos=[tch.Punto(0, 0, -30)],
                                          creado="2026-08-19 10:00"))
        biblioteca.agregar(tch.Movimiento(nombre="Baile 1 - Vals",
                                          puntos=[tch.Punto(1, 1, -30)],
                                          creado="2026-08-19 11:00"))

        versionados = json.loads(archivo.read_text(encoding="utf-8"))["movimientos"]
        locales = json.loads(
            biblioteca.archivo_local.read_text(encoding="utf-8"))["movimientos"]

        assert [m["nombre"] for m in versionados] == ["Baile 1 - Vals"]
        assert [m["nombre"] for m in locales] == ["Movimiento 7"]

        # Para el operador siguen siendo una biblioteca sola.
        assert len(tch.Biblioteca(archivo).movimientos) == 2

        # Y renombrar uno lo muda de archivo, que es cómo se lo salva.
        indice = [m.nombre for m in biblioteca.movimientos].index("Movimiento 7")
        biblioteca.renombrar(indice, "Saludo")

        versionados = json.loads(archivo.read_text(encoding="utf-8"))["movimientos"]

        assert sorted(m["nombre"] for m in versionados) == ["Baile 1 - Vals", "Saludo"]
        assert json.loads(
            biblioteca.archivo_local.read_text(encoding="utf-8"))["movimientos"] == []


def test_un_archivo_roto_no_impide_arrancar():
    with tempfile.TemporaryDirectory() as carpeta:
        archivo = Path(carpeta) / "movimientos.json"
        archivo.write_text("{esto no es json", encoding="utf-8")

        assert tch.Biblioteca(archivo).movimientos == []


# ==================================================================
#  La pestaña, de punta a punta
# ==================================================================
def test_el_jog_manda_direcciones_y_frena_al_soltar():
    def prueba(banco: Banco):
        interfaz = banco.interfaz
        interfaz._cambio_pestana(SimpleNamespace(value="Teach"))
        banco.limpiar()

        # W = alejarse (+Y), D = a la derecha (+X).
        interfaz._teach_tecla(_tecla("KeyW"))
        interfaz._teach_tecla(_tecla("KeyD"))
        interfaz._teach_tick()

        jog = banco.ultimas("JD")

        assert jog, "no salió ninguna dirección de jog"

        vx, vy, vz = (float(v) for v in jog[-1][2:].split(","))

        assert vx > 0 and vy > 0, f"dirección equivocada: {jog[-1]}"
        assert abs(vz) < 1e-9
        # Las diagonales no van más rápido que las rectas.
        assert abs((vx * vx + vy * vy) ** 0.5 - 1.0) < 0.02

        # Soltar tiene que frenar, y una sola vez.
        interfaz._teach_tecla(_tecla("KeyW", abajo=False))
        interfaz._teach_tecla(_tecla("KeyD", abajo=False))
        banco.limpiar()
        interfaz._teach_tick()
        interfaz._teach_tick()
        interfaz._teach_tick()

        assert banco.ultimas("JD") == ["JD0.00,0.00,0.00"], banco.ultimas("JD")

    _correr_en_pagina(prueba)


def test_las_flechas_mueven_la_altura_y_el_joystick_el_plano():
    def prueba(banco: Banco):
        interfaz = banco.interfaz
        interfaz._cambio_pestana(SimpleNamespace(value="Teach"))

        interfaz._teach_tecla(_tecla("ArrowUp"))
        banco.limpiar()
        interfaz._teach_tick()

        assert banco.ultimas("JD")[-1].endswith(",1.00")

        interfaz._teach_tecla(_tecla("ArrowUp", abajo=False))
        interfaz._teach_tecla(_tecla("ArrowDown"))
        banco.limpiar()
        interfaz._teach_tick()

        assert banco.ultimas("JD")[-1].endswith(",-1.00")

        interfaz._teach_tecla(_tecla("ArrowDown", abajo=False))
        interfaz._teach_tick()

        # Joystick arrastrado a la izquierda del centro (el div mide 150 px).
        interfaz._joy_apretar(_raton(10, 75))
        banco.limpiar()
        interfaz._teach_tick()

        vx = float(banco.ultimas("JD")[-1][2:].split(",")[0])

        assert vx < -0.8, f"el joystick no se leyó: {banco.ultimas('JD')}"

        # Soltar el botón del mouse lo devuelve al centro solo.
        interfaz._joy_mover(_raton(10, 75, botones=0))

        assert interfaz.joy_x == 0.0 and interfaz.joy_y == 0.0

    _correr_en_pagina(prueba)


def test_fuera_de_la_pestana_el_teclado_no_mueve_nada():
    def prueba(banco: Banco):
        interfaz = banco.interfaz
        interfaz._cambio_pestana(SimpleNamespace(value="Operacion"))
        banco.limpiar()

        interfaz._teach_tecla(_tecla("KeyW"))
        interfaz._teach_tick()

        assert not [l for l in banco.lineas if l.startswith("JD") and l != "JD0.00,0.00,0.00"]

    _correr_en_pagina(prueba)


def test_grabar_reproducir_y_verificar_por_etapas():
    def prueba(banco: Banco):
        interfaz = banco.interfaz
        est = banco.estado

        interfaz._cambio_pestana(SimpleNamespace(value="Teach"))

        # --- grabación: R, se mueve, R ---
        interfaz._teach_grabar()

        assert interfaz.teach_grabando

        for i in range(30):
            est.teach_pos = (i * 0.3, 0.0, -28.0)
            est.teach_bomba = (i > 15)
            interfaz._teach_tick()

        interfaz._teach_grabar()

        assert not interfaz.teach_grabando
        assert len(interfaz.biblioteca.movimientos) == 1

        movimiento = interfaz.biblioteca.movimientos[0]

        assert movimiento.verificado == tch.SIN_VERIFICAR
        assert movimiento.siguiente_escalon == 15
        assert interfaz.teach_sel == 0

        # Se guardó en disco, no sólo en memoria.
        assert tch.Biblioteca(interfaz.biblioteca.archivo).movimientos

        # --- reproducción: primero al 15 % ---
        banco.limpiar()
        interfaz._teach_reproducir()

        # La carga es a pedacitos; se la fuerza hasta que termina.
        for _ in range(40):
            if not interfaz._cola_subida:
                break
            interfaz._teach_subir_trozo()

        assert banco.ultimas("JC") == ["JC"], "no se vació el buffer antes de cargar"
        assert len(banco.ultimas("JA")) == len(movimiento.puntos)
        assert banco.ultimas("JR") == ["JR15"], banco.ultimas("JR")

        # --- el cartel de "¿salió bien?" ---
        est.teach_evento = pr.parsear("[TEACH] fin")
        est.teach_evento_n += 1
        interfaz._teach_tick()

        assert interfaz._dialogo_teach is not None, "no salió el cartel"

        # Confirmar encadena la etapa siguiente sola.
        banco.limpiar()
        interfaz._teach_aprobar(interfaz._dialogo_teach, movimiento, 15)

        assert movimiento.verificado == 15

        for _ in range(40):
            if not interfaz._cola_subida:
                break
            interfaz._teach_subir_trozo()

        assert banco.ultimas("JR") == ["JR50"], banco.ultimas("JR")

        # --- y de ahí al 100 %, que es el último ---
        interfaz._teach_aprobar(interfaz._dialogo_teach, movimiento, 50)

        assert movimiento.verificado == 50

        for _ in range(40):
            if not interfaz._cola_subida:
                break
            interfaz._teach_subir_trozo()

        assert banco.ultimas("JR")[-1] == "JR100"

        interfaz._teach_aprobar(interfaz._dialogo_teach, movimiento, 100)

        assert movimiento.verificado == 100
        assert not movimiento.falta_verificar

        # Y queda verificado también en el archivo.
        guardado = tch.Biblioteca(interfaz.biblioteca.archivo).movimientos[0]

        assert guardado.verificado == 100

    _correr_en_pagina(prueba)


def test_decir_que_no_deja_el_escalon_donde_estaba():
    def prueba(banco: Banco):
        interfaz = banco.interfaz

        movimiento = interfaz.biblioteca.agregar(tch.Movimiento(
            nombre="prueba",
            puntos=[tch.Punto(0, 0, -30), tch.Punto(2, 0, -30)]))

        interfaz._teach_rearmar_lista()
        interfaz.teach_sel = 0
        interfaz._cambio_pestana(SimpleNamespace(value="Teach"))

        interfaz._teach_reproducir()

        for _ in range(40):
            if not interfaz._cola_subida:
                break
            interfaz._teach_subir_trozo()

        banco.estado.teach_evento = pr.parsear("[TEACH] fin")
        banco.estado.teach_evento_n += 1
        interfaz._teach_tick()

        banco.limpiar()
        interfaz._teach_rechazar(interfaz._dialogo_teach)

        assert movimiento.verificado == tch.SIN_VERIFICAR
        assert movimiento.siguiente_escalon == 15
        assert not banco.ultimas("JR"), "se reprodujo igual después de decir que no"

    _correr_en_pagina(prueba)


def test_la_carga_no_se_pasa_de_la_capacidad_del_firmware():
    """El buffer del ESP32 entra 150 puntos y el cargador nunca manda más."""

    fuente = (Path(__file__).resolve().parents[2] / "src" / "robot" / "Robot.h") \
        .read_text(encoding="utf-8")

    import re

    m = re.search(r"TEACH_MAX_PUNTOS\s*=\s*(\d+)", fuente)

    assert m, "no se encontró TEACH_MAX_PUNTOS en el firmware"
    assert tch.MAX_PUNTOS <= int(m.group(1)), \
        f"Python simplifica a {tch.MAX_PUNTOS} y el firmware entra {m.group(1)}"


def test_el_volcado_de_posicion_se_enciende_y_se_apaga_solo():
    """760 B/s no se gastan con la pantalla mirando otra cosa."""

    def prueba(banco: Banco):
        interfaz = banco.interfaz

        interfaz._cambio_pestana(SimpleNamespace(value="Teach"))
        banco.limpiar()
        interfaz._teach_tick()

        assert "JG1" in banco.lineas

        interfaz._cambio_pestana(SimpleNamespace(value="Proceso"))
        banco.limpiar()
        interfaz._teach_tick()

        assert "JG0" in banco.lineas

    _correr_en_pagina(prueba)


def test_ir_a_una_coordenada_manda_el_pedido_una_sola_vez():
    def prueba(banco: Banco):
        interfaz = banco.interfaz
        interfaz._cambio_pestana(SimpleNamespace(value="Teach"))
        banco.limpiar()

        def escribir(x, y, z):
            interfaz.campos_ir["x"].value = x
            interfaz.campos_ir["y"].value = y
            interfaz.campos_ir["z"].value = z

        escribir(2.0, 3.0, -29.0)
        interfaz._teach_ir()

        assert banco.ultimas("JI") == ["JI2.00,3.00,-29.00"], banco.lineas

        # Con uno en curso no se manda otro: el firmware lo rechazaría y el
        # operador vería una línea roja sin entender por qué.
        banco.limpiar()
        interfaz._teach_ir()

        assert banco.ultimas("JI") == []

        # Mientras el firmware no confirme, el bloqueo es CORTO. Un firmware
        # viejo no contesta nada, y la pantalla tiene que volver sola en vez
        # de quedarse diciendo que el brazo va.
        falta = interfaz.teach_yendo_hasta - time.monotonic()

        assert falta <= ui_mod.ESPERA_IR_CONFIRMA_S, falta

        # Confirmado, en cambio, se banca todo el movimiento.
        banco.estado.teach_evento = pr.parsear(
            "[TEACH] ir x=2.00 y=3.00 z=-29.00 home=1")
        banco.estado.teach_evento_n += 1
        interfaz._teach_eventos()

        assert interfaz.teach_yendo_hasta - time.monotonic() > ui_mod.ESPERA_IR_CONFIRMA_S

        # Y el evento de llegada lo desbloquea.
        banco.estado.teach_evento = pr.parsear("[TEACH] irfin")
        banco.estado.teach_evento_n += 1
        interfaz._teach_eventos()

        interfaz._teach_ir()

        assert banco.ultimas("JI") == ["JI2.00,3.00,-29.00"]

        # Fuera del volumen no se manda nada: el firmware recorta, pero
        # recortar en silencio sería mover el brazo a otro lado del pedido.
        banco.limpiar()
        banco.estado.teach_evento = pr.parsear("[TEACH] irfin")
        banco.estado.teach_evento_n += 1
        interfaz._teach_eventos()

        escribir(200.0, 3.0, -29.0)
        interfaz._teach_ir()

        assert banco.ultimas("JI") == []

    _correr_en_pagina(prueba)


def test_con_algo_en_curso_el_boton_de_reproducir_es_el_de_parar():
    def prueba(banco: Banco):
        interfaz = banco.interfaz
        est = banco.estado

        interfaz._cambio_pestana(SimpleNamespace(value="Teach"))
        interfaz.biblioteca.agregar(tch.Movimiento(
            nombre="Saludo", puntos=[tch.Punto(0, 0, -30)],
            verificado=100, creado="2026-08-19 10:00"))
        interfaz._teach_rearmar_lista()
        interfaz.teach_sel = 0

        # Reproduciendo: el firmware lo dice en [E] con el índice del punto.
        est.e.teach_indice = 3
        banco.limpiar()

        interfaz._refrescar_teach()

        assert interfaz.boton_reproducir.text == "Parar  ·  P"

        interfaz._teach_reproducir()

        assert banco.ultimas("JX") == ["JX"], banco.lineas
        assert banco.ultimas("JR") == [], "arrancó una reproducción en vez de parar"

        # Y quieto vuelve a ser el de reproducir, sin el porcentaje encima.
        est.e.teach_indice = 0
        interfaz._refrescar_teach()

        assert interfaz.boton_reproducir.text == "Reproducir  ·  P"

    _correr_en_pagina(prueba)


def test_un_movimiento_ya_verificado_no_vuelve_a_preguntar():
    def prueba(banco: Banco):
        interfaz = banco.interfaz
        est = banco.estado

        interfaz._cambio_pestana(SimpleNamespace(value="Teach"))

        movimiento = interfaz.biblioteca.agregar(tch.Movimiento(
            nombre="Saludo", puntos=[tch.Punto(0, 0, -30)],
            verificado=100, creado="2026-08-19 10:00"))

        interfaz.teach_mov_en_curso = movimiento
        interfaz.teach_pct_en_curso = 100

        est.teach_evento = pr.parsear("[TEACH] fin")
        est.teach_evento_n += 1
        interfaz._teach_tick()

        assert interfaz._dialogo_teach is None, "preguntó por un 100 % ya verificado"

        # Con uno a medio verificar, en cambio, tiene que preguntar.
        movimiento.verificado = 50
        interfaz.teach_mov_en_curso = movimiento
        est.teach_evento_n += 1
        interfaz._teach_tick()

        assert interfaz._dialogo_teach is not None

    _correr_en_pagina(prueba)


def test_la_pestana_dibuja_el_volumen_y_la_lista():
    def prueba(banco: Banco):
        interfaz = banco.interfaz
        interfaz.biblioteca.agregar(tch.Movimiento(
            nombre="Ida y vuelta al tacho",
            puntos=[tch.Punto(0, 0, -30), tch.Punto(6, -4, -28, True, 200)],
            verificado=50, creado="2026-08-19 10:00", duracion_s=3.5))

        interfaz._cambio_pestana(SimpleNamespace(value="Teach"))
        interfaz._teach_rearmar_lista()
        interfaz.teach_sel = 0
        interfaz._refrescar_teach()

        plano = interfaz.html_plano.content

        assert plano.startswith("<svg") and "</svg>" in plano
        assert interfaz.html_joystick.content.startswith("<svg")

        # La posición real sale de la cinemática directa de los encoders.
        assert "sin lectura de los encoders" not in plano

        assert "50 %" in interfaz._filas_teach[0]["insignia"].content
        assert "2 puntos" in interfaz._filas_teach[0]["info"].text

    _correr_en_pagina(prueba)


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
