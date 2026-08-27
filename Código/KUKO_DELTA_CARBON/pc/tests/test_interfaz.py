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
sys.path.insert(0, str(Path(__file__).resolve().parent))

from kuko import parametros as par
from kuko import protocolo as pr
from kuko import rendimiento as rd
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

    _corrida_de_prueba(estado)

    return estado


def _corrida_de_prueba(estado: EstadoSistema) -> None:
    """Le mete al historial una corrida corta con todo lo que sabe dibujar.

    Sin esto, la pestaña de Rendimiento se renderizaría con todos los
    gráficos en su rama de "sin datos" y las siete funciones que arman las
    opciones de verdad no las tocaría nadie hasta tener el robot delante.

    El reloj del módulo se reemplaza mientras dura el armado: cuatro minutos
    de corrida no se pueden simular esperándolos.
    """

    hist = estado.rendimiento
    real = rd.time.time
    ahora = real()
    rd.time.time = lambda: ahora                    # type: ignore[assignment]

    t_ms = 5000
    contadores = dict(detectadas=0, depositadas=0, descartadas=0, fallos=0)

    fotogramas = 0
    camara_ok = True
    ultimo_aviso = 0.0

    # El robot y la cámara informan A LA VEZ, cada uno por su lado, así que
    # el mismo avance de reloj tiene que mover a los dos. Alimentarlos por
    # separado —primero el robot, después la cámara— dejaba a la cámara
    # mandando todas sus muestras en el mismo instante, y una serie de FPS
    # con todos los puntos en el mismo segundo no es una serie: la duración
    # entre muestras da cero y no hay de dónde sacar unos fotogramas por
    # segundo.
    def estar(st, segundos, paso=0.25):
        nonlocal ahora, t_ms, fotogramas, ultimo_aviso

        for _ in range(max(1, int(segundos / paso))):
            ahora += paso
            t_ms += int(paso * 1000)
            hist.observar_telemetria(pr.Telemetria(crudo="", t_ms=t_ms, estado=st))

            if camara_ok:
                fotogramas += int(30 * paso)         # una USB corriente

            # Mismo ritmo que `Vision.PERIODO_AVISO_S`.
            if ahora - ultimo_aviso >= 0.5:
                ultimo_aviso = ahora
                hist.observar_camara(
                    viva=camara_ok, fotogramas=fotogramas,
                    detalle="" if camara_ok else "sin imagen")

    def proceso(st, **cambios):
        contadores.update(cambios)
        hist.observar_proceso(pr.Proceso(crudo="", t_ms=t_ms, estado=st, **contadores))

    try:
        estar(pr.EstadoRobot.IDLE, 3)
        estar(pr.EstadoRobot.HOMING, 8)

        for i in range(24):
            estar(pr.EstadoRobot.WAIT_PIECE, 3)

            # A mitad de la corrida el brazo choca EN PLENA MANIOBRA: parada,
            # rehoming, y dos piezas que se pasan sin agarrar mientras tanto.
            # Tiene que ser en el medio de la maniobra y no entre dos, porque
            # es lo que hace que la maniobra quede marcada como cortada.
            if i == 11:
                estar(pr.EstadoRobot.PICK_APPROACH, 0.75)
                estar(pr.EstadoRobot.PICK_DESCEND, 0.75)
                hist.observar_fallo(pr.parsear(
                    f"[FALLO] n=1 t={t_ms} tipo=COLISION eje=2 err=13.20 "
                    "dcmd=44.10 denc=1.90 estado=PICK_DESCEND pieza=1 enmano=0 "
                    "color=B forma=C py=4.20 px=-1.30 tacho=3"))
                estar(pr.EstadoRobot.COLLISION_STOP, 4)
                estar(pr.EstadoRobot.HOMING, 9)
                proceso(pr.EstadoRobot.WAIT_PIECE, descartadas=2, fallos=1)
                continue

            for st in (pr.EstadoRobot.PICK_APPROACH, pr.EstadoRobot.PICK_DESCEND,
                       pr.EstadoRobot.PICK_LIFT, pr.EstadoRobot.GO_BIN,
                       pr.EstadoRobot.RELEASE_WAIT):
                estar(st, 0.75)

            proceso(pr.EstadoRobot.WAIT_PIECE, detectadas=i + 1, depositadas=i + 1)

            # A los tres cuartos de la corrida se desenchufa la camara un
            # rato: es lo que hace que la tarjeta y el grafico de FPS se
            # dibujen con un corte de verdad y no con la nube plana, que es
            # el caso facil.
            if i == 17:
                camara_ok = False

            if i == 20:
                camara_ok = True

        hist.observar_resumen(pr.parsear(
            "[FALLOS] total=3 COLISION=2 ENCODER=1 HOMING=0 MANUAL=0 "
            "DESCALIBRACION=0 guardados=3"))
        hist.observar_fallo(pr.parsear(
            f"[FALLO] n=2 t={t_ms} tipo=ENCODER eje=3 err=2.10 dcmd=38.00 "
            "denc=37.10 estado=WAIT_PIECE pieza=0 enmano=0"))
    finally:
        rd.time.time = real                         # type: ignore[assignment]


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

    Cubre las cuatro pestañas de una: cualquier excepción al construirlas o
    al refrescarlas sale como error del pedido, no como un panel en blanco.
    """

    import pagina

    from kuko import ui as interfaz_ui

    estado = _estado_completo()
    interfaz = interfaz_ui.Interfaz(estado, lambda linea: True, None)

    def cuerpo():
        interfaz.construir()

        # Lo que normalmente harían los temporizadores de 0,1 s y 0,5 s. El
        # segundo refresco es a propósito: el primero arma las listas y el
        # segundo pasa por el camino de "ya estaban armadas".
        interfaz._refrescar_rapido()
        interfaz._refrescar_lento()
        interfaz._refrescar_lento()

        # La de rendimiento solo se dibuja con la pestana a la vista, asi
        # que hay que decirle que lo esta: es el mismo camino que corre el
        # temporizador, gate incluido.
        interfaz.tab_activa = "Rendimiento"
        interfaz._refrescar_rendimiento()
        interfaz._refrescar_rendimiento()

    respuesta = pagina.pedir(cuerpo)

    assert respuesta.status_code == 200

    for texto in ("Presion sobre la pieza", "Supervision de colisiones",
                  "Corrimiento de la caja", "Consola del robot",
                  "medida por la vision", "Guardar en la placa",
                  "Modo Teach", "Jog manual", "Volumen de trabajo",
                  "Disponibilidad", "Piezas sin agarrar", "Que hizo el robot",
                  "Registro de fallos", "Cronologia de eventos",
                  "Se trabo, o mide mal el encoder",
                  "Camara", "Fotogramas por segundo",
                  "fps, promedio de 5 min"):
        assert texto in respuesta.text, f"no se renderizo {texto!r}"

    # El registro de fallos dibuja la fila con su diagnostico, no solo el
    # marco de la tabla.
    assert "brazo frenado" in respuesta.text
    assert "COLISION" in respuesta.text

    # Y los graficos salieron con datos, no en su rama de "sin datos": una
    # maniobra cortada por el choque y un punto en la nube de trabas.
    cortadas = interfaz.g_maniobras.options["series"][1]["data"]
    trabados = interfaz.g_trabas.options["series"][3]["data"]

    assert cortadas, "la maniobra que corto la colision no se dibujo"
    assert trabados, "la colision con el brazo frenado no se dibujo"

    # La camara: la curva de FPS con el corte adentro, no la nube plana.
    fps = interfaz.g_fps.options["series"][0]["data"]

    assert len(fps) > 20, "la curva de FPS no se dibujo"
    assert any(v < 1.0 for _, v in fps), "el corte de camara no quedo dibujado"
    assert any(v > 20.0 for _, v in fps), "no se dibujo la camara andando"

    # El chip de arriba y el panel de componentes tienen su fila de camara.
    assert "camara" in estado.chequeos()

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


def test_el_rendimiento_sin_datos_no_rompe():
    """La pestana tiene que dibujarse igual con el robot apagado.

    Es el estado en que arranca siempre: la interfaz se abre antes de que el
    ESP32 diga nada, y los siete graficos tienen que caer en su rama de "sin
    datos" en vez de reventar contra una lista vacia.
    """

    import pagina

    from kuko import ui as interfaz_ui

    interfaz = interfaz_ui.Interfaz(EstadoSistema(), lambda linea: False, None)

    def cuerpo():
        interfaz.construir()
        interfaz.tab_activa = "Rendimiento"
        interfaz._refrescar_rendimiento()

    respuesta = pagina.pedir(cuerpo)

    assert respuesta.status_code == 200
    assert "Ningun fallo registrado" in respuesta.text
    assert "Todavia no llego telemetria" in respuesta.text


def test_el_rendimiento_no_se_dibuja_en_otra_pestana():
    """Siete graficos por websocket cada segundo sin que nadie los mire, no.

    Se verifica sobre el efecto observable: con la pestana en otra cosa, el
    refresco no toca el contenido de ningun panel.
    """

    import pagina

    from kuko import ui as interfaz_ui

    interfaz = interfaz_ui.Interfaz(_estado_completo(), lambda linea: True, None)

    def cuerpo():
        interfaz.construir()
        interfaz.tab_activa = "Operacion"
        interfaz._refrescar_rendimiento()

        assert interfaz.html_cronologia.content == ""
        assert interfaz.g_reparto.options == {"series": []}

    assert pagina.pedir(cuerpo).status_code == 200


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
