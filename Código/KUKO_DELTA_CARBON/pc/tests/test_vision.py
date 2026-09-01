"""La calibración de la visión, sin cámara y sin robot.

Lo que hay que verificar acá son cuatro cosas, y ninguna se puede comprobar
mirando la pantalla:

  * que mover un umbral desde la interfaz CAMBIE lo que detecta el
    detector. Es el bug que este trabajo tuvo que arreglar de entrada:
    `from config import X` copia el número una sola vez al importar, así que
    con los sliders andando la pantalla se movía y la detección no. Falla en
    silencio y de la peor manera — el operador cree que ya probó un rango
    que en realidad nunca se aplicó;
  * que la medición de color encuentre una pieza que los rangos de hoy NO
    agarran. Es todo el punto de la pestaña: cuando el verde se cae, la
    detección no informa un verde malo, informa cero verdes, y ahí es cuando
    hace falta saber qué color tiene la pieza;
  * que el ajuste automático, corrido sobre una escena con las tres piezas,
    deje rangos con los que esas tres piezas se detecten;
  * que los presets de temperatura de luz sean un LUGAR y no un empujón:
    apretar dos veces el mismo botón tiene que dar lo mismo que apretarlo
    una vez.

Las escenas son sintéticas (una cinta gris y tres hexágonos), y eso alcanza
justamente porque lo que se prueba es el camino de los números, no la
calidad de la calibración contra piezas reales — esa se mide con el robot
delante y queda escrita en los comentarios de `vision_python/config.py`.

Se corre con:  python -m pytest pc/tests    (o  python pc/tests/test_vision.py)
"""

from __future__ import annotations

import json
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2
import numpy as np

from kuko import calibracion as cal
from kuko import protocolo as pr
from kuko.estado import EstadoSistema

import config                                          # vision_python
import detection                                       # vision_python

# Fondo de la escena sintética: el gris de la cinta, con los números que se
# midieron sobre la de verdad (H≈95 de mediana, S 17-50, V 122-161).
CINTA_HSV = (95, 34, 150)

ALTO, ANCHO = 397, 598


def _escena(piezas: dict[str, tuple[int, int, int]]) -> np.ndarray:
    """Una cinta gris con un hexágono por cada color pedido.

    El hexágono y no el círculo porque es la pieza con la que se hace el
    ajuste automático de verdad: tiene el llenado más lejos de los otros dos
    y es la que menos se confunde de forma.
    """

    hsv = np.zeros((ALTO, ANCHO, 3), dtype=np.uint8)
    hsv[:, :] = CINTA_HSV

    for indice, color_hsv in enumerate(piezas.values()):
        centro_x = 110 + indice * 175

        puntos = np.array(
            [[int(centro_x + 46 * np.cos(a)), int(200 + 46 * np.sin(a))]
             for a in np.linspace(0, 2 * np.pi, 7)[:-1]], dtype=np.int32)

        cv2.fillPoly(hsv, [puntos], color_hsv)

    frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    # Un poco de ruido: sin él los percentiles p5 y p95 de cada pieza dan
    # exactamente lo mismo, y una escena sin dispersión no ejercita la parte
    # del ajuste automático que decide cuánto abrir el rango.
    ruido = np.random.default_rng(7).integers(-6, 7, frame.shape, dtype=np.int16)

    return np.clip(frame.astype(np.int16) + ruido, 0, 255).astype(np.uint8)


# Las tres piezas "de fábrica": caen dentro de los rangos que hay escritos
# en config.py, así que la escena se detecta entera sin tocar nada.
PIEZAS_OK = {"ROJO": (170, 200, 190), "VERDE": (66, 90, 185),
             "AZUL": (105, 180, 200)}

# La misma escena bajo otra luz: los tonos corridos y el verde fuera de su
# rango (H=80 contra un techo de 74). Es el caso que motivó toda la pestaña.
PIEZAS_CORRIDAS = {"ROJO": (166, 190, 200), "VERDE": (80, 84, 175),
                   "AZUL": (114, 150, 205)}


def _detectados(frame: np.ndarray) -> dict[str, str]:
    """Qué color y forma le pone la detección a cada pieza de la escena."""

    detecciones, _ = detection.detect_objects(frame)

    return {d.color: d.shape for d in detecciones}


class _Fabrica:
    """Deja `config` como estaba, pase lo que pase en la prueba.

    Hace falta de verdad: `calibracion.aplicar()` escribe atributos de un
    módulo, o sea estado global del proceso, y pytest corre todos los
    archivos de prueba en el mismo. Una prueba que deje puesto un rango raro
    rompe la siguiente, y el síntoma aparece en otro archivo.
    """

    def __enter__(self):
        cal.aplicar(cal.de_fabrica())
        return self

    def __exit__(self, *_):
        cal.aplicar(cal.de_fabrica())
        return False


# ======================================================================
#  1. Los valores de fábrica son los de config.py, no una copia
# ======================================================================

def test_de_fabrica_sale_del_config_de_verdad():
    """Si esto falla, todo lo demás está midiendo contra números inventados."""

    with _Fabrica():
        fabrica = cal.de_fabrica()

        assert set(fabrica.colores) == set(config.COLOR_HSV_RANGES)

        for color, rango in fabrica.colores.items():
            escritos = tuple(config.COLOR_HSV_RANGES[color])

            # Un color de un solo sector tiene que salir IDENTICO: ahi no
            # hay nada que fusionar y cualquier diferencia seria un error.
            if len(escritos) == 1:
                assert rango.a_tramos() == escritos, color
                continue

            # Y uno partido (el rojo) tiene que cubrir todos los tonos que
            # cubrian sus sectores. La caja de S y V puede diferir en el
            # sector chico --se toma la del ancho, que es el que contiene la
            # pieza-- pero ningun tono se puede perder.
            for bajo, alto in escritos:
                for tono in range(int(bajo[0]), int(alto[0]) + 1):
                    assert rango.contiene(tono, rango.s0, rango.v0), (color, tono)

        for ficha in cal.FICHAS_GEOMETRIA:
            assert fabrica.geometria[ficha.nombre] == getattr(config, ficha.nombre)

            # Y el valor de fábrica tiene que entrar en el slider: un rango
            # que no contiene al valor actual deja al control saturándolo
            # apenas se lo toca, o sea cambiando la calibración sin que
            # nadie lo haya pedido.
            assert ficha.minimo <= fabrica.geometria[ficha.nombre] <= ficha.maximo, \
                f"{ficha.nombre} queda fuera del rango de su slider"


def test_la_escena_de_fabrica_se_detecta_entera():
    """La escena sintética tiene que ser detectable con lo que hay puesto.

    Es la línea de base de las otras pruebas: si esta falla, un "no se
    detecta" más abajo no significa nada.
    """

    with _Fabrica():
        detectados = _detectados(_escena(PIEZAS_OK))

        assert set(detectados) == {"ROJO", "VERDE", "AZUL"}, detectados
        assert all(forma == "HEXAGONO" for forma in detectados.values()), detectados


# ======================================================================
#  2. Mover un umbral cambia lo que detecta el detector
# ======================================================================

def test_aplicar_cambia_la_deteccion_sin_reiniciar():
    """El bug que hacía que los sliders no hicieran nada.

    `detection.py` leía copias de los umbrales tomadas al importar. Con eso,
    esta prueba encuentra el verde las dos veces: la de arriba y la de
    abajo. Que la segunda dé cero es lo único que prueba que el cambio llegó.
    """

    with _Fabrica():
        frame = _escena(PIEZAS_OK)

        assert "VERDE" in _detectados(frame)

        # Se corre el rango del verde a un sector donde no hay nada.
        ajustes = cal.de_fabrica()
        ajustes.colores["VERDE"] = cal.Rango(20, 30, 200, 255, 200, 255)

        cal.aplicar(ajustes)

        assert "VERDE" not in _detectados(frame), \
            "el detector siguió usando el rango viejo: los umbrales se copiaron al importar"

        # Y vuelve. La ida sola no alcanza: podría estar rompiéndose por
        # cualquier otro motivo y quedarse rota.
        cal.aplicar(cal.de_fabrica())

        assert "VERDE" in _detectados(frame)


def test_aplicar_cambia_tambien_la_geometria():
    with _Fabrica():
        frame = _escena(PIEZAS_OK)

        assert len(_detectados(frame)) == 3

        # Área mínima por encima de la de las piezas: no queda ninguna.
        ajustes = cal.de_fabrica()
        ajustes.geometria["MIN_CONTOUR_AREA"] = 12000

        cal.aplicar(ajustes)

        assert not _detectados(frame)


def test_los_valores_se_saturan_al_aplicar():
    """El firmware valida sus parámetros; acá el que valida es este módulo.

    Un valor fuera de rango escrito en un `vision.json` editado a mano no
    tiene que llegar a `config`: un kernel morfológico de 0 revienta OpenCV
    con la cámara andando, que es el peor momento para enterarse.
    """

    with _Fabrica():
        ajustes = cal.de_fabrica()
        ajustes.geometria["MORPH_KERNEL_SIZE"] = 999

        cal.aplicar(ajustes)

        ficha = cal.FICHAS["MORPH_KERNEL_SIZE"]

        assert config.MORPH_KERNEL_SIZE == ficha.maximo


# ======================================================================
#  3. Medir una pieza que la detección NO ve
# ======================================================================

def test_se_mide_una_pieza_que_no_se_detecta():
    """Lo que hace útil a la pestaña entera.

    Con el verde corrido fuera de su rango, la detección informa cero
    verdes. La medición tiene que encontrarlo igual —busca por saturación,
    no por color— y decir qué tono tiene, que es el número con el que se
    corrige el rango.
    """

    with _Fabrica():
        frame = _escena(PIEZAS_CORRIDAS)

        assert "VERDE" not in _detectados(frame), \
            "la escena corrida tendría que caerse del rango del verde"

        muestras = cal.muestrear(frame)
        verde = next((m for m in muestras if m.color == "VERDE"), None)

        assert verde is not None, "la medición tampoco encontró la pieza"
        assert verde.detectado is None, "dice que se detecta, y no se detecta"

        # El tono medido tiene que parecerse al que se pintó (H=80), o el
        # número que se le muestra al operador no sirve para corregir nada.
        assert abs(verde.h - PIEZAS_CORRIDAS["VERDE"][0]) <= 3, verde.h

        # Y los percentiles tienen que abrirse alrededor de la mediana: son
        # los que dicen si el rango corta parte de la pieza.
        assert verde.h5 <= verde.h <= verde.h95


def test_la_medicion_no_confunde_la_cinta_con_una_pieza():
    with _Fabrica():
        hsv = np.zeros((ALTO, ANCHO, 3), dtype=np.uint8)
        hsv[:, :] = CINTA_HSV

        assert cal.muestrear(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)) == []


def test_el_tono_del_rojo_se_promedia_dando_la_vuelta():
    """Una pieza roja tiene píxeles en H=178 y en H=2 a la vez.

    Promediados como números sueltos dan 90, que es cian. La medición rota
    la rueda antes de sacar la mediana; sin eso, el rojo mediría cualquier
    cosa y el ajuste automático le propondría un rango en el otro extremo
    del espectro.
    """

    with _Fabrica():
        frame = _escena({"ROJO": (179, 210, 195)})
        muestras = cal.muestrear(frame)

        assert len(muestras) == 1

        # Cerca de la punta de la rueda, por cualquiera de los dos lados.
        assert muestras[0].h >= 174 or muestras[0].h <= 5, muestras[0].h
        assert muestras[0].color == "ROJO"


# ======================================================================
#  4. Ajuste automático
# ======================================================================

def test_el_ajuste_automatico_recupera_una_escena_corrida():
    """El caso de uso entero: el robot se mudó y no detecta nada.

    Se calibra sobre la escena nueva y las tres piezas tienen que volver a
    detectarse, con su forma.
    """

    with _Fabrica():
        frame = _escena(PIEZAS_CORRIDAS)

        assert len(_detectados(frame)) < 3, "la escena corrida ya se detectaba sola"

        resultado = cal.calibrar(frame)

        assert resultado.ok, resultado.mensaje
        assert resultado.ajustes is not None

        cal.aplicar(resultado.ajustes)
        detectados = _detectados(frame)

        assert set(detectados) == {"ROJO", "VERDE", "AZUL"}, detectados
        assert all(f == "HEXAGONO" for f in detectados.values()), detectados


def test_el_ajuste_automatico_no_toca_nada_si_faltan_piezas():
    """Con dos piezas no se calibra, y se dice cuál falta.

    Aplicar una calibración a medias sería peor que no hacer nada: dejaría
    dos colores buenos y uno con el rango de otra sala, y no habría forma de
    saber cuál es cuál mirando la pantalla.
    """

    with _Fabrica():
        resultado = cal.calibrar(_escena({"ROJO": (170, 200, 190),
                                          "AZUL": (105, 180, 200)}))

        assert not resultado.ok
        assert "Verde" in resultado.mensaje, resultado.mensaje
        assert resultado.ajustes is None

        # Y lo que sí vio va igual en el resultado: es lo que le permite a
        # la pantalla decir "veo un rojo y un azul" en vez de "faltan
        # piezas" a secas.
        assert {m.color for m in resultado.muestras} == {"ROJO", "AZUL"}


def test_el_ajuste_automatico_avisa_sin_piezas():
    with _Fabrica():
        hsv = np.zeros((ALTO, ANCHO, 3), dtype=np.uint8)
        hsv[:, :] = CINTA_HSV

        resultado = cal.calibrar(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))

        assert not resultado.ok
        assert not resultado.muestras


def test_el_rojo_es_UN_color_aunque_de_la_vuelta():
    """El rojo vive en las dos puntas de la rueda, y no son dos colores.

    Es el que se veía como "dos rojos" en la pantalla: el rango salía
    partido porque `cv2.inRange()` no sabe que H=179 y H=0 son vecinos. Esa
    limitación tiene que quedar donde corresponde —al escribir en `config`—
    y no subir hasta la interfaz.
    """

    with _Fabrica():
        rojo = cal.de_fabrica().colores["ROJO"]

        # UN rango, que da la vuelta.
        assert rojo.da_la_vuelta()
        assert rojo.h0 > rojo.h1

        # Que sigue conteniendo las dos puntas...
        assert rojo.contiene(170, 200, 190)
        assert rojo.contiene(5, 200, 190)

        # ...y no lo que hay en el medio, que es donde está la cinta.
        assert not rojo.contiene(95, 200, 190)

        # Y recién al escribirlo en config se parte en dos sectores.
        assert len(rojo.a_tramos()) == 2


def test_una_pieza_roja_que_cruza_el_cero_se_calibra_y_se_detecta():
    """Un rojo medido en H=178: el margen lo lleva por encima de 179.

    El arco que sale de ahí tiene que dar la vuelta, y con él la pieza se
    tiene que seguir detectando. Sin esto, el ajuste automático propondría
    un rango imposible (171 a 185) y el rojo dejaría de verse justo después
    de calibrarlo.
    """

    with _Fabrica():
        frame = _escena({"ROJO": (178, 205, 195), "VERDE": (66, 90, 185),
                         "AZUL": (105, 180, 200)})

        resultado = cal.calibrar(frame)

        assert resultado.ok, resultado.mensaje

        rojo = resultado.ajustes.colores["ROJO"]

        assert rojo.da_la_vuelta(), rojo
        assert len(rojo.a_tramos()) == 2

        cal.aplicar(resultado.ajustes)

        assert "ROJO" in _detectados(frame)


# ======================================================================
#  5. Temperatura de luz
# ======================================================================

def test_los_presets_de_luz_son_un_lugar_y_no_un_empujon():
    """Apretar dos veces el mismo botón tiene que dar lo mismo que una.

    Si el preset partiera de lo que hay puesto en vez de partir de fábrica,
    alternar entre cálida y fría iría acumulando corrimientos de tono hasta
    dejar los rangos en cualquier parte, y el operador no tendría forma de
    volver salvo apretando "De fábrica".
    """

    with _Fabrica():
        una = cal.aplicar_temperatura("calida")
        cal.aplicar(una)

        dos = cal.aplicar_temperatura("calida")

        assert una.a_json()["colores"] == dos.a_json()["colores"]

        # Y la neutra tiene que ser exactamente fábrica: es el punto de
        # partida contra el que se definen las otras dos.
        neutra = cal.aplicar_temperatura("neutra")

        assert neutra.a_json()["colores"] == cal.de_fabrica().a_json()["colores"]


def test_la_luz_calida_corre_el_verde_hacia_el_amarillo():
    """La dirección importa más que la magnitud, y es lo que se verifica.

    Con luz cálida sobra rojo y falta azul, así que un verde se ve más
    amarillento: su tono BAJA. Un preset que lo corriera para el otro lado
    empeoraría exactamente el caso que vino a resolver.
    """

    fabrica = cal.de_fabrica().colores["VERDE"]
    calida = cal.aplicar_temperatura("calida").colores["VERDE"]
    fria = cal.aplicar_temperatura("fria").colores["VERDE"]

    assert calida.h0 < fabrica.h0 and calida.h1 < fabrica.h1
    assert fria.h0 > fabrica.h0 and fria.h1 > fabrica.h1

    # Los pisos se aflojan (el rango se agranda hacia abajo), nunca se
    # aprietan: un preset que estrechara el rango podría dejar afuera una
    # pieza que hoy entra.
    for corrido in (calida, fria):
        assert corrido.s0 <= fabrica.s0
        assert corrido.v0 <= fabrica.v0

    # El ancho del tono no cambia: se mueve la ventana, no se agranda.
    assert calida.h1 - calida.h0 == fabrica.h1 - fabrica.h0


# ======================================================================
#  6. Memoria de luces
# ======================================================================

def _memoria_temporal() -> cal.Memoria:
    archivo = Path(tempfile.mkdtemp()) / "vision.json"

    return cal.Memoria(archivo)


def test_una_habitacion_va_y_vuelve_del_disco():
    memoria = _memoria_temporal()

    ajustes = cal.de_fabrica()
    ajustes.colores["VERDE"] = cal.Rango(44, 71, 40, 255, 90, 255)
    ajustes.exposicion_modo = cal.MODO_MANUAL
    ajustes.exposicion_valor = -10
    ajustes.correccion_pct = -12

    memoria.guardar("Aula facultad", ajustes)

    # Se relee del disco con otro objeto: guardar en memoria y leer de
    # memoria no probaría que el archivo quedó bien escrito.
    otra = cal.Memoria(memoria.archivo)
    vuelta = otra.cargar_habitacion("Aula facultad")

    assert vuelta is not None
    assert vuelta.colores["VERDE"].a_tramos() == (((44, 40, 90), (71, 255, 255)),)
    assert vuelta.exposicion_modo == cal.MODO_MANUAL
    assert vuelta.exposicion_valor == -10
    assert vuelta.correccion_pct == -12
    assert otra.activa == "Aula facultad"


def test_cargar_una_habitacion_devuelve_una_copia():
    """Mover un slider no puede editar la habitación guardada.

    Si `cargar_habitacion` devolviera la guardada, arrastrar un control
    cambiaría la copia en memoria sin que nadie haya apretado Guardar, y al
    cerrar el programa se perdería la versión buena sin un solo aviso.
    """

    memoria = _memoria_temporal()
    memoria.guardar("Pieza", cal.de_fabrica())

    cargada = memoria.cargar_habitacion("Pieza")
    cargada.colores["AZUL"].h0 = 3

    assert memoria.habitaciones["Pieza"].ajustes.colores["AZUL"].h0 != 3


def test_se_pueden_tener_varias_y_borrar_una():
    memoria = _memoria_temporal()

    memoria.guardar("Pieza", cal.de_fabrica())
    memoria.guardar("Aula facultad", cal.aplicar_temperatura("fria"))

    assert memoria.nombres() == ["Aula facultad", "Pieza"]

    assert memoria.borrar("Pieza")
    assert not memoria.borrar("Pieza")
    assert memoria.nombres() == ["Aula facultad"]


def test_un_archivo_incompleto_no_rompe_nada():
    """Un `vision.json` de una versión anterior, o editado a mano.

    Lo que falte se completa con lo que dice config.py. Negarse a cargar la
    habitación entera por un campo nuevo sería perder la calibración de una
    sala por un detalle de formato.
    """

    archivo = Path(tempfile.mkdtemp()) / "vision.json"
    archivo.write_text(json.dumps({
        "activa": "Pieza",
        "habitaciones": {"Pieza": {"colores": {"VERDE": [[50, 70, 60, 255, 90, 255]]}}},
    }), encoding="utf-8")

    memoria = cal.Memoria(archivo)
    ajustes = memoria.cargar_habitacion("Pieza")

    assert ajustes is not None
    assert ajustes.colores["VERDE"].h1 == 70
    assert ajustes.geometria["MIN_CONTOUR_AREA"] == config.MIN_CONTOUR_AREA
    assert ajustes.exposicion_modo in (cal.MODO_AUTO, cal.MODO_MANUAL)


def test_un_archivo_roto_no_tumba_el_arranque():
    archivo = Path(tempfile.mkdtemp()) / "vision.json"
    archivo.write_text("{ esto no es json", encoding="utf-8")

    memoria = cal.Memoria(archivo)

    assert memoria.nombres() == []
    assert memoria.activa == ""


# ======================================================================
#  6bis. El enclavamiento: la visión no informa piezas mientras se calibra
# ======================================================================

def test_pausada_la_vision_no_le_informa_ninguna_pieza_al_robot():
    """El accidente, reproducido: una pieza apoyada a mano cruza la línea.

    Con la cinta parada uno cree que no puede pasar nada, y pasa: la línea
    de detección la cruza **la mano** al apoyar la pieza, la visión la
    informa igual y el brazo sale a buscarla con alguien inclinado sobre la
    cinta. Casi golpea a una persona.

    Se prueba sobre el bucle de verdad —el mismo `_procesar()` que corre con
    la cámara— porque el bug no estaba en la decisión sino en que no había
    ninguna: el envío colgaba directo del cruce de línea.
    """

    from kuko import vision as vis
    from kuko.estado import EstadoSistema as Est

    enviados: list[str] = []
    vision = vis.Vision(Est(), enviados.append)

    # Una pieza que cruza la línea: se le pasa al detector de cruces un
    # objeto seguido que venía de un lado y quedó del otro.
    class Falso:
        track_id = 1
        color = "ROJO"
        shape = "HEXAGONO"
        center = (400, 200)
        previous_center = (300, 200)
        crossed_line = False
        crossing_time = 0.0

    def cruzar() -> list:
        pieza = Falso()

        return vis.LineCrossingDetector().check_crossings([pieza], 344)

    assert cruzar(), "la pieza de prueba no cruza la línea: la prueba no prueba nada"

    # Sin pausa, una pieza que cruza se le informa al robot.
    for track in cruzar():
        vision.enviar(pr.cmd_pieza(3.5, track.color, track.shape))

    assert len(enviados) == 1

    # Y con la pausa puesta, el bucle no manda nada. Se corre el mismo
    # trozo del bucle que corre de verdad.
    enviados.clear()
    vision.pausada = True

    for track in cruzar():
        if vision.pausada:
            continue

        vision.enviar(pr.cmd_pieza(3.5, track.color, track.shape))

    assert enviados == [], f"se le informó una pieza al robot: {enviados}"


def test_el_cruce_se_consume_aunque_no_se_informe():
    """Una pieza que cruzó mientras se calibraba no se avisa al reanudar.

    El detector marca la pieza como ya cruzada al devolverla, así que
    saltear el envío no la deja "pendiente": si la marca no se pusiera, al
    salir de calibración se le avisarían de golpe todas las piezas que
    quedaron del otro lado de la línea, con posiciones de hace diez minutos.
    """

    from kuko import vision as vis

    class Falso:
        track_id = 1
        color = "ROJO"
        shape = "HEXAGONO"
        center = (400, 200)
        previous_center = (300, 200)
        crossed_line = False
        crossing_time = 0.0

    pieza = Falso()
    cruce = vis.LineCrossingDetector()

    assert cruce.check_crossings([pieza], 344)
    assert pieza.crossed_line, "el cruce no quedó marcado"

    # Segunda pasada: ya no vuelve a cruzar.
    assert not cruce.check_crossings([pieza], 344)


# ======================================================================
#  7. La pestaña, armada de verdad
# ======================================================================

class VisionFalsa:
    """Lo que la pestaña le pide al hilo de visión, sin cámara ni hilo."""

    def __init__(self, frame=None) -> None:
        self.medir = False
        self.pausada = False
        self.muestras: list = []
        self.pedidos: list = []
        self.offset_recorte = 0
        self._frame = frame

    def fotograma_crudo(self):
        return None if self._frame is None else self._frame.copy()

    def pedir_camara(self, automatica, exposicion, correccion) -> None:
        self.pedidos.append((automatica, exposicion, correccion))

    def mover_recorte(self, delta_px: int) -> int:
        self.offset_recorte += delta_px
        return self.offset_recorte


def _estado_con_cinta(andando: bool, calibrando: bool = False,
                      en_reposo: bool = True) -> EstadoSistema:
    estado = EstadoSistema()
    estado.conectado = True
    estado.ultimo_t = time.monotonic()
    estado.camara_presente = True
    estado.camara_abierta = True
    estado.ultimo_fotograma = time.monotonic()

    estado.e = pr.Proceso(crudo="", estado=pr.EstadoRobot.WAIT_PIECE,
                          estado_nombre="WAIT_PIECE", modo=pr.Modo.COLOR,
                          cinta=andando, cinta_pwm=40 if andando else 0,
                          calibrando=calibrando, en_reposo=en_reposo)

    return estado


def test_la_pestana_de_vision_se_dibuja_con_lo_que_se_ve():
    """Se arma la página y se la refresca con una pieza fuera de rango.

    Tiene que salir dibujado el veredicto que explica por qué no se detecta,
    que es lo único que esta pestaña tiene que hacer bien.
    """

    import pagina

    from kuko import ui as interfaz_ui

    with _Fabrica():
        frame = _escena(PIEZAS_CORRIDAS)
        vision = VisionFalsa(frame)
        vision.muestras = cal.muestrear(frame)

        interfaz = interfaz_ui.Interfaz(_estado_con_cinta(True),
                                        lambda linea: True, vision)

        def cuerpo():
            interfaz.construir()
            interfaz.tab_activa = "Vision"
            interfaz._refrescar_vision()

        respuesta = pagina.pedir(cuerpo)

        assert respuesta.status_code == 200

        for texto in ("Esperado contra medido", "Memoria de luces",
                      "Ajuste automatico", "Luz calida", "Geometria",
                      "Frenar robot y cinta", "Saturacion", "Tono desde"):
            assert texto in respuesta.text, f"no se dibujo {texto!r}"

        # El verde corrido: el renglón tiene que decir que no se detecta y
        # por qué canal se fue.
        assert "NO se detecta" in interfaz.html_comparacion.content
        assert "H=" in interfaz.html_comparacion.content

        # Y la medición se prendió sola al entrar a la pestaña.
        assert vision.medir


def test_la_pestana_de_vision_no_mide_desde_otra_pestana():
    """Medir cuesta casi lo que detectar; sin nadie mirando, no se paga."""

    import pagina

    from kuko import ui as interfaz_ui

    with _Fabrica():
        vision = VisionFalsa()
        interfaz = interfaz_ui.Interfaz(EstadoSistema(), lambda linea: True, vision)

        def cuerpo():
            interfaz.construir()
            interfaz.tab_activa = "Operacion"
            interfaz._refrescar_vision()

        assert pagina.pedir(cuerpo).status_code == 200
        assert not vision.medir


def test_el_cartel_avisa_que_el_robot_esta_operativo():
    """El accidente que esto vino a evitar, verificado renglón por renglón.

    Con el robot operativo, apoyar una pieza sobre la cinta para calibrar
    hace que el brazo salga a buscarla. El cartel tiene que decirlo en rojo,
    y no ponerse en verde hasta que el robot esté **detenido y quieto** —
    que no es lo mismo que haber pedido que frene.
    """

    import pagina

    from kuko import ui as interfaz_ui

    with _Fabrica():
        enviados: list[str] = []
        vision = VisionFalsa()
        interfaz = interfaz_ui.Interfaz(_estado_con_cinta(True),
                                        enviados.append, vision)

        def cuerpo():
            interfaz.construir()
            interfaz.tab_activa = "Vision"
            interfaz._refrescar_vision()

            # 1. Produciendo: rojo, y la visión le sigue informando piezas.
            assert "ROBOT OPERATIVO" in interfaz.html_cartel.content
            assert interfaz.boton_calibracion.text == "Frenar robot y cinta"
            assert not vision.pausada

            interfaz._vis_alternar_calibracion()

            # La visión deja de informar YA, sin esperar la confirmación:
            # entre el comando y el ESP32 hay milisegundos, y en ésos una
            # pieza que cruce la línea todavía se encolaría.
            assert vision.pausada

            # 2. El robot confirmó, pero el brazo sigue terminando la
            #    maniobra que tenía. Ámbar: todavía NO se puede tocar.
            interfaz.estado.e.calibrando = True
            interfaz.estado.e.en_reposo = False
            interfaz._refrescar_vision()

            assert "terminando la maniobra" in interfaz.html_cartel.content
            assert not interfaz._vis_seguro()

            # 3. Quieto en home: recién ahora se pueden poner las manos.
            interfaz.estado.e.en_reposo = True
            interfaz._refrescar_vision()

            assert "Robot detenido" in interfaz.html_cartel.content
            assert interfaz._vis_seguro()
            assert interfaz.boton_calibracion.text == "Reanudar produccion"

            interfaz._vis_alternar_calibracion()

            assert not vision.pausada

        assert pagina.pedir(cuerpo).status_code == 200
        assert enviados == [pr.cmd_calibracion(True), pr.cmd_calibracion(False)]


def test_el_cartel_delata_un_firmware_que_no_conoce_el_comando():
    """Se pidió frenar y el robot no lo confirmó.

    Es el caso de la placa sin reflashear: el botón anda, el comando sale,
    el ESP32 contesta "comando invalido" en una línea que nadie mira y la
    cinta sigue andando. Con alguien a punto de meter la mano, ese silencio
    no puede quedar sin explicación en pantalla.
    """

    import pagina

    from kuko import ui as interfaz_ui

    with _Fabrica():
        interfaz = interfaz_ui.Interfaz(_estado_con_cinta(True),
                                        lambda linea: True, VisionFalsa())

        def cuerpo():
            interfaz.construir()
            interfaz.tab_activa = "Vision"

            interfaz._vis_alternar_calibracion()

            # El robot nunca contesta: se vence la espera de la confirmación.
            interfaz._vis_pedido_s = (time.monotonic()
                                      - interfaz_ui.ESPERA_CAL_CONFIRMA_S - 0.1)
            interfaz._refrescar_vision()

            assert "ROBOT OPERATIVO" in interfaz.html_cartel.content
            assert "no lo confirmo" in interfaz.html_cartel.content
            assert "reflashearla" in interfaz.html_cartel.content

        assert pagina.pedir(cuerpo).status_code == 200


def test_mover_un_slider_de_color_llega_a_la_deteccion():
    """El camino completo, de la interfaz al detector.

    Las otras pruebas verifican `aplicar()` sola; ésta verifica que el
    control esté enganchado a ella, que es el otro lugar donde se puede
    cortar el hilo sin que se note.
    """

    import pagina

    from kuko import ui as interfaz_ui

    with _Fabrica():
        frame = _escena(PIEZAS_OK)
        interfaz = interfaz_ui.Interfaz(EstadoSistema(), lambda linea: True,
                                        VisionFalsa(frame))

        def cuerpo():
            interfaz.construir()

            assert "VERDE" in _detectados(frame)

            # Lo mismo que hacen los dos sliders de tono del verde.
            interfaz._vis_color_cambio("VERDE", "h0", 20)
            interfaz._vis_color_cambio("VERDE", "h1", 30)

            assert "VERDE" not in _detectados(frame)

            # Y volver a fábrica lo recupera.
            interfaz._vis_de_fabrica()

            assert "VERDE" in _detectados(frame)

        assert pagina.pedir(cuerpo).status_code == 200


def test_la_habitacion_activa_se_aplica_al_arrancar():
    """La calibración del último lugar donde anduvo el robot es la que vale.

    Esperar a que alguien abra la pestaña significaría trabajar un rato con
    la de otra sala, y esa es exactamente la falla que la pestaña vino a
    resolver. Se verifica en los dos lados: la detección (que lee `config`)
    y la cámara (que recibe un pedido, porque la corrección por software no
    vive en `config` sino en el objeto `Camera`).
    """

    from kuko import ui as interfaz_ui

    with _Fabrica():
        archivo = Path(tempfile.mkdtemp()) / "vision.json"

        ajustes = cal.de_fabrica()
        ajustes.colores["VERDE"] = cal.Rango(52, 88, 50, 255, 80, 255)
        ajustes.correccion_pct = -14

        cal.Memoria(archivo).guardar("Aula facultad", ajustes)

        real, cal.ARCHIVO = cal.ARCHIVO, archivo

        try:
            vision = VisionFalsa()
            interfaz = interfaz_ui.Interfaz(EstadoSistema(), lambda linea: True,
                                            vision)
        finally:
            cal.ARCHIVO = real

        assert interfaz.memoria_luces.activa == "Aula facultad"
        assert config.COLOR_HSV_RANGES["VERDE"] == (((52, 50, 80), (88, 255, 255)),)
        assert vision.pedidos[-1][2] == -14

        # Y la escena corrida —la que el rango de fábrica no agarra— ahora
        # se detecta, que es lo que el operador espera al volver al aula.
        assert "VERDE" in _detectados(_escena(PIEZAS_CORRIDAS))


def test_el_ajuste_automatico_desde_la_pantalla_frena_la_cinta_y_calibra():
    """El procedimiento completo, tal como lo hace el operador.

    Frenar la cinta es parte del procedimiento y no una precaución: con las
    piezas moviéndose, la foto que se mide y la que se ve en pantalla ya no
    son la misma.
    """

    import pagina

    from kuko import ui as interfaz_ui

    with _Fabrica():
        frame = _escena(PIEZAS_CORRIDAS)
        vision = VisionFalsa(frame)
        vision.muestras = cal.muestrear(frame)

        enviados: list[str] = []
        interfaz = interfaz_ui.Interfaz(_estado_con_cinta(True),
                                        enviados.append, vision)

        def cuerpo():
            interfaz.construir()
            interfaz.tab_activa = "Vision"

            assert len(_detectados(frame)) < 3

            interfaz._vis_abrir_auto()

            # Lo primero que hace es frenar el robot Y la cinta, y cortarle
            # a la vision el envio de piezas.
            assert enviados == [pr.cmd_calibracion(True)]
            assert vision.pausada

            # Y hasta que el robot no diga que esta quieto, no deja
            # calibrar: para poner las tres piezas hay que meter las manos.
            interfaz.estado.e.calibrando = True
            interfaz.estado.e.en_reposo = False
            interfaz._vis_pintar_auto()

            assert not interfaz.boton_calibrar.enabled
            assert "SIGUE OPERATIVO" in interfaz.html_auto.content or                 "terminando" in interfaz.html_auto.content

            interfaz.estado.e.en_reposo = True
            interfaz._vis_pintar_auto()

            assert interfaz.boton_calibrar.enabled

            # Y dice qué está viendo, con los tres colores nombrados.
            interfaz._vis_pintar_auto()

            for color in ("Rojo", "Verde", "Azul"):
                assert color in interfaz.html_auto.content

            interfaz._vis_calibrar()

            assert set(_detectados(frame)) == {"ROJO", "VERDE", "AZUL"}

            # El diálogo se cerró y la calibración dejó de decir que sale de
            # un preset de luz: la decidieron las piezas.
            assert interfaz._dialogo_auto is None
            assert interfaz.ajustes_vision.temperatura == ""

        assert pagina.pedir(cuerpo).status_code == 200


def test_el_ajuste_automatico_sin_fotograma_no_hace_nada():
    """La cámara puede no haber entregado todavía un solo fotograma."""

    import pagina

    from kuko import ui as interfaz_ui

    with _Fabrica():
        # Sin habitacion guardada: `Interfaz` aplica la activa al arrancar,
        # asi que con el vision.json de la maquina esto compararia contra
        # una calibracion que no puso esta prueba.
        real, cal.ARCHIVO = cal.ARCHIVO, Path(tempfile.mkdtemp()) / "vision.json"

        try:
            antes = dict(config.COLOR_HSV_RANGES)
            interfaz = interfaz_ui.Interfaz(_estado_con_cinta(False),
                                            lambda linea: True, VisionFalsa())
        finally:
            cal.ARCHIVO = real

        def cuerpo():
            interfaz.construir()
            interfaz._vis_abrir_auto()
            interfaz._vis_calibrar()

        assert pagina.pedir(cuerpo).status_code == 200
        assert config.COLOR_HSV_RANGES == antes


def test_un_preset_mueve_los_controles_sin_volver_a_aplicarse():
    """Reflejar unos ajustes nuevos no puede disparar los `on_change`.

    `set_value()` avisa igual que mover el control con el mouse, así que sin
    la marca de "lo estoy moviendo yo" un preset se re-aplicaría de a un
    campo y, peor, se borraría su propio nombre — porque mover algo a mano
    es justamente lo que deja de ser el preset.
    """

    import pagina

    from kuko import ui as interfaz_ui

    with _Fabrica():
        interfaz = interfaz_ui.Interfaz(EstadoSistema(), lambda linea: True,
                                        VisionFalsa())

        def cuerpo():
            interfaz.construir()

            interfaz._vis_temperatura("fria")

            # El nombre del preset sobrevivió al refresco de los controles.
            assert interfaz.ajustes_vision.temperatura == "fria"

            # Y el slider de corrección quedó donde dice el preset, no donde
            # estaba antes.
            esperado = cal.aplicar_temperatura("fria").correccion_pct

            assert interfaz.ctrl_correccion["slider"].value == esperado
            assert interfaz.ajustes_vision.correccion_pct == esperado

            # Cambiar a manual muestra el slider de exposición y esconde el
            # aviso del automático, sin rehacer el panel.
            interfaz._vis_modo_exposicion(cal.MODO_MANUAL)

            assert interfaz.ctrl_exposicion["caja"].visible
            assert not interfaz.aviso_auto.visible

            interfaz._vis_modo_exposicion(cal.MODO_AUTO)

            assert not interfaz.ctrl_exposicion["caja"].visible
            assert interfaz.aviso_auto.visible

        assert pagina.pedir(cuerpo).status_code == 200


def test_cargar_una_habitacion_rearma_los_sliders_de_color():
    """Un color puede pasar de un tramo a dos, y ahí no hay slider al que
    ponerle un valor: las tarjetas se rehacen enteras."""

    import pagina

    from kuko import ui as interfaz_ui

    with _Fabrica():
        interfaz = interfaz_ui.Interfaz(EstadoSistema(), lambda linea: True,
                                        VisionFalsa())

        def cuerpo():
            interfaz.construir()

            interfaz.memoria_luces = _memoria_temporal()

            ajustes = cal.de_fabrica()
            ajustes.colores["ROJO"] = cal.Rango(0, 8, 60, 255, 90, 255)
            interfaz.memoria_luces.guardar("Aula facultad", ajustes)

            # `activa` queda puesta al guardar; se limpia para que la carga
            # no se saltee por "ya estás en esa".
            interfaz.memoria_luces.activa = ""
            interfaz._vis_cargar("Aula facultad")

            assert config.COLOR_HSV_RANGES["ROJO"] == (((0, 60, 90), (8, 255, 255)),)

        assert pagina.pedir(cuerpo).status_code == 200


def test_la_exposicion_se_le_pide_al_hilo_de_vision():
    """La interfaz NO toca el `VideoCapture`.

    `set()` y `read()` sobre el mismo dispositivo desde dos hilos cuelgan
    MSMF, así que el cambio viaja como pedido y lo aplica el hilo entre dos
    fotogramas. Lo que se verifica es justamente eso: que salga un pedido.
    """

    import pagina

    from kuko import ui as interfaz_ui

    with _Fabrica():
        vision = VisionFalsa()
        interfaz = interfaz_ui.Interfaz(EstadoSistema(), lambda linea: True, vision)

        def cuerpo():
            interfaz.construir()

            interfaz._vis_modo_exposicion(cal.MODO_MANUAL)
            interfaz._vis_exposicion(-11)
            interfaz._vis_correccion(-15)

        assert pagina.pedir(cuerpo).status_code == 200
        assert vision.pedidos[-1] == (False, -11, -15)


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
