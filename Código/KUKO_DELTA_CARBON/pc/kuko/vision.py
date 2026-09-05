"""Hilo de vision: camara, deteccion, seguimiento y envio de piezas.

Es el UNICO bucle de vision del sistema. No abre ninguna ventana de
OpenCV: deja el ultimo fotograma anotado en memoria, codificado a JPEG,
para que el servidor lo sirva como MJPEG.

Reutiliza los modulos de vision que ya existen y estan calibrados
(vision_python/): camara, deteccion de color y forma, seguimiento y cruce de
linea. No se copiaron aca a proposito -- son los que andan.
"""

from __future__ import annotations

import statistics
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import cv2

# Los modulos de vision viven en vision_python/, al lado de pc/. La ruta se
# arma desde este archivo y no desde el directorio de trabajo, para que ande
# igual en cualquier maquina y desde cualquier lado que se lo invoque.
RAIZ = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RAIZ / "vision_python"))

import config                                     # noqa: E402
from camera import Camera                         # noqa: E402
from coordinates import pixels_to_robot_cm        # noqa: E402
from detection import detect_objects              # noqa: E402
from line_crossing import LineCrossingDetector, draw_detection_line  # noqa: E402
from tracker import CentroidTracker, TrackedObject                    # noqa: E402

from . import ajustes
from . import calibracion as cal
from . import estado as est_mod
from . import protocolo as pr
from .estado import EstadoSistema

COLORES_DIBUJO = {"ROJO": (0, 0, 255), "AZUL": (255, 0, 0), "VERDE": (0, 200, 0)}
COLOR_POR_DEFECTO = (255, 255, 255)

# Muestras de velocidad de cinta que se promedian. Con la mediana de 15
# muestras el ruido de un fotograma suelto no mueve el numero.
MUESTRAS_CINTA = 15

# Cuando se da por perdida la camara lo decide `EstadoSistema.camara_viva()`,
# con el vencimiento de estado.py: el mismo dato que dibuja el punto rojo de
# la pantalla tiene que ser el que decide reabrirla, o el punto y el hilo
# terminan opinando distinto. Hace falta porque una camara USB desenchufada
# NO da error -- `VideoCapture.read()` devuelve False para siempre --, y el
# bucle viejo se quedaba dando vueltas en silencio con la ultima imagen
# congelada y los FPS clavados en el ultimo valor bueno, que es la peor
# forma posible de fallar: todo parece andar.

# Cuanto se espera antes de reintentar abrirla, y hasta cuanto crece esa
# espera si sigue sin aparecer. Mismo criterio que el enlace serie
# (RETARDO_RECONEXION_S): reintentar sin pausa contra un dispositivo que no
# esta solo gasta CPU. La espera crece porque cada intento fallido escupe
# tres lineas de aviso de OpenCV por la salida de error -- no las imprime
# este programa y no se pueden apagar sin apagar tambien las utiles --, y a
# un intento cada tres segundos eso tapa la consola en un minuto.
RETARDO_REAPERTURA_S = 3.0
RETARDO_REAPERTURA_MAX_S = 20.0

# Cada cuanto se le informa el estado al historial mientras la camara NO
# esta andando. Con la camara andando informa cada fotograma; sin ella hay
# que seguir diciendo "sigo sin imagen" o el historial no podria distinguir
# una camara caida de un hilo de vision muerto.
PERIODO_AVISO_S = 0.5

# Cada cuanto se mide el color real de las piezas mientras la pestana de
# Vision esta a la vista. Es una conversion a HSV y una busqueda de
# contornos mas por medicion, o sea casi lo mismo que cuesta un fotograma
# entero de deteccion: a cada fotograma seria pagar el doble para mirar un
# numero que no cambia tan rapido. Con la cinta parada --que es como se
# calibra-- dos por segundo sobra.
PERIODO_MEDICION_S = 0.5


def _dibujar(frame, track: TrackedObject, line_x: int) -> None:
    color = COLORES_DIBUJO.get(track.color, COLOR_POR_DEFECTO)

    # Casco convexo y no contorno crudo: es lo mismo que mira
    # classify_shape(), asi que lo que se ve coincide con lo clasificado.
    cv2.drawContours(frame, [cv2.convexHull(track.contour)], -1, color, 3)

    x, y, _, alto = track.bbox
    cv2.circle(frame, track.center, 6, (0, 255, 0), -1)

    alto_img, ancho_img = frame.shape[:2]
    x_cm, y_cm = pixels_to_robot_cm(track.center, ancho_img, alto_img, line_x)

    cv2.putText(frame, f"ID {track.track_id} | {track.shape} {track.color}",
                (x, max(y - 35, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.putText(frame, f"X:{x_cm:.1f} cm Y:{y_cm:.1f} cm",
                (x, max(y - 12, 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 2)

    if track.crossed_line:
        cv2.putText(frame, "CRUZO LA LINEA", (x, y + alto + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)


class Vision:
    def __init__(self, estado: EstadoSistema, enviar):
        self.estado = estado
        self.enviar = enviar          # callable(str) -> bool, del enlace

        # Que la camara "esta prevista" lo decide la existencia de este
        # objeto: con --sin-vision no se construye ninguno, y ahi no
        # corresponde ni un punto rojo ni una alarma. Es distinto de que la
        # camara este prevista y no aparezca, que si es una falla.
        estado.camara_presente = True

        self._parar = threading.Event()
        self._hilo: Optional[threading.Thread] = None
        self._jpeg: Optional[bytes] = None
        self._lock = threading.Lock()

        self.piezas_vistas = 0
        self._camara = None

        # El recorte arranca donde quedo la ultima vez, no en un valor
        # predefinido: es lo que se pidio expresamente para no tener que
        # recentrar la camara en cada arranque.
        self._offset_pedido = int(ajustes.cargar().get("recorte_y_px", 0))

        # Para medir la velocidad de la cinta: ultima posicion conocida de
        # cada pieza seguida, y las ultimas muestras de velocidad.
        self._ultima_pos: dict[int, tuple[float, float]] = {}
        self._muestras: list[float] = []

        self._ultimo_aviso = 0.0
        self._ultimo_viva: Optional[bool] = None

        # Veniamos sin imagen. Sirve para dos cosas que no se pueden decidir
        # mirando un instante suelto: contar una reconexion solo cuando la
        # camara VUELVE (y no en cada reintento contra una que no esta), y
        # avisar por consola una vez por corte en vez de una cada dos
        # segundos mientras dure.
        self._caida = False

        # `time.monotonic()` en que se abrio el dispositivo. Es la referencia
        # para dar por perdida una camara que abrio pero nunca entrego nada.
        self._abierta_en = 0.0

        # ---------------- Calibracion (pestana de Vision) --------------
        # Todo lo de aca abajo esta APAGADO salvo con esa pestana a la
        # vista: guardar una copia del fotograma y medirle el color a cada
        # pieza cuesta casi tanto como detectarla, y con el robot
        # produciendo no lo mira nadie.
        self.medir = False

        #: Modo calibracion: NO se le informa ninguna pieza al robot.
        #:
        #: Es la primera de las dos defensas contra el accidente que motivo
        #: todo esto: con la cinta parada y alguien apoyando piezas a mano
        #: para calibrar, cada pieza apoyada CRUZA la linea de deteccion --
        #: la cruza la mano, no la cinta -- y el brazo salia a buscarla.
        #:
        #: La segunda defensa esta en el firmware (`calibrando`), y las dos
        #: hacen falta: esta corta el mensaje en el origen, aquella protege
        #: contra una PC vieja, un navegador que quedo abierto o alguien
        #: mandando una pieza a mano por el monitor serie.
        self.pausada = False

        #: Ultimo fotograma SIN anotar. Tiene que ser sin anotar: sobre el
        #: dibujado, el contorno verde y el texto blanco que se le pintan
        #: encima entran a la medicion de color como si fueran la pieza.
        self._crudo: Optional[object] = None

        #: Lo ultimo que se midio de cada pieza a la vista, y cuando.
        self.muestras: list = []
        self.medido_en = 0.0
        self._ultima_medicion = 0.0

        #: Cambio de exposicion pedido desde la interfaz, todavia sin
        #: aplicar. NO se aplica desde el hilo de la interfaz: `set()` y
        #: `read()` sobre el mismo VideoCapture desde dos hilos cuelgan
        #: MSMF. Lo toma el bucle entre dos fotogramas, que es el unico
        #: momento en que nadie esta leyendo.
        self._camara_pedida: Optional[tuple] = None
        self.correccion_pct = 0.0

        #: Ultimo error avisado por consola, para no repetirlo cada vuelta.
        self._error_camara = ""
        self._error_medicion = ""

    def arrancar(self) -> None:
        self._hilo = threading.Thread(target=self._correr, daemon=True, name="vision")
        self._hilo.start()

    def parar(self) -> None:
        self._parar.set()

        if self._hilo:
            self._hilo.join(timeout=3.0)

    def fotograma(self) -> Optional[bytes]:
        with self._lock:
            return self._jpeg

    def mover_recorte(self, delta_px: int) -> int:
        """Corre el area recortada hacia arriba o abajo y lo deja guardado.

        El alto no cambia, solo se desplaza la ventana: la escala pixel->cm
        se mantiene y por lo tanto tambien las coordenadas que se le mandan
        al robot.
        """

        if self._camara is None:
            return self._offset_pedido

        self._offset_pedido = self._camara.mover_recorte(delta_px)
        ajustes.guardar("recorte_y_px", self._offset_pedido)

        return self._offset_pedido

    @property
    def offset_recorte(self) -> int:
        return self._camara.offset_y if self._camara else self._offset_pedido

    # ==================================================================
    #  Calibracion: lo que usa la pestana de Vision
    # ==================================================================
    def fotograma_crudo(self):
        """El ultimo fotograma sin anotar, o None.

        Solo hay uno mientras `medir` este encendido: guardarlo cuesta una
        copia por fotograma y con la pestana de Vision cerrada no lo mira
        nadie.
        """

        with self._lock:
            return None if self._crudo is None else self._crudo.copy()

    def pedir_camara(self, automatica: bool, exposicion: float,
                     correccion: float) -> None:
        """Deja pedido un cambio de exposicion; lo aplica el hilo.

        Es un PEDIDO y no una aplicacion directa a proposito: ver
        `_camara_pedida` en el constructor. Se pisa el pedido anterior si
        todavia no se aplico -- arrastrando un slider salen veinte pedidos
        por segundo y el unico que importa es el ultimo.
        """

        self.correccion_pct = float(correccion)
        self._camara_pedida = (bool(automatica), float(exposicion))

    def _aplicar_camara(self) -> None:
        camara = self._camara

        if camara is None:
            # SIN consumir el pedido: la interfaz puede haber movido la
            # exposicion con la camara desenchufada, y ese ajuste tiene que
            # sobrevivir hasta que vuelva a abrir.
            return

        # La correccion se reescribe siempre y no solo cuando cambia: vive
        # en el objeto `Camera`, y una camara que se desenchufa y vuelve es
        # un objeto NUEVO, que arranca sin ella.
        camara.correccion_pct = self.correccion_pct

        pedido, self._camara_pedida = self._camara_pedida, None

        if pedido is None:
            return

        automatica, exposicion = pedido

        try:
            camara.aplicar_exposicion(automatica, exposicion)
        except Exception as err:                      # noqa: BLE001
            # Una camara que no acepta el modo de exposicion no es motivo
            # para matar el hilo entero: se sigue con la que tenia y se
            # avisa una vez.
            if self._error_camara != str(err):
                self._error_camara = str(err)
                self.estado.consola.append(
                    f"la camara rechazo la exposicion: {err}")

    def _medir(self, frame) -> None:
        """Mide el color real de las piezas del fotograma, cada tanto."""

        ahora = time.monotonic()

        if ahora - self._ultima_medicion < PERIODO_MEDICION_S:
            return

        self._ultima_medicion = ahora

        try:
            muestras = cal.muestrear(frame)
        except Exception as err:                      # noqa: BLE001
            # Una vez por racha y no una cada medio segundo: la pestana de
            # Vision queda abierta, asi que un error que se repite llenaria
            # la consola y taparia todo lo demas.
            if self._error_medicion != str(err):
                self._error_medicion = str(err)
                self.estado.consola.append(f"no se pudo medir el color: {err}")

            return

        self._error_medicion = ""
        self.muestras = muestras
        self.medido_en = ahora

    # ==================================================================
    #  Bucle principal
    # ==================================================================
    #
    #  Dos bucles, uno adentro del otro, igual que el enlace serie: el de
    #  afuera abre la camara y la vuelve a abrir cuando se pierde, el de
    #  adentro procesa fotogramas mientras haya. Antes habia uno solo, y de
    #  ahi salian los dos modos de fallar en silencio que esto arregla:
    #
    #    * si la camara no abria al arrancar, el hilo se moria y no habia
    #      forma de recuperarlo sin reiniciar el programa entero;
    #    * si se desenchufaba andando, `read()` devolvia False para siempre
    #      y el bucle giraba sin hacer nada, con la ultima imagen congelada
    #      en la pantalla y los FPS clavados en el ultimo valor bueno.
    #
    #  Reabrirla hace falta de verdad y no es un lujo: cuando se desenchufa
    #  un USB, el `VideoCapture` queda muerto, y vuelve a andar recien con
    #  un `VideoCapture` nuevo. Sin esto, volver a enchufar la camara no
    #  arregla nada.

    def _correr(self) -> None:
        espera = RETARDO_REAPERTURA_S

        while not self._parar.is_set():
            if not self._abrir():
                self._esperar(espera)
                espera = min(espera * 2.0, RETARDO_REAPERTURA_MAX_S)
                continue

            # Abrio: la proxima caida vuelve a reintentar rapido. Lo que se
            # quiere espaciar es el machaque contra una camara que no esta,
            # no la recuperacion de una que se desenchufo un momento.
            espera = RETARDO_REAPERTURA_S

            self._procesar()
            self._cerrar()

    def _esperar(self, segundos: float) -> None:
        """Espera avisando: el historial tiene que seguir viendo el hueco."""

        limite = time.monotonic() + segundos

        while not self._parar.is_set() and time.monotonic() < limite:
            self._avisar()
            self._parar.wait(0.2)

    def _abrir(self) -> bool:
        est = self.estado

        try:
            camara = Camera()
        except Exception as err:                      # noqa: BLE001
            # A la consola una sola vez por caida, no cada tres segundos:
            # un mensaje repetido cada tres segundos tapa todo lo demas y
            # termina haciendo que nadie mire la consola.
            if not est.camara_error:
                est.consola.append(f"no se pudo abrir la camara: {err}")

            est.camara_abierta = False
            est.camara_error = str(err)
            self._caida = True
            self._avisar(forzar=True)

            return False

        camara.offset_y = self._offset_pedido
        self._camara = camara
        self._abierta_en = time.monotonic()

        est.camara_abierta = True
        est.camara_backend = camara.backend_name
        est.camara_error = ""

        # `ultimo_fotograma` NO se toca aca, y es lo que hace que esto ande:
        # abrir el dispositivo no es tener imagen. Marcarlo como si acabara
        # de llegar un fotograma -- para que el arranque no se viera rojo --
        # hacia que una camara desenchufada, que se reabre igual porque el
        # `VideoCapture` se construye sin quejarse, se pusiera verde dos
        # segundos en cada reintento: el punto parpadeaba y el historial se
        # llenaba de pares "se cayo"/"volvio" que no eran ninguna de las dos.
        # El arranque se ve gris ("abriendo") hasta el primer fotograma, que
        # es lo que de verdad esta pasando.
        return True

    def _cerrar(self) -> None:
        camara, self._camara = self._camara, None
        self.estado.camara_abierta = False

        # Lo guardado deja de ser "lo que se ve": calibrar contra la ultima
        # imagen de antes de que se cayera la camara es exactamente el modo
        # de fallar que evita todo el resto de este archivo.
        with self._lock:
            self._crudo = None

        self.muestras = []

        if camara is not None:
            try:
                camara.release()
            except Exception:                         # noqa: BLE001
                pass

    def _avisar(self, forzar: bool = False) -> None:
        """Le pasa al historial el estado de la camara y el conteo de fotogramas.

        Se manda el CONTADOR y no los FPS a proposito: el promedio de los
        ultimos cinco minutos sale de restar dos lecturas del contador y
        dividir por el tiempo entre ellas, y asi los ratos sin imagen bajan
        el promedio solos, sin ningun caso especial. Mandando una tasa ya
        calculada habria que acordarse de mandar ceros mientras no hay
        camara, y ese es justo el momento en que nadie manda nada.
        """

        ahora = time.monotonic()
        est = self.estado
        viva = est.camara_viva()

        # Un CAMBIO de estado se manda en el acto y no espera el periodo. Dos
        # motivos: la hora del aviso es la que va a quedar en la cronologia,
        # y ese renglon se lee contra la linea de tiempo del robot, asi que
        # medio segundo de atraso corre el corte de lugar; y sin esto el
        # periodo de aviso y el vencimiento de la camara quedan atados en
        # silencio -- un corte mas corto que el periodo no se anotaria nunca.
        if not forzar and viva == self._ultimo_viva                 and ahora - self._ultimo_aviso < PERIODO_AVISO_S:
            return

        self._ultimo_aviso = ahora
        self._ultimo_viva = viva

        est.rendimiento.observar_camara(
            viva=viva,
            fotogramas=est.fotogramas,
            detalle=est.camara_error)

    # ------------------------------------------------------------------
    def _procesar(self) -> None:
        camara = self._camara
        est = self.estado

        tracker = CentroidTracker()
        cruce = LineCrossingDetector()
        anterior = time.perf_counter()

        while not self._parar.is_set():
            # Entre dos fotogramas, que es el unico momento en que nadie
            # esta leyendo el dispositivo (ver `pedir_camara`).
            self._aplicar_camara()

            ok, frame = camara.read()

            if not ok:
                # Una lectura fallida suelta no es nada -- pasa con
                # cualquier camara USB --; lo que importa es la RACHA. Se
                # cuenta desde el ultimo fotograma o desde que se abrio el
                # dispositivo, lo que sea mas reciente: una camara que abre
                # y no entrega nunca nada tambien tiene que vencer.
                self._avisar()

                desde = max(est.ultimo_fotograma, self._abierta_en)

                if time.monotonic() - desde > est_mod.CAMARA_TIMEOUT_S:
                    if not self._caida:
                        est.consola.append("la camara dejo de entregar imagen")

                    self._caida = True
                    return

                self._parar.wait(0.05)
                continue

            if self._caida:
                # Volvio. Se cuenta aca y no al abrir el dispositivo porque
                # abrirlo no prueba nada: contra una camara desenchufada, el
                # `VideoCapture` se construye igual y el contador subiria en
                # cada reintento. Varias reconexiones en un turno son un
                # cable flojo, y ese numero tiene que significar eso.
                self._caida = False
                est.reconexiones_camara += 1
                est.consola.append("la camara volvio")

            est.fotogramas += 1
            est.ultimo_fotograma = time.monotonic()

            alto, ancho = frame.shape[:2]
            line_x = int(ancho * config.LINE_X_RATIO)

            # La copia se toma ACA, antes de dibujar nada encima: el
            # contorno y el texto que se le pintan a cada pieza son pixeles
            # saturados y entrarian a la medicion de color como si fueran
            # parte de la pieza.
            if self.medir:
                with self._lock:
                    self._crudo = frame.copy()

                self._medir(frame)

            detecciones, _ = detect_objects(frame)
            seguidas = tracker.update(detecciones)

            self._medir_cinta(seguidas, ancho, alto, line_x)

            for track in cruce.check_crossings(seguidas, line_x):
                # El cruce se consume igual aunque no se informe: la marca
                # `crossed_line` del tracker tiene que quedar puesta o al
                # salir de calibracion se avisaria de golpe cada pieza que
                # quedo del lado de alla de la linea.
                if self.pausada:
                    continue

                _, y_cm = pixels_to_robot_cm(track.center, ancho, alto, line_x)

                self.piezas_vistas += 1
                self.enviar(pr.cmd_pieza(y_cm, track.color, track.shape))

            draw_detection_line(frame, line_x)

            for track in seguidas:
                _dibujar(frame, track, line_x)

            ahora = time.perf_counter()
            dt = ahora - anterior
            anterior = ahora
            est.fps_camara = 1.0 / dt if dt > 0 else 0.0

            self._avisar()

            ok, buffer = cv2.imencode(".jpg", frame,
                                      [int(cv2.IMWRITE_JPEG_QUALITY), 80])

            if ok:
                with self._lock:
                    self._jpeg = buffer.tobytes()

    def _medir_cinta(self, seguidas, ancho: int, alto: int, line_x: int) -> None:
        """Velocidad real de la cinta, siguiendo cuanto avanza cada pieza.

        Es el unico chequeo que puede decir si la cinta se trabo: no hay
        sensor en el eje. De paso valida la constante de la que depende toda
        la intercepcion.
        """

        ahora = time.perf_counter()
        vistos = set()

        for track in seguidas:
            x_cm, _ = pixels_to_robot_cm(track.center, ancho, alto, line_x)
            vistos.add(track.track_id)

            previo = self._ultima_pos.get(track.track_id)
            self._ultima_pos[track.track_id] = (ahora, x_cm)

            if previo is None:
                continue

            dt = ahora - previo[0]

            # Ventana chica: por debajo de 0,2 s el ruido del centroide
            # domina sobre el avance real.
            if dt < 0.2:
                self._ultima_pos[track.track_id] = previo
                continue

            self._muestras.append((x_cm - previo[1]) / dt)
            self._muestras[:] = self._muestras[-MUESTRAS_CINTA:]

        for perdido in set(self._ultima_pos) - vistos:
            del self._ultima_pos[perdido]

        self.estado.cinta_medida = (statistics.median(self._muestras)
                                    if len(self._muestras) >= 5 else None)
