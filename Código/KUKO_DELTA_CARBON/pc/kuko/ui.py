"""Interfaz NiceGUI. No toca hardware: lee el estado y manda comandos.

La pantalla se refresca sola: un temporizador a 10 Hz redibuja lo que se
mueve (diales, finales, ventosa) y otro mas lento el resto. NiceGUI manda
por websocket solo lo que cambio, asi que no hay recarga de pagina.

La pestana de operacion entra ENTERA en la ventana, sin scroll: es una
pantalla para mirar de reojo con el robot andando, y algo que hay que
scrollear para ver completo no sirve para eso.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional

from fastapi import Response
from fastapi.responses import StreamingResponse
from nicegui import app, ui

from . import cinematica as cin
from . import parametros as par
from . import protocolo as pr
from . import teach as tch
from .estado import AMBAR, GRIS, ROJO, VERDE, EstadoSistema

FONDO = "#14171C"
PANEL = "#1B1F26"
BORDE = "#2A313B"
TEXTO = "#E6EAF0"
APAGADO = "#8A94A6"
CELESTE = "#38BDF8"
INACTIVO = "#252B34"
ROJO_STOP = "#E5484D"
COLOR_ESTADO = {VERDE: "#3DD68C", AMBAR: "#F5B942", ROJO: "#FF5C5C", GRIS: "#4B5563"}

COLOR_PIEZA = {"R": "#E5484D", "G": "#3DD68C", "B": "#3B82F6"}

# Cuanto se corre el recorte por click. 4 px es fino para recentrar sin
# volverse eterno manteniendo apretado.
PASO_RECORTE = 4

# Escala de toda la interfaz. Es exactamente lo mismo que poner el zoom del
# navegador en 110 %, pero de fabrica: a tamaño 1,0 los paneles quedan chicos
# para la pantalla en la que se usa esto. Si en otra pantalla queda grande o
# chico, este es el unico numero a tocar.
ZOOM = 1.1

DIAL_MIN, DIAL_MAX = -70.0, 30.0

# Cada cuanto corre el lazo del modo teach: manda la direccion del jog y, si
# se esta grabando, toma una muestra. 20 Hz es lo mismo a lo que el firmware
# vuelca la posicion comandada, asi que no se pierde ni se repite ninguna.
PERIODO_TEACH_S = 0.05

# Cada cuanto se refresca la direccion del jog aunque no haya cambiado. Tiene
# que ser bastante mas rapido que la vigencia que le da el firmware (350 ms),
# porque esa vigencia es lo que frena el brazo si la interfaz se muere.
REFRESCO_JOG_S = 0.10

# Resolucion con la que se pinta la zona alcanzable del plano XY. La cuenta
# es una cinematica inversa por celda, asi que se guarda en cache por altura
# redondeada: con el jog subiendo y bajando, recalcularla en cada refresco
# seria el unico gasto serio de CPU de toda la interfaz.
COLUMNAS_ALCANCE = 33
FILAS_ALCANCE = 48
PASO_CACHE_Z = 0.25

# Lienzo del plano isometrico del modo teach. Es una relacion de aspecto
# fija, y el panel que lo contiene se dimensiona con ella (ver `_teach`): si
# el panel fuera mas ancho, el SVG se centraria adentro y las dos franjas que
# quedan al costado son ancho de pantalla tirado.
ANCHO_PLANO, ALTO_PLANO = 640.0, 470.0

# Lado del joystick en pixeles. Lo necesitan el div, el dibujo y la cuenta
# que pasa de pixeles a direccion, asi que va en un solo lugar.
LADO_JOYSTICK = 150.0

# Cuanto se da por buena la marca de "hay un ir a una coordenada en curso"
# sin noticias del firmware. El movimiento real son uno o dos segundos; esto
# es el vencimiento por si se pierde el evento de llegada, y por eso es tan
# holgado: sobra para cualquier recorrido y no deja la pantalla trabada.
ESPERA_IR_S = 20.0

# Lo que se espera la CONFIRMACION del firmware ('[TEACH] ir'). Es corto a
# proposito: un pedido que no se confirma no es un movimiento largo, es un
# firmware que no conoce el comando, y ahi la pantalla tiene que volver sola.
ESPERA_IR_CONFIRMA_S = 1.5


def _svg_dial(indice: int, comandado: Optional[float], medido: Optional[float]) -> str:
    """Un dial: sector celeste = angulo comandado, aguja amarilla = encoder."""

    cx = cy = 60
    r = 46

    def punto(grados: float, radio: float) -> tuple[float, float]:
        # 0 grados = horizontal a la derecha; los negativos van hacia abajo,
        # que es como cuelga el brazo del delta.
        rad = math.radians(-grados)
        return cx + radio * math.cos(rad), cy + radio * math.sin(rad)

    partes = [
        '<svg viewBox="0 0 120 124" style="width:100%;height:auto">',
        f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="none" stroke="{BORDE}" stroke-width="2"/>',
        f'<line x1="{cx - r}" y1="{cy}" x2="{cx + r}" y2="{cy}" stroke="{BORDE}"'
        ' stroke-width="1" stroke-dasharray="3 3"/>',
    ]

    if comandado is not None:
        limite = max(DIAL_MIN, min(DIAL_MAX, comandado))
        x0, y0 = punto(0, r - 6)
        x1, y1 = punto(limite, r - 6)

        partes.append(
            f'<path d="M {cx} {cy} L {x0:.1f} {y0:.1f} '
            f'A {r - 6} {r - 6} 0 0 {1 if limite < 0 else 0} {x1:.1f} {y1:.1f} Z" '
            f'fill="{CELESTE}" fill-opacity="0.30" stroke="{CELESTE}" stroke-width="1.5"/>')

    # Un valor fuera de la escala del dial se recortaba sin decir nada, y la
    # aguja clavada en el tope se lee como "el brazo esta ahi". Fue justo lo
    # que tapo un angulo de 355 grados que llegaba de un encoder mal
    # filtrado: el numero decia la verdad y la aguja decia otra cosa. Ahora
    # el numero se pinta en ambar cuando esta fuera de escala.
    fuera_de_escala = medido is not None and not (DIAL_MIN <= medido <= DIAL_MAX)

    if medido is not None:
        x, y = punto(max(DIAL_MIN, min(DIAL_MAX, medido)), r - 4)
        partes.append(f'<line x1="{cx}" y1="{cy}" x2="{x:.1f}" y2="{y:.1f}" '
                      f'stroke="{"#F5B942" if fuera_de_escala else "#F5D442"}" '
                      f'stroke-width="3" stroke-linecap="round" '
                      f'stroke-dasharray="{"4 3" if fuera_de_escala else "none"}"/>')

    partes.append(f'<circle cx="{cx}" cy="{cy}" r="4" fill="{TEXTO}"/>')
    partes.append(f'<text x="{cx}" y="15" fill="{APAGADO}" font-size="13" '
                  f'text-anchor="middle" font-family="system-ui">{indice}</text>')
    partes.append(f'<text x="{cx}" y="119" '
                  f'fill="{COLOR_ESTADO[AMBAR] if fuera_de_escala else TEXTO}" '
                  f'font-size="14" text-anchor="middle" font-family="system-ui">'
                  f'{f"{medido:.1f}°" if medido is not None else "--"}</text>')
    partes.append("</svg>")

    return "".join(partes)


def _svg_ventosa(activa: bool) -> str:
    """El rectangulo de arriba se pinta de celeste cuando la bomba esta on."""

    trazo = CELESTE if activa else BORDE

    return f"""<svg viewBox="0 0 120 70" style="width:100%;height:auto">
      <rect x="22" y="2" width="76" height="18" rx="3"
            fill="{CELESTE if activa else 'none'}" fill-opacity="{0.75 if activa else 0}"
            stroke="{trazo}" stroke-width="2"/>
      <path d="M 38 20 L 38 40 L 24 62 L 96 62 L 82 40 L 82 20"
            fill="none" stroke="{trazo}" stroke-width="2" stroke-linejoin="round"/>
    </svg>"""


def _svg_finales(finales: list[bool]) -> str:
    """Tres rectangulos con su etiqueta al lado: || FC1  || FC2  || FC3."""

    partes = []

    for i in range(3):
        pisado = finales[i] if i < len(finales) else False
        x = 3 + i * 63

        partes.append(
            f'<rect x="{x}" y="3" width="19" height="50" rx="2" '
            f'fill="{CELESTE if pisado else "none"}" fill-opacity="0.75" '
            f'stroke="{CELESTE if pisado else BORDE}" stroke-width="2"/>'
            f'<text x="{x + 26}" y="34" fill="{APAGADO}" font-size="13" '
            f'font-family="system-ui">FC{i + 1}</text>')

    return f'<svg viewBox="0 0 190 56" style="width:100%;height:auto">{"".join(partes)}</svg>'


def _svg_forma(codigo: str, color: str) -> str:
    """Cuadrado, hexagono o circulo dibujados, en vez de la letra."""

    if codigo == "S":
        figura = (f'<rect x="6" y="6" width="24" height="24" rx="2" fill="none" '
                  f'stroke="{color}" stroke-width="2.2"/>')
    elif codigo == "C":
        figura = f'<circle cx="18" cy="18" r="13" fill="none" stroke="{color}" stroke-width="2.2"/>'
    else:
        # El hexagono se dibuja con el radio al VERTICE, asi que con el
        # mismo radio que el circulo se ve mas chico: entre dos caras mide
        # un 13 % menos. Se compensa agrandando el radio, y se lo apoya
        # sobre una cara plana (arranca en 30 grados) para que se lea como
        # hexagono y no como algo torcido.
        puntos = " ".join(
            f"{18 + 14.9 * math.cos(math.radians(a)):.1f},"
            f"{18 + 14.9 * math.sin(math.radians(a)):.1f}"
            for a in range(30, 390, 60))
        figura = f'<polygon points="{puntos}" fill="none" stroke="{color}" stroke-width="2.2"/>'

    return f'<svg viewBox="0 0 36 36" style="width:34px;height:34px">{figura}</svg>'


def _archivo_portada() -> str:
    """Ruta del archivo de portada, sea cual sea su formato.

    Asi se puede dejar caer la imagen como PNG, JPG o SVG sin tocar codigo.
    """

    carpeta = Path(__file__).resolve().parents[1] / "assets"

    for extension in ("png", "svg", "jpg", "jpeg", "webp"):
        if (carpeta / f"portada.{extension}").exists():
            return f"/assets/portada.{extension}"

    return "/assets/portada.png"


def _punto(estado: str) -> str:
    return (f'<span style="display:inline-block;width:10px;height:10px;border-radius:50%;'
            f'background:{COLOR_ESTADO[estado]};margin-right:9px"></span>')


class Interfaz:
    def __init__(self, estado: EstadoSistema, enviar, vision=None):
        self.estado = estado
        self.enviar = enviar
        self.vision = vision

        # Disposicion de la caja que se esta editando. Solo viaja al apretar
        # Confirmar: mandarla tecla por tecla generaria disposiciones
        # intermedias invalidas (cuatro rojos, por ejemplo).
        self.layout_editado: list[str] = []

        # Contadores: el firmware los lleva desde que arranco, y lo que se
        # muestra es lo producido DESDE EL ULTIMO CAMBIO DE MODO. Se guarda
        # la foto del contador en ese instante y se resta.
        self.base_color: dict[str, int] = {}
        self.base_forma: dict[str, int] = {}
        self.modo_anterior: Optional[pr.Modo] = None

        self.caja_avisada = False
        self._dialogo_caja = None
        self._repeticion = None

        # ---------------- Modo teach ----------------
        # Las secuencias viven en disco (pc/config/movimientos.json): son el
        # producto de una sesion de ensenanza y tienen que sobrevivir a que
        # se cierre el programa.
        self.biblioteca = tch.Biblioteca()

        # Que pestana se esta mirando. El teclado del jog es global -- no hay
        # forma de escuchar teclas "solo dentro de un panel" -- asi que la
        # pestana activa es lo que decide si una W mueve el brazo o no.
        self.tab_activa = "Operacion"

        self.teach_teclas: set[str] = set()
        self.joy_x = 0.0
        self.joy_y = 0.0
        self.joy_tomado = False

        self.jog_ultimo = (0.0, 0.0, 0.0)
        self.jog_enviado_s = 0.0
        self._volcado_on = False

        self.teach_grabando = False
        self.teach_muestras: list[tch.Muestra] = []
        self.teach_t0 = 0.0

        self.teach_sel: Optional[int] = None
        self.teach_mov_en_curso: Optional[tch.Movimiento] = None
        self.teach_pct_en_curso = 0

        # Hay un "ir a una coordenada" en curso. El firmware no lo informa en
        # la telemetría periódica -- son uno o dos segundos --, así que se
        # sigue por los eventos `[TEACH] ir` / `irfin`. Por si alguno se
        # pierde, la marca vence sola: el peor caso es un botón que se
        # rehabilita tarde, no uno que no se rehabilita nunca.
        self.teach_yendo_hasta = 0.0

        self._teach_evento_visto = 0
        self._cola_subida: list = []
        self._timer_subida = None
        self._filas_teach: list[dict] = []
        self._cache_alcance: tuple = (None, [])
        self._dialogo_teach = None

        # Ajustes: los controles se arman con lo que contesta 'P?', asi que
        # no existen hasta que llega. `_firma` guarda que parametros habia la
        # ultima vez para rearmar la lista SOLO cuando cambia el conjunto --
        # rearmarla en cada refresco perderia el foco del teclado a mitad de
        # tipear un numero.
        self._firma: dict[int, tuple] = {}
        self._nav: dict[int, object] = {}
        self._lista: dict[int, object] = {}
        self._pie_texto: dict[int, object] = {}
        self._filas: dict[str, dict] = {}

        self._registrar_rutas()

    # ------------------------------------------------------------------
    def _registrar_rutas(self) -> None:
        # Carpeta de imagenes de la interfaz (la portada con el logo y los
        # nombres). Se sirve estatica para poder cambiar el archivo sin
        # tocar codigo ni reiniciar nada.
        app.add_static_files("/assets", str(Path(__file__).resolve().parents[1] / "assets"))

        limite = "--kukoframe"

        def generar():
            import time

            while True:
                jpeg = self.vision.fotograma() if self.vision else None

                if jpeg:
                    yield (f"--{limite}\r\nContent-Type: image/jpeg\r\n"
                           f"Content-Length: {len(jpeg)}\r\n\r\n").encode() + jpeg + b"\r\n"

                time.sleep(1 / 25)

        @app.get("/video")
        def video():
            if not self.vision:
                return Response(status_code=503)

            return StreamingResponse(
                generar(), media_type=f"multipart/x-mixed-replace; boundary={limite}")

        # Estado en JSON: sirve para ver si el nucleo esta vivo sin abrir la
        # interfaz (desde el celular, desde otra maquina, o desde un script).
        @app.get("/salud")
        def salud():
            est = self.estado

            return {
                "enlace": est.enlace_vivo(),
                "puerto": est.puerto,
                "error": est.error_enlace,
                "fps": round(est.fps_camara, 1),
                "estado": est.e.estado_nombre if est.e else "",
                "modo": est.e.modo.name if est.e and est.e.modo else "",
                "cola": est.e.cola if est.e else None,
                "parametros": len(est.parametros),
                "cinta_medida": est.cinta_medida,
                "recorte_y": self.vision.offset_recorte if self.vision else None,
                "chequeos": {k: [v.estado, v.detalle] for k, v in est.chequeos().items()},
            }

    # ------------------------------------------------------------------
    def construir(self) -> None:
        ui.add_head_html(f"""<style>
          body {{ background: {FONDO}; color: {TEXTO};
                  font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
                  overflow: hidden; zoom: {ZOOM}; }}
          .nicegui-content {{ padding: 0 !important; gap: 0 !important; }}
          .panel {{ background: {PANEL}; border: 1px solid {BORDE}; border-radius: 8px;
                    min-height: 0; }}
          .fila {{ white-space: nowrap; overflow: hidden; text-overflow: ellipsis;
                   line-height: 1.55; }}
          .titulo {{ color: {APAGADO}; font-size: 15px; font-weight: 600;
                     letter-spacing: .07em; text-transform: uppercase; }}
        </style>""")

        with ui.row().classes("w-full items-center gap-4 px-4").style("height:38px") \
                .style(f"background:{PANEL};border-bottom:1px solid {BORDE}"):
            ui.label("KUKO").style(f"color:{CELESTE};font-weight:700;letter-spacing:.16em")

            with ui.tabs().props("dense indicator-color=cyan-4") as tabs:
                self.tab_operacion = ui.tab("Operacion")
                self.tab_teach = ui.tab("Teach")
                self.tab_proceso = ui.tab("Proceso")
                self.tab_servicio = ui.tab("Servicio")

            ui.space()
            self.chip_enlace = ui.html()

        with ui.tab_panels(tabs, value=self.tab_operacion).classes("w-full") \
                .style(f"background:{FONDO};height:calc(100vh / {ZOOM} - 42px);overflow:hidden"):
            with ui.tab_panel(self.tab_operacion).classes("p-0").style("height:100%"):
                self._operacion()

            with ui.tab_panel(self.tab_teach).classes("p-0").style("height:100%"):
                self._teach()

            with ui.tab_panel(self.tab_proceso).classes("p-0").style("height:100%"):
                self._ajustes(pr.NIVEL_PROCESO, self._panel_en_vivo)

            with ui.tab_panel(self.tab_servicio).classes("p-0").style("height:100%"):
                self._ajustes(pr.NIVEL_SERVICIO, self._panel_servicio)

        tabs.on_value_change(self._cambio_pestana)

        # El teclado se escucha a nivel pagina: no existe "escuchar solo
        # dentro de este panel". Los campos de texto quedan afuera (ignore),
        # asi que renombrar un movimiento no mueve el brazo.
        ui.keyboard(on_key=self._teach_tecla,
                    ignore=["input", "select", "button", "textarea"])

        ui.timer(0.1, self._refrescar_rapido)
        ui.timer(0.5, self._refrescar_lento)
        ui.timer(PERIODO_TEACH_S, self._teach_tick)

    # ------------------------------------------------------------------
    def _operacion(self) -> None:
        # Disposicion: dos columnas. A la izquierda el video con los finales
        # y la ventosa debajo; a la derecha tres filas -- componentes y
        # motores, guard/stop y latencia, y abajo clasificacion y caja.
        # Todo con alturas en flex para que entre en la ventana sin scroll.
        with ui.row().classes("w-full gap-2 p-2 no-wrap").style("height:100%"):
            # ================= Columna izquierda =================
            with ui.column().classes("gap-2 no-wrap items-stretch").style("flex:1.18 1 0;height:100%;min-height:0"):
                with ui.column().classes("panel p-2 gap-1").style("flex:0 0 auto;overflow:hidden"):
                    with ui.row().classes("w-full items-center gap-2 no-wrap px-1"):
                        ui.label("Vision IA").classes("titulo")
                        ui.space()

                        # Recentrado del recorte. Mantener apretado repite:
                        # la cinta se corre de a poco y buscar el centro de
                        # a un click seria tedioso.
                        self.etiqueta_recorte = ui.label("").style(
                            f"color:{APAGADO};font-size:12px")

                        for icono, paso in (("keyboard_arrow_up", -PASO_RECORTE),
                                            ("keyboard_arrow_down", PASO_RECORTE)):
                            boton = ui.button(icon=icono).props("dense flat round size=sm") \
                                .style(f"color:{CELESTE}")
                            boton.on("mousedown", lambda _, p=paso: self._empezar_a_mover(p))
                            boton.on("mouseup", lambda _: self._dejar_de_mover())
                            boton.on("mouseleave", lambda _: self._dejar_de_mover())

                    ui.html('<img src="/video" style="width:100%;height:auto;'
                            'object-fit:contain;border-radius:4px;display:block" alt="camara"/>') \
                        .style("width:100%;display:flex;justify-content:center")

                # --- Portada: logo de la facultad, nombres, etc. --------
                # La imagen se deja caer en pc/assets/portada.png y aparece
                # sola; mientras no exista se muestra el marco vacio, que es
                # mas honesto que una imagen de relleno.
                with ui.row().classes("panel items-center justify-center overflow-hidden") \
                        .style("flex:1 1 0;min-height:0;padding:6px"):
                    ui.html(f'<img src="{_archivo_portada()}" alt="" '
                            'onerror="this.style.display=\'none\'" '
                            'style="max-width:100%;max-height:100%;object-fit:contain;'
                            'display:block"/>') \
                        .style("max-height:100%;display:flex;align-items:center")
            # ================= Columna derecha =================
            with ui.column().classes("gap-2 no-wrap") \
                    .style("flex:1.25 1 0;height:100%;min-height:0"):
                # --- Fila 1: componentes y motores ----------------------
                with ui.row().classes("w-full gap-2 no-wrap items-stretch") \
                        .style("flex:0 0 auto"):
                    with ui.column().classes("panel p-3 gap-1").style("flex:1 1 0"):
                        ui.label("Componentes").classes("titulo")
                        self.filas_chequeo = {}

                        for clave in ("cinta", "encoders", "endstops", "motores", "neumatica"):
                            self.filas_chequeo[clave] = ui.html().classes("text-sm fila w-full")

                    with ui.column().classes("panel p-2 gap-1").style("flex:0 0 420px"):
                        ui.label("Motores").classes("titulo")

                        with ui.row().classes("w-full gap-2 items-center no-wrap"):
                            self.diales = [ui.html().style("flex:1 1 0") for _ in range(3)]

                # --- Fila 2: guard + stop, y latencia -------------------
                with ui.row().classes("w-full gap-2 no-wrap items-stretch") \
                        .style("flex:0 0 auto"):
                    with ui.row().classes("panel px-3 py-2 gap-2 items-center no-wrap") \
                            .style("flex:0 0 290px"):
                        self.fila_guard = ui.html().classes("text-sm").style("flex:1 1 0")
                        self.boton_paro = ui.button("STOP", on_click=self._paro) \
                            .props("unelevated dense no-caps").style("min-width:104px")

                    with ui.row().classes("panel px-3 py-2 gap-2 items-center no-wrap") \
                            .style("flex:1 1 0"):
                        ui.label("Latencia").classes("titulo").style("flex:0 0 auto")

                        # El slider no se crea aca: sus topes son el rango que
                        # declara el firmware para 'vis_lat', y eso llega con
                        # el 'P?' un par de segundos despues. Tenerlos escritos
                        # tambien de este lado es la receta para que un dia el
                        # slider ofrezca un rango que el robot rechaza -- ya
                        # paso con el minimo, que quedo en -0,10 s despues de
                        # que el firmware pasara a -0,20 s.
                        self.caja_latencia = ui.row() \
                            .classes("items-center no-wrap").style("flex:1 1 0")
                        self.slider_latencia = None

                        # Los botones ajustan de a 0,1 cm y no de a
                        # milisegundos: el centimetro es lo que se ve errarle
                        # al gripper, y es con eso que uno corrige.
                        for icono, paso in (("remove", -0.1), ("add", 0.1)):
                            ui.button(icon=icono,
                                      on_click=lambda _, d=paso: self._ajustar_latencia(d)) \
                                .props("dense flat round size=sm").style(f"color:{CELESTE}")

                        self.etiqueta_latencia = ui.label("—").classes("text-sm") \
                            .style("flex:0 0 122px;text-align:right")

                # --- Fila 3: clasificacion y caja -----------------------
                # Un panel por modo, y el TITULO de cada panel es el boton que
                # lo selecciona: no hay un titulo decorativo y un boton
                # aparte diciendo lo mismo. "Clasificacion" quedaba ademas
                # mal puesto sobre el modo caja, que no clasifica nada.
                with ui.row().classes("w-full gap-2 no-wrap items-stretch") \
                        .style("flex:1 1 0;min-height:0"):
                    self.botones_modo = {}

                    with ui.column().classes("panel p-3 gap-2") \
                            .style("flex:1 1 0;min-height:0"):
                        self.botones_modo[pr.Modo.COLOR] = self._boton_titulo(
                            "Por color", pr.Modo.COLOR)
                        self.contadores_color = ui.html().classes("w-full")

                    with ui.column().classes("panel p-3 gap-2") \
                            .style("flex:1 1 0;min-height:0"):
                        self.botones_modo[pr.Modo.FORMA] = self._boton_titulo(
                            "Por forma", pr.Modo.FORMA)
                        self.contadores_forma = ui.html().classes("w-full")

                    with ui.column().classes("panel p-3 gap-2").style("flex:0 0 236px"):
                        self.botones_modo[pr.Modo.ALFAJORES] = self._boton_titulo(
                            "Box", pr.Modo.ALFAJORES)

                        self.celdas = []

                        with ui.grid(columns=3).classes("gap-2 w-full"):
                            for i in range(6):
                                self.celdas.append(
                                    ui.html().style("cursor:pointer")
                                    .on("click", lambda _, c=i: self._rotar_celda(c)))

                        self.boton_confirmar = ui.button(
                            "Confirmar", on_click=self._confirmar_caja) \
                            .props("dense unelevated no-caps").classes("w-full")

                # Dos paneles separados: son dos cosas distintas y compartir
                # un marco las hacia parecer una sola.
                with ui.row().classes("w-full gap-2 no-wrap items-stretch") \
                        .style("flex:0 0 116px"):
                    with ui.column().classes("panel p-2 gap-1 items-center")                             .style("flex:1 1 0"):
                        ui.label("Finales de carrera").classes("titulo w-full")
                        self.html_finales = ui.html().style("width:100%;max-width:250px")

                    with ui.column().classes("panel p-2 gap-1 items-center")                             .style("flex:1 1 0"):
                        ui.label("Succion").classes("titulo w-full")
                        self.html_ventosa = ui.html().style("width:100%;max-width:130px")


    def _boton_titulo(self, texto: str, modo: pr.Modo):
        return (ui.button(texto, on_click=lambda _, m=modo: self._modo(m))
                .props("flat dense no-caps align=left")
                .classes("w-full")
                .style("font-size:15px;font-weight:600;letter-spacing:.07em;"
                       "text-transform:uppercase;padding:2px 8px"))

    # ==================================================================
    #  AJUSTES  (pestanas de proceso y servicio)
    # ==================================================================
    #
    #  Una lista larga y scrolleable, con indice a la izquierda: el mismo
    #  patron que un menu de opciones de un juego, y por el mismo motivo --
    #  hay muchos valores, cada uno se toca poco, y lo que importa es
    #  encontrar rapido el que se busca y entender que hace antes de moverlo.
    #
    #  NINGUN parametro esta escrito aca. La lista se arma recorriendo lo que
    #  contesto 'P?', con el rango y el nivel que declaro el firmware, asi que
    #  agregar uno es una linea de C++ y aparece solo, en la pestana que le
    #  corresponde. Lo unico que pone Python es el nombre en castellano y la
    #  explicacion (parametros.py), y hasta eso es opcional.

    def _ajustes(self, nivel: int, extra=None) -> None:
        with ui.row().classes("w-full gap-2 p-2 no-wrap").style("height:100%"):
            with ui.column().classes("panel p-2 gap-1 no-wrap") \
                    .style("flex:0 0 186px;height:100%;overflow-y:auto"):
                ui.label("Secciones").classes("titulo").style("padding:0 6px 4px")
                self._nav[nivel] = ui.column().classes("w-full gap-0")

            with ui.column().classes("panel no-wrap") \
                    .style("flex:1 1 0;height:100%;min-height:0;gap:0;overflow:hidden"):
                # La clase 'lista-ajustes' la usa _ir_a() para encontrar
                # ESTE contenedor desde el ancla de la seccion.
                self._lista[nivel] = ui.column().classes("w-full gap-0 lista-ajustes") \
                    .style("flex:1 1 0;min-height:0;overflow-y:auto;"
                           "overscroll-behavior:contain")
                self._barra_inferior(nivel)

            if extra is not None:
                with ui.column().classes("gap-2 no-wrap") \
                        .style("flex:0 0 336px;height:100%;min-height:0"):
                    extra()

    def _barra_inferior(self, nivel: int) -> None:
        """Guardar / restaurar / releer, y cuantos difieren de fabrica.

        Va abajo y siempre visible: 'P*' es lo que hace que la calibracion
        sobreviva al proximo reflasheo, y si hubiera que scrollear hasta el
        final de la lista para encontrarlo, tarde o temprano alguien ajusta
        media hora el robot y despues sube firmware sin haber guardado.
        """

        with ui.row().classes("w-full items-center gap-2 no-wrap px-3 py-2") \
                .style(f"flex:0 0 auto;border-top:1px solid {BORDE}"):
            self._pie_texto[nivel] = ui.label("").style(
                f"color:{APAGADO};font-size:12px;flex:1 1 0")

            ui.button("Releer", on_click=lambda: self.enviar(pr.cmd_listar_parametros())) \
                .props("flat dense no-caps").style(f"color:{APAGADO}")
            ui.button("De fabrica", on_click=self._confirmar_de_fabrica) \
                .props("flat dense no-caps").style(f"color:{APAGADO}")
            ui.button("Guardar en la placa", on_click=self._guardar_parametros) \
                .props("unelevated dense no-caps") \
                .style(f"background:{CELESTE}!important;color:#0B1220!important")

    # ------------------------------------------------------------------
    def _reconstruir(self, nivel: int) -> None:
        """Rearma la lista solo si cambio el conjunto de parametros.

        Rearmarla en cada refresco perderia el foco del teclado a mitad de
        tipear un numero, dos veces por segundo.
        """

        contenedor = self._lista.get(nivel)

        if contenedor is None:
            return

        grupos = par.agrupar(self.estado.parametros.values(), nivel)
        firma = tuple(p.nombre for _, ps in grupos for p in ps)

        if firma == self._firma.get(nivel):
            return

        self._firma[nivel] = firma

        for nombre in firma:
            self._filas.pop(nombre, None)

        contenedor.clear()
        self._nav[nivel].clear()

        if not firma:
            with contenedor:
                ui.label("Esperando la tabla de parametros del robot...") \
                    .style(f"color:{APAGADO};padding:14px")
            return

        for i, (grupo, ps) in enumerate(grupos):
            ident = f"sec-{nivel}-{i}"

            with self._nav[nivel]:
                ui.button(grupo, on_click=lambda _, d=ident: self._ir_a(d)) \
                    .props("flat dense no-caps align=left").classes("w-full") \
                    .style(f"color:{APAGADO};font-size:13px;padding:3px 6px")

            with contenedor:
                ui.html(f'<div id="{ident}"></div>').style("height:0")

                with ui.row().classes("w-full items-center no-wrap") \
                        .style(f"background:{INACTIVO};padding:6px 12px;"
                               "position:sticky;top:0;z-index:2"):
                    ui.label(grupo).classes("titulo").style("font-size:13px")

                for parametro in ps:
                    self._fila_param(parametro)

    def _ir_a(self, ident: str) -> None:
        """Scrollea SOLO la lista, no la pagina.

        Aca habia un scrollIntoView(), y estaba mal: ese metodo scrollea
        todos los ancestros scrolleables del elemento, y el navegador
        considera scrolleable tambien un contenedor con overflow:hidden --
        no deja que lo mueva la rueda del mouse, pero si lo mueve por
        codigo. Resultado: al saltar a una seccion del final, la pagina
        entera se corria hacia arriba, la barra de pestanas quedaba fuera de
        vista y no habia forma de volver, porque justamente la rueda no
        actua sobre un overflow:hidden.

        Movemos el scrollTop del contenedor de la lista a mano y de paso
        enderezamos cualquier ancestro que haya quedado corrido de antes.
        """

        ui.run_javascript(f"""
            const ancla = document.getElementById("{ident}");
            const lista = ancla ? ancla.closest(".lista-ajustes") : null;

            if (lista) {{
                lista.scrollTop += ancla.getBoundingClientRect().top
                                 - lista.getBoundingClientRect().top;

                for (let n = lista.parentElement; n; n = n.parentElement) {{
                    if (n.scrollTop) {{ n.scrollTop = 0; }}
                }}

                window.scrollTo(0, 0);
            }}
        """)

    # ------------------------------------------------------------------
    def _fila_param(self, p: pr.Parametro) -> None:
        ficha = par.describir(p.nombre)
        entero = p.tipo in ("i", "b")

        minimo = p.minimo if p.minimo is not None else 0.0
        maximo = p.maximo if p.maximo is not None else 1.0
        rango = max(maximo - minimo, 1e-6)

        # El paso sale del rango declarado y no de una tabla de casos: un
        # parametro que va de 0 a 0,3 cm necesita milesimas y uno que va de
        # 20000 a 150000 pasos/s no necesita ni decimas.
        if entero:
            paso = 1.0
        elif rango <= 1.0:
            paso = 0.005
        elif rango <= 15.0:
            paso = 0.01
        elif rango <= 60.0:
            paso = 0.05
        else:
            paso = 0.5

        decimales = 0 if entero else max(0, -int(math.floor(math.log10(paso))))

        fila = {"paso": paso, "hasta": 0.0, "timer": None, "mudo": False,
                "pendiente": None, "editando": False}
        self._filas[p.nombre] = fila

        with ui.row().classes("w-full items-center no-wrap gap-3") \
                .style(f"padding:7px 12px;border-bottom:1px solid {BORDE}"):
            with ui.column().classes("gap-0").style("flex:1 1 0;min-width:0"):
                with ui.row().classes("items-center gap-2 no-wrap"):
                    fila["punto"] = ui.html()
                    ui.label(ficha.etiqueta).style(f"color:{TEXTO};font-size:14px")
                    ui.label(p.nombre).style(
                        f"color:{APAGADO};font-size:11px;opacity:.55;"
                        "font-family:ui-monospace,monospace")

                ui.label(ficha.ayuda).style(
                    f"color:{APAGADO};font-size:12px;line-height:1.35;"
                    "white-space:normal")

            fila["slider"] = ui.slider(
                min=minimo, max=maximo, step=paso, value=p.valor,
                on_change=lambda e, n=p.nombre: self._tocar(n, e.value)) \
                .props("dense").style("flex:0 0 176px")

            fila["num"] = ui.number(
                value=p.valor, format=f"%.{decimales}f",
                step=paso, min=minimo, max=maximo,
                on_change=lambda e, n=p.nombre: self._tocar(n, e.value, 0.8)) \
                .props("dense outlined").style("flex:0 0 92px")

            # Mientras el cursor esta DENTRO del campo, el refresco no lo
            # toca. Sin esto es imposible escribir un numero: a los 0,5 s el
            # refresco reescribe el valor que dice el firmware y se pierde lo
            # tipeado a medio tipear.
            fila["num"].on("focus", lambda _, n=p.nombre: self._editando(n, True))
            fila["num"].on("blur", lambda _, n=p.nombre: self._editando(n, False))

            # Y la demora de 0,8 s de arriba es para que escribir "-32" no
            # mande primero un 3 y despues un -3 (que esta dentro de rango
            # para varios de estos parametros). Enter no espera.
            fila["num"].on("keydown.enter", lambda _, n=p.nombre: self._tocar_numero(n))

            ui.label(p.unidad or "").style(
                f"color:{APAGADO};font-size:12px;flex:0 0 46px")

            fila["reset"] = ui.button(
                icon="restart_alt", on_click=lambda _, n=p.nombre: self._de_fabrica(n)) \
                .props("flat dense round size=sm").style(f"color:{APAGADO}")

            if p.defecto is not None:
                fila["reset"].tooltip(
                    f"de fabrica: {p.defecto:.{decimales}f} {p.unidad}".rstrip())

    # ------------------------------------------------------------------
    def _tocar(self, nombre: str, valor, demora: float = 0.22) -> None:
        """Un control se movio. Se manda con un respiro, no en el acto.

        Arrastrar un slider genera decenas de eventos por segundo y el
        puerto serie lo comparte la vision: mandarlos todos llenaria el
        enlace de escrituras de un valor que el operador todavia esta
        eligiendo. Tipear un numero es igual, pero mas lento (de ahi la
        demora mas larga del campo de texto).
        """

        fila = self._filas.get(nombre)

        if fila is None or fila["mudo"] or valor is None:
            return

        # Mientras el operador toca, el refresco no le pisa el control con
        # lo que dice el firmware (que todavia es el valor viejo). Se marca
        # ANTES del descarte de abajo: si no, volver al valor original con
        # las flechitas dejaria el campo destapado a mitad de la edicion.
        fila["hasta"] = time.monotonic() + max(1.5, demora + 1.0)

        actual = self.estado.parametros.get(nombre)

        if actual is not None and actual.valor is not None and \
                abs(actual.valor - float(valor)) < fila["paso"] / 4:
            return

        fila["pendiente"] = float(valor)

        if fila["timer"] is not None:
            fila["timer"].cancel()
            fila["timer"] = None

        if demora <= 0.0:
            self._despachar_param(nombre)
            return

        fila["timer"] = ui.timer(demora, lambda n=nombre: self._despachar_param(n),
                                 once=True)

    def _tocar_numero(self, nombre: str) -> None:
        fila = self._filas.get(nombre)

        if fila is not None:
            self._tocar(nombre, fila["num"].value, 0.0)

    def _editando(self, nombre: str, activo: bool) -> None:
        """El campo de texto gano o perdio el foco.

        Con el foco adentro el refresco no lo toca en absoluto (ni por
        tiempo): alguien puede quedarse pensando delante del campo mas de lo
        que dure cualquier ventana. Al salir se manda lo que quedo escrito,
        sin esperar la demora.
        """

        fila = self._filas.get(nombre)

        if fila is None:
            return

        fila["editando"] = activo

        if not activo:
            self._tocar(nombre, fila["num"].value, 0.0)

    def _despachar_param(self, nombre: str) -> None:
        fila = self._filas.get(nombre)

        if fila is None or fila["pendiente"] is None:
            return

        valor = fila["pendiente"]
        fila["pendiente"] = None
        fila["timer"] = None

        if not self.enviar(pr.cmd_parametro(nombre, valor)):
            ui.notify("sin enlace con el robot", color="negative")

    def _de_fabrica(self, nombre: str) -> None:
        fila = self._filas.get(nombre)
        parametro = self.estado.parametros.get(nombre)

        if fila is None or parametro is None or parametro.defecto is None:
            return

        fila["hasta"] = 0.0
        self.enviar(pr.cmd_parametro(nombre, parametro.defecto))

    def _guardar_parametros(self) -> None:
        if self.enviar(pr.cmd_guardar_parametros()):
            ui.notify("Guardado en la placa: sobrevive al reinicio y al reflasheo",
                      color="positive")

    def _confirmar_de_fabrica(self) -> None:
        modificados = [p for p in self.estado.parametros.values() if p.modificado]

        with ui.dialog() as dialogo, ui.card().style(f"background:{PANEL};color:{TEXTO}"):
            ui.label("Volver todo a los valores de fabrica").classes("text-lg")
            ui.label(f"Se pierden los {len(modificados)} valores ajustados a mano, "
                     "de las tres pestanas, no solo de esta. Los de fabrica son "
                     "los que estan escritos en el codigo del firmware.") \
                .style(f"color:{APAGADO};max-width:400px")

            with ui.row().classes("w-full justify-end gap-2"):
                ui.button("Cancelar", on_click=dialogo.close).props("flat dense no-caps")
                ui.button("Restaurar", on_click=lambda: self._restaurar(dialogo)) \
                    .props("dense unelevated no-caps") \
                    .style(f"background:{ROJO_STOP}!important;color:#fff!important")

        dialogo.open()

    def _restaurar(self, dialogo) -> None:
        self.enviar(pr.cmd_parametros_de_fabrica())

        # El firmware no vuelca la tabla despues de 'P0': sin este pedido, la
        # pantalla seguiria mostrando los valores viejos hasta el proximo
        # arranque.
        self.enviar(pr.cmd_listar_parametros())

        for fila in self._filas.values():
            fila["hasta"] = 0.0

        dialogo.close()
        ui.notify("Valores de fabrica restaurados", color="warning")

    # ------------------------------------------------------------------
    def _refrescar_ajustes(self) -> None:
        self._reconstruir(pr.NIVEL_PROCESO)
        self._reconstruir(pr.NIVEL_SERVICIO)

        ahora = time.monotonic()
        vivo = self.estado.enlace_vivo()

        for nombre, fila in self._filas.items():
            p = self.estado.parametros.get(nombre)

            if p is None or p.valor is None:
                continue

            fila["slider"].set_enabled(vivo)
            fila["num"].set_enabled(vivo)

            # Un punto celeste marca lo que no esta en su valor de fabrica.
            # Es lo unico que distingue "el robot viene asi" de "alguien lo
            # ajusto y no lo anoto en ningun lado".
            fila["punto"].content = (
                '<span style="display:inline-block;width:7px;height:7px;'
                f'border-radius:50%;background:'
                f'{CELESTE if p.modificado else "transparent"}"></span>')

            # El campo con el foco adentro no se toca nunca, y el resto de la
            # fila queda quieto un rato despues de cada cambio: el firmware
            # tarda en contestar y hasta que contesta sigue diciendo el valor
            # viejo, que es exactamente el que no hay que volver a escribir.
            if fila["editando"] or ahora < fila["hasta"]:
                continue

            # Poner .value dispara on_change igual que si lo hubiera movido
            # una persona; sin el mudo, cada refresco reenviaria el valor.
            fila["mudo"] = True

            if abs((fila["slider"].value or 0.0) - p.valor) > fila["paso"] / 2:
                fila["slider"].value = p.valor

            if fila["num"].value is None or \
                    abs(float(fila["num"].value) - p.valor) > 1e-6:
                fila["num"].value = p.valor

            fila["mudo"] = False

        for nivel, etiqueta in self._pie_texto.items():
            total = sum(1 for p in self.estado.parametros.values() if p.nivel == nivel)
            cuantos = sum(1 for p in self.estado.parametros.values()
                          if p.nivel == nivel and p.modificado)

            etiqueta.text = (f"{total} ajustes · {cuantos} distintos de fabrica"
                             if total else "esperando la tabla del robot")

    # ==================================================================
    #  Paneles laterales de las dos pestanas
    # ==================================================================
    def _panel_en_vivo(self) -> None:
        """Lo que hay que estar mirando MIENTRAS se mueven estos numeros.

        Los umbrales del guard no se eligen leyendo el manual: se eligen
        mirando cuanto error hay de verdad en cada tramo. Tener el error y
        el umbral efectivo al lado del slider evita el ida y vuelta entre
        pestanas, que es donde se pierde la referencia de lo que uno movio.
        """

        with ui.column().classes("panel p-3 gap-2").style("flex:0 0 auto"):
            ui.label("Supervision en vivo").classes("titulo")
            self.html_guard_vivo = ui.html().classes("w-full")

        with ui.column().classes("panel p-3 gap-2").style("flex:0 0 auto"):
            ui.label("Cinta").classes("titulo")
            self.html_cinta_vivo = ui.html().classes("w-full")

        with ui.column().classes("panel p-3 gap-2").style("flex:1 1 0;min-height:0"):
            ui.label("Produccion").classes("titulo")
            self.html_produccion = ui.html().classes("w-full")

    def _panel_servicio(self) -> None:
        with ui.column().classes("panel p-3 gap-2").style("flex:0 0 auto"):
            ui.label("Acciones").classes("titulo")

            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("Frenar por colision").style(f"color:{TEXTO};font-size:13px")
                self.sw_paradas = ui.switch(
                    on_change=lambda e: self._alternar_paradas(bool(e.value))) \
                    .props("dense color=cyan-4")

            ui.label("Apagado, la supervision sigue midiendo y avisando pero no "
                     "frena el robot. Es para calibrar umbrales sin que cada "
                     "prueba termine en un rehoming.") \
                .style(f"color:{APAGADO};font-size:11.5px;line-height:1.35")

            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("Stream de telemetria").style(f"color:{TEXTO};font-size:13px")
                self.sw_telemetria = ui.switch(
                    value=True,
                    on_change=lambda e: self.enviar(pr.cmd_telemetria(bool(e.value)))) \
                    .props("dense color=cyan-4")

            ui.label("Apagarlo deja la pantalla ciega: los diales y los puntitos "
                     "se congelan. Solo sirve para leer el puerto a mano.") \
                .style(f"color:{APAGADO};font-size:11.5px;line-height:1.35")

            with ui.row().classes("w-full gap-2 no-wrap"):
                for texto, comando in (("Estado guard", pr.cmd_estado_supervision),
                                       ("Traza", pr.cmd_alternar_traza),
                                       ("Fallos", pr.cmd_historial_fallos)):
                    ui.button(texto, on_click=lambda _, c=comando: self.enviar(c())) \
                        .props("flat dense no-caps").classes("flex-1") \
                        .style(f"color:{CELESTE};font-size:12px")

        with ui.column().classes("panel p-3 gap-1").style("flex:1 1 0;min-height:0"):
            ui.label("Consola del robot").classes("titulo")
            self.html_consola = ui.html().style(
                "flex:1 1 0;min-height:0;overflow-y:auto;width:100%")

    def _alternar_paradas(self, activar: bool) -> None:
        # El switch refleja al robot; solo se manda el comando si lo que se
        # pide es distinto de lo que el robot dice tener. Sin esto, el
        # set_value() del refresco dispararia el comando de vuelta.
        e = self.estado.e
        actual = bool(e.paradas_activas) if e and e.paradas_activas is not None else True

        if activar != actual:
            self.enviar(pr.cmd_alternar_paradas())

    # ------------------------------------------------------------------
    def _refrescar_paneles(self) -> None:
        est = self.estado
        e, t = est.e, est.t

        def linea(clave: str, valor) -> str:
            return (f'<div style="display:flex;justify-content:space-between;'
                    f'font-size:13px;margin:3px 0"><span style="color:{APAGADO}">'
                    f'{clave}</span><span style="color:{TEXTO}">{valor}</span></div>')

        if hasattr(self, "html_guard_vivo"):
            filas = []

            for i in range(3):
                err = abs(t.error[i]) if t and t.error[i] is not None else None
                umb = t.umbral[i] if t and t.umbral[i] else None

                # margen() ya es error/umbral: la misma cuenta que decide la
                # colision, para que la barra no pueda decir una cosa
                # distinta de la que va a hacer el robot.
                margen = t.margen(i) if t else None
                frac = min(1.0, margen) if margen is not None else 0.0
                color = COLOR_ESTADO[VERDE if frac < 0.7
                                     else (AMBAR if frac < 1.0 else ROJO)]

                filas.append(
                    f'<div style="margin:7px 0">'
                    f'<div style="display:flex;justify-content:space-between;'
                    f'font-size:12px;color:{APAGADO}"><span>Eje {i + 1}</span>'
                    f'<span>{"—" if err is None else f"{err:.1f}"} de '
                    f'{"—" if umb is None else f"{umb:.1f}"}°</span></div>'
                    f'<div style="height:6px;border-radius:3px;background:{INACTIVO};'
                    f'margin-top:3px"><div style="height:6px;border-radius:3px;'
                    f'width:{frac * 100:.0f}%;background:{color}"></div></div></div>')

            self.html_guard_vivo.content = "".join(filas)

        if hasattr(self, "html_cinta_vivo"):
            conf = est.parametros.get("cinta_cms")
            pwm = est.parametros.get("cinta_pwm")

            self.html_cinta_vivo.content = (
                linea("configurada", "—" if not conf or conf.valor is None
                      else f"{conf.valor:.2f} cm/s")
                + linea("medida por la vision", "—" if est.cinta_medida is None
                        else f"{est.cinta_medida:.2f} cm/s")
                + linea("PWM", "—" if not pwm or pwm.valor is None
                        else f"{pwm.valor:.0f} %")
                + linea("en marcha", "si" if e and e.cinta else "no")
                + f'<div style="color:{APAGADO};font-size:11.5px;margin-top:6px;'
                  'line-height:1.35">Las dos primeras tienen que coincidir. Si no, '
                  'la planificacion apunta a donde la pieza no va a estar.</div>')

        if hasattr(self, "html_produccion"):
            tasa = e.tasa_exito if e else None

            self.html_produccion.content = (
                linea("detectadas", e.detectadas if e else "—")
                + linea("depositadas", e.depositadas if e else "—")
                + linea("descartadas", e.descartadas if e else "—")
                + linea("fallos", e.fallos if e else "—")
                + linea("efectividad", "—" if tasa is None else f"{tasa * 100:.0f} %"))

        if hasattr(self, "sw_paradas"):
            activas = (bool(e.paradas_activas)
                       if e and e.paradas_activas is not None else True)

            if self.sw_paradas.value != activas:
                self.sw_paradas.set_value(activas)

            self.sw_paradas.set_enabled(est.enlace_vivo())
            self.sw_telemetria.set_enabled(est.enlace_vivo())

        if hasattr(self, "html_consola"):
            self.html_consola.content = "".join(
                f'<div style="font-family:ui-monospace,monospace;font-size:11.5px;'
                f'color:{APAGADO};line-height:1.5;white-space:pre-wrap;'
                f'word-break:break-word">{l}</div>'
                for l in est.consola[-40:]) or (
                f'<div style="color:{APAGADO};font-size:12px">sin novedades</div>')

    # ------------------------------------------------------------------
    #  Acciones
    # ------------------------------------------------------------------
    def _ajustar_latencia(self, delta_cm: float) -> None:
        """Corre la latencia lo que tarde la cinta en avanzar delta_cm."""

        param = self.estado.parametros.get("vis_lat")
        cinta = self.estado.parametros.get("cinta_cms")

        if not param or param.valor is None or not cinta or not cinta.valor:
            return

        nuevo = param.valor + delta_cm / cinta.valor

        # Se satura contra el rango que declaro el firmware: si no, el
        # ultimo click contra el tope contestaria err=rango y ensuciaria la
        # consola sin que el operador entienda por que.
        nuevo = max(param.minimo, min(param.maximo, nuevo))

        self.enviar(pr.cmd_parametro("vis_lat", nuevo))

    def _empezar_a_mover(self, paso: int) -> None:
        self._mover_recorte(paso)
        self._dejar_de_mover()

        # Repeticion mientras se mantiene apretado, con una pausa inicial
        # para que un click suelto no dispare dos pasos.
        self._repeticion = ui.timer(0.35, lambda: self._repetir(paso), once=True)

    def _repetir(self, paso: int) -> None:
        self._repeticion = ui.timer(0.09, lambda: self._mover_recorte(paso))

    def _dejar_de_mover(self) -> None:
        if self._repeticion:
            self._repeticion.cancel()
            self._repeticion = None

    def _mover_recorte(self, paso: int) -> None:
        if self.vision:
            self.vision.mover_recorte(paso)

    def _paro(self) -> None:
        est = self.estado
        en_error = bool(est.e and est.e.estado is pr.EstadoRobot.ERROR)

        self.enviar(pr.cmd_paro())
        ui.notify("Rehomeando" if en_error else "Parada manual enviada",
                  color="info" if en_error else "warning")

    def _modo(self, modo: pr.Modo) -> None:
        est = self.estado
        actual = est.e.modo if est.e else None

        # Entrar o salir del modo box implica poner o sacar la tapa. El
        # firmware ya pide el comando dos veces; el dialogo es esa segunda
        # vez, con el motivo escrito.
        if (modo is pr.Modo.ALFAJORES) == (actual is pr.Modo.ALFAJORES):
            self.enviar(pr.cmd_modo(modo))
            return

        poner = modo is pr.Modo.ALFAJORES

        with ui.dialog() as dialogo, ui.card().style(f"background:{PANEL};color:{TEXTO}"):
            ui.label(f"Hay que {'COLOCAR' if poner else 'RETIRAR'} la tapa").classes("text-lg")
            ui.label("Sin la tapa puesta, los alfajores se apoyan en el aire."
                     if poner else
                     "Con la tapa puesta, las piezas rebotan contra ella.") \
                .style(f"color:{APAGADO};max-width:380px")

            with ui.row().classes("w-full justify-end gap-2"):
                ui.button("Cancelar", on_click=dialogo.close).props("flat dense no-caps")
                ui.button(f"Ya {'puse' if poner else 'saque'} la tapa",
                          on_click=lambda: (self.enviar(pr.cmd_modo(modo)),
                                            self.enviar(pr.cmd_modo(modo)),
                                            dialogo.close())) \
                    .props("dense unelevated no-caps") \
                    .style(f"background:{CELESTE}!important;color:#0B1220!important")

        dialogo.open()

    def _cambiar_latencia(self, evento) -> None:
        self.enviar(pr.cmd_parametro("vis_lat", float(evento.value)))

    def _rotar_celda(self, celda: int) -> None:
        if not self.layout_editado:
            return

        orden = "RGB"
        self.layout_editado[celda] = orden[(orden.find(self.layout_editado[celda]) + 1) % 3]

        # El tope de 3 por color lo valida cmd_layout_caja al confirmar; se
        # avisa antes para no dejar que el operador arme algo imposible sin
        # enterarse hasta el final.
        self._pintar_caja()

    def _confirmar_caja(self) -> None:
        try:
            comando = pr.cmd_layout_caja("".join(self.layout_editado))
        except ValueError as err:
            ui.notify(str(err), color="negative")
            return

        self.enviar(comando)
        ui.notify("Disposicion enviada", color="positive")

    def _caja_nueva(self, dialogo) -> None:
        self.enviar(pr.cmd_caja_nueva())
        self.caja_avisada = False
        dialogo.close()

    # ------------------------------------------------------------------
    #  Refresco
    # ------------------------------------------------------------------
    def _refrescar_rapido(self) -> None:
        est = self.estado
        vivo = est.enlace_vivo()

        for i in range(3):
            self.diales[i].content = _svg_dial(
                i + 1,
                est.t.comandado[i] if est.t and vivo else None,
                est.angulo_suave[i] if vivo else None)

        self.html_finales.content = _svg_finales(est.t.finales if est.t and vivo else [])
        self.html_ventosa.content = _svg_ventosa(bool(est.t and vivo and est.t.bomba))

        # El plano de teach se redibuja aca y no en el refresco lento: la
        # gracia de un grafico en vivo es que la mano y el punto de la
        # pantalla se muevan juntos, y a 2 Hz el punto va siempre medio
        # segundo atrasado.
        self._refrescar_teach()

    def _refrescar_lento(self) -> None:
        est = self.estado
        vivo = est.enlace_vivo()

        # Un firmware con otra versión de protocolo se avisa ACÁ y no sólo
        # en la consola. Es la falla que más caro sale de encontrar: la
        # interfaz nueva dibuja controles que la placa vieja no conoce, así
        # que el botón anda, el comando sale, el ESP32 contesta "comando
        # invalido" en una línea que nadie está mirando y el brazo no se
        # mueve. Pasó de verdad con `JI`.
        vieja = bool(est.boot and not est.boot.compatible)

        if vivo and vieja:
            senal, color = ROJO, COLOR_ESTADO[ROJO]
            texto = (f"firmware desparejo (proto={est.boot.proto}, "
                     f"la interfaz habla {pr.VERSION_PROTOCOLO}): hay que reflashear")
        elif vivo:
            senal, color = VERDE, COLOR_ESTADO[VERDE]
            texto = f"{est.puerto} · {est.fps_camara:.0f} fps"
        else:
            senal, color = ROJO, COLOR_ESTADO[ROJO]
            texto = est.error_enlace or "sin enlace"

        self.chip_enlace.content = (
            f'{_punto(senal)}'
            f'<span style="color:{color};font-size:13px">{texto}</span>')

        for clave, chequeo in est.chequeos().items():
            self.filas_chequeo[clave].content = (
                f'{_punto(chequeo.estado)}<span>{clave.capitalize()}</span>'
                f'<span style="color:{APAGADO};font-size:12px"> — {chequeo.detalle}</span>')

        if self.vision:
            self.etiqueta_recorte.text = f"recorte {self.vision.offset_recorte:+d} px"

        self._guard_y_paro()
        self._modo_y_contadores()
        self._avisar_caja_completa()
        self._latencia()
        self._refrescar_ajustes()
        self._refrescar_paneles()

    def _guard_y_paro(self) -> None:
        est = self.estado
        e = est.e

        if not est.enlace_vivo() or not e:
            estado_guard, detalle = GRIS, "sin datos"
        elif e.guard is pr.EstadoGuard.ARMADO:
            estado_guard = VERDE
            detalle = "observando" if e.observando else "calibrado"
        elif e.guard is pr.EstadoGuard.PROMEDIANDO:
            estado_guard, detalle = AMBAR, "calibrando"
        else:
            estado_guard, detalle = ROJO, "sin calibrar"

        self.fila_guard.content = (
            f'{_punto(estado_guard)}Guard'
            f'<span style="color:{APAGADO};font-size:12px"> — {detalle}</span>')

        # El boton cambia de cara segun el estado: desde ERROR la misma 'R'
        # rehomea, y no avisarlo hace que el operador apriete STOP dos veces
        # sin entender por que el robot arranco.
        en_error = bool(e and e.estado is pr.EstadoRobot.ERROR)

        self.boton_paro.text = "Re-Homing" if en_error else "STOP"
        self.boton_paro.style(
            f'background:{CELESTE if en_error else ROJO_STOP}!important;'
            f'color:{"#0B1220" if en_error else "#fff"}!important;font-weight:700')
        self.boton_paro.set_enabled(est.enlace_vivo())

    def _modo_y_contadores(self) -> None:
        est = self.estado
        e = est.e
        modo = e.modo if e else None

        # Al cambiar de modo los contadores vuelven a cero: lo que interesa
        # es lo producido en la corrida actual. El firmware sigue contando
        # desde que arranco, asi que se guarda la foto y se resta.
        if modo is not self.modo_anterior:
            self.base_color = dict(e.por_color) if e else {}
            self.base_forma = dict(e.por_forma) if e else {}
            self.modo_anterior = modo

        for m, boton in self.botones_modo.items():
            activo = modo is m
            boton.style(f'background:{CELESTE if activo else "transparent"}!important;'
                        f'color:{"#0B1220" if activo else APAGADO}!important;'
                        'font-size:15px;font-weight:600;letter-spacing:.07em;'
                        'text-transform:uppercase;padding:2px 8px')
            boton.set_enabled(est.enlace_vivo())

        def bloque(codigos: dict, cuentas: dict, base: dict, forma: bool) -> str:
            filas = []

            for codigo, marca in codigos.items():
                bruto = cuentas.get(codigo)
                valor = "—" if bruto is None else max(0, bruto - base.get(codigo, 0))

                filas.append(
                    f'<div style="display:flex;align-items:center;gap:12px;margin:10px 0">'
                    f'{marca}<span style="font-size:26px;font-weight:600;'
                    f'color:{TEXTO}">{valor}</span></div>')

            return "".join(filas)

        cuadrado = {c: (f'<span style="display:inline-block;width:30px;height:30px;'
                        f'border-radius:3px;background:{COLOR_PIEZA[c]}"></span>')
                    for c in "RGB"}
        formas = {c: _svg_forma(c, APAGADO) for c in "SHC"}

        self.contadores_color.content = bloque(
            cuadrado, e.por_color if e else {}, self.base_color, False)
        self.contadores_forma.content = bloque(
            formas, e.por_forma if e else {}, self.base_forma, True)

        # La grilla se sincroniza con el firmware mientras no se este
        # editando: en cuanto el operador toca una celda, manda lo que ve.
        if e and e.layout and not self.layout_editado:
            self.layout_editado = list(e.layout)

        self._pintar_caja()

        en_box = modo is pr.Modo.ALFAJORES

        self.boton_confirmar.set_enabled(est.enlace_vivo() and bool(self.layout_editado))
        self.boton_confirmar.style(
            f'background:{CELESTE if en_box else INACTIVO}!important;'
            f'color:{"#0B1220" if en_box else APAGADO}!important')

    def _pintar_caja(self) -> None:
        e = self.estado.e
        llenas = e.llenas if e and e.llenas else [False] * 6

        # Fuera del modo box la grilla se muestra en gris: sigue siendo
        # editable, pero no esta rigiendo nada en este momento.
        en_box = bool(e and e.modo is pr.Modo.ALFAJORES)

        for i, celda in enumerate(self.celdas):
            codigo = self.layout_editado[i] if i < len(self.layout_editado) else None
            color = (COLOR_PIEZA.get(codigo) if en_box else APAGADO) if codigo else None
            llena = llenas[i] if i < len(llenas) else False

            circulo = (f'<circle cx="32" cy="26" r="17" fill="{color}" '
                       f'fill-opacity="{1.0 if (llena and en_box) else 0.30}" '
                       f'stroke="{color}" stroke-width="2"/>' if color else "")

            celda.content = (
                f'<svg viewBox="0 0 64 52" style="width:100%;height:auto">'
                f'<rect x="1" y="1" width="62" height="50" rx="4" fill="none" '
                f'stroke="{BORDE}" stroke-width="1"/>{circulo}</svg>')

    def _avisar_caja_completa(self) -> None:
        e = self.estado.e

        if not e or e.modo is not pr.Modo.ALFAJORES or not e.llenas:
            return

        completa = len(e.llenas) == 6 and all(e.llenas)

        if not completa:
            # Se rearma solo cuando la caja deja de estar llena, asi el
            # aviso vuelve a aparecer con la caja siguiente.
            self.caja_avisada = False
            return

        if self.caja_avisada:
            return

        self.caja_avisada = True

        if self._dialogo_caja is not None:
            self._dialogo_caja.open()
            return

        with ui.dialog() as dialogo, ui.card().style(f"background:{PANEL};color:{TEXTO}"):
            ui.label("Caja completa").classes("text-lg")
            ui.label("Las 6 celdas estan llenas. Retira la caja, poné una vacia "
                     "y confirmá para seguir.").style(f"color:{APAGADO};max-width:380px")

            with ui.row().classes("w-full justify-end gap-2"):
                ui.button("Despues", on_click=dialogo.close).props("flat dense no-caps")
                ui.button("Puse una caja nueva",
                          on_click=lambda: self._caja_nueva(dialogo)) \
                    .props("dense unelevated no-caps") \
                    .style(f"background:{CELESTE}!important;color:#0B1220!important")

        self._dialogo_caja = dialogo
        dialogo.open()

    def _latencia(self) -> None:
        est = self.estado
        param = est.parametros.get("vis_lat")

        if not param or param.valor is None:
            self.etiqueta_latencia.text = "—"
            return

        # Recien ahora se conoce el rango, asi que recien ahora existe el
        # slider. Se crea una sola vez, cuando llega la tabla.
        if self.slider_latencia is None:
            minimo = param.minimo if param.minimo is not None else -0.2
            maximo = param.maximo if param.maximo is not None else 0.3

            with self.caja_latencia:
                self.slider_latencia = ui.slider(
                    min=minimo, max=maximo, step=0.005, value=param.valor,
                    on_change=self._cambiar_latencia) \
                    .props("dense").classes("w-full")

        # El slider no se pisa mientras el operador lo esta arrastrando: se
        # sincroniza solo cuando el valor del firmware difiere de verdad.
        if self.slider_latencia.value is None or \
                abs((self.slider_latencia.value or 0) - param.valor) > 0.004:
            self.slider_latencia.value = param.valor

        # Lo que importa no son los segundos sino los centimetros que la
        # pieza avanza en ese tiempo: es lo que se ve errarle al gripper.
        cinta = est.parametros.get("cinta_cms")
        velocidad = cinta.valor if cinta and cinta.valor else 0.0

        self.etiqueta_latencia.text = (
            f"{param.valor * 1000:.0f} ms · {param.valor * velocidad:+.2f} cm")

    # ==================================================================
    #  MODO TEACH
    # ==================================================================
    #
    #  Reparto de trabajo con el firmware (ver pc/PROTOCOLO.md §6):
    #
    #    ESP32    recorta al volumen de trabajo, resuelve la cinemática,
    #             mueve y encadena los puntos de una ruta ya cargada.
    #    acá      dibuja, graba, guarda las secuencias con nombre y lleva la
    #             cuenta de a qué porcentaje se verificó cada una.
    #
    #  Lo que se manda para joguear es una DIRECCIÓN, no un destino, y esa
    #  dirección vence sola en el firmware. Es la diferencia entre que el
    #  brazo se pare cuando se cierra el navegador y que siga viaje.

    def _teach(self) -> None:
        with ui.row().classes("w-full gap-2 p-2 no-wrap").style("height:100%"):
            # ================= Movimientos grabados =================
            # A la izquierda y de alto completo: es la lista que se recorre
            # para elegir que reproducir, y con doce secuencias adentro de un
            # panel de un cuarto de alto habia que scrollear para ver
            # cualquier cosa. Se lleva el ancho que sobra (flex:1) porque el
            # volumen de al lado ya no lo usa: su panel se ajusta al dibujo.
            with ui.column().classes("panel p-3 gap-2 no-wrap") \
                    .style("flex:1 1 0;min-width:270px;height:100%;min-height:0"):
                with ui.row().classes("w-full items-center no-wrap gap-2"):
                    ui.label("Movimientos").classes("titulo").style("flex:1 1 0")
                    self.etiqueta_grabacion = ui.label("").style(
                        f"color:{APAGADO};font-size:12px")

                with ui.row().classes("w-full gap-2 no-wrap"):
                    self.boton_grabar = ui.button(
                        "Grabar  ·  R", on_click=self._teach_grabar) \
                        .props("unelevated dense no-caps").style("flex:1 1 0")
                    self.boton_reproducir = ui.button(
                        "Reproducir  ·  P", on_click=self._teach_reproducir) \
                        .props("unelevated dense no-caps").style("flex:1 1 0")

                self.lista_teach = ui.column().classes("w-full gap-0") \
                    .style("flex:1 1 0;min-height:0;overflow-y:auto")

            # ================= Volumen de trabajo =================
            # La columna se ajusta al ANCHO DEL DIBUJO en vez de estirarse.
            # El SVG tiene una relacion de aspecto fija, asi que adentro de
            # un panel mas ancho se centraba y dejaba dos franjas muertas a
            # los costados -- que es lo que se veia. Con 'aspect-ratio' sobre
            # la columna, el ancho sale del alto y el panel termina justo
            # donde termina el dibujo: mismo tamaño, sin los margenes.
            #
            # El alto tiene que ser el de la propiedad 'height' y no el que
            # reparte el flex: el ancho de una fila se resuelve ANTES que el
            # alto de sus hijos, asi que un alto que sale de estirarse todavia
            # no existe cuando hace falta. Por eso va aca (height:100%, que
            # cuelga de una cadena de altos definidos) y no en el panel.
            #
            # Los 60 px de mas son lo que el dibujo no ocupa: el titulo, el
            # padding y el panel de posicion de abajo. Aproximado a proposito
            # -- errarle por unos pixeles deja un margen de unos pocos, no las
            # franjas de antes.
            with ui.column().classes("gap-2 no-wrap items-stretch") \
                    .style(f"flex:0 1 auto;height:100%;min-height:0;"
                           f"min-width:320px;"
                           f"aspect-ratio:{ANCHO_PLANO:.0f} / {ALTO_PLANO + 60:.0f}"):
                with ui.column().classes("panel p-2 gap-1 no-wrap") \
                        .style("flex:1 1 0;min-height:0;min-width:0"):
                    ui.label("Volumen de trabajo").classes("titulo")
                    self.html_plano = ui.html().style(
                        "flex:1 1 0;min-height:0;width:100%")

                with ui.row().classes("panel px-3 py-2 gap-2").style("flex:0 0 auto"):
                    self.html_teach_pos = ui.html().classes("w-full")

            # ================= Controles =================
            with ui.column().classes("gap-2 no-wrap") \
                    .style("flex:0 0 360px;height:100%;min-height:0;align-items:stretch"):
                # --- modo ---
                with ui.column().classes("panel p-3 gap-2").style("flex:0 0 auto"):
                    with ui.row().classes("w-full items-center no-wrap gap-2"):
                        ui.label("Modo Teach").classes("titulo").style("flex:1 1 0")
                        self.chip_teach = ui.html()

                    self.boton_teach = ui.button("Entrar a Teach",
                                                 on_click=self._teach_alternar) \
                        .props("unelevated dense no-caps").classes("w-full")

                    ui.label("Se entra desde home y con las manos vacías: si hay "
                             "una pieza en vuelo, el robot la termina primero. "
                             "Mientras dure, la cinta queda parada y las piezas "
                             "que informe la visión se ignoran.") \
                        .style(f"color:{APAGADO};font-size:11.5px;line-height:1.35")

                # --- jog ---
                with ui.column().classes("panel p-3 gap-2").style("flex:0 0 auto"):
                    ui.label("Jog manual").classes("titulo")

                    with ui.row().classes("w-full gap-3 no-wrap items-center"):
                        self.zona_joystick = ui.element("div").style(
                            f"flex:0 0 {LADO_JOYSTICK:.0f}px;"
                            f"height:{LADO_JOYSTICK:.0f}px;position:relative;"
                            "cursor:crosshair;touch-action:none;user-select:none")

                        with self.zona_joystick:
                            self.html_joystick = ui.html().style(
                                "width:100%;height:100%;pointer-events:none")

                        # Eventos de PUNTERO y no de mouse, por la captura:
                        # 'setPointerCapture' hace que todos los eventos de
                        # ese puntero sigan llegando a este div aunque el
                        # cursor se vaya afuera. Sin eso el navegador deja de
                        # mandar 'mousemove' apenas se sale del cuadrado y el
                        # jog se cortaba solo -- que con un circulo de 150 px
                        # es a cada rato, porque para pedir velocidad maxima
                        # hay que estar justo contra el borde.
                        #
                        # Ojo: el que se suelta ya no es 'mouseleave' sino
                        # 'pointerup'/'pointercancel', que con la captura
                        # llegan igual con el cursor afuera. 'mouseleave'
                        # NO puede seguir estando: se dispara al cruzar el
                        # borde arrastrando, que es justamente lo que se
                        # quiere permitir.
                        self.zona_joystick.on(
                            "pointerdown", self._joy_apretar,
                            ["offsetX", "offsetY"],
                            js_handler="(e) => { "
                                       "e.currentTarget.setPointerCapture(e.pointerId); "
                                       "emit(e); }")
                        self.zona_joystick.on("pointermove", self._joy_mover,
                                              ["offsetX", "offsetY", "buttons"],
                                              throttle=PERIODO_TEACH_S)
                        self.zona_joystick.on("pointerup", self._joy_soltar)
                        self.zona_joystick.on("pointercancel", self._joy_soltar)

                        with ui.column().classes("gap-2 no-wrap").style("flex:1 1 0"):
                            self.boton_z_sube = self._boton_pulsado(
                                "keyboard_arrow_up", "Subir  ↑", "arriba")
                            self.boton_z_baja = self._boton_pulsado(
                                "keyboard_arrow_down", "Bajar  ↓", "abajo")
                            self.boton_bomba = ui.button(
                                "Vacío  ·  E", on_click=self._teach_bomba) \
                                .props("unelevated dense no-caps").classes("w-full")

                    ui.label("Arrastrá el joystick con el mouse (podés salirte del "
                             "círculo sin soltar) o usá W A S D "
                             "en el plano, ↑ ↓ para la altura y E para el vacío. "
                             "La velocidad y la aceleración van al 15 %.") \
                        .style(f"color:{APAGADO};font-size:11.5px;line-height:1.35")

                # El hueco empuja el módulo de coordenada contra el borde de
                # abajo: el modo y el jog no crecen, y la lista de
                # movimientos -- lo único que tiene sentido estirar -- está
                # del otro lado.
                ui.element("div").style("flex:1 1 0;min-height:0")

                # --- ir a una coordenada ---
                with ui.column().classes("panel p-3 gap-2").style("flex:0 0 auto"):
                    ui.label("Ir a una coordenada").classes("titulo")

                    with ui.row().classes("w-full gap-2 no-wrap items-center"):
                        self.campos_ir = {}

                        for eje in ("x", "y", "z"):
                            campo = ui.number(label=eje.upper(), value=None,
                                              step=0.5, format="%.2f") \
                                .props("dense outlined").style("flex:1 1 0")
                            campo.on("keydown.enter", lambda _: self._teach_ir())
                            self.campos_ir[eje] = campo

                        self.boton_ir = ui.button("Ir", on_click=self._teach_ir) \
                            .props("unelevated dense no-caps") \
                            .style("flex:0 0 66px")

                    # El rango se escribe acá y no se deja para el rechazo: es
                    # la diferencia entre elegir un número que entra y probar
                    # a ver cuál entra.
                    # El rango es lo ÚNICO que va debajo de los campos: el
                    # panel tiene que entrar en la columna sin scrollear, y
                    # una explicación de tres renglones que se lee una vez en
                    # la vida no vale ese precio. Lo que hace el botón está
                    # en PROTOCOLO.md §6.3.1.
                    self.etiqueta_ir = ui.label("").style(
                        f"color:{APAGADO};font-size:11.5px;line-height:1.35")

        self._teach_rearmar_lista()

    def _boton_pulsado(self, icono: str, texto: str, tecla: str):
        """Botón que jogea MIENTRAS se lo mantiene apretado.

        No es un click: soltar tiene que frenar el brazo, igual que soltar la
        flecha del teclado. `mouseleave` está a propósito -- si el mouse se
        va del botón con el dedo apretado, el navegador nunca manda el
        `mouseup` y el eje se quedaría subiendo solo.
        """

        boton = ui.button(texto, icon=icono) \
            .props("unelevated dense no-caps").classes("w-full")

        boton.on("mousedown", lambda _, k=tecla: self.teach_teclas.add(k))
        boton.on("mouseup", lambda _, k=tecla: self.teach_teclas.discard(k))
        boton.on("mouseleave", lambda _, k=tecla: self.teach_teclas.discard(k))

        return boton

    # ------------------------------------------------------------------
    #  Volumen de trabajo
    # ------------------------------------------------------------------
    def _teach_limites(self) -> tuple:
        """(xmin,xmax), (ymin,ymax), (zmin,zmax) del volumen del jog.

        Primero lo que contestó `J?`, que ya trae el piso en Z resuelto; si
        todavía no llegó, se arma con la tabla de parámetros. Los números
        escritos acá son el último recurso y NO son la fuente de verdad:
        están para que la pantalla dibuje algo coherente antes de que
        conteste el robot, no para decidir hasta dónde se mueve.
        """

        t = self.estado.teach

        if t and t.limite_x and t.limite_y and t.limite_z:
            return t.limite_x, t.limite_y, t.limite_z

        p = self.estado.parametros

        def val(nombre: str, defecto: float) -> float:
            q = p.get(nombre)
            return q.valor if q and q.valor is not None else defecto

        zmin = val("grab_z", -32.6)

        return ((val("t_xmin", -12.0), val("t_xmax", 12.0)),
                (val("t_ymin", -9.55), val("t_ymax", 11.05)),
                (zmin, zmin + val("t_zup", 6.0)))

    def _en_teach(self) -> bool:
        e = self.estado.e
        return bool(e and e.estado is pr.EstadoRobot.TEACH and self.estado.enlace_vivo())

    def _reproduciendo(self) -> bool:
        e = self.estado.e
        return bool(e and e.teach_indice)

    # ------------------------------------------------------------------
    #  Entradas del operador
    # ------------------------------------------------------------------
    def _cambio_pestana(self, evento) -> None:
        self.tab_activa = str(evento.value)

        # Cambiar de pestaña con una tecla apretada dejaría el brazo yendo:
        # el timer manda el vector nulo en cuanto la pestaña deja de ser la
        # de teach, pero el estado local se limpia acá igual.
        self.teach_teclas.clear()
        self.joy_x = self.joy_y = 0.0
        self.joy_tomado = False

        if self.tab_activa == "Teach":
            self.enviar(pr.cmd_teach_estado())

    def _teach_tecla(self, evento) -> None:
        if self.tab_activa != "Teach":
            return

        codigo = evento.key.code

        movimiento = {"KeyW": "arriba_y", "KeyS": "abajo_y",
                      "KeyA": "izq", "KeyD": "der",
                      "ArrowUp": "arriba", "ArrowDown": "abajo"}

        if codigo in movimiento:
            if evento.action.keydown:
                self.teach_teclas.add(movimiento[codigo])
            elif evento.action.keyup:
                self.teach_teclas.discard(movimiento[codigo])
            return

        # Las que alternan algo se atienden una sola vez: `repeat` es el
        # autorepeat del sistema operativo y prendería y apagaría la bomba
        # treinta veces por segundo con la tecla apretada.
        if not evento.action.keydown or evento.action.repeat:
            return

        if codigo == "KeyE":
            self._teach_bomba()
        elif codigo == "KeyR":
            self._teach_grabar()
        elif codigo == "KeyP":
            self._teach_reproducir()

    def _joy_apretar(self, evento) -> None:
        self.joy_tomado = True
        self._joy_desde_pixeles(evento)

    def _joy_mover(self, evento) -> None:
        # `buttons` es el bitmap de botones apretados AHORA. Sin mirarlo, el
        # joystick seguiría a un mouse que ya se soltó.
        if not self.joy_tomado or not evento.args.get("buttons"):
            self._joy_soltar()
            return

        self._joy_desde_pixeles(evento)

    def _joy_soltar(self, evento=None) -> None:
        self.joy_tomado = False
        self.joy_x = 0.0
        self.joy_y = 0.0

    def _joy_desde_pixeles(self, evento) -> None:
        lado = LADO_JOYSTICK
        radio = lado / 2.0

        dx = (float(evento.args.get("offsetX", radio)) - radio) / radio
        dy = (radio - float(evento.args.get("offsetY", radio))) / radio

        largo = math.hypot(dx, dy)

        # Fuera del círculo se satura en el borde en vez de crecer: el
        # cuadrado del div tiene esquinas, y el brazo no tiene por qué ir más
        # rápido en diagonal que de frente. Con la captura del puntero esto
        # dejó de ser el caso raro y pasó a ser el normal: el cursor se va
        # bien afuera del div (offsets negativos o mayores al lado) y lo que
        # se manda es el borde en esa dirección, o sea velocidad máxima hacia
        # donde apunta la mano. Es lo que hace que no haya que apuntarle a un
        # anillo de pocos píxeles para ir rápido.
        if largo > 1.0:
            dx, dy = dx / largo, dy / largo

        # Zona muerta: sin esto, el temblor de la mano manda un jog de 0,02
        # que igual dispara tramos y hace vibrar el brazo.
        if math.hypot(dx, dy) < 0.12:
            dx = dy = 0.0

        self.joy_x, self.joy_y = dx, dy

    # ------------------------------------------------------------------
    #  Lazo del jog
    # ------------------------------------------------------------------
    def _teach_tick(self) -> None:
        est = self.estado
        activo = (self.tab_activa == "Teach") and self._en_teach()

        # El volcado de la posición comandada se enciende sólo mientras hace
        # falta: son 760 B/s del enlace, y con la pantalla en otra pestaña no
        # los mira nadie.
        if activo != self._volcado_on:
            self._volcado_on = activo
            self.enviar(pr.cmd_teach_volcado(activo))

        self._teach_eventos()

        if not activo or self._reproduciendo() or self._yendo():
            self.teach_teclas.clear()
            self.joy_x = self.joy_y = 0.0
            self._jog_enviar(0.0, 0.0, 0.0)
            return

        teclas = self.teach_teclas

        vx = self.joy_x + ("der" in teclas) - ("izq" in teclas)
        vy = self.joy_y + ("arriba_y" in teclas) - ("abajo_y" in teclas)
        vz = ("arriba" in teclas) - ("abajo" in teclas)

        largo = math.hypot(vx, vy)

        if largo > 1.0:
            vx, vy = vx / largo, vy / largo

        self._jog_enviar(vx, vy, float(vz))

        if self.teach_grabando and est.teach_pos is not None:
            x, y, z = est.teach_pos

            # Tope de seguridad: 20 Hz por 20 minutos son 24.000 muestras, y
            # a esa altura ya no es un movimiento enseñado sino un olvido.
            if len(self.teach_muestras) < 24000:
                self.teach_muestras.append(
                    tch.Muestra(time.monotonic() - self.teach_t0,
                                x, y, z, bool(est.teach_bomba)))

    def _jog_enviar(self, vx: float, vy: float, vz: float) -> None:
        """Manda la dirección, con el mínimo de líneas posible.

        Un vector que no cambió igual se reenvía cada REFRESCO_JOG_S: en el
        firmware la dirección vence, y dejar de refrescarla es justamente
        cómo se frena el brazo si esta interfaz se muere.
        """

        v = (round(vx, 2), round(vy, 2), round(vz, 2))
        quieto = (v == (0.0, 0.0, 0.0))
        ahora = time.monotonic()

        if v == self.jog_ultimo:
            if quieto or (ahora - self.jog_enviado_s) < REFRESCO_JOG_S:
                return

        self.jog_ultimo = v
        self.jog_enviado_s = ahora
        self.enviar(pr.cmd_teach_jog(*v))

    def _sin_foco(self) -> None:
        """Le saca el foco al botón que se acaba de apretar.

        Si no, la barra espaciadora vuelve a activarlo en vez de subir el
        brazo: el navegador activa con Espacio el elemento enfocado, y eso
        pasa por encima de cualquier atajo de teclado.
        """

        ui.run_javascript("document.activeElement && document.activeElement.blur()")

    def _teach_alternar(self) -> None:
        self._sin_foco()

        if self._en_teach():
            # Salir cancela todo lo que estuviera en curso, así que la marca
            # del "ir a" se limpia acá y no vence sola: si no, al volver a
            # entrar la pantalla seguiría creyendo que hay algo yendo.
            self.teach_yendo_hasta = 0.0
            self.enviar(pr.cmd_teach(False))
            return

        e = self.estado.e

        if e and e.homed is False:
            ui.notify("El robot no está calibrado: hay que hacer el homing antes",
                      color="negative")
            return

        if e and e.estado is pr.EstadoRobot.ERROR:
            ui.notify("Desde ERROR hay que rehomear antes de mover el brazo a mano",
                      color="negative")
            return

        self.enviar(pr.cmd_teach(True))
        ui.notify("Entrando a Teach: el robot termina lo que tenga y vuelve a home",
                  color="info")

    def _teach_bomba(self) -> None:
        self._sin_foco()

        if not self._en_teach():
            return

        t = self.estado.t
        self.enviar(pr.cmd_teach_bomba(not bool(t and t.bomba)))

    # ------------------------------------------------------------------
    #  Ir a una coordenada escrita
    # ------------------------------------------------------------------
    def _teach_ir(self) -> None:
        """Manda el brazo a la coordenada de los tres campos.

        El camino (por home si hace falta) y el recorte al volumen los
        resuelve el firmware, que es el que no puede equivocarse. Lo que se
        hace acá es no mandarle un pedido que ya se sabe que va a rechazar:
        un rechazo se ve como una línea roja en la consola, y el operador se
        queda sin saber cuál de los tres números estaba mal.
        """

        self._sin_foco()

        if not self._en_teach():
            ui.notify("Primero hay que entrar al modo Teach", color="warning")
            return

        if self.teach_grabando or self._reproduciendo() or self._yendo():
            return

        valores = {}

        for eje, campo in self.campos_ir.items():
            if campo.value is None:
                ui.notify("Escribí las tres coordenadas", color="warning")
                return

            valores[eje] = float(campo.value)

        limites = dict(zip("xyz", self._teach_limites()))

        for eje, (lo, hi) in limites.items():
            v = valores[eje]

            if not lo <= v <= hi:
                ui.notify(f"{eje.upper()} = {v:.2f} cm queda fuera del volumen "
                          f"({lo:.2f} a {hi:.2f} cm)", color="warning")
                return

        x, y, z = valores["x"], valores["y"], valores["z"]

        # El cajón tiene esquinas a las que un delta no llega: estar adentro
        # de los límites no alcanza para que el punto exista.
        if not cin.alcanzable(x, y, z):
            ui.notify("El brazo no llega a ese punto: está adentro de los "
                      "límites pero fuera de su alcance", color="warning")
            return

        if not self.enviar(pr.cmd_teach_ir(x, y, z)):
            ui.notify("sin enlace con el robot", color="negative")
            return

        # Se bloquea CORTO hasta que el firmware confirme (`[TEACH] ir`), y
        # recién ahí por el tiempo largo del movimiento. Si la placa no
        # entiende el comando -- firmware viejo -- no contesta nada, y con el
        # tiempo largo la pantalla se quedaba veinte segundos diciendo que
        # estaba yendo, con el botón de parar sin nada que parar.
        self.teach_yendo_hasta = time.monotonic() + ESPERA_IR_CONFIRMA_S

    def _yendo(self) -> bool:
        return time.monotonic() < self.teach_yendo_hasta

    # ------------------------------------------------------------------
    #  Grabación
    # ------------------------------------------------------------------
    def _teach_grabar(self) -> None:
        self._sin_foco()

        if not self.teach_grabando:
            if not self._en_teach():
                ui.notify("Primero hay que entrar al modo Teach", color="warning")
                return

            self.teach_muestras = []
            self.teach_t0 = time.monotonic()
            self.teach_grabando = True

            ui.notify("Grabando. Mové el brazo; R de nuevo para terminar.",
                      color="info")
            return

        self.teach_grabando = False

        puntos = tch.simplificar(self.teach_muestras)
        duracion = self.teach_muestras[-1].t if self.teach_muestras else 0.0

        if len(puntos) < 2:
            ui.notify("No se movió nada: no hay movimiento que guardar",
                      color="warning")
            return

        movimiento = self.biblioteca.agregar(tch.Movimiento(
            nombre=self.biblioteca.nombre_libre(),
            puntos=puntos,
            verificado=tch.SIN_VERIFICAR,
            creado=tch.ahora_texto(),
            duracion_s=duracion))

        self.teach_sel = len(self.biblioteca.movimientos) - 1
        self._teach_rearmar_lista()

        ui.notify(f"Guardado «{movimiento.nombre}»: {len(puntos)} puntos, "
                  f"{duracion:.1f} s de grabación", color="positive")

    # ------------------------------------------------------------------
    #  Reproducción por etapas
    # ------------------------------------------------------------------
    def _teach_reproducir(self) -> None:
        self._sin_foco()

        # Con algo en curso, el mismo botón (y la misma tecla) es el de
        # parar. Es a propósito: parar es lo único que hay que poder hacer
        # sin buscar nada, y con el brazo yendo no existe otra cosa que ese
        # botón pueda significar.
        if self._reproduciendo() or self._yendo():
            self.enviar(pr.cmd_teach_abortar())
            return

        movimiento = self.biblioteca.obtener(self.teach_sel) \
            if self.teach_sel is not None else None

        if movimiento is None:
            ui.notify("Elegí un movimiento de la lista", color="warning")
            return

        if not self._en_teach():
            ui.notify("Primero hay que entrar al modo Teach", color="warning")
            return

        if self.teach_grabando:
            return

        self._teach_lanzar(movimiento, movimiento.siguiente_escalon)

    def _teach_lanzar(self, movimiento, pct: int) -> None:
        """Sube la ruta al firmware y la arranca al `pct` % de vel y acel.

        La subida va de a poco y no de un saque: son hasta 150 líneas, y
        escribirlas todas juntas dejaría la interfaz congelada casi un
        segundo mientras el puerto las traga.
        """

        if not self.enviar(pr.cmd_teach_limpiar()):
            ui.notify("sin enlace con el robot", color="negative")
            return

        self.teach_mov_en_curso = movimiento
        self.teach_pct_en_curso = int(pct)
        self._cola_subida = list(movimiento.puntos)

        if self._timer_subida is not None:
            self._timer_subida.cancel()

        self._timer_subida = ui.timer(0.04, self._teach_subir_trozo)

    def _teach_subir_trozo(self) -> None:
        for _ in range(10):
            if not self._cola_subida:
                if self._timer_subida is not None:
                    self._timer_subida.cancel()
                    self._timer_subida = None

                self.enviar(pr.cmd_teach_reproducir(self.teach_pct_en_curso))
                return

            p = self._cola_subida.pop(0)

            if not self.enviar(pr.cmd_teach_punto(p.x, p.y, p.z, p.bomba, p.espera_ms)):
                self._cola_subida.clear()
                self.teach_mov_en_curso = None

                if self._timer_subida is not None:
                    self._timer_subida.cancel()
                    self._timer_subida = None

                ui.notify("se cortó el enlace durante la carga", color="negative")
                return

    def _teach_eventos(self) -> None:
        """Consume los eventos `[TEACH]` que todavía no se atendieron."""

        est = self.estado

        if est.teach_evento_n == self._teach_evento_visto:
            return

        self._teach_evento_visto = est.teach_evento_n
        evento = est.teach_evento

        if evento is None:
            return

        if evento.evento == "fin":
            self._teach_preguntar()

        elif evento.evento == "ir":
            # El firmware confirma que lo tomó: a partir de acá el jog y los
            # botones quedan en pausa hasta que llegue (o hasta que venza).
            self.teach_yendo_hasta = time.monotonic() + ESPERA_IR_S

        elif evento.evento == "irfin":
            self.teach_yendo_hasta = 0.0

        elif evento.evento == "abort":
            self.teach_mov_en_curso = None
            self.teach_yendo_hasta = 0.0
            ui.notify(f"Movimiento cortado ({evento.motivo or 'sin motivo'})",
                      color="negative")

        elif evento.evento == "err":
            self.teach_mov_en_curso = None
            self.teach_yendo_hasta = 0.0
            ui.notify(f"Teach: {evento.error}", color="negative")

    def _teach_preguntar(self) -> None:
        """El cartel de "¿salió bien?" que separa una etapa de la siguiente.

        Es lo único que sube el movimiento de escalón. La primera pasada va a
        la misma velocidad a la que se lo enseñó (15 %) y recién con una
        confirmación pasa al 50 %, y con otra al 100 %. Un movimiento recién
        grabado nunca se estrena a fondo: la trayectoria puede pasar cerca de
        algo que a paso de hombre no roza y a toda velocidad sí.
        """

        movimiento = self.teach_mov_en_curso
        pct = self.teach_pct_en_curso

        self.teach_mov_en_curso = None

        if movimiento is None:
            return

        # Un movimiento que YA estaba verificado al 100 % no tiene nada que
        # confirmar: el cartel existe para subir de escalón, y de acá no se
        # sube. Se pregunta por el 100 % una sola vez, la que lo verifica.
        if not movimiento.falta_verificar:
            return

        if self._dialogo_teach is not None:
            self._dialogo_teach.delete()
            self._dialogo_teach = None

        ultimo = pct >= 100
        siguiente = tch._SIGUIENTE.get(pct, 100)

        with ui.dialog() as dialogo, ui.card().style(f"background:{PANEL};color:{TEXTO}"):
            ui.label(f"¿Salió bien al {pct} %?").classes("text-lg")
            ui.label(f"«{movimiento.nombre}» terminó de reproducirse al {pct} % "
                     "de la velocidad y la aceleración máximas."
                     + ("" if ultimo else
                        f" Si estuvo limpio, la próxima pasada va al {siguiente} %.")) \
                .style(f"color:{APAGADO};max-width:420px")

            with ui.row().classes("w-full justify-end gap-2"):
                ui.button("No", on_click=lambda: self._teach_rechazar(dialogo)) \
                    .props("flat dense no-caps")

                ui.button("Sí, quedó verificado" if ultimo
                          else f"Sí — probar al {siguiente} %",
                          on_click=lambda m=movimiento, p=pct:
                              self._teach_aprobar(dialogo, m, p)) \
                    .props("dense unelevated no-caps") \
                    .style(f"background:{CELESTE}!important;color:#0B1220!important")

        self._dialogo_teach = dialogo
        dialogo.open()

    def _teach_aprobar(self, dialogo, movimiento, pct: int) -> None:
        movimiento.aprobar(pct)
        self.biblioteca.guardar()
        self._teach_rearmar_lista()

        dialogo.close()

        if pct >= 100:
            ui.notify(f"«{movimiento.nombre}» verificado al 100 %", color="positive")
            return

        # Se encadena sola: el operador ya dijo que la pasada anterior estuvo
        # bien, y hacerle buscar otra vez el botón entre etapa y etapa es la
        # forma más fácil de que termine salteándose la verificación.
        self._teach_lanzar(movimiento, movimiento.siguiente_escalon)

    def _teach_rechazar(self, dialogo) -> None:
        dialogo.close()
        ui.notify("Se deja como estaba: la próxima vuelve a arrancar por el "
                  "escalón que faltaba", color="warning")

    # ------------------------------------------------------------------
    #  Lista de movimientos
    # ------------------------------------------------------------------
    def _teach_rearmar_lista(self) -> None:
        """Rehace la lista entera. Se llama sólo cuando cambia el conjunto.

        Rearmarla en cada refresco le sacaría el foco al campo del nombre a
        mitad de escribirlo, que es el mismo motivo por el que la lista de
        parámetros tampoco se rearma sola.
        """

        if not hasattr(self, "lista_teach"):
            return

        self.lista_teach.clear()
        self._filas_teach = []

        if not self.biblioteca.movimientos:
            with self.lista_teach:
                ui.label("Todavía no hay movimientos grabados. Entrá a Teach, "
                         "apretá R, mové el brazo y volvé a apretar R.") \
                    .style(f"color:{APAGADO};font-size:12px;padding:10px 4px;"
                           "white-space:normal")
            return

        for i, movimiento in enumerate(self.biblioteca.movimientos):
            with self.lista_teach:
                fila = ui.row().classes("w-full items-center no-wrap gap-2") \
                    .style(f"padding:6px 4px;border-bottom:1px solid {BORDE};"
                           "cursor:pointer")

                with fila:
                    fila.on("click", lambda _, k=i: self._teach_elegir(k))

                    marca = ui.html().style("flex:0 0 10px")

                    with ui.column().classes("gap-0").style("flex:1 1 0;min-width:0"):
                        campo = ui.input(value=movimiento.nombre) \
                            .props("dense borderless").classes("w-full") \
                            .style(f"color:{TEXTO};font-size:13px")
                        campo.on("blur", lambda _, k=i: self._teach_renombrar(k))
                        campo.on("keydown.enter", lambda _, k=i: self._teach_renombrar(k))

                        info = ui.label("").style(
                            f"color:{APAGADO};font-size:11px")

                    insignia = ui.html().style("flex:0 0 auto")

                    ui.button(icon="play_arrow",
                              on_click=lambda _, k=i: self._teach_elegir(k, True)) \
                        .props("flat dense round size=sm").style(f"color:{CELESTE}")

                    ui.button(icon="delete_outline",
                              on_click=lambda _, k=i: self._teach_borrar(k)) \
                        .props("flat dense round size=sm").style(f"color:{APAGADO}")

            self._filas_teach.append({"fila": fila, "marca": marca,
                                      "campo": campo, "info": info,
                                      "insignia": insignia})

        self._teach_pintar_lista()

    def _teach_pintar_lista(self) -> None:
        colores = {0: ROJO, 15: AMBAR, 50: AMBAR, 100: VERDE}

        for i, fila in enumerate(self._filas_teach):
            movimiento = self.biblioteca.obtener(i)

            if movimiento is None:
                continue

            elegido = (i == self.teach_sel)

            fila["fila"].style(
                f"padding:6px 4px;border-bottom:1px solid {BORDE};cursor:pointer;"
                f"background:{INACTIVO if elegido else 'transparent'}")

            fila["marca"].content = (
                '<span style="display:inline-block;width:4px;height:26px;'
                f'border-radius:2px;background:{CELESTE if elegido else "transparent"}">'
                '</span>')

            # El aviso de "sólo en esta PC" hace visible la regla de
            # `teach.es_local`: lo que conserva el nombre de fábrica no viaja
            # con el proyecto. Sin esto, la regla es magia y un movimiento
            # que costó una tarde se pierde en la próxima máquina.
            fila["info"].text = (f"{len(movimiento.puntos)} puntos · "
                                 f"{movimiento.duracion_s:.1f} s · {movimiento.creado}"
                                 + (" · sólo en esta PC"
                                    if tch.es_local(movimiento.nombre) else ""))

            color = COLOR_ESTADO[colores.get(movimiento.verificado, ROJO)]
            texto = ("sin verificar" if movimiento.verificado == 0
                     else f"{movimiento.verificado} %")

            fila["insignia"].content = (
                f'<span style="border:1px solid {color};color:{color};'
                'font-size:10.5px;border-radius:9px;padding:1px 7px;'
                f'white-space:nowrap">{texto}</span>')

    def _teach_elegir(self, indice: int, reproducir: bool = False) -> None:
        self.teach_sel = indice
        self._teach_pintar_lista()

        if reproducir:
            self._teach_reproducir()

    def _teach_renombrar(self, indice: int) -> None:
        fila = self._filas_teach[indice] if indice < len(self._filas_teach) else None

        if fila is None:
            return

        self.biblioteca.renombrar(indice, str(fila["campo"].value or ""))

        movimiento = self.biblioteca.obtener(indice)

        if movimiento is not None:
            fila["campo"].value = movimiento.nombre

            # Renombrar puede haber sacado el movimiento del archivo local:
            # el aviso de la fila tiene que reflejarlo en el acto.
            self._teach_pintar_lista()

    def _teach_borrar(self, indice: int) -> None:
        movimiento = self.biblioteca.obtener(indice)

        if movimiento is None:
            return

        with ui.dialog() as dialogo, ui.card().style(f"background:{PANEL};color:{TEXTO}"):
            ui.label(f"Borrar «{movimiento.nombre}»").classes("text-lg")
            ui.label("No se puede deshacer: la secuencia se borra del archivo.") \
                .style(f"color:{APAGADO};max-width:360px")

            with ui.row().classes("w-full justify-end gap-2"):
                ui.button("Cancelar", on_click=dialogo.close).props("flat dense no-caps")
                ui.button("Borrar",
                          on_click=lambda: self._teach_borrar_ya(dialogo, indice)) \
                    .props("dense unelevated no-caps") \
                    .style(f"background:{ROJO_STOP}!important;color:#fff!important")

        dialogo.open()

    def _teach_borrar_ya(self, dialogo, indice: int) -> None:
        self.biblioteca.borrar(indice)

        if self.teach_sel is not None:
            if self.teach_sel == indice:
                self.teach_sel = None
            elif self.teach_sel > indice:
                self.teach_sel -= 1

        dialogo.close()
        self._teach_rearmar_lista()

    # ------------------------------------------------------------------
    #  Dibujo
    # ------------------------------------------------------------------
    def _zona_alcanzable(self, z: float, limites) -> list:
        """Franjas verticales de la zona a la que el brazo llega a esa altura.

        El volumen del jog es un cajón y el alcance real de un delta no lo es:
        las esquinas quedan afuera. Pintarlo evita el desconcierto de ver el
        brazo dejar de avanzar sin que nada avise.

        Se resuelve una cinemática inversa por celda, así que el resultado se
        guarda en caché por altura redondeada -- si no, subir y bajar con el
        jog recalcularía la grilla entera diez veces por segundo.
        """

        clave = (round(z / PASO_CACHE_Z), limites)

        if self._cache_alcance[0] == clave:
            return self._cache_alcance[1]

        (xmin, xmax), (ymin, ymax) = limites[0], limites[1]

        partes = []
        ancho = (xmax - xmin) / COLUMNAS_ALCANCE

        for i in range(COLUMNAS_ALCANCE):
            x = xmin + ancho * (i + 0.5)
            alcanza = [ymin + (ymax - ymin) * j / (FILAS_ALCANCE - 1)
                       for j in range(FILAS_ALCANCE)
                       if cin.alcanzable(x, ymin + (ymax - ymin) * j / (FILAS_ALCANCE - 1), z)]

            if alcanza:
                partes.append((x - ancho / 2.0, x + ancho / 2.0,
                               min(alcanza), max(alcanza)))

        self._cache_alcance = (clave, partes)

        return partes

    def _svg_plano(self) -> str:
        """El volumen de trabajo en isométrica, con el gripper adentro.

        Un cubo dibujado con tres ejes a 30°: X baja hacia la derecha, Y se
        aleja hacia arriba-derecha (la cinta queda al fondo) y Z es vertical.
        Es la misma vista de un plano en perspectiva de taller, y se lee de
        un vistazo — que era lo que no pasaba con un plano XY y una barra de
        altura por separado.

        La profundidad de una isométrica es ambigua a propósito (un punto
        alto y cerca se dibuja igual que uno bajo y lejos), así que la altura
        NO se deja librada al dibujo: el gripper lleva una vertical hasta el
        piso y una sombra ahí abajo, y el número sigue estando escrito.
        """

        est = self.estado
        limites = self._teach_limites()
        (xmin, xmax), (ymin, ymax), (zmin, zmax) = limites

        ancho, alto = ANCHO_PLANO, ALTO_PLANO

        cos30 = 0.8660254
        sin30 = 0.5

        # --- proyección ---
        # u = X (a la derecha), v = cuánto se ACERCA al operador, w = altura.
        # Con v así, la cinta (Y grande) queda al fondo de la escena, que es
        # como la ve el operador parado delante del robot.
        def iso(x: float, y: float, z: float) -> tuple[float, float]:
            u = x - xmin
            v = ymax - y
            w = z - zmin

            return ((u - v) * cos30, (u + v) * sin30 - w)

        esquinas = [iso(x, y, z)
                    for x in (xmin, xmax) for y in (ymin, ymax) for z in (zmin, zmax)]

        sx0 = min(p[0] for p in esquinas)
        sx1 = max(p[0] for p in esquinas)
        sy0 = min(p[1] for p in esquinas)
        sy1 = max(p[1] for p in esquinas)

        margen = 34.0
        escala = min((ancho - 2 * margen) / max(sx1 - sx0, 1e-3),
                     (alto - 2 * margen) / max(sy1 - sy0, 1e-3))

        cx = (ancho - (sx1 - sx0) * escala) / 2.0 - sx0 * escala
        cy = (alto - (sy1 - sy0) * escala) / 2.0 - sy0 * escala

        def pt(x: float, y: float, z: float) -> tuple[float, float]:
            sx, sy = iso(x, y, z)
            return (cx + sx * escala, cy + sy * escala)

        def linea(a, b, color, gruesa=1.0, guion=None, opacidad=1.0) -> str:
            (x1, y1), (x2, y2) = pt(*a), pt(*b)
            d = f' stroke-dasharray="{guion}"' if guion else ""
            return (f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
                    f'stroke="{color}" stroke-width="{gruesa}" '
                    f'stroke-opacity="{opacidad}"{d}/>')

        p = [f'<svg viewBox="0 0 {ancho:.0f} {alto:.0f}" '
             'style="width:100%;height:100%;display:block">',
             f'<rect x="0" y="0" width="{ancho}" height="{alto}" fill="{FONDO}"/>']

        medido = cin.directa_desde(est.angulo_suave) if est.enlace_vivo() else None
        destino = est.teach_pos
        z_actual = destino[2] if destino else (medido[2] if medido else zmin)

        # --- piso: el rectángulo de abajo, relleno ---
        piso = [pt(xmin, ymin, zmin), pt(xmax, ymin, zmin),
                pt(xmax, ymax, zmin), pt(xmin, ymax, zmin)]
        p.append('<polygon points="'
                 + " ".join(f"{q[0]:.1f},{q[1]:.1f}" for q in piso)
                 + f'" fill="{PANEL}" fill-opacity="0.55" stroke="none"/>')

        # --- zona alcanzable a la altura actual, apoyada en el piso ---
        for x0, x1, y0, y1 in self._zona_alcanzable(z_actual, limites):
            franja = [pt(x0, y0, zmin), pt(x1, y0, zmin),
                      pt(x1, y1, zmin), pt(x0, y1, zmin)]
            p.append('<polygon points="'
                     + " ".join(f"{q[0]:.1f},{q[1]:.1f}" for q in franja)
                     + f'" fill="{CELESTE}" fill-opacity="0.10" stroke="none"/>')

        # --- grilla del piso, cada 4 cm ---
        for x in range(int(math.ceil(xmin / 4)) * 4, int(xmax) + 1, 4):
            p.append(linea((x, ymin, zmin), (x, ymax, zmin), BORDE, 0.6, "2 5"))

        for y in range(int(math.ceil(ymin / 4)) * 4, int(ymax) + 1, 4):
            p.append(linea((xmin, y, zmin), (xmax, y, zmin), BORDE, 0.6, "2 5"))

        # --- aristas ocultas (las tres que salen del vértice del fondo) ---
        oculto = (xmin, ymax, zmin)
        for otro in ((xmax, ymax, zmin), (xmin, ymin, zmin), (xmin, ymax, zmax)):
            p.append(linea(oculto, otro, BORDE, 1.0, "3 4", 0.55))

        # --- la caja: piso, techo y las cuatro verticales ---
        for z in (zmin, zmax):
            for a, b in (((xmin, ymin, z), (xmax, ymin, z)),
                         ((xmax, ymin, z), (xmax, ymax, z)),
                         ((xmax, ymax, z), (xmin, ymax, z)),
                         ((xmin, ymax, z), (xmin, ymin, z))):
                if (xmin, ymax, zmin) in (a, b) and z == zmin:
                    continue  # ya se dibujó punteada
                p.append(linea(a, b, BORDE, 1.2))

        for x in (xmin, xmax):
            for y in (ymin, ymax):
                if (x, y) == (xmin, ymax):
                    continue
                p.append(linea((x, y, zmin), (x, y, zmax), BORDE, 1.2))

        # --- referencias de la mesa ---
        cinta_y = min(ymax, 12.05)

        if ymin < cinta_y <= ymax:
            p.append(linea((xmin, cinta_y, zmin), (xmax, cinta_y, zmin),
                           APAGADO, 1.2, "6 4", 0.8))
            qx, qy = pt(xmax, cinta_y, zmin)
            p.append(f'<text x="{qx + 6:.1f}" y="{qy + 4:.1f}" fill="{APAGADO}" '
                     'font-size="10" font-family="system-ui">cinta</text>')

        par_ = est.parametros

        def valor(nombre, defecto):
            q = par_.get(nombre)
            return q.valor if q and q.valor is not None else defecto

        bin_y = valor("bin_y", -9.55)

        for nombre in ("bin_x1", "bin_x2", "bin_x3"):
            bx = valor(nombre, None)

            if bx is None or not (xmin <= bx <= xmax) or not (ymin <= bin_y <= ymax):
                continue

            qx, qy = pt(bx, bin_y, zmin)
            p.append(f'<ellipse cx="{qx:.1f}" cy="{qy:.1f}" rx="11" ry="6.4" '
                     f'fill="none" stroke="{APAGADO}" stroke-width="1" '
                     'stroke-dasharray="3 3"/>')

        # --- trayectoria ---
        camino = []

        if self.teach_grabando:
            camino = [(m.x, m.y, m.z, m.bomba) for m in self.teach_muestras[-1200:]]
        else:
            movimiento = self.biblioteca.obtener(self.teach_sel) \
                if self.teach_sel is not None else None

            if movimiento is not None:
                camino = [(q.x, q.y, q.z, q.bomba) for q in movimiento.puntos]

        # La sombra del camino en el piso: sin ella, en isométrica no se
        # distingue un tramo que sube de uno que se aleja.
        for a, b in zip(camino, camino[1:]):
            p.append(linea((a[0], a[1], zmin), (b[0], b[1], zmin),
                           APAGADO, 1.0, None, 0.25))

        for a, b in zip(camino, camino[1:]):
            p.append(linea(a[:3], b[:3], CELESTE if a[3] else APAGADO,
                           2.2, None, 0.95 if a[3] else 0.55))

        if not self.teach_grabando:
            for qx_, qy_, qz_, qb in camino:
                px_, py_ = pt(qx_, qy_, qz_)
                p.append(f'<circle cx="{px_:.1f}" cy="{py_:.1f}" r="2.6" '
                         f'fill="{CELESTE if qb else APAGADO}"/>')

        # --- destino comandado ---
        if destino is not None:
            dx_, dy_ = pt(*destino)
            p.append(f'<path d="M {dx_ - 8:.1f} {dy_:.1f} H {dx_ + 8:.1f} '
                     f'M {dx_:.1f} {dy_ - 8:.1f} V {dy_ + 8:.1f}" '
                     f'stroke="{TEXTO}" stroke-width="1" stroke-opacity="0.4"/>')

        # --- el gripper ---
        if medido is not None:
            mx, my = pt(*medido)
            sx_, sy_ = pt(medido[0], medido[1], zmin)
            bomba = bool(est.t and est.t.bomba)

            # Vertical hasta el piso y sombra: es lo que hace legible la
            # altura en una vista donde la profundidad es ambigua.
            p.append(f'<line x1="{mx:.1f}" y1="{my:.1f}" x2="{sx_:.1f}" '
                     f'y2="{sy_:.1f}" stroke="{TEXTO}" stroke-width="1" '
                     'stroke-opacity="0.30" stroke-dasharray="3 3"/>')
            p.append(f'<ellipse cx="{sx_:.1f}" cy="{sy_:.1f}" rx="7" ry="4" '
                     f'fill="{TEXTO}" fill-opacity="0.16"/>')

            if bomba:
                p.append(f'<circle cx="{mx:.1f}" cy="{my:.1f}" r="15" fill="none" '
                         f'stroke="{CELESTE}" stroke-width="1.5" stroke-opacity="0.55"/>')

            p.append(f'<circle cx="{mx:.1f}" cy="{my:.1f}" r="8" '
                     f'fill="{CELESTE if bomba else "#F5D442"}" '
                     f'stroke="{FONDO}" stroke-width="2"/>')
        else:
            p.append(f'<text x="{ancho / 2:.0f}" y="{alto / 2:.0f}" fill="{APAGADO}" '
                     'font-size="13" text-anchor="middle" '
                     'font-family="system-ui">sin lectura de los encoders</text>')

        # --- ejes rotulados sobre las aristas del frente ---
        def rotulo(a, b, texto: str, desvio: tuple[float, float]) -> str:
            (x1, y1), (x2, y2) = pt(*a), pt(*b)
            return (f'<text x="{(x1 + x2) / 2 + desvio[0]:.1f}" '
                    f'y="{(y1 + y2) / 2 + desvio[1]:.1f}" fill="{APAGADO}" '
                    f'font-size="11" text-anchor="middle" '
                    f'font-family="system-ui">{texto}</text>')

        p.append(rotulo((xmin, ymin, zmin), (xmax, ymin, zmin),
                        f"X  {xmin:.0f} a {xmax:.0f} cm", (8, 18)))
        p.append(rotulo((xmax, ymin, zmin), (xmax, ymax, zmin),
                        f"Y  {ymin:.0f} a {ymax:.0f} cm", (34, 6)))
        p.append(rotulo((xmin, ymin, zmin), (xmin, ymin, zmax),
                        f"Z  {zmax - zmin:.0f} cm", (-26, 0)))

        # Escala de altura sobre la vertical de adelante: sin esto la Z se
        # lee sólo en el número de abajo, y la idea es verla en el dibujo.
        for i in range(int(zmax - zmin) + 1):
            z = zmin + i
            qx, qy = pt(xmin, ymin, z)
            largo = 7 if i % 2 == 0 else 4
            p.append(f'<line x1="{qx - largo:.1f}" y1="{qy:.1f}" x2="{qx:.1f}" '
                     f'y2="{qy:.1f}" stroke="{BORDE}" stroke-width="1"/>')

        if medido is not None:
            qx, qy = pt(xmin, ymin, max(zmin, min(zmax, medido[2])))
            p.append(f'<line x1="{qx - 11:.1f}" y1="{qy:.1f}" x2="{qx + 4:.1f}" '
                     f'y2="{qy:.1f}" stroke="{CELESTE}" stroke-width="2"/>')

        p.append("</svg>")

        return "".join(p)

    def _svg_joystick(self) -> str:
        lado = LADO_JOYSTICK
        centro = lado / 2.0
        radio = centro - 6.0

        activo = self.tab_activa == "Teach" and self._en_teach()
        trazo = CELESTE if activo else BORDE

        kx = centro + self.joy_x * radio
        ky = centro - self.joy_y * radio

        return (f'<svg viewBox="0 0 {lado:.0f} {lado:.0f}" '
                'style="width:100%;height:100%">'
                f'<circle cx="{centro}" cy="{centro}" r="{radio}" fill="{INACTIVO}" '
                f'stroke="{trazo}" stroke-width="1.5"/>'
                f'<line x1="{centro - radio}" y1="{centro}" x2="{centro + radio}" '
                f'y2="{centro}" stroke="{BORDE}" stroke-width="1" '
                'stroke-dasharray="3 4"/>'
                f'<line x1="{centro}" y1="{centro - radio}" x2="{centro}" '
                f'y2="{centro + radio}" stroke="{BORDE}" stroke-width="1" '
                'stroke-dasharray="3 4"/>'
                f'<text x="{centro}" y="14" fill="{APAGADO}" font-size="10" '
                'text-anchor="middle" font-family="system-ui">W  (+Y)</text>'
                f'<text x="{centro}" y="{lado - 5}" fill="{APAGADO}" font-size="10" '
                'text-anchor="middle" font-family="system-ui">S  (-Y)</text>'
                f'<circle cx="{kx:.1f}" cy="{ky:.1f}" r="15" '
                f'fill="{CELESTE if activo else BORDE}" fill-opacity="0.85"/>'
                '</svg>')

    # ------------------------------------------------------------------
    def _refrescar_teach(self) -> None:
        # Con la pestana en otra cosa no se dibuja nada: armar el SVG del
        # plano diez veces por segundo para que no lo mire nadie es el unico
        # gasto de CPU que esta interfaz podria llegar a notar.
        if not hasattr(self, "html_plano") or self.tab_activa != "Teach":
            return

        est = self.estado
        en_teach = self._en_teach()

        self.html_plano.content = self._svg_plano()
        self.html_joystick.content = self._svg_joystick()

        # --- chip de estado ---
        e = est.e

        if not est.enlace_vivo():
            color, texto = ROJO, "sin enlace"
        elif en_teach and self._reproduciendo():
            color = AMBAR
            texto = f"reproduciendo {e.teach_indice}/{e.teach_puntos or '?'}"
        elif en_teach:
            color, texto = VERDE, "activo"
        elif e and e.estado is pr.EstadoRobot.ERROR:
            color, texto = ROJO, "en ERROR: hay que rehomear"
        elif e and e.homed is False:
            color, texto = ROJO, "falta homing"
        else:
            color, texto = GRIS, "apagado"

        self.chip_teach.content = (
            f'{_punto(color)}<span style="color:{APAGADO};font-size:12.5px">'
            f'{texto}</span>')

        self.boton_teach.text = "Salir de Teach" if en_teach else "Entrar a Teach"
        self.boton_teach.style(
            f'background:{ROJO_STOP if en_teach else CELESTE}!important;'
            f'color:{"#fff" if en_teach else "#0B1220"}!important;font-weight:600')
        # Desde ERROR no se entra: el robot no vuelve solo a home, y después
        # de un paro la posición real dejó de corresponderse con los pasos.
        self.boton_teach.set_enabled(
            est.enlace_vivo()
            and (en_teach or not (e and e.estado is pr.EstadoRobot.ERROR)))

        # --- lectura de posición ---
        medido = cin.directa_desde(est.angulo_suave) if est.enlace_vivo() else None
        destino = est.teach_pos

        def celda(clave: str, valor) -> str:
            return (f'<div style="display:flex;justify-content:space-between;'
                    f'font-size:13px;margin:3px 0"><span style="color:{APAGADO}">'
                    f'{clave}</span><span style="color:{TEXTO};'
                    'font-family:ui-monospace,monospace">'
                    f'{valor}</span></div>')

        def terna(p) -> str:
            if p is None:
                return "—"
            return f"X {p[0]:+6.2f}   Y {p[1]:+6.2f}   Z {p[2]:+6.2f}"

        self.html_teach_pos.content = (
            celda("posición real (encoders)", terna(medido))
            + celda("destino comandado", terna(destino))
            + celda("vacío", "activo" if (est.t and est.t.bomba) else "apagado"))

        # --- botones que dependen del estado ---
        # "Ocupado" es cualquier movimiento que el operador no está manejando
        # en vivo: una reproducción o un "ir a una coordenada". Los dos dejan
        # el jog en pausa y los dos se cortan con el mismo botón.
        ocupado = self._reproduciendo() or self._yendo()

        bomba = bool(est.t and est.t.bomba)

        self.boton_bomba.style(
            f'background:{CELESTE if bomba else INACTIVO}!important;'
            f'color:{"#0B1220" if bomba else APAGADO}!important')
        self.boton_bomba.set_enabled(en_teach and not ocupado)

        for boton in (self.boton_z_sube, self.boton_z_baja):
            boton.set_enabled(en_teach and not ocupado)
            boton.style(f'background:{INACTIVO}!important;color:{TEXTO}!important')

        self.boton_grabar.text = "Terminar  ·  R" if self.teach_grabando else "Grabar  ·  R"
        self.boton_grabar.style(
            f'background:{ROJO_STOP if self.teach_grabando else INACTIVO}!important;'
            f'color:{"#fff" if self.teach_grabando else TEXTO}!important')
        self.boton_grabar.set_enabled(en_teach and not ocupado)

        elegido = self.biblioteca.obtener(self.teach_sel) \
            if self.teach_sel is not None else None

        # El porcentaje de la próxima pasada NO va en el botón: es el texto
        # más largo del panel y desbordaba la columna. Se sigue viendo en la
        # insignia de cada fila de la lista, en la etiqueta de acá abajo
        # mientras corre y en el cartel del final, que son los tres lugares
        # donde hace falta.
        self.boton_reproducir.set_enabled(
            en_teach and not self.teach_grabando
            and (ocupado or elegido is not None))
        self.boton_reproducir.text = "Parar  ·  P" if ocupado else "Reproducir  ·  P"

        listo = ocupado or elegido is not None

        self.boton_reproducir.style(
            f'background:{ROJO_STOP if ocupado else (CELESTE if listo else INACTIVO)}'
            '!important;'
            f'color:{"#fff" if ocupado else ("#0B1220" if listo else APAGADO)}'
            '!important')

        # --- ir a una coordenada ---
        (xmin, xmax), (ymin, ymax), (zmin, zmax) = self._teach_limites()

        self.etiqueta_ir.text = (
            f"X {xmin:.1f} a {xmax:.1f} · Y {ymin:.1f} a {ymax:.1f} · "
            f"Z {zmin:.1f} a {zmax:.1f} cm")

        self.boton_ir.set_enabled(en_teach and not ocupado and not self.teach_grabando)
        self.boton_ir.style(
            f'background:{CELESTE if (en_teach and not ocupado) else INACTIVO}'
            '!important;'
            f'color:{"#0B1220" if (en_teach and not ocupado) else APAGADO}!important')

        for campo in self.campos_ir.values():
            campo.set_enabled(en_teach and not ocupado)

        if self.teach_grabando:
            self.etiqueta_grabacion.text = (
                f"grabando {time.monotonic() - self.teach_t0:5.1f} s · "
                f"{len(self.teach_muestras)} muestras")
        elif self._reproduciendo():
            self.etiqueta_grabacion.text = f"reproduciendo al {self.teach_pct_en_curso} %"
        else:
            self.etiqueta_grabacion.text = f"{len(self.biblioteca.movimientos)} guardados"



def montar(estado: EstadoSistema, enviar, vision=None) -> Interfaz:
    interfaz = Interfaz(estado, enviar, vision)

    @ui.page("/")
    def pagina():
        interfaz.construir()

    return interfaz
