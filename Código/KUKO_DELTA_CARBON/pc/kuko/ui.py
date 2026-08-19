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

from . import parametros as par
from . import protocolo as pr
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
                self.tab_proceso = ui.tab("Proceso")
                self.tab_servicio = ui.tab("Servicio")

            ui.space()
            self.chip_enlace = ui.html()

        with ui.tab_panels(tabs, value=self.tab_operacion).classes("w-full") \
                .style(f"background:{FONDO};height:calc(100vh / {ZOOM} - 42px);overflow:hidden"):
            with ui.tab_panel(self.tab_operacion).classes("p-0").style("height:100%"):
                self._operacion()

            with ui.tab_panel(self.tab_proceso).classes("p-0").style("height:100%"):
                self._ajustes(pr.NIVEL_PROCESO, self._panel_en_vivo)

            with ui.tab_panel(self.tab_servicio).classes("p-0").style("height:100%"):
                self._ajustes(pr.NIVEL_SERVICIO, self._panel_servicio)

        ui.timer(0.1, self._refrescar_rapido)
        ui.timer(0.5, self._refrescar_lento)

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

    def _refrescar_lento(self) -> None:
        est = self.estado
        vivo = est.enlace_vivo()

        color = COLOR_ESTADO[VERDE] if vivo else COLOR_ESTADO[ROJO]
        texto = (f"{est.puerto} · {est.fps_camara:.0f} fps"
                 if vivo else (est.error_enlace or "sin enlace"))
        self.chip_enlace.content = (
            f'{_punto(VERDE if vivo else ROJO)}'
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


def montar(estado: EstadoSistema, enviar, vision=None) -> Interfaz:
    interfaz = Interfaz(estado, enviar, vision)

    @ui.page("/")
    def pagina():
        interfaz.construir()

    return interfaz
