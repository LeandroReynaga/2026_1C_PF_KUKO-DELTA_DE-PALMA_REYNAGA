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
from pathlib import Path
from typing import Optional

from fastapi import Response
from fastapi.responses import StreamingResponse
from nicegui import app, ui

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

    if medido is not None:
        x, y = punto(max(DIAL_MIN, min(DIAL_MAX, medido)), r - 4)
        partes.append(f'<line x1="{cx}" y1="{cy}" x2="{x:.1f}" y2="{y:.1f}" '
                      f'stroke="#F5D442" stroke-width="3" stroke-linecap="round"/>')

    partes.append(f'<circle cx="{cx}" cy="{cy}" r="4" fill="{TEXTO}"/>')
    partes.append(f'<text x="{cx}" y="15" fill="{APAGADO}" font-size="13" '
                  f'text-anchor="middle" font-family="system-ui">{indice}</text>')
    partes.append(f'<text x="{cx}" y="119" fill="{TEXTO}" font-size="14" '
                  f'text-anchor="middle" font-family="system-ui">'
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

            with ui.tab_panel(self.tab_proceso):
                ui.label("Parametros de proceso — en construccion").classes("titulo")

            with ui.tab_panel(self.tab_servicio):
                ui.label("Calibracion de servicio — en construccion").classes("titulo")

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

                        self.slider_latencia = ui.slider(
                            min=-0.10, max=0.30, step=0.005,
                            on_change=self._cambiar_latencia).props("dense").style("flex:1 1 0")

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
