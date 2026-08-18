"""Punto de entrada unico: levanta camara, puerto serie e interfaz.

    pc\\.venv\\Scripts\\python pc/kuko_app.py
    pc\\.venv\\Scripts\\python pc/kuko_app.py --puerto COM5 --sin-vision

Un solo proceso, tres hilos: la vision (camara + deteccion), el enlace serie
y el servidor web. La interfaz no toca hardware -- lee el estado que dejan
los otros dos y manda comandos por el enlace.

Cerrar el navegador NO detiene el robot: el servidor sigue corriendo. Para
parar todo, Ctrl+C en esta consola.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from nicegui import app, ui

from kuko import ui as interfaz_ui
from kuko.enlace import Enlace
from kuko.estado import EstadoSistema
from kuko.vision import Vision

# Configuracion propia de cada maquina (puerto COM, camara). No va al
# repositorio: cada PC tiene la suya. Ver pc/config/local.ejemplo.json.
CONFIG_LOCAL = Path(__file__).resolve().parent / "config" / "local.json"


def cargar_config() -> dict:
    if CONFIG_LOCAL.exists():
        try:
            return json.loads(CONFIG_LOCAL.read_text(encoding="utf-8"))
        except json.JSONDecodeError as err:
            print(f"[config] {CONFIG_LOCAL.name} tiene un error de formato: {err}")

    return {}


def main() -> None:
    config = cargar_config()

    parser = argparse.ArgumentParser(description="Interfaz del KUKO Delta Carbon")
    parser.add_argument("--puerto", default=config.get("puerto", "AUTO"),
                        help="COM del ESP32, o AUTO para buscarlo solo")
    parser.add_argument("--sin-vision", action="store_true",
                        help="no abre la camara (para trabajar en la interfaz sin robot)")
    parser.add_argument("--sin-navegador", action="store_true")
    parser.add_argument("--puerto-web", type=int, default=config.get("puerto_web", 8080))
    args = parser.parse_args()

    estado = EstadoSistema()

    enlace = Enlace(estado, puerto=args.puerto)
    vision = None if args.sin_vision else Vision(estado, enlace.enviar)

    interfaz_ui.montar(estado, enlace.enviar, vision)

    # Los hilos arrancan con el servidor ya levantado: si la camara tarda en
    # abrir, la pantalla igual aparece y muestra que esta esperando.
    app.on_startup(enlace.arrancar)

    if vision:
        app.on_startup(vision.arrancar)
        app.on_shutdown(vision.parar)

    app.on_shutdown(enlace.parar)

    ui.run(title="KUKO Delta Carbon", port=args.puerto_web, dark=True,
           show=not args.sin_navegador, reload=False, favicon="🤖")


# NiceGUI relanza el modulo con __name__ == "__mp_main__" en algunos modos;
# se acepta el nombre real tambien para que ande igual con reload apagado.
if __name__ in {"__main__", "__mp_main__"}:
    main()
