"""Ajustes que sobreviven al cierre del programa, propios de cada maquina.

Van a pc/config/local.json, que NO esta en el repositorio: el puerto COM y
el encuadre de la camara son distintos en cada PC a proposito.

Hoy guarda el desplazamiento del recorte de la camara. Es el unico ajuste
que se pide explicitamente que NO tenga un valor por defecto al arrancar:
tiene que quedar donde lo dejo el operador la ultima vez.
"""

from __future__ import annotations

import json
from pathlib import Path

ARCHIVO = Path(__file__).resolve().parents[1] / "config" / "local.json"


def cargar() -> dict:
    if not ARCHIVO.exists():
        return {}

    try:
        return json.loads(ARCHIVO.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError) as err:
        print(f"[ajustes] no se pudo leer {ARCHIVO.name}: {err}")
        return {}


def guardar(clave: str, valor) -> None:
    datos = cargar()
    datos[clave] = valor

    try:
        ARCHIVO.parent.mkdir(parents=True, exist_ok=True)
        ARCHIVO.write_text(json.dumps(datos, indent=2, ensure_ascii=False),
                           encoding="utf-8")
    except OSError as err:
        print(f"[ajustes] no se pudo guardar {ARCHIVO.name}: {err}")
