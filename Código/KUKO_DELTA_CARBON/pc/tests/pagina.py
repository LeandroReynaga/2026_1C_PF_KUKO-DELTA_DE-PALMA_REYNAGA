"""Un solo servidor de NiceGUI para todas las pruebas del proceso.

`ui.run_with()` instala middleware en la aplicación, y middleware sólo se
puede agregar **antes** de que arranque. Con un servidor por archivo de
pruebas eso anda mientras se corra un archivo por vez, pero `pytest` los
corre a todos en el mismo proceso y el segundo revienta con
"Cannot add middleware after an application has started".

Así que el servidor se arma una sola vez y después se le van agregando
rutas: agregar rutas después del arranque sí se puede.

    respuesta = pagina.pedir(lambda: interfaz.construir())
"""

from __future__ import annotations

import atexit

_cliente = None
_contador = 0


def _arrancar():
    global _cliente

    if _cliente is None:
        from fastapi import FastAPI
        from fastapi.testclient import TestClient
        from nicegui import ui

        servidor = FastAPI()
        ui.run_with(servidor)

        _cliente = TestClient(servidor)
        _cliente.__enter__()

        # Sin esto el proceso puede quedarse esperando el apagado del
        # servidor cuando la prueba termina bien.
        atexit.register(_apagar)

    return _cliente


def _apagar() -> None:
    global _cliente

    if _cliente is not None:
        try:
            _cliente.__exit__(None, None, None)
        except Exception:                               # noqa: BLE001
            pass

        _cliente = None


def pedir(cuerpo):
    """Registra una página que ejecuta `cuerpo()`, la pide y devuelve la respuesta.

    El cuerpo corre dentro de un cliente de verdad, así que puede usar
    `ui.notify` y `ui.dialog`, que fuera de una página no funcionan.
    """

    from nicegui import ui

    global _contador

    _contador += 1
    ruta = f"/prueba-{_contador}"

    cliente = _arrancar()
    ui.page(ruta)(cuerpo)

    return cliente.get(ruta)
