"""Aplicación de PC del KUKO Delta Carbon.

Tres capas, separadas a propósito:

    protocolo.py   traduce entre las líneas del serie y objetos de Python
    nucleo/        el proceso que es dueño de la cámara y del puerto COM
    ui/            la interfaz, que no toca hardware ni por accidente

La regla que sostiene todo el diseño: **un solo proceso abre el puerto**.
En Windows un COM lo toma un único proceso, y además abrirlo resetea el
ESP32, así que si la interfaz lo abriera por su cuenta, cada recarga de la
página rehomearía el robot.
"""

__version__ = "0.1.0"
