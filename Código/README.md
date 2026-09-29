# Código

Código fuente del **KUKO Delta Carbon** y el driver USB para conectar la placa a la PC.

El sistema son dos partes que se comunican por puerto serie: el **firmware** del ESP32
(PlatformIO / Arduino) y la **aplicación de PC** (Python).

## Contenido

| Carpeta | Contenido |
|---|---|
| [`KUKO_DELTA_CARBON/`](KUKO_DELTA_CARBON/) | Proyecto completo: firmware y aplicación de PC. |
| [`Driver_USB_CP2102/`](Driver_USB_CP2102/) | Driver de Windows para el conversor USB-serie CP2102 de la NodeMCU-32S, comprimido y descomprimido. |

## KUKO_DELTA_CARBON

| Ubicación | Contenido |
|---|---|
| `platformio.ini` | Configuración de PlatformIO: entorno `main` (firmware) y `test` (pruebas de placa). |
| `include/Pinout.h` | Asignación de pines del ESP32. Es el único lugar donde se definen. |
| `src/` | Firmware: máquina de estados, generación de pasos, cinemática, trayectorias, encoders, finales de carrera, neumática y cinta. |
| `pc/` | Aplicación de PC: enlace serie, visión, interfaz, teach y rendimiento. |
| `pc/PROTOCOLO.md` | Protocolo de comunicación serie entre la PC y el ESP32. |
| `pc/tests/` | Pruebas de la aplicación de PC. |
| `vision_python/` | Módulos de visión calibrados: cámara, detección por color y forma, seguimiento y conversión a coordenadas del robot. |
| `pruebas/` | Programas de prueba de la puesta en marcha de la placa. |
| `estructura.txt` | Estructura del proyecto, archivo por archivo. |
| `CLAUDE.md` | Guía del proyecto para Claude Code. |

**Volumen:** ~11.400 líneas de firmware (C++) y ~19.600 de aplicación de PC (Python).
