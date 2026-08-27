# Datasheets

Hojas de datos de los componentes principales del KUKO Delta Carbon.

| Componente | Archivo esperado | Dato que interesa |
| :--- | :--- | :--- |
| ESP32 NodeMCU-32S | `esp32-nodemcu-32s.pdf` | Pines con ADC, canales PWM, timers de hardware |
| Driver DM556 | `dm556.pdf` | Tabla de microstepping y de corriente (DIP switches) |
| Motor NEMA 23 | `nema23.pdf` | Par, corriente por fase, inercia del rotor |
| Encoder AS5600 | `as5600.pdf` | Salida analógica, resolución, alimentación ratiométrica |
| Bomba de vacío | `bomba-vacio.pdf` | Consumo y caudal |
| Cámara USB | `camara-usb.pdf` | Modos de captura soportados (MJPG vs. YUY2) |

> **Por qué importan dos de estas en particular**
>
> - El **AS5600** tiene salida **ratiométrica a su VCC**: una caída del riel de
>   alimentación corre las tres lecturas de ángulo a la vez sin que el robot se mueva.
>   Ese comportamiento es la razón de los 300 ms de blanqueo al conmutar la bomba.
> - La **cámara** solo llega a 30 fps a 720p si negocia **MJPG**; sin compresión el
>   video no entra en el ancho de banda de USB 2.0 y el driver baja a 10 fps.

Nombres de archivo sin espacios, sin acentos y en minúscula.
