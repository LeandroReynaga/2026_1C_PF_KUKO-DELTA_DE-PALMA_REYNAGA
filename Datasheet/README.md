# Datasheets

Hojas de datos de los componentes principales del KUKO Delta Carbon.

| Componente | Archivo esperado | Dato que interesa |
| :--- | :--- | :--- |
| ESP32 NodeMCU-32S | `nodemcu-32s.pdf` | Pines con ADC, canales PWM, timers de hardware |
| Driver DM556 | `dm556.pdf` | Tabla de microstepping y de corriente (DIP switches) |
| Motor NEMA 23 | `nema23.pdf` | Par, corriente por fase, inercia del rotor |
| Encoder AS5600 | `as5600.pdf` | Salida analógica, resolución, alimentación ratiométrica |
| Bomba de vacío | `bomba-vacio.pdf` | Consumo y caudal |
| Electroválvula | `electrovalvula.pdf` | Alimentación, acción directa, rango de presiones|
| Cámara USB | `camara-usb.pdf` | Resolución, modos de captura |
| Driver DRV8833 | `dvr8833.pdf` | Pinout, corriente máxima |
| Motorreductor 60RPM | `motorreductor.pdf` | Torque a tensión nominal, rpm |
| Switch Final de Carrera | `final-de-carrera.pdf` | Dimensiones, NO y NC|

---

> **Los aspectos más importantes**

> - La **placa ESP32** debe tener suficiente capacidad de procesamiento y cantidad de pines para todos los componentes del robot.
> - La selección del modelo de **motores paso a paso** es fundamental para garantizar **torque sostenido a altas RPM** ya que una caída
>   del torque genera pérdida de pasos.
> - El **circuito neumático** debe ser capaz de generar vacío y liberar el sistema **a alta velocidad**.
> - La **cámara** debe tener suficiente resolución y contraste.
> - La **cinta transportadora** no debe trabarse durante el funcionamiento.

Nombres de archivo sin espacios, sin acentos y en minúscula.
