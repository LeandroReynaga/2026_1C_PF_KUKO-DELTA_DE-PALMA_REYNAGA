# Flujograma del sistema de visión artificial

El bucle vive en `pc/kuko/vision.py` y corre en su propio hilo, dentro de la
aplicación de PC. Usa los módulos de este directorio (`camera.py`,
`detection.py`, `tracker.py`, `line_crossing.py`, `coordinates.py`), que son
los que están calibrados.

Dos cosas que el diagrama muestra y conviene no perder de vista:

- **Son dos bucles, uno adentro del otro.** El de afuera abre la cámara y la
  vuelve a abrir cuando se pierde; el de adentro procesa fotogramas mientras
  haya. Una cámara USB desenchufada **no da error**: `read()` devuelve `False`
  para siempre y `isOpened()` sigue diciendo que sí, así que lo único que la
  delata es cuánto hace que no llega un fotograma. Y reabrirla hace falta de
  verdad: el `VideoCapture` queda muerto y volver a enchufar el cable no
  arregla nada sin un objeto nuevo.
- **En modo calibración no se informa ninguna pieza.** Con la cinta parada y
  alguien apoyando piezas a mano, cada pieza apoyada cruza la línea —la cruza
  la mano— y el brazo salía a buscarla. El cruce igual se marca en el
  seguimiento, para que al salir de calibración no se avise de golpe todo lo
  que quedó del otro lado.

```mermaid
flowchart TD

    A([Arranca el hilo de visión]) --> B["camera.py<br/>Abrir la cámara (MSMF, con respaldo)"]

    B --> C{"¿Abrió?"}

    C -- "No" --> ESPERA["Esperar y reintentar<br/>(la espera crece hasta 20 s)"]
    ESPERA --> B

    C -- "Sí" --> D["camera.py<br/>Leer fotograma:<br/>rotar 90°, recortar a la cinta<br/>y aplicar corrección de exposición"]

    D --> E{"¿Llegó el fotograma?"}

    E -- "No" --> VENCE{"¿Hace más de<br/>2 s que no llega?"}
    VENCE -- "No" --> D
    VENCE -- "Sí" --> CAIDA["Dar la cámara por perdida<br/>y cerrarla"]
    CAIDA --> B

    E -- "Sí" --> MEDIR{"¿Pestaña de Visión<br/>a la vista?"}

    MEDIR -- "Sí" --> CAL["calibracion.py<br/>Guardar el fotograma SIN anotar<br/>y medir el color real de cada pieza"]
    MEDIR -- "No" --> F

    CAL --> F["Calcular dónde cae<br/>la línea de detección"]

    F --> G["detection.py<br/>Desenfocar y pasar a HSV"]

    G --> H["Crear las máscaras de<br/>ROJO, VERDE y AZUL<br/>(el rojo da la vuelta<br/>a la rueda de tono)"]

    H --> I["Limpiar con operaciones<br/>morfológicas y suavizar el borde"]

    I --> J["Buscar contornos"]

    J --> SPLIT["Separar por watershed<br/>las piezas del mismo color<br/>que se están tocando"]

    SPLIT --> L{"¿Quedan contornos<br/>por analizar?"}

    L -- "Sí" --> M{"¿El área está dentro<br/>del rango permitido?"}

    M -- "No" --> L

    M -- "Sí" --> N["Sobre el casco convexo:<br/>vértices, llenado y circularidad"]

    N --> O{"¿Cuadrado,<br/>hexágono o círculo?"}

    O -- "No reconocido" --> L

    O -- "Reconocido" --> P["Calcular centroide y bounding box"]

    P --> Q["Crear objeto Detection"]

    Q --> L

    L -- "No" --> TRACKER["tracker.py<br/>Actualizar el seguimiento"]

    TRACKER --> R{"¿Coincide con<br/>un ID existente?"}

    R -- "Sí" --> S["Mantener el ID y<br/>actualizar la posición"]
    R -- "No" --> T["Asignar un ID nuevo"]

    S --> CINTA["Medir la velocidad real de la cinta<br/>siguiendo cuánto avanzó cada pieza"]
    T --> CINTA

    CINTA --> U["line_crossing.py<br/>Comparar la posición anterior<br/>con la actual"]

    U --> V{"¿Cruzó la línea?"}

    V -- "No" --> DIBUJAR

    V -- "Sí" --> W["Marcar el cruce en el seguimiento<br/>(para no avisarlo dos veces)"]

    W --> PAUSA{"¿Modo calibración?"}

    PAUSA -- "Sí" --> DIBUJAR

    PAUSA -- "No" --> COORD["coordinates.py<br/>Pasar el centro de píxeles<br/>a centímetros del robot"]

    COORD --> SERIAL["enlace.py<br/>Enviar Y,color,forma al ESP32"]

    SERIAL --> DIBUJAR["Dibujar contorno, ID, forma,<br/>color, coordenadas y la línea"]

    DIBUJAR --> JPEG["Codificar a JPEG y dejarlo<br/>para el stream MJPEG de la interfaz"]

    JPEG --> D
```
