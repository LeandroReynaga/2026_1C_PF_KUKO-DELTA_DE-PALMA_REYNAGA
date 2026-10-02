<div align="center">
<img width="100%" alt="Banner_KUKO_Delta_Carbon" src="https://github.com/user-attachments/assets/3c6adcc1-a618-4cc2-b73e-77055bdcb245" />
<br>



# KUKO DELTA CARBON

### Robot delta de 3 GDL con visión artificial para clasificación de piezas sobre cinta en movimiento

<br>

[![Tipo](https://img.shields.io/badge/Tipo-Proyecto%20Final-1f6feb?style=for-the-badge)](#)
[![Periodo](https://img.shields.io/badge/2026-1er%20Cuatrimestre-30363d?style=for-the-badge)](#)
[![Estado](https://img.shields.io/badge/Estado-Funcional-2ea043?style=for-the-badge)](#)

[![UNLZ](https://img.shields.io/badge/FI--UNLZ-Ingenier%C3%ADa%20Mecatr%C3%B3nica-0b7285?style=flat-square)](https://ingenieria.unlz.edu.ar/)
[![ESP32](https://img.shields.io/badge/ESP32-PlatformIO-FF7F00?style=flat-square&logo=platformio&logoColor=white)](https://platformio.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-Arduino%20framework-00599C?style=flat-square&logo=cplusplus&logoColor=white)](#)
[![Python](https://img.shields.io/badge/Python-3.13-3776AB?style=flat-square&logo=python&logoColor=white)](#)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.10-5C3EE8?style=flat-square&logo=opencv&logoColor=white)](#)
[![NiceGUI](https://img.shields.io/badge/Interfaz-NiceGUI-0ea5e9?style=flat-square)](#)
[![Pruebas](https://img.shields.io/badge/Pruebas-98%20autom%C3%A1ticas-2ea043?style=flat-square)](#)

<br>

<img src="https://github.com/JonatanBogadoUNLZ/PPS-Jonatan-Bogado/blob/9952aac097aca83a1aadfc26679fc7ec57369d82/LOGO%20AZUL%20HORIZONTAL%20-%20fondo%20transparente.png?raw=true" alt="Universidad Nacional de Lomas de Zamora — Facultad de Ingeniería" width="420">

**Universidad Nacional de Lomas de Zamora · Facultad de Ingeniería**<br>
**Ingeniería Mecatrónica**

</div>

---

## Ficha del proyecto

|                   |                                                               |
| :---------------- | :------------------------------------------------------------ |
| **Proyecto**      | KUKO Delta Carbon                                              |
| **Tipo**          | Proyecto Final (PF)                                            |
| **Año / Período** | 2026 — 1.er Cuatrimestre                                       |
| **Carrera**       | Ingeniería Mecatrónica                                         |
| **Autores**       | **DE PALMA, Marcos Agustín** · **REYNAGA RÍOS, Leandro Joel**  |
| **Institución**   | Facultad de Ingeniería — UNLZ                                  |
| **Estado**        | 🟢 Prototipo funcional     |
| **Repositorio**   | `2026_1C_PF_KUKO-DELTA_DE-PALMA_REYNAGA`                       |

---

<p align="center">
  <img src="Multimedia/01_Robot_completo.jpg" alt="KUKO Delta Carbon" width="760">
  <br>
  <em>Imagen 1: KUKO Delta Carbon</em>
</p>

---

<a id="indice"></a>

## 📑 Índice

|  #  | Sección                                      | Contenido                              |
| :-: | :------------------------------------------- | :------------------------------------- |
|  1  | [Introducción y objetivos](#introduccion)     | Contexto, problema y objetivos         |
|  2  | [Brief](#brief)                               | Pitch, solución, alcance y estado      |
|  3  | [Multimedia](#multimedia)                     | GIF, videos y visor 3D con AR          |
|  4  | [Descripción técnica](#descripcion-tecnica)   | Cómo funciona por dentro               |
|  5  | [Instrucciones de uso](#uso)                  | Puesta en marcha reproducible          |
|  6  | [Desarrollo del proyecto](#desarrollo)        | Diagrama de Gantt                      |
|  7  | [Autores](#autores)                           | Contacto                               |

---

<a id="introduccion"></a>

## 1 · Introducción y objetivos

### Contexto

**KUKO Delta Carbon** es una celda de clasificación, de escala didáctica pero con criterios
industriales, construida íntegramente por los autores: mecánica, electrónica, firmware, visión
artificial e interfaz de operación.

### Problema a resolver

Tomar piezas **en movimiento** sobre una cinta transportadora y depositarlas en el destino que les
corresponde, sin detener la cinta, es un problema de sincronización. Resolverlo exige medir la latencia del sistema
de visión, predecir la posición futura de la pieza y planificar la intercepción, todo con hardware
de bajo costo y sobre motores paso a paso.

### Objetivo general

> Diseñar, construir y poner en marcha una celda robótica de clasificación automática basada en un
> robot delta de 3 grados de libertad, capaz de detectar piezas por **color** y **forma** mediante
> visión artificial, interceptarlas sobre una cinta en movimiento y depositarlas en su destino,
> supervisada desde una interfaz de operación propia.

---

<a id="brief"></a>

## 2 · Brief

> **KUKO Delta Carbon** es una celda robótica *pick & place* que clasifica piezas por color y forma
> sobre una cinta en movimiento, pensada para tareas de selección repetitivas.

Este proyecto **KUKO Delta Carbon** (Proyecto Final, **2026 · 1.er Cuatrimestre**) resuelve la
**clasificación manual de piezas en línea de producción** mediante un **robot delta de 3 GDL con
visión artificial que intercepta las piezas sin detener la cinta**.
Permite **clasificar
por color, por forma o armar una caja de 6 posiciones**, con una interfaz que muestra en vivo el
estado de cada componente, la producción acumulada y la disponibilidad de la celda.

Se implementa con **ESP32 + C++ (PlatformIO)** para el control en tiempo real y **Python + OpenCV +
NiceGUI** para la visión y la operación, y se valida con **98 pruebas automáticas** que verifican
tanto el protocolo de comunicación como la matemática del movimiento.

<table>
<tr><td width="52%" valign="top">

**Para lograrlo:**

- 👁️ **Detecta** piezas por color (rojo · verde · azul) y forma (cuadrado · hexágono · círculo)
- 🎯 **Intercepta** la pieza en movimiento
- 🗑️ **Clasifica** en 3 tachos, por color o por forma
- 📦 **Modo Box:** llena una caja de 6 celdas con una disposición de colores configurable
- 🛡️ **Supervisa colisiones** con encoders magnéticos y se recupera sola rehomeando
- 🧑‍🏫 **Modo Teach:** el operador mueve el brazo a mano, graba secuencias y las reproduce
- 📊 **Mide su propia producción:** piezas, ritmo, fallos y disponibilidad
- ⚙️ **62 parámetros ajustables**, sin recompilar y persistidos en la placa

</td><td width="48%" valign="top">

```
      Cámara USB 720p
             ↓  OpenCV (HSV + contornos)
  Color · Forma · Posición Y
             ↓  Serie 115200 8N1
  ESP32 — planificación e intercepción
             ↓  Cinemática inversa delta
  3 × NEMA 23 (10.000 µpasos/vuelta)
             ↓  Ventosa de vacío
  Pieza depositada en su destino
             ↑
  3 × AS5600 → supervisión de colisión
```

</td></tr>
</table>

### Alcance

<table>
<tr>
<th width="50%">✅ Incluye</th>
<th width="50%">🚫 No incluye</th>
</tr>
<tr><td valign="top">

- Robot delta de 3 GDL con ventosa de vacío
- Cinta transportadora con velocidad regulada por PWM
- Visión artificial por color y forma, con seguimiento
- Intercepción de piezas en movimiento
- Detección de colisiones y recuperación automática
- Registro de fallos e indicadores de producción
- Interfaz web de operación, proceso y servicio
- Modo Teach con grabación y reproducción de rutas
- Movimiento lineal (movL) en el espacio cartesiano

</td><td valign="top">


- Alimentador automático de piezas
- Vacuostato (confirmación física del vacío)
- Vision Artificial con Redes neuronales

</td></tr>
</table>

### Estado del proyecto

| Aspecto              | Estado                                                                                    |
| :------------------- | :---------------------------------------------------------------------------------------- |
| **Madurez**          | ✅ Prototipo funcional                                 |
| **Ciclo completo**   | ✅ Homing → detección → intercepción → agarre → clasificación → retorno                    |
| **Modos operativos** | ✅ Color · Forma · Box · Teach                                                    |
| **Seguridad**        | ✅ Supervisión de colisión activa, con recuperación automática                             |
| **Interfaz**         | ✅ 6 pestañas operativas (Operación, Teach, Rendimiento, Visión, Proceso, Servicio)          |

---

<a id="multimedia"></a>

## 3 · Multimedia

<p align="center">
  <a href="https://leandroreynaga.github.io/2026_1C_PF_KUKO-DELTA_DE-PALMA_REYNAGA/Multimedia/KUKO_AR/">
    <img src="Multimedia/10_Render_giratorio.gif" alt="Render giratorio: clic para ver el robot en 3D y realidad aumentada" width="760">
  </a>
  <br>
  <em>Figura 1: Render giratorio del modelo 3D</em>
</p>

<p align="center">
  <a href="https://leandroreynaga.github.io/2026_1C_PF_KUKO-DELTA_DE-PALMA_REYNAGA/Multimedia/KUKO_AR/"><b>🔍 Ver en 3D y Realidad Aumentada</b></a>
</p>
<p align="center">
  <img src="Multimedia/KUKO_AR/qr_ar.png" alt="Código QR para ver el robot en realidad aumentada" width="160">
  <br>
  <em>Enlace 1: Haz clic en el render o escanea el QR con el celular para ver el robot en 3D y en realidad aumentada.</em>
</p>

<p align="center">
  <img src="Multimedia/11_Clasificaci%C3%B3n_por_Color.gif" alt="Clasificación por color" width="760">
  <br>
  <em>Figura 2: Clasificación por color</em>
</p>

<p align="center">
  <img src="Multimedia/12_Clasificaci%C3%B3n_por_Forma.gif" alt="Clasificación por forma" width="760">
  <br>
  <em>Figura 3: Clasificación por forma</em>
</p>

<p align="center">
  <img src="Multimedia/13_Modo_Box.gif" alt="Modo Box" width="760">
  <br>
  <em>Figura 4: Modo Box</em>
</p>

<p align="center">
  <img src="Multimedia/14_Modo_Teach.gif" alt="Modo Teach" width="760">
  <br>
  <em>Figura 5: Modo Teach</em>
</p>

<p align="center">
  <a href="https://youtube.com/playlist?list=PLV_DUMoOxinw&amp;si=bVj7ZXU2DtyZ1oJc">
    <img width="760" alt="Playlist de videos del KUKO Delta Carbon" src="Multimedia/15_Portada_playlist.png" />
  </a>
</p>
<p align="center">
  <em>Enlace 2: Haz clic en la imagen para ver la playlist con los videos de los modos.</em>
</p>

<p align="center">
  <a href="https://drive.google.com/drive/folders/15TBgdvEbJJ4iwdDsnXzpPRDG5-t90HLm?usp=drive_link">
    <img width="320" alt="Google Drive_Logo" src="https://github.com/user-attachments/assets/b0655d57-e569-428e-a687-4a39b1f1422e" />

  </a>
</p>
<p align="center">
  <em>Enlace 3: Haz clic en la imagen para descargar el modelo 3D y los videos.</em>
</p>

---

<a id="descripcion-tecnica"></a>

## 4 · Descripción técnica

El sistema se reparte responsabilidades: el ESP32 hace
lo que **no puede esperar** (generar pasos, leer encoders, decidir cuándo bajar el brazo) y la PC hace
lo que **necesita memoria y potencia** (procesar imagen, dibujar, recordar sucesos).

### 4.1 · Ciclo de clasificación

```mermaid
stateDiagram-v2
    direction LR
    [*] --> HOMING
    HOMING --> WAIT_PIECE : finales de carrera + calibración de encoders
    WAIT_PIECE --> PICK_APPROACH : llega una pieza alcanzable
    PICK_APPROACH --> PICK_DESCEND : espera sobre el punto de encuentro
    PICK_DESCEND --> PICK_LIFT : baja a favor de la cinta y toma la pieza
    PICK_LIFT --> GO_BIN : modo Color / Forma
    PICK_LIFT --> BOX_TRANSIT : modo Box
    GO_BIN --> BIN_SETTLE
    BIN_SETTLE --> RELEASE_WAIT
    RELEASE_WAIT --> WAIT_PIECE
    BOX_TRANSIT --> BOX_APPROACH
    BOX_APPROACH --> BOX_DESCEND
    BOX_DESCEND --> BOX_LIFT
    BOX_LIFT --> WAIT_PIECE
    WAIT_PIECE --> GO_HOME_IDLE : sin piezas en cola
    GO_HOME_IDLE --> WAIT_PIECE
    HOMING --> COLLISION_STOP : cualquier estado puede caer acá
    COLLISION_STOP --> HOMING : recalibra conservando la cola
    WAIT_PIECE --> TEACH : pedido del operador (J1)
    TEACH --> WAIT_PIECE
```

<p align="center"><em>Figura 6: Máquina de estados del ciclo de clasificación</em></p>

### 4.2 · Modos de clasificación

| Modo | Comando | Qué hace |
| :--- | :-----: | :------- |
| **Color** | `C` | Tacho 1 rojo · Tacho 2 verde · Tacho 3 azul |
| **Forma** | `F` | Tacho 1 cuadrado · Tacho 2 hexágono · Tacho 3 círculo |
| **Box** | `A` | Llena una caja de 6 celdas (2 filas × 3 columnas) con una disposición de colores configurable; máximo 3 piezas por color. El resto de las piezas siguen de largo |

### 4.3 · Visión artificial

La detección corre en la PC sobre **OpenCV**, con procesamiento clásico (sin redes neuronales): es
determinístico, se calibra a mano.

**La `X` de la pieza no viaja por el enlace.** Como el aviso se emite exactamente en el cruce de la
 línea de detección, la `X` es siempre la misma y es conocida por las dos partes. Solo viajan
 `Y`, color y forma — tres campos, dos comas, una línea.

### 4.4 · Cinemática

`DeltaKinematics` es un espacio de nombres puramente matemático —sin entrada/salida ni llamadas a
motores— que resuelve la **cinemática inversa** del delta. La **cinemática directa** vive del lado de
Python (`cinematica.py`), y la necesita para dibujar el brazo en pantalla.

### 4.5 · Generación de pasos y perfiles de movimiento

Cada eje se maneja con un **timer de hardware dedicado** del ESP32 y una rampa trapezoidal calculada
con el **algoritmo de Austin (2004)**.

**Coordinación multi-eje.** `Motors::moveSynchronized` escala velocidad y aceleración de cada eje en
proporción a su recorrido, de modo que los tres motores **arrancan y llegan juntos** aunque recorran
distancias distintas.

### 4.6 · Supervisión de colisiones

**No es control de posición** — la posición sigue siendo lazo abierto por micropasos. Es un
 **detector de discrepancia**: en cada vuelta del loop compara el ángulo medido contra el que dicen
 los pasos emitidos, y si la diferencia supera el umbral **y se sostiene** durante un tiempo de
 confirmación, declara colisión.

**El umbral no es fijo:**

```
umbral_efectivo = UMBRAL_DEG + MARGEN_VELOCIDAD_MS × velocidad
```

**Reacción ante colisión:** frena los 3 ejes → suelta la pieza → espera 3 s → rehomea conservando la
cola de piezas. Tras 3 colisiones seguidas, pasa a `ERROR`.

### 4.7 · Modo Teach

Un modo aparte del ciclo de clasificación: el operador maneja el brazo a mano desde la interfaz, graba
secuencias y las reproduce. Incluye verificación por etapas.

### 4.8 · Interfaz de operación

Aplicación web local hecha con **NiceGUI**, en un solo proceso y tres hilos: visión, enlace serie y
servidor web. Sirve para controlar el proceso, modificar variables, enterarse de errores y verificar componentes. 

---

<a id="uso"></a>

## 5 · Instrucciones de uso

### 5.1 · Requisitos previos

| Categoría | Requisito |
| :--- | :--- |
| **Sistema operativo** | Windows 10/11  |
| **Firmware** | [PlatformIO](https://platformio.org/) (CLI `pio` o la extensión de VS Code) |
| **PC** | Python **3.13** o superior |
| **Driver USB** | CP2102 — incluido en [`Código/Driver_USB_CP2102/`](C%C3%B3digo/Driver_USB_CP2102/) |
| **Hardware** | Robot Kuko Delta Carbon |

### 5.2 · Instalación

**a) Clonar el repositorio**

```bash
git clone https://github.com/<usuario>/2026_1C_PF_KUKO-DELTA_DE-PALMA_REYNAGA.git
cd "2026_1C_PF_KUKO-DELTA_DE-PALMA_REYNAGA/Código/KUKO_DELTA_CARBON"
```

**b) Instalar el driver USB** (solo la primera vez)

Ejecutar el instalador de `Código/Driver_USB_CP2102/CP210x_Universal_Windows_Driver/` según la
arquitectura de la PC. Sin esto, el ESP32 no aparece como puerto COM.

**c) Compilar y cargar el firmware**

```bash
pio run                        # compila el entorno "main"
pio run -e main -t upload      # flashea la placa
pio device monitor             # monitor serie a 115200 baudios
```

<details>
<summary><b>Entornos de compilación disponibles</b></summary>

<br>

La carpeta `pruebas/` guarda sketches históricos (`.bak`) de motores, encoders y cinemática, además de
`test.cpp`, que es lo que compila el entorno `test`. Los `.bak` no forman parte de ningún build, pero
sirven de referencia al depurar un subsistema por separado.

</details>

**d) Preparar la aplicación de PC**

```bash
python -m venv pc/.venv
pc\.venv\Scripts\pip install -r pc/requirements.txt
```

### 5.3 · Puesta en marcha

```bash
pc\.venv\Scripts\python pc/kuko_app.py
```

Opciones útiles:

```bash
pc\.venv\Scripts\python pc/kuko_app.py --puerto COM5      # forzar el puerto serie
pc\.venv\Scripts\python pc/kuko_app.py --sin-vision       # trabajar en la interfaz sin cámara
pc\.venv\Scripts\python pc/kuko_app.py --sin-navegador    # no abrir el navegador solo
```

La interfaz queda en **`http://localhost:8080`**.

---

<a id="desarrollo"></a>

## 6 · Desarrollo del proyecto

El proyecto se construyó **de abajo hacia arriba**: primero cada subsistema por separado y verificado
en aislamiento, después la integración. Cada capa nueva se apoyó en una que ya estaba medida.

### Diagrama de Gantt

Cronograma del proyecto de marzo a octubre de 2026, en seis etapas. El detalle de cada tarea está en el
[Diagrama de Gantt final](Documentaci%C3%B3n/Diagrama%20de%20Gantt/Diagrama_de_Gantt_final.pdf).

<p align="center">
  <img src="Multimedia/09_Diagrama_de_Gantt.png" alt="Diagrama de Gantt final" width="760">
  <br>
  <em>Figura 7: Diagrama de Gantt final</em>
</p>

---

<a id="autores"></a>

## 7 · Autores

<table align="center">
<tr><td width="50%" align="center">

### DE PALMA<br>Marcos Agustín

Ingeniería Mecatrónica — FI-UNLZ

📧 <marcosdepalma03@gmail.com>

💻 GitHub [@MarcosDePalma](https://github.com/MarcosDePalma)

</td><td width="50%" align="center">

### REYNAGA RÍOS<br>Leandro Joel

Ingeniería Mecatrónica — FI-UNLZ

📧 <leandro_05_01@hotmail.com>

💻 GitHub [@LeandroReynaga](https://github.com/LeandroReynaga)

</td></tr>
</table>

---

<div align="center">

**KUKO DELTA CARBON**

**Facultad de Ingeniería — Universidad Nacional de Lomas de Zamora**<br>
Proyecto Final · 2026 · 1.er Cuatrimestre

</div>
