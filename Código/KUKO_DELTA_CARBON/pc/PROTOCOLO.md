# Protocolo serie KUKO Delta Carbon — versión 1

Contrato entre el firmware del ESP32 y la aplicación de PC. **Es la única
referencia válida**: si el firmware y Python no coinciden acá, el error está
en el que se apartó del documento, no en el otro.

- **Transporte:** UART sobre USB, 115200 baudios, 8N1.
- **Codificación:** ASCII de 7 bits. Sin acentos, sin UTF-8. El firmware no
  puede imprimir acentos y Python no debe esperarlos.
- **Unidad:** una línea terminada en `\n`. Nunca hay dos mensajes en la
  misma línea ni un mensaje partido en dos.
- **Ancho de banda:** 115200 baudios ≈ 11,5 kB/s. La telemetría periódica
  descrita acá consume ~1,8 kB/s (16 %). El resto queda libre para eventos
  y para los volcados largos (`D`, `P?`).

## Quién abre el puerto

**Un solo proceso**: `kuko.nucleo`. Ni la interfaz, ni el monitor serie de
PlatformIO, ni un script suelto pueden tenerlo abierto al mismo tiempo — en
Windows un COM lo toma un único proceso.

Abrir el puerto **resetea el ESP32** (pulso de DTR). Es el comportamiento
buscado —se arranca desde un estado conocido— pero implica que la conexión
se hace **una vez, al arrancar el núcleo**, y no se reintenta por cada acción
del operador. Después de abrir hay que esperar ~2 s y descartar lo que haya
en el buffer: son los bytes previos al arranque del firmware.

---

## 1. PC → ESP32 (comandos)

Una línea por comando. El firmware recorta espacios en los extremos y no
distingue mayúsculas de minúsculas en la letra del comando.

### 1.1 Pieza detectada por la visión

    <Y>,<color>,<forma>          ej: 3.50,B,S

Exactamente tres campos y dos comas. `Y` en centímetros del sistema del
robot, `color` ∈ {`R`,`G`,`B`}, `forma` ∈ {`S`,`H`,`C`}. La `X` no viaja: la
pieza se informa justo al cruzar la línea de detección, así que su `X` es
siempre la de la línea (`DETECTION_LINE_X` en el firmware, `LINE_X_CM` en la
visión — **ver §5, están duplicados**).

Es el único mensaje con comas, y por eso nunca se confunde con un comando de
una letra aunque compartan letra (`C` de color vs. `C` de círculo).

### 1.2 Comandos de una letra

| Cmd | Acción |
|-----|--------|
| `C` | Modo clasificación por color |
| `F` | Modo clasificación por forma |
| `A` | Modo alfajores (pide confirmación de tapa, ver §3.4) |
| `N` | Caja nueva: borra el mapa de celdas llenas |
| `R` | Paro manual. Desde `ERROR`, rehomea |
| `D` | Vuelca el historial de fallos |
| `S` | Vuelca el estado detallado de la supervisión |
| `G` | Alterna frenar-por-colisión / sólo observar |
| `M` | Alterna la traza de diagnóstico a 20 Hz |

### 1.3 Caja de alfajores

    X<c1><c2><c3><c4><c5><c6>    ej: XBRGRBG

Los seis colores de la celda 1 a la 6. Máximo 3 de cada color. Se rechaza si
hay una pieza en vuelo hacia una celda.

### 1.4 Parámetros

    P?                  vuelca la tabla completa (una línea [P] por parámetro)
    P<nombre>=<valor>   fija un parámetro       ej: Pvis_lat=0.18
    P*                  guarda los valores actuales en NVS (sobreviven al reinicio)
    P0                  vuelve a los valores de fábrica y borra lo guardado

El firmware **valida contra su propio mín/máx y rechaza** lo que se va de
rango (no lo satura en silencio: saturar dejaría a la interfaz mostrando un
número que el robot no está usando, que es peor que un error visible). La
interfaz valida por comodidad del operador; el firmware valida por
seguridad. Nunca al revés.

**Los valores negativos son válidos** y el parser tiene que aceptarlos. No es
un detalle: la latencia de visión se calibra en un rango que cruza el cero
(−0,10 a +0,30 s), porque si la cámara llegara a informar la pieza *antes* de
que cruce la línea, la corrección va para el otro lado. Los comandos
históricos `U`/`T`/`K`/`L`/`Q` sí rechazan negativos, y así se quedan: ahí
ninguno tiene sentido.

Los comandos históricos `U`, `T`, `K`, `L`, `Q` (umbral, confirmación, margen
por velocidad, retardo, umbral en reposo) siguen andando y escriben en la
misma tabla, así que la interfaz se entera igual por el `[P] set` de vuelta.

### 1.5 Telemetría

    V1     enciende el envío periódico de [T], [E] y [H]
    V0     lo apaga
    V?     manda una vez [E] y [H] (foto del estado, sin encender el stream)

Arranca **apagada**: el firmware tiene que poder usarse con un monitor serie
común sin que lo inunde de líneas.

---

## 2. ESP32 → PC

Toda línea de máquina empieza con una etiqueta entre corchetes y sigue con
pares `clave=valor` separados por un espacio. Los valores nunca llevan
espacios; sí pueden llevar comas.

La interfaz **parsea** `[T]`, `[E]`, `[H]`, `[P]`, `[FALLO]`, `[PIEZA]` y
`[BOOT]`. Todo lo demás (`[GUARD]`, `[MODO]`, `[CAJA]`, `[SERIAL]`,
`[EMERGENCIA]`, `[TAPA]`, `[COLA]`, `[TRAZA]`…) se muestra tal cual en la
consola de la interfaz. Una etiqueta desconocida **nunca** es un error: se
trata como texto.

### 2.1 `[BOOT]` — al arrancar el firmware

    [BOOT] proto=1 fw=2026-08-16 estados=16 params=24

`proto` es la versión de este documento. Si no coincide con la que espera
Python, la interfaz avisa en grande y no habilita los controles: un
protocolo desparejo es exactamente el tipo de falla que hay que ver antes de
mover el brazo, no después.

### 2.2 `[T]` — telemetría rápida (por defecto 10 Hz, parámetro `tele_ms`)

    [T] t=125430 st=4 pm=1 fc=010 a1=-45.10 a2=-44.28 a3=-44.51
        c1=-45.00 c2=-44.30 c3=-44.50 e1=0.31 e2=-0.12 e3=0.44
        u1=8.2 u2=8.1 u3=8.3 v1=1200 v2=-300 v3=0

| Clave | Significado |
|-------|-------------|
| `t` | `millis()` del ESP32 |
| `st` | Estado (índice del enum, §3.1) |
| `pm` | Bomba de vacío, 0/1 |
| `fc` | Finales de carrera, un dígito por eje (1 = pisado) |
| `aN` | Ángulo **medido** por el encoder del eje N, grados |
| `cN` | Ángulo **comandado** (según los pasos emitidos), grados |
| `eN` | Error que mira el guard (`enc − cmd`), grados |
| `uN` | Umbral **efectivo** de ese eje en este instante, grados |
| `vN` | Velocidad comandada, grados/s |

`aN` es lo que usa la interfaz para dibujar la aguja amarilla de cada dial y
para la cinemática directa. **La FK se calcula en Python**: no hay motivo
para gastar ciclos del ESP32 en algo que no necesita para moverse.

`fc` viaja en la línea rápida —y no en la de proceso— porque los finales de
carrera se dibujan en vivo: es la única forma de verificar uno sin desarmar
nada, empujándolo con el dedo y viendo si el rectángulo de la pantalla se
pinta. Hoy el firmware sólo los lee durante el homing.

`eN` contra `uN` es la barra de "cuán cerca está de declarar colisión", que es
la visualización más útil del tablero de diagnóstico.

### 2.3 `[E]` — estado de proceso (por defecto 1 Hz, parámetro `est_ms`)

    [E] t=125400 st=4 sn=PICK_APPROACH md=C mp=- cf=0 cr=0 q=3 qa=2100
        hm=1 gd=2 ob=0 sup=1 cv=1 cvp=60 bx=BRGRBG bf=000100 bc=4
        pc=B pf=C py=4.20 pb=3 nd=41 nk=38 nx=3 nf=2
        kr=12 kg=14 kb=12 ks=15 kh=11 kc=12

| Clave | Significado |
|-------|-------------|
| `st` / `sn` | Estado como índice y como nombre (permite verificar que las tablas coinciden) |
| `md` | Modo activo: `C`, `F` o `A` |
| `mp` | Modo pendiente (`-` si no hay) |
| `cf` / `cr` | Esperando confirmación de tapa (0/1) y ms que quedan |
| `q` / `qa` | Piezas en cola y antigüedad de la más vieja, ms |
| `hm` | Homing hecho, 0/1 |
| `gd` | Estado del guard: 0 desarmado, 1 promediando, 2 armado |
| `ob` | Guard en modo observador (mide y avisa pero no frena), 0/1 |
| `sup` | Paradas por colisión habilitadas, 0/1 |
| `cv` / `cvp` | Cinta en marcha (0/1) y su PWM en % |
| `bx` | Disposición de la caja, 6 colores; `-` fuera del modo alfajores |
| `bf` | Celdas llenas, 6 dígitos 0/1 |
| `bc` | Celda reservada por la pieza en la mano (0 = ninguna) |
| `pc` `pf` `py` `pb` | Pieza en curso: color, forma, Y en cm, tacho destino (0 = ninguno) |
| `nd` `nk` `nx` `nf` | Detectadas, depositadas OK, descartadas por inalcanzables, fallos |
| `kr` `kg` `kb` | Depositadas por color: rojas, verdes, azules |
| `ks` `kh` `kc` | Depositadas por forma: cuadrados, hexágonos, círculos |

`nd`/`nk`/`nx` son los contadores del indicador de rendimiento: la tasa de
éxito del sistema completo es `nk / nd`.

`kr…kc` son los contadores que la interfaz muestra al lado de cada color y
de cada forma en el panel de modo. Se cuentan **al soltar la pieza**, no al
detectarla: lo que interesa es lo que efectivamente terminó en el tacho.
Viven en el firmware y no en Python para que sobrevivan a que se cierre la
interfaz, que es lo primero que uno hace cuando algo anda mal.

### 2.4 `[H]` — salud (por defecto cada 2 s, parámetro `sal_ms`)

    [H] t=125000 up=125 loop=980 heap=182340
        enc1=ok gan1=1.000 atr1=70 pic1=2.10 rep1=0.80 fug1=0.30 rmn1=210 rmx1=3800 rsy1=0
        enc2=ok ... enc3=caido ...

| Clave | Significado |
|-------|-------------|
| `up` | Segundos desde el arranque |
| `loop` | Vueltas del loop por segundo (si se desploma, algo está bloqueando) |
| `heap` | RAM libre en bytes (si baja sin parar, hay una fuga) |
| `encN` | `ok`, `caido` o `rango` |
| `ganN` | Ganancia encoder/pasos. **Tiene que dar ~1,00**; por debajo de 0,97 se están perdiendo cuentas y ningún umbral lo arregla |
| `atrN` | Atraso de medición estimado, ms |
| `picN` | Peor error desde el homing, grados (con esto se elige el umbral) |
| `repN` | Peor error quieto en home, grados (con esto se elige el umbral en reposo) |
| `fugN` | Deriva absorbida por la fuga en reposo, grados. Si crece siempre para el mismo lado, son pasos perdidos de verdad |
| `rmnN`/`rmxN` | Extremos de lectura cruda (útiles: 60…3950) |
| `rsyN` | Resincronizaciones del canal desde el homing |

### 2.5 `[P]` — parámetros

Ante `P?`, una línea por parámetro y una de cierre:

    [P] n=vis_lat v=0.150 d=0.150 min=0.000 max=0.500 u=s l=2 t=f
    [P] fin n=24

Ante un cambio (venga de `P`, de `U`/`T`/`K`/`L`/`Q` o de donde sea):

    [P] set n=vis_lat v=0.180 ok
    [P] set n=vis_lat v=9.900 err=rango
    [P] set n=inventado err=desconocido
    [P] set n=grab_z err=bloqueado          (no se puede cambiar en marcha)

| Clave | Significado |
|-------|-------------|
| `n` | Nombre corto, ≤ 12 caracteres, sin espacios |
| `v` `d` | Valor actual y valor de fábrica |
| `min` `max` | Rango que el firmware hace cumplir |
| `u` | Unidad para mostrar (`cm`, `s`, `ms`, `deg`, `%`, `-`) |
| `l` | Nivel: 1 operación, 2 proceso, 3 servicio |
| `t` | Tipo: `f` real, `i` entero, `b` booleano |

**La tabla del firmware es la fuente de verdad.** La interfaz no lleva una
lista propia de parámetros: pide `P?` al conectarse y arma los controles con
lo que recibe. Agregar un parámetro nuevo es tocar una sola tabla en C++ y
aparece solo en la pantalla, en la pestaña que le corresponde por su nivel.

Los nombres bonitos y la ayuda de cada parámetro sí viven en Python
(`kuko/etiquetas.py`): el firmware no puede imprimir acentos y no tiene
sentido gastarle flash en textos.

### 2.6 `[PIEZA]` y `[FALLO]` — sin cambios

Ya venían en `clave=valor` y se parsean tal cual están hoy.

    [PIEZA] Y=4.20 color=B forma=C  en cola: 3
    [FALLO] n=2 t=125430 tipo=COLISION eje=1 err=13.20 dcmd=45.10 denc=31.90
            estado=6 pieza=1 enmano=1 color=B forma=C py=4.20 px=-1.30 tacho=3

---

## 3. Tablas de códigos

### 3.1 Estados del robot (`st`)

El índice **es** el orden del `enum RobotState` en `src/robot/Robot.h`. Si se
agrega un estado en el medio, se rompe la correspondencia: agregarlos siempre
**al final**, o actualizar los dos lados a la vez (`sn` está justamente para
detectar la discrepancia).

| # | Nombre | # | Nombre |
|---|--------|---|--------|
| 0 | `IDLE` | 8 | `BIN_SETTLE` |
| 1 | `HOMING` | 9 | `RELEASE_WAIT` |
| 2 | `WAIT_PIECE` | 10 | `BOX_TRANSIT` |
| 3 | `GO_HOME_IDLE` | 11 | `BOX_APPROACH` |
| 4 | `PICK_APPROACH` | 12 | `BOX_DESCEND` |
| 5 | `PICK_DESCEND` | 13 | `BOX_LIFT` |
| 6 | `PICK_LIFT` | 14 | `COLLISION_STOP` |
| 7 | `GO_BIN` | 15 | `ERROR` |

### 3.2 Colores y formas

`R` rojo · `G` verde · `B` azul — `S` cuadrado · `H` hexágono · `C` círculo

### 3.3 Tipos de fallo

`COLISION` · `ENCODER` · `HOMING` · `MANUAL` · `DESCALIBRACION`

### 3.4 Confirmación de tapa

Entrar o salir del modo alfajores requiere poner o sacar la tapa con forma de
caja. El firmware no puede verla, y arrancar con el modo equivocado significa
tirar piezas contra la tapa. Por eso el cambio se pide **dos veces**: el
primer `A` deja el pedido esperando (`cf=1`, `cr` cuenta atrás desde 10 s) y
el segundo lo confirma.

La interfaz muestra eso como un diálogo con la cuenta regresiva. Que el
operador apriete "Confirmar" es exactamente lo mismo que mandar la letra de
nuevo: **el firmware no cambia**, sólo se ve mejor.

---

## 4. Reglas de robustez

1. **Línea que no se entiende, se ignora** (y se muestra como texto). Nunca
   tirar abajo la conexión por un mensaje raro: el ESP32 puede reiniciarse a
   mitad de una línea y dejar basura.
2. **Campo que falta, se conserva el anterior.** La interfaz mantiene el
   último valor conocido y su antigüedad, no lo borra.
3. **Watchdog de enlace:** si pasan más de 1000 ms sin ninguna línea `[T]`
   con la telemetría encendida, el enlace se marca caído y la interfaz lo
   muestra en rojo. Es el check más importante de todos.
4. **Los comandos no se encolan a ciegas:** si el enlace está caído, el botón
   se deshabilita en vez de escribir a un puerto muerto.
5. El botón de PARO de la interfaz es una **conveniencia**, no un paro de
   emergencia. El paro de emergencia real corta la alimentación de los
   drivers por hardware.

---

## 5. Los cinco chequeos de componentes

Son los puntitos verde/rojo del panel de la pestaña de operación. La lógica
vive en Python (`kuko/chequeos.py`); acá se documenta **con qué dato se
decide cada uno**, porque un chequeo que no puede fallar es decoración y hay
que saber cuáles son cuáles.

| Componente | Se decide con | ¿Es una verificación real? |
|---|---|---|
| **Encoders** | `[H]`: `encN`, `ganN`, `rsyN`, margen de `rmnN`/`rmxN` | **Sí.** Rojo si un canal está caído o fuera de rango, o si la ganancia bajó de 0,97 (pérdida de cuentas real) |
| **Endstops** | `[T]`: `fc` + el estado del robot | **Sí.** Rojo si un final queda pisado con el brazo lejos de home (pegado o cable en corto), o si el homing venció sin encontrar uno |
| **Motores** | `[T]`: `eN` contra `uN`; `[H]`: `fugN` creciendo siempre para el mismo lado | **Sí.** Es el lazo cerrado de seguridad que ya existe: mide si el brazo está donde los pasos dicen |
| **Cinta** | `[E]`: `cv`/`cvp`, contra la **velocidad medida por la visión** | **Sí**, y es el más valioso: el tracker ya sigue las piezas fotograma a fotograma, así que se puede medir cuántos cm/s avanzan de verdad y compararlos con `cinta_cms`. Verifica de paso la constante de la que depende toda la intercepción |
| **Neumática** | `[E]`/`[T]`: `pm`, más los fallos con `enmano=1` | **No del todo.** Sin un vacuostato, el firmware sabe si *mandó* prender la bomba, no si hay vacío. El punto refleja el estado comandado y se pone en ámbar —no verde— si se acumulan piezas perdidas con la pieza en la mano |

La distinción importa para la defensa: cuatro de los cinco puntos son
mediciones, el quinto es una intención. Un vacuostato de $5 en la línea de
vacío lo convertiría en el quinto chequeo real, y es la mejora de hardware
más barata que tiene el proyecto pendiente.

---

## 6. Parámetros duplicados entre firmware y visión

Estos valores existen **en los dos lados** y tienen que valer lo mismo. Si se
editan de un solo lado, el robot le empieza a errar a las piezas sin que nada
avise, y es de las fallas más caras de encontrar.

| Visión (`config.py`) | Firmware (`Robot.cpp`) | Valor |
|---|---|---|
| `LINE_X_CM` | `DETECTION_LINE_X` | −23,0 cm |
| `IMAGE_BOTTOM_Y_CM` | `BELT_MIN_Y` | −2,8 cm |
| `IMAGE_BOTTOM_Y_CM + IMAGE_HEIGHT_CM` | `BELT_MAX_Y` | 11,2 cm |
| `SERIAL_BAUDRATE` | `Serial.begin()` | 115200 |

**El núcleo compara los dos lados al conectarse** (con lo que devuelve `P?`)
y avisa si difieren. Ese chequeo solo justifica la mitad del trabajo de tener
los parámetros expuestos.

Caso aparte: `BELT_VELOCITY_CMS`. Toda la intercepción depende de ese número.
Si alguna vez se hace variable la velocidad de la cinta, el parámetro tiene
que ser la **velocidad en cm/s** —y que el firmware derive el PWM de una
tabla medida—, nunca el PWM suelto: un PWM que no se corresponde con la
velocidad real deja la planificación mintiendo en silencio.
