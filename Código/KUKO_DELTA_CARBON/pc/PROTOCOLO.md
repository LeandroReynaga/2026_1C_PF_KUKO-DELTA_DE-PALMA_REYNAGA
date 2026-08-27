# Protocolo serie KUKO Delta Carbon — versión 3

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

**Sin cambio de versión** (§2.6): la interfaz pasó a parsear `[FALLOS]` y a
aceptar el `estado` de `[FALLO]` por nombre. El firmware **no cambia** —las
dos líneas son las que viene imprimiendo desde siempre—, así que la versión
no sube: subirla marcaría como incompatible a una placa que habla exactamente
el mismo protocolo.

**Cambios de la versión 3** (§6.3.1 y §6.3.2): `JI`, ir a una coordenada
escrita pasando por home, con sus eventos `[TEACH] ir` / `irfin`; y `JL`,
movimiento lineal, con `[TEACH] l` / `lfin`. También sube el tope de
`vis_lat`, hoy en 1,00 s. Nada de lo anterior cambió, pero la versión sube
igual, y esta vez con un caso real que lo justifica: con el firmware viejo en
la placa, el módulo de coordenadas de la interfaz se dibujaba, aceptaba los
números, decía que estaba yendo y el brazo no se movía — el ESP32 tiraba un
`comando de teach invalido` a la consola que nadie estaba mirando. **Un
firmware que no coincide hay que verlo al conectar, no descubrirlo con el
brazo quieto.**

**Cambios de la versión 2** (§6): modo teach. Comandos `J...`, línea
`[TEACH]`, dos campos nuevos en `[E]` y el estado `TEACH` al final del enum.
Todo lo anterior quedó igual, pero la versión sube igual: si la interfaz y el
firmware no coinciden, la pestaña de Teach parecería andar y no haría nada,
que es la clase de falla que hay que ver antes y no delante del jurado.

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
| `A` | Modo Box (pide confirmación de tapa, ver §3.4). La letra es histórica: el modo se llamaba ALFAJORES. No se cambió a `B` porque acá `B` ya significa AZUL |
| `N` | Caja nueva: borra el mapa de celdas llenas |
| `R` | Paro manual. Desde `ERROR`, rehomea |
| `D` | Vuelca el historial de fallos |
| `S` | Vuelca el estado detallado de la supervisión |
| `G` | Alterna frenar-por-colisión / sólo observar |
| `M` | Alterna la traza de diagnóstico a 20 Hz |

Los comandos del modo teach empiezan todos con `J` y van en §6.

### 1.3 Caja del modo Box

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

La interfaz **parsea** `[T]`, `[E]`, `[H]`, `[P]`, `[FALLO]`, `[FALLOS]`,
`[PIEZA]`, `[TEACH]` y `[BOOT]`. Todo lo demás (`[GUARD]`, `[MODO]`,
`[CAJA]`, `[SERIAL]`, `[EMERGENCIA]`, `[TAPA]`, `[COLA]`, `[TRAZA]`…) se
muestra tal cual en la consola de la interfaz. Una etiqueta desconocida
**nunca** es un error: se trata como texto.

### 2.1 `[BOOT]` — al arrancar el firmware

    [BOOT] proto=3 fw=2026-08-21 estados=17 params=55

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
        pc=B pf=C py=4.20 pb=3 tw=0 ti=0 nd=41 nk=38 nx=3 nf=2
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
| `bx` | Disposición de la caja, 6 colores; `-` fuera del modo Box |
| `bf` | Celdas llenas, 6 dígitos 0/1 |
| `bc` | Celda reservada por la pieza en la mano (0 = ninguna) |
| `pc` `pf` `py` `pb` | Pieza en curso: color, forma, Y en cm, tacho destino (0 = ninguno) |
| `tw` / `ti` | Teach: puntos cargados en la ruta, y cuál se está ejecutando (0 = ninguno) |
| `nd` `nk` `nx` `nf` | Detectadas, depositadas OK, descartadas por inalcanzables, fallos |
| `kr` `kg` `kb` | Depositadas por color: rojas, verdes, azules |
| `ks` `kh` `kc` | Depositadas por forma: cuadrados, hexágonos, círculos |

`tw`/`ti` viajan en la línea periódica **además** de en los eventos
`[TEACH]`: un evento suelto se puede perder en un reinicio y dejaría a la
pantalla creyendo que el robot todavía está reproduciendo algo.

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

### 2.6 `[PIEZA]`, `[FALLO]` y `[FALLOS]` — el registro de fallos

    [PIEZA] Y=4.20 color=B forma=C  en cola: 3
    [FALLO] n=2 t=125430 tipo=COLISION eje=1 err=13.20 dcmd=45.10 denc=31.90
            estado=GO_BIN pieza=1 enmano=1 color=B forma=C py=4.20 px=-1.30 tacho=3

`estado` viaja como **nombre**, no como índice: `FaultLog` guarda el literal
que devuelve `Robot::nombreEstado()`. Este documento mostró durante mucho
tiempo un `estado=6` que el firmware nunca emitió, y el parser de Python
estaba escrito contra el ejemplo, así que el campo valía `None` en todos los
fallos reales. Python acepta hoy **las dos formas** —cuatro líneas— para que
deje de importar con qué firmware esté flasheada la placa.

`dcmd` y `denc` no son decorativos: son los grados que se le ordenaron al eje
y los que giró de verdad, y su cociente es lo único que separa *el brazo se
trabó* (`denc ≈ 0`) de *el encoder midió mal* (`denc ≈ dcmd`). La interfaz lo
muestra como una columna de diagnóstico y como el gráfico de dispersión de la
pestaña de Rendimiento.

El comando `D` vuelca el registro entero, encerrado entre un encabezado y un
cierre con la misma etiqueta:

    [FALLOS] total=41 COLISION=38 ENCODER=2 HOMING=0 MANUAL=1 DESCALIBRACION=0 guardados=16
    [FALLO] ...                         (los guardados, del más viejo al más nuevo)
    [FALLOS] fin

| Clave | Significado |
|-------|-------------|
| `total` | Fallos desde el encendido. **No** se pierde aunque el buffer dé la vuelta |
| `<TIPO>` | Un contador por cada tipo de §3.3, en el orden del `enum TipoFallo` |
| `guardados` | Cuántos quedan en el registro (tope 16) |

Los contadores por tipo son el **único** lugar donde está el histórico
completo: el registro guarda los últimos 16 fallos y nada más. La interfaz
pide `D` sola al conectarse, así que la pestaña de Rendimiento arranca con lo
que el robot ya tenía guardado en vez de vacía — que es justo lo que uno
necesita cuando abre la interfaz *después* de que algo anduvo mal.

Los `[FALLO]` de ese volcado son los mismos que ya se vieron en vivo. La
interfaz **deduplica por `n`**, así que ni se cuentan dos veces ni se repiten
en la consola.

---

### 2.7 `[TEACH]` — modo aprendizaje

Todas sus formas están en §6, junto con los comandos que las provocan: el
modo entero se documenta en un solo lugar en vez de repartirlo entre las dos
direcciones.

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
|   |          | 16 | `TEACH` |

`TEACH` está **después** de `ERROR` justamente por la regla de arriba:
apéndice al final, aunque quede feo, antes que correr la numeración.
`test_los_estados_coinciden_con_el_firmware` compara las dos tablas leyendo
`Robot.h`, así que una desincronización falla en las pruebas y no en marcha.

### 3.2 Colores y formas

`R` rojo · `G` verde · `B` azul — `S` cuadrado · `H` hexágono · `C` círculo

### 3.3 Tipos de fallo

`COLISION` · `ENCODER` · `HOMING` · `MANUAL` · `DESCALIBRACION`

### 3.4 Confirmación de tapa

Entrar o salir del modo Box requiere poner o sacar la tapa con forma de
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

## 5. Los seis chequeos de componentes

Son los puntitos verde/rojo del panel de la pestaña de operación. La lógica
vive en Python (`kuko/estado.py`); acá se documenta **con qué dato se decide
cada uno**, porque un chequeo que no puede fallar es decoración y hay que
saber cuáles son cuáles.

Cinco se deciden con la telemetría del ESP32 y se apagan a gris cuando no
hay enlace. El de la **cámara** no: cuelga del USB de la PC y no del robot,
así que se caen por separado y tiene que poder decir algo cuando el otro no
está.

| Componente | Se decide con | ¿Es una verificación real? |
|---|---|---|
| **Encoders** | `[H]`: `encN`, `ganN`, `rsyN`, margen de `rmnN`/`rmxN` | **Sí.** Rojo si un canal está caído o fuera de rango, o si la ganancia bajó de 0,97 (pérdida de cuentas real) |
| **Endstops** | `[T]`: `fc` + el estado del robot | **Sí.** Rojo si un final queda pisado con el brazo lejos de home (pegado o cable en corto), o si el homing venció sin encontrar uno |
| **Motores** | `[T]`: `eN` contra `uN`; `[H]`: `fugN` creciendo siempre para el mismo lado | **Sí.** Es el lazo cerrado de seguridad que ya existe: mide si el brazo está donde los pasos dicen |
| **Cinta** | `[E]`: `cv`/`cvp`, contra la **velocidad medida por la visión** | **Sí**, y es el más valioso: el tracker ya sigue las piezas fotograma a fotograma, así que se puede medir cuántos cm/s avanzan de verdad y compararlos con `cinta_cms`. Verifica de paso la constante de la que depende toda la intercepción |
| **Neumática** | `[E]`/`[T]`: `pm`, más los fallos con `enmano=1` | **No del todo.** Sin un vacuostato, el firmware sabe si *mandó* prender la bomba, no si hay vacío. El punto refleja el estado comandado y se pone en ámbar —no verde— si se acumulan piezas perdidas con la pieza en la mano |
| **Cámara** | Cuánto hace que llegó el último fotograma (`CAMARA_TIMEOUT_S`), no lo que diga `VideoCapture` | **Sí.** Rojo si hace más de 2 s que no entra imagen, o si no se puede abrir el dispositivo; ámbar si anda pero ya se reenganchó alguna vez, que casi siempre es el cable |

La distinción importa para la defensa: cinco de los seis puntos son
mediciones, el sexto es una intención. Un vacuostato de $5 en la línea de
vacío convertiría a la neumática en el sexto chequeo real, y es la mejora de
hardware más barata que tiene el proyecto pendiente.

### 5.1 Por qué la cámara se mide por el reloj y no preguntándole

Una cámara USB desconectada **no da error**. `VideoCapture.read()` devuelve
`False` para siempre y `isOpened()` sigue diciendo que sí: preguntarle al
objeto es preguntarle al que no se enteró. El único dato que sirve es
**cuánto hace que no llega un fotograma**, y por eso el chequeo se decide
contra el reloj.

Importa más de lo que parece. Sin cámara, el firmware no recibe una sola
pieza y el robot se queda en `WAIT_PIECE`: o sea con 100 % de
disponibilidad, sin un solo fallo y sin producir nada. Es la falla que mejor
se disfraza de "hoy no vinieron piezas", y ningún otro indicador la delata
—por eso encabeza el veredicto de la pestaña de Rendimiento—.

Recuperarse **exige volver a abrir el dispositivo**: el `VideoCapture` de un
USB que se desenchufó queda muerto y no revive solo, así que sin reabrirlo
volver a enchufar la cámara no arreglaría nada. El hilo de visión reintenta,
espaciando cada vez más si sigue sin aparecer, y cuenta las reconexiones —
varias en un turno son un cable flojo, y eso no se ve en ningún otro lado.

---

## 6. Modo teach (aprendizaje)

Un modo aparte del ciclo de clasificación: el brazo lo maneja el operador
desde la pestaña *Teach* de la interfaz, o reproduce una secuencia que se le
enseñó antes. Mientras dura, la cinta queda parada y las piezas que informe
la visión **se ignoran en silencio** — no son un fallo del robot ni una pieza
perdida, así que tampoco se cuentan.

### 6.1 Quién hace qué

| | |
|---|---|
| **ESP32** | recorta al volumen de trabajo, resuelve la cinemática, mueve, y encadena los puntos de una ruta ya cargada |
| **Interfaz** | dibuja, graba, guarda las secuencias con nombre y lleva la cuenta de a qué porcentaje se verificó cada una |

El corte está ahí porque lo único que no se puede delegar es lo que protege
al robot: el recorte del volumen y el chequeo de alcance **están de los dos
lados**, y el que manda es el del firmware. La interfaz los repite nada más
que para no ofrecerle al operador un punto que va a ser rechazado.

Encadenar los puntos también es del firmware. Si cada punto esperara la
confirmación de llegada por serie, entre punto y punto se metería el ida y
vuelta del enlace y la secuencia se reproduciría a los tirones.

### 6.2 Entrar y salir

    J1     pedir entrada al modo teach
    J0     salir
    J?     volcar el estado y el volumen de trabajo

`J1` **no entra en el acto**: deja el pedido y el robot entra recién cuando
llega a `WAIT_PIECE`, o sea quieto, en home y con las manos vacías. Con una
pieza en vuelo, primero la termina. Esas tres condiciones juntas son lo que
hace que la posición de partida del jog sea conocida sin que el firmware
tenga que resolver cinemática **directa** — que no la tiene, y no la
necesita para nada más.

Se sabe que entró porque `st` pasa a `TEACH` (16), no porque conteste.
Al salir vuelve por `GO_HOME_IDLE`, que es el estado que lo lleva a home
antes de aceptar piezas de nuevo.

Una colisión o un paro manual sacan del modo: después de cualquiera de los
dos hay que rehomear, y hasta que eso pase la posición dejó de ser conocida.
Por lo mismo, `J1` desde `ERROR` o `IDLE` se rechaza con `err=rehomear`: desde
ahí el robot no llega a `WAIT_PIECE` por su cuenta y el pedido quedaría
esperando para siempre.

| Rechazo | Cuándo |
|---|---|
| `err=sinhoming` | Nunca se calibró |
| `err=rehomear` | Está en `ERROR` o `IDLE` |
| `err=nomodo` | El comando mueve el brazo y no está en teach |
| `err=ocupado` | Hay una reproducción en curso |
| `err=ik` | El punto no tiene solución |
| `err=lleno` / `err=vacio` | El buffer se llenó / no hay ruta cargada |
| `err=formato` | Los números no se entienden |

### 6.3 Jog manual

    JD<vx>,<vy>,<vz>     dirección, cada componente en [−1, 1]
    JM<x>,<y>,<z>        destino absoluto de la punta, en cm
    JP1 / JP0            bomba de vacío

**Se manda una dirección y no una posición**, y la dirección **vence sola**
a los 350 ms. Es el hombre-muerto: si la interfaz deja de refrescarla —
navegador cerrado, enlace caído, la pestaña que pasa a segundo plano — el
brazo termina el tramo que está haciendo y se para. Un destino, en cambio,
se seguiría cumpliendo con nadie mirando. La interfaz la refresca a 10 Hz
mientras haya una tecla o el joystick apretados, y manda el vector nulo en
cuanto se sueltan.

El firmware integra esa dirección en tramos cortos (uno cada 80 ms, del
largo que dé `t_jog`) y **lanza el siguiente sólo cuando el anterior
terminó**. No es un detalle de implementación: `Stepper::moveTo()` reinicia
la rampa en cada destino, así que reemitir uno con el eje todavía andando
dejaría al brazo persiguiendo un objetivo que se le escapa, y al soltar la
tecla tendría por delante todo el atraso acumulado. Con esta forma, el brazo
va siempre a un punto que ya se sabe alcanzable, y si no llega a seguir el
ritmo el jog se frena solo en vez de acumular deuda.

Velocidad y aceleración van al `t_jogpct` % del tope (15 % de fábrica).

### 6.3.1 Ir a una coordenada escrita

    JI<x>,<y>,<z>        ir al punto pasando por home, a máxima

    [TEACH] ir x=1.50 y=-2.00 z=-30.10 home=1
    [TEACH] irfin

La diferencia con `JM` no es la velocidad sino **el camino**: si el brazo no
está en home, primero sube a home y recién desde ahí baja al punto. Quien
escribe una coordenada no ve por dónde va a pasar el brazo, y la recta entre
dos puntos bajos del volumen va raspando la cinta todo el camino. Home es
brazos horizontales, o sea lo más arriba que llega el robot: subir primero y
bajar después no puede tocar nada de lo que haya abajo. `home=1` en el evento
dice que hizo el rodeo; `home=0`, que ya estaba arriba y fue derecho.

Los dos tramos van a `FAST_LIMITS` —la velocidad y la aceleración del ciclo
normal, no las de teach— y **no se encadenan sin frenar**: redondear la
esquina de home sería justamente cortar el rodeo que es todo el punto.

La cinemática se resuelve **antes** de arrancar: un punto sin solución se
rechaza con `err=ik` sin mover nada, en vez de subir a home para después
descubrir que no se puede bajar. Durante el tramo a home la posición
comandada que informa `JG` se queda en el punto de partida — el firmware no
tiene cinemática directa para decir en cartesiano dónde está home — y se
corrige al lanzar el segundo tramo.

`JX` lo corta igual que a una reproducción, y mientras dura, el jog se
ignora en silencio (no `err=ocupado`: el operador puede tener una tecla
apretada de antes y serían diez líneas por nada).

### 6.3.2 Movimiento lineal (movL)

    JL<x>,<y>,<z>        ir hasta el punto en LINEA RECTA

    [TEACH] l x=1.50 y=-2.00 z=-30.10 paso=1.00 vel=20.0
    [TEACH] lfin tramos=27 frenadas=0 paso=1.00

Todo lo demás que mueve el brazo (`JM`, `JI`, la reproducción, el ciclo
normal) es **movJ**: `Motors::moveSynchronized` reparte velocidad y
aceleración en proporción al recorrido de cada eje, así que los tres motores
arrancan y llegan juntos. Eso es una recta en el espacio de **los ángulos**,
no en el de la punta, y en un delta las dos cosas no se parecen. Medido con
la cinemática de este robot (`pc/tests/test_lineal.py`):

| Tramo | Largo | Se aparta de la recta |
|---|---|---|
| Cruzar la cinta en X | 24,0 cm | **27,3 mm** |
| De un tacho al otro | 16,0 cm | 10,2 mm |
| Bajada vertical en el centro | 6,0 cm | 0,0 mm |
| Bajada vertical en una esquina | 6,0 cm | 1,2 mm |
| Diagonal larga | 27,4 cm | **33,4 mm** |

`JL` parte la recta en tramos de `movl_paso` y resuelve la cinemática en cada
punto intermedio. Adentro de un tramo sigue siendo movJ, pero el error cae
con el **cuadrado** del largo: 1 cm da 0,05 mm y 2 cm da 0,19 mm. Los tramos
se encadenan sin frenar (`Stepper::redirigir`), así que el movimiento es uno
solo y no una sucesión de arranques.

**El paso y la velocidad están atados**, y esto es lo que hay que entender
antes de tocarlos: `redirigir` no acepta un destino más cerca que la
distancia de frenado, así que un tramo más corto que eso no se puede
encadenar. Como no hay planificador con *look-ahead* que reparta la frenada
entre varios tramos, la velocidad le pone un piso al paso — a 20 cm/s son
0,6 cm, a 50 cm/s son 7 cm. Por eso el firmware **sube el paso solo** si hace
falta, y por eso movL a toda velocidad no existe: sería movJ otra vez. El
valor de fábrica, 1 cm a 20 cm/s, es donde las dos cosas cierran.

`JL` puede fallar donde `JM` no falla: **la recta puede salirse del alcance
aunque las dos puntas estén adentro** (el volumen de un delta no es convexo).
El firmware recorre la recta entera resolviendo la inversa *antes* de mover
un paso y contesta `err=ik` sin haberse movido.

En `lfin`, `frenadas` es el número que explica un movimiento que se vio a los
tirones: son los tramos en los que un eje tuvo que invertir el sentido y no
se pudo encadenar. Con 0 el movimiento salió de un tirón solo.

### 6.4 Volumen de trabajo

Un cajón, más chico que el alcance real del brazo:

| Eje | De | A | Parámetro |
|---|---|---|---|
| X | −12 cm | +12 cm | `t_xmin` / `t_xmax` |
| Y | centro de los tachos | un centímetro antes del borde lejano de la cinta | `t_ymin` / `t_ymax` |
| Z | `grab_z` | `grab_z` + `t_zup` | `t_zup` |

El piso en Z **no es un parámetro propio**: cuelga de `grab_z`, la altura a
la que se agarra una pieza apoyada. Así, recalibrar el agarre mueve también
el límite del jog, en vez de dejar los dos diciendo cosas distintas — que es
como se termina clavando la ventosa contra la cinta.

El cajón tiene esquinas a las que un delta no llega. El firmware simplemente
no toma esos destinos (el brazo deja de avanzar para ese lado) y la interfaz
las pinta, resolviendo la inversa con `kuko/cinematica.py`, para que el
operador vea por qué.

`J?` contesta con todo junto, ya resuelto:

    [TEACH] est=on n=12 i=0 pct=15 x=1.50 y=-2.00 z=-30.10
            xmin=-12.00 xmax=12.00 ymin=-9.55 ymax=12.05
            zmin=-32.60 zmax=-26.60 cap=150

| Clave | Significado |
|-------|-------------|
| `est` | `on`, `pedido` u `off` |
| `n` / `i` | Puntos cargados, y cuál se está ejecutando (0 = ninguno) |
| `pct` | Porcentaje de la última reproducción |
| `x` `y` `z` | Posición **comandada** de la punta |
| `xmin`…`zmax` | El cajón, con el piso en Z ya resuelto |
| `cap` | Cuántos puntos entran en el buffer del firmware |

### 6.5 Volcado de la posición comandada

    JG1 / JG0     enciende / apaga el volcado a 20 Hz

    [TEACH] p x=-3.20 y=4.50 z=-30.10 b=1

Es **lo que se graba**. Podría grabarse en cambio la posición medida, que ya
viaja en `[T]`, pero el AS5600 analógico tiene ~1° de ruido y en cartesiano
eso son milímetros que después se reproducen como temblor: lo que el
operador enseñó es a dónde llevó el brazo, no cómo vibró el sensor. Y podría
calcularla la interfaz integrando el jog, pero el firmware saltea tramos
cuando el brazo no llega, así que las dos cuentas divergirían sin que nadie
se entere.

Son 760 B/s, así que se enciende sólo con la pestaña de teach a la vista.

### 6.6 Reproducción

    JC                          vaciar la ruta
    JA<x>,<y>,<z>,<b>,<w>       agregar un punto (b = bomba 0/1, w = espera ms)
    JR<pct>                     reproducir al <pct> % de velocidad y aceleración
    JX                          cortar

La reproducción **no frena en cada punto**. A `t_mezcla` cm de un punto
intermedio, el firmware redirige al siguiente conservando la velocidad
(`Stepper::redirigir`), así que el brazo pasa *cerca* del punto en vez de
clavarse en él. Sin eso, una trayectoria de veinte puntos son veinte frenadas
y veinte arranques, y a la aceleración del ciclo normal eso se siente como un
traqueteo — que es exactamente lo que se vio en el robot.

No se redondea todo: un punto con espera o con cambio de bomba se cumple
exacto (ahí se agarra o se suelta una pieza), y una esquina más cerrada que
`t_esquina` tampoco, porque ahí el tirón lo produce la esquina misma y un eje
que invierte el sentido tiene que pasar por velocidad cero de todos modos.
Cuando el redondeo no se puede, se frena y se arranca: nunca queda a medias.

La aceleración de todo el modo teach sale de `t_acel`, **no** de la del ciclo
normal. Es el primer número a bajar si el brazo vibra al reproducir.

La grabación se muestrea a 20 Hz y se **simplifica** antes de subirla
(Ramer–Douglas–Peucker, `kuko/teach.py`). El motivo no es la memoria: cada
punto es un `moveTo`, o sea un arranque y una frenada, y reproducir 400
puntos sería un movimiento a los tirones y lentísimo. Se conservan siempre
los extremos, **los cambios de bomba** y las pausas — un `E` apretado y medio
segundo quieto esperando que el vacío agarre es exactamente lo que hay que
guardar, y lo primero que borraría un simplificador que sólo mire geometría.

La espera de cada punto **no se escala** con el porcentaje. El porcentaje es
velocidad y aceleración; una espera es el tiempo que tarda el vacío en
formarse, y eso no cambia porque el brazo vaya más rápido entre punto y
punto.

Al terminar sale `[TEACH] fin`; si se corta, `[TEACH] abort motivo=<...>`.

`JX` corta **cualquier** movimiento que el operador no esté manejando en vivo
—una reproducción o un `JI`—, y por eso en la interfaz el botón de reproducir
se convierte en *Parar* mientras hay algo andando, con la misma tecla. Es una
parada seca (`Stepper::stop()`, sin rampa): a media velocidad puede perder
pasos y dejar la posición corrida, que es lo que corresponde a un botón de
parar y por lo que después conviene rehomear.

### 6.7 Verificación por etapas

Un movimiento recién grabado **no se estrena a fondo**. Lleva una marca de
hasta dónde se verificó, con cuatro valores:

| Marca | Qué significa |
|---|---|
| `0` | Sin verificar. Se reproduce al **15 %**, que es la velocidad a la que se lo enseñó |
| `15` | Salió bien despacio. Se reproduce al **50 %** |
| `50` | Salió bien a media máquina. Se reproduce al **100 %** |
| `100` | Verificado. De acá en más va siempre al 100 % |

Entre etapa y etapa la interfaz pregunta *«¿salió bien?»*, y **sólo esa
confirmación sube el escalón**. Decir que no lo deja donde estaba, así que la
próxima vez vuelve a arrancar por el escalón que faltaba.

La pregunta aparece **mientras falte verificar algo**. Un movimiento que ya
está en 100 se reproduce sin cartel al terminar: no hay escalón que subir, y
un cartel que sólo se puede contestar de una forma es un click de más en cada
pasada.

Existe porque una trayectoria hecha a mano puede pasar cerca de algo que a
paso de hombre no roza y a toda velocidad sí, y porque a 97.000 pasos/s² un
error de enseñanza no se corrige con reflejos. La marca se guarda junto con
la secuencia (`pc/config/movimientos.json`) y **vuelve a cero si se regraba**:
es otro movimiento, aunque conserve el nombre.

---

## 7. Parámetros duplicados entre firmware y visión

Estos valores existen **en los dos lados** y tienen que valer lo mismo. Si se
editan de un solo lado, el robot le empieza a errar a las piezas sin que nada
avise, y es de las fallas más caras de encontrar.

| Visión (`config.py`) | Firmware (`Robot.cpp`) | Valor |
|---|---|---|
| `LINE_X_CM` | `DETECTION_LINE_X` | −23,0 cm |
| `IMAGE_BOTTOM_Y_CM` | `BELT_MIN_Y` | −2,8 cm |
| `IMAGE_BOTTOM_Y_CM + IMAGE_HEIGHT_CM` | `BELT_MAX_Y` | 11,2 cm |
| `SERIAL_BAUDRATE` | `Serial.begin()` | 115200 |

La geometría del brazo es el otro caso, y es más peligroso porque no da
ningún síntoma: `pc/kuko/cinematica.py` repite las constantes de
`src/kinematics/DeltaKinematics.h` (largos de brazos, radios, offset de
herramienta, límites articulares y las dos ganancias de calibración). Si se
vuelve a medir el robot y se toca un solo lado, la pantalla dibuja la punta
unos milímetros corrida de donde está y nadie se entera nunca.
`test_cinematica.py` lee las dos y las compara, que es la única defensa real
contra esto.

**El núcleo compara los dos lados al conectarse** (con lo que devuelve `P?`)
y avisa si difieren. Ese chequeo solo justifica la mitad del trabajo de tener
los parámetros expuestos.

Caso aparte: `BELT_VELOCITY_CMS`. Toda la intercepción depende de ese número.
Si alguna vez se hace variable la velocidad de la cinta, el parámetro tiene
que ser la **velocidad en cm/s** —y que el firmware derive el PWM de una
tabla medida—, nunca el PWM suelto: un PWM que no se corresponde con la
velocidad real deja la planificación mintiendo en silencio.
