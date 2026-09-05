# CLAUDE.md

Este archivo brinda guía a Claude Code (claude.ai/code) al trabajar con código en este repositorio.

## Qué es esto

Firmware para KUKO Delta Carbon, un robot delta de 3 GDL (ESP32) que toma
una pieza y la deposita sobre una cinta transportadora en movimiento.
PlatformIO + framework Arduino. Los comentarios e identificadores del código
están escritos en español; mantené los comentarios nuevos en español para
mantener la consistencia.

## Build / upload / monitor

Requiere el CLI de PlatformIO (`pio`) o la extensión de PlatformIO para VS Code.

```
pio run                       # compila el entorno "main" (por defecto)
pio run -e main -t upload     # flashea la placa
pio device monitor             # monitor serie (115200 baudios)
pio run -t upload -t monitor  # flashea y abre el monitor inmediatamente
```

Hay dos entornos en `platformio.ini`, ambos apuntando a
`espressif32` / `nodemcu-32s`:
- `env:main` — compila `src/` (el firmware real), punto de entrada `src/main.cpp`.
- `env:test` — cambia el filtro de fuentes para compilar solo `pruebas/test.cpp`
  en lugar de `src/`. Usá `pio run -e test -t upload` para flashear eso en su lugar.
  No es un setup de PlatformIO Unit Testing (no hay `pio test`), sino un
  build alternativo de un solo archivo para bring-up de placa/código de prueba.

`pruebas/` contiene, además de `test.cpp`, archivos `.bak` de
prueba/referencia (sketches independientes viejos para motores, encoders,
cinemática, etc.) — no forman parte de ningún build, útiles como referencia
histórica al depurar un subsistema específico de forma aislada.

## Arquitectura

Todo corre en un único loop de core del ESP32 (`src/main.cpp`) más ISRs de
timer de hardware para la generación de pulsos de paso. No hay uso de RTOS
tasks.

**Máquina de estados del robot** (`src/robot/Robot.h/.cpp`) es el
orquestador. Antes del `switch`, cada `update()` corre `supervisarColision()`
(ver más abajo), que puede meter al robot en `COLLISION_STOP` desde
cualquier estado. `Robot::update()` es un `switch` sobre `RobotState` (HOMING →
GO_POSITION → GRAB → GO_UP → CONVEYOR_RUN → GO_DOWN → RELEASE → GO_ZERO2 →
CONVEYOR_STOP → READY), cada uno con un handler privado `updateX()` llamado
en cada tick del loop. `TEACH` es el único estado fuera de ese ciclo (ver
abajo) y es el **último** del enum, después de `ERROR`: el índice viaja crudo
en la telemetría, así que un estado nuevo va siempre al final aunque quede
feo, nunca en el medio. Todas las esperas dentro de estos handlers son
no bloqueantes, basadas en timestamps (`millis()`) — nunca agregues un
`delay()` acá, congela la generación de pasos, las lecturas de encoders y
toda la máquina de estados simultáneamente (ver los comentarios en
`updateRelease()`/`updateConveyorStop()` sobre el incidente que este patrón
reemplazó). Los handlers de estado que emiten un movimiento lo hacen una
sola vez al entrar (protegidos por una flag como `positionMoveIssued`), no
en cada tick.

**Generación de pasos** (`src/robot/Stepper.h/.cpp`) maneja cada eje
mediante un timer de hardware del ESP32 dedicado (uno de los 4 disponibles,
`timerIndex` 0-2 usados para los 3 motores) corriendo un tick base de
frecuencia fija (`BASE_TICK_US`). El perfil de movimiento es una rampa
trapezoidal (algoritmo de Austin 2004, la misma matemática que usa
AccelStepper) calculada en **aritmética de punto fijo entero dentro de la
ISR** — sin float/double ahí, ya que el core Arduino del ESP32 no garantiza
que el contexto de la FPU se guarde al entrar a la ISR de timer, y hacer
matemática de punto flotante en la ISR ha causado crashes `LoadProhibited`
anteriormente. Float solo se usa fuera de la ISR (en
`moveTo()`/`moveContinuous()`/`setAcceleration()`) para precalcular los
límites de rampa antes de que empiecen las secciones críticas. El estado
compartido entre `loop()` y la ISR es `volatile` y está protegido por
`portENTER_CRITICAL`/`portEXIT_CRITICAL` con un mutex por instancia.

**Supervisión de colisiones** (`src/robot/CollisionGuard.h/.cpp`) es la
única parte del sistema que usa los encoders en marcha. NO es control de
posición —eso sigue siendo lazo abierto por micropasos—: compara en cada
vuelta del loop el ángulo medido contra el que dicen los pasos emitidos, y
si la diferencia pasa el umbral **y se mantiene** `CONFIRMACION_MS`, declara
colisión. El umbral no es fijo: `UMBRAL_DEG + MARGEN_VELOCIDAD_MS × velocidad`,
porque la medición del encoder llega atrasada un tiempo dado y el error que
eso produce crece con la velocidad (un umbral fijo alto tapa el problema pero
deja ciega la detección con el robot quieto). El atraso se puede además
cancelar con `RETARDO_ENCODER_MS`, que pasa la posición comandada por el mismo
pasabajos que sufre el sensor. `S` informa el atraso medido y la ganancia
encoder/pasos, que son los dos números para calibrar todo esto; `M` vuelca una
traza en vivo a 20 Hz.

El guard **casi no imprime nada por sí solo**, y es a propósito: los números
que hacen falta para calibrarlo ya viajan en `[T]` (error contra umbral, 10 Hz)
y en `[H]` (ganancia, atraso, picos y fuga por eje), así que las líneas
`[GUARD]` que salían solas —el volcado cada 15 s, el salto de la bomba en cada
conmutación, la firma de reposo en cada parada en home— eran varias por
segundo con el robot produciendo y no las lee nadie en vivo. Quedan sólo los
avisos de algo que pasó: encoder caído, salto de calibración entre paradas,
deriva acumulada y encoder invertido. El volcado periódico sigue disponible
con el parámetro `diag_ms`, que viene en 0.

Contra falsos positivos hay tres defensas además del umbral, y conviene
entender qué ataca cada una antes de tocarlas: el margen por velocidad decae
con constante de tiempo (no con ventana fija) para que siga en pie durante
todo el frenado; una sospecha cuyo error está **bajando** se cancela (el
sensor poniéndose al día decae, un brazo trabado sostiene el error); y con el
eje asentado la referencia se fuga despacio hacia lo medido, para que lo que
sobra de cada tramo no se acumule ciclo a ciclo. Además la detección se
silencia `BLANQUEO_NEUMATICA_MS` al conmutar la bomba: es una carga fuerte y
la salida del AS5600 es ratiométrica a su VCC, así que una hundida del riel
corre las tres lecturas juntas sin que se mueva nada (el guard imprime cuánto
saltó cada eje, para poder distinguirlo). La comparación es DIFERENCIAL contra
una referencia que se promedia durante la ventana de asentamiento del
homing, así no depende de la calibración absoluta del encoder (±1° de
ruido, no linealidad del AS5600 analógico, offset de home). `Robot` reacciona
frenando los 3 ejes, soltando la pieza, esperando `COLLISION_PAUSE_MS` y
rehomeando con `startHoming(true)` (conserva la cola de piezas: la cinta
nunca se detuvo). Los parámetros a tocar están todos en `GuardConfig`, al
principio de `CollisionGuard.h`, y se pueden barrer en caliente con los
comandos `U`/`T` por Serial.

**Registro de fallos** (`src/robot/FaultLog.h/.cpp`) guarda los últimos 16
fallos (colisión, encoder caído, homing vencido, parada manual) con el
contexto de la pieza involucrada, y los imprime en formato clave=valor
pensado para que la interfaz de Python los parsee. Se vuelca con `D`, que
además imprime `[FALLOS]` con los contadores **por tipo**: esos no se pierden
aunque el buffer de 16 dé la vuelta, y son el único histórico completo que
hay. El campo `estado` de cada `[FALLO]` viaja como **nombre**, no como
índice (`PROTOCOLO.md` decía lo contrario y el parser de Python estaba
escrito contra el documento, así que la columna venía vacía; hoy se aceptan
las dos formas).

`dcmd` y `denc` —grados ordenados contra grados girados— son el par que
separa *el brazo se trabó* de *el encoder midió mal*, que es la pregunta que
decide qué se va a arreglar. Está expuesto como `Fallo.brazo_frenado` en
`protocolo.py` y es de lo que vive media pestaña de Rendimiento.

**Telemetría** (`src/robot/Telemetry.h/.cpp`) son las tres líneas periódicas
que consume la interfaz: `[T]` a 10 Hz (ángulos medidos y comandados, error
del guard, umbral efectivo, finales de carrera, bomba), `[E]` a 1 Hz (modo,
cola, caja, contadores de producción) y `[H]` cada 2 s (salud de encoders,
vueltas de loop por segundo, RAM libre). Arranca **apagada**: se enciende con
`V1`, se apaga con `V0` y `V?` pide una foto sin encender el stream. La clase
no sabe nada del robot — `Robot::emitirTelemetria()` llena structs planas y
`Telemetria` sólo formatea y lleva los relojes.

**Tabla de parámetros** (`src/robot/Params.h/.cpp`) es lo que la interfaz
puede ajustar sin recompilar: cada entrada lleva su rango, unidad y nivel
(1 operación / 2 proceso / 3 servicio), el firmware **rechaza** lo que se va
de rango, y `P*` persiste en NVS (sobrevive al reflasheo, que es donde antes
se perdía toda la calibración hecha a mano). Se registran en
`Robot::registrarParametros()`; las constantes correspondientes de
`Robot.cpp` dejaron de ser `const` pero **los puntos de uso no cambiaron**.
Los cinco parámetros del guard tienen copia local en `Robot`
(`pGuardUmbral`…) porque el guard los guarda adentro con setters:
`sincronizarParametros()` los baja cuando cambia la generación de la tabla,
y de paso rearma los límites de `Motors` y reaplica el PWM de la cinta —
son los otros dos casos donde alguien guarda copia en vez de leer la
variable en el punto de uso. Por
eso los comandos históricos `U`/`T`/`K`/`L`/`Q` ahora escriben en la tabla en
vez de tocar el guard directo — si no, la tabla diría una cosa y el guard
otra, y el próximo cambio de cualquier parámetro pisaría el ajuste hecho a
mano.

El contrato completo de todo esto está en `pc/PROTOCOLO.md`, y es la
referencia válida: si el firmware y ese documento no coinciden, el que se
apartó es el firmware.

**Modo Teach** (`Robot::updateTeach()` y compañía, más `pc/kuko/teach.py` y
`pc/kuko/cinematica.py`) es un modo aparte del ciclo de clasificación: el
operador maneja el brazo a mano desde la pestaña *Teach* de la interfaz, graba
secuencias y las reproduce. El reparto está explicado entero en
`pc/PROTOCOLO.md` §6, y las tres decisiones que conviene entender antes de
tocarlo son:

- **Se manda una dirección, no un destino**, y esa dirección vence sola a los
  350 ms en el firmware. Es el hombre-muerto: si la interfaz deja de
  refrescarla, el brazo termina el tramo que tiene y se para.
- **La reproducción no frena en cada punto.** `Stepper::redirigir()` cambia
  el destino conservando la rampa, así que el brazo redondea las esquinas en
  vez de detenerse en cada una. Cuánto puede frenar desde la velocidad que
  trae **no se calcula con `v²/(2a)`**: esa fórmula es la del movimiento
  continuo y la rampa de Austin en punto fijo se aparta de ella hasta un
  190 %. El dato exacto es `|n|`, el propio índice de rampa.
  `pc/tests/test_rampa.py` simula la aritmética entera de la ISR y verifica
  que ningún encadenado deje el eje llegando rápido a un destino — que sería
  perder pasos sin choque y sin aviso. Esa prueba es un **espejo** del
  código: si se toca la rampa, hay que tocarla también ahí.
- **El jog avanza por tramos cortos, y el siguiente sale sólo cuando el
  anterior terminó.** `Stepper::moveTo()` reinicia la rampa en cada destino,
  así que reemitir uno con el eje andando dejaría al brazo persiguiendo un
  objetivo que se le escapa. Con esta forma se frena solo en vez de acumular
  atraso.
- **«Ir a una coordenada» (`JI`) pasa por home si el brazo no está ahí.**
  Quien escribe X, Y, Z no ve por dónde va a pasar el brazo, y la recta entre
  dos puntos bajos del volumen raspa la cinta. Home es brazos horizontales,
  lo más alto que llega el robot, así que subir y bajar no toca nada. Los dos
  tramos van a `FAST_LIMITS` y **no** se encadenan sin frenar: redondear la
  esquina de home cortaría el rodeo, que es todo el punto.
- **Se entra desde `WAIT_PIECE` y no en el acto** (`J1` deja el pedido). Ahí el
  brazo está quieto, en home y con las manos vacías, que es lo que hace
  conocida la posición de partida sin que el firmware necesite cinemática
  DIRECTA — no la tiene. La directa está en Python (`cinematica.py`), que es
  quien la necesita para dibujar.

Las secuencias grabadas viven en `pc/config/movimientos.json` con una marca de
hasta qué porcentaje se verificaron (0 → 15 → 50 → 100). Un movimiento recién
grabado no se estrena a fondo: se reproduce al 15 %, la interfaz pregunta si
salió bien, y sólo esa confirmación lo sube de escalón.

**Movimiento lineal** (`src/motion/Trajectory.h/.cpp`, clase
`MovimientoLineal`) es movL: la punta va **derecho** entre dos puntos, contra
el movJ de `Motors::moveSynchronized`, que va derecho en el espacio de los
ángulos y describe una curva en el de la punta. No es un detalle fino:
medido con la cinemática de este robot, cruzar la cinta con movJ se aparta
**27 mm** de la recta, y de un tacho al otro, 10 mm. Se implementa partiendo
la recta en tramos de `movl_paso` y encadenándolos sin frenar con
`Motors::redirigirSincronizado`; el error contra la recta cae con el
**cuadrado** del largo del tramo (1 cm → 0,05 mm). Dos cosas que hay que
saber antes de tocarlo, las dos explicadas largo en el header:

- **El paso y la velocidad están atados.** `Stepper::redirigir` no acepta un
  destino más cerca que la distancia de frenado y no hay planificador con
  look-ahead, así que la velocidad le pone un piso al paso: a 20 cm/s son
  0,6 cm y a 50 cm/s son 7 cm. `comenzar()` sube el paso solo cuando hace
  falta, y por eso movL a toda velocidad no existe — sería movJ otra vez.
- **La recta puede salirse del alcance con las dos puntas adentro** (el
  volumen de un delta no es convexo: 342 de 60.000 rectas al azar). Por eso
  se valida la recta entera antes de mover un paso.

`pc/tests/test_lineal.py` es el espejo en Python: mide los desvíos y verifica
la regla del paso. Los números escritos en los comentarios salen de ahí, así
que si se vuelve a medir el robot, esa prueba falla — que es lo que se
quiere.

**Cinemática** (`src/kinematics/DeltaKinematics.h/.cpp`) es un namespace
puramente matemático (sin I/O, sin llamadas a motores): `solveIK(x, y, z)`
devuelve los ángulos de las articulaciones y los targets de pasos de motor
equivalentes para la geometría del robot delta. Todas las constantes de
geometría física del robot (largos de brazos, radios de base/efector,
límites articulares, pasos por revolución, flags de inversión por motor)
viven al principio de `DeltaKinematics.h` — ese es el único lugar a tocar
después de una remedición o un cambio de microstepping del driver.
`Robot::goToPositionIK()` es el puente entre un target cartesiano y
`Motors::moveSynchronized`.

**Coordinación multi-eje** (`src/hardware/Motors.h/.cpp`,
`Motors::moveSynchronized`) escala la velocidad/aceleración de cada eje de
forma proporcional a la fracción de la distancia de recorrido más grande
entre los tres, para que los tres motores lleguen simultáneamente pese a
tener distancias distintas. `Motors::VEL_MAX` / `ACC_RAPIDA` / `ACC_SUAVE` son el
único techo de velocidad global para todo el sistema. Dejaron de ser
`constexpr` para poder barrerlos desde la interfaz: `FAST_LIMITS`,
`SOFT_LIMITS` y `DEFAULT_LIMITS` se leen igual que antes en los puntos de
uso, pero los rearma `Motors::aplicarLimites()`.

**Intercepción de la cinta transportadora** (`src/motion/ConveyorIntercept.h/.cpp`)
es un namespace de puro cálculo (sin llamadas a motores/IO) que resuelve
dónde y cuándo el brazo debe moverse para atrapar una pieza que se mueve
sobre la cinta, mediante iteración de punto fijo (el tiempo de viaje
depende del punto objetivo, que a su vez depende del tiempo de viaje).
Converge en pocas iteraciones; todavía no está conectado a la máquina de
estados de `Robot`.

**Wrappers de hardware** bajo `src/hardware/`: `Motors` (coordinación
multi-eje, arriba), `Encoders` (encoders magnéticos AS5600, uno por motor,
sobremuestreados + filtrados exponencialmente + con rechazo de saltos
implausibles; también soporta promediado por ventana de asentamiento para
la calibración de homing vía
`iniciarAsentamientoHoming()`/`calibrarHoming()`; un canal que quedó
enganchado rechazando todo se reengancha solo y levanta la bandera
`huboResincronizacion()`, que le dice a `CollisionGuard` que deje de confiar
en ese eje hasta el próximo homing), `Endstops` (lecturas de
fin de carrera por motor, usadas solo durante el homing), `Pneumatics`
(agarre/liberación por vacío), `Conveyor` (control de velocidad PWM de la cinta).

**Asignación de pines** vive en `include/Pinout.h` — el único lugar donde
deben definirse números de pin; el código de hardware/robot debería
referenciar estas macros en lugar de hardcodear números de GPIO.
Es el único header de `include/`: el resto de las constantes vive al
principio del módulo que las usa, junto a la medición que las eligió.

## Estado WIP conocido

- `MovimientoLineal` (movL) está implementado y probado, pero **sólo
  conectado al modo teach** (`JL`). Los tres lugares del ciclo normal donde
  serviría —la bajada sobre la pieza, la entrada a una celda de la caja y la
  reproducción de una secuencia grabada— están sin conectar a propósito:
  cada uno cambia el tiempo de ciclo y hay que medirlo con el robot antes de
  dejarlo puesto.
- `ConveyorIntercept` está implementado pero todavía no llamado desde `Robot`.
- Los ángulos de homing (`HOME_ANGLE_M1/2/3`, ahora en `Robot.cpp` y
  registrados como parámetros de nivel servicio) se ajustan a mano contra el
  robot físico — se espera que sigan cambiando a medida que se calibra el
  hardware.
- **La cámara se supervisa por el reloj**, no preguntándole a OpenCV: una
  USB desconectada no da error, `read()` devuelve `False` para siempre y
  `isOpened()` sigue diciendo que sí. El único dato válido es cuánto hace
  que no llega un fotograma (`EstadoSistema.camara_viva()`, ver
  `PROTOCOLO.md` §5.1). El hilo de `vision.py` son dos bucles anidados como
  los del enlace serie —uno abre el dispositivo, el otro procesa
  fotogramas—, porque el `VideoCapture` de un USB desenchufado queda muerto
  y sin **reabrirlo** volver a enchufar la cámara no arregla nada. Dos
  detalles que ya se rompieron una vez y están comentados en el código:
  abrir el dispositivo **no** cuenta como tener imagen (marcarlo así hacía
  parpadear el punto en cada reintento), y una reconexión se cuenta cuando
  vuelve un fotograma, **no** al abrir —contra una cámara ausente el
  `VideoCapture` se construye igual—. Todo esto se prueba sin cámara, con
  una falsa que se "desenchufa" desde la prueba (`pc/tests/test_camara.py`).
- La aplicación de PC vive en `pc/`: `pc/kuko/protocolo.py` es la mitad en
  Python del contrato serie (con pruebas en `pc/tests/`), `enlace.py` es el
  dueño del puerto, `vision.py` el de la cámara y `ui.py` la interfaz
  NiceGUI (`pc/kuko_app.py` levanta las tres). `cinematica.py` es la
  cinemática directa e inversa del lado de Python, `teach.py` la
  biblioteca de secuencias grabadas, `rendimiento.py` la historia de la
  corrida y `calibracion.py` los umbrales de la visión (ver abajo los dos
  últimos).
- Las pruebas se corren con `pc\.venv\Scripts\python -m pytest pc/tests`, o
  archivo por archivo (`python pc/tests/test_teach.py`). **Todas las que
  arman una página comparten un solo servidor** (`pc/tests/pagina.py`):
  `ui.run_with()` instala middleware y eso sólo se puede hacer antes de que
  la aplicación arranque, así que un servidor por archivo revienta apenas se
  corren dos en el mismo proceso.
- Varias pruebas comparan las dos mitades leyendo el C++: la tabla de
  parámetros contra sus fichas, el enum de estados contra `protocolo.py`, y
  las constantes de `DeltaKinematics.h` contra `cinematica.py`. Es la única
  defensa contra tocar un solo lado de algo que está escrito dos veces.
- **Pestaña de Rendimiento** (`pc/kuko/rendimiento.py` y los métodos
  `_rend_*` de `ui.py`). El firmware lleva contadores pero **no lleva
  historia**: no sabe cuándo pasó cada cosa ni cuánto tiempo estuvo en cada
  estado, y no debería —lo que gaste en recordar el pasado se lo saca a la
  generación de pasos—. La PC ya está leyendo todas las líneas igual, así
  que la memoria vive ahí. `Rendimiento` la alimenta el enlace y la dibuja
  la interfaz; no toca ni el puerto ni la pantalla, así que se prueba sin
  robot mintiéndole el reloj (`pc/tests/test_rendimiento.py`).

  Cuatro decisiones que conviene entender antes de tocarlo:

  - **Cada estado cae en un cajón**: trabajando, esperando pieza, parado por
    falla, arranque, teach o sin enlace. **La disponibilidad se mide contra
    el tiempo en que el robot estaba PARA trabajar**: el homing inicial, el
    modo teach y el rato apagado no van ni al numerador ni al denominador.
    Esperar pieza **no** es estar caído — si contara, dejar la cinta vacía
    cinco minutos daría 0 % con el robot perfecto. Una prueba verifica que
    ningún estado del enum quede sin cajón.
  - **El homing es el mismo estado antes y después de un choque, y no
    significa lo mismo**: se distinguen por lo que venía antes, que es lo
    único que los diferencia.
  - **El reloj es el del ESP32.** Los fallos que vuelca `D` ocurrieron hace
    minutos, así que se fechan traduciendo su `millis()` con un ancla
    (`millis()` ↔ hora de la PC del último mensaje). Fecharlos con la hora
    de llegada los amontonaría todos en el instante de conectarse.
  - **Un hueco entre dos muestras no es tiempo del robot.** Un salto mayor a
    `HUECO_MAX_S` se contabiliza como "sin enlace" y queda fuera de la
    disponibilidad: no se afirma nada sobre lo que no se vio.
  - **La cámara encabeza el veredicto.** Sin ella el firmware no recibe una
    sola pieza y el robot se queda en `WAIT_PIECE`: 100 % de disponibilidad,
    cero fallos y cero producción. Ningún otro número de la pantalla la
    delata. El promedio de FPS se calcula restando el **contador** de
    fotogramas sobre el tiempo transcurrido, no promediando tasas: así un
    rato sin cámara baja el promedio solo, mientras que promediando tasas
    ese rato no existiría —no hay muestras que promediar— y el número diría
    que todo anduvo perfecto.

  La cronología se dibuja como SVG a mano y no como un gráfico: lo que hay
  que ver de un vistazo es *dónde* están los tramos rojos, y tres paradas
  cortas repartidas en una hora dan la misma torta que una sola de quince
  minutos. Los otros siete gráficos son ECharts, que NiceGUI trae adentro
  (no sale a buscar nada a internet: la PC de la celda puede no tener red).
  Sólo se redibuja con la pestaña a la vista. Tiene scroll, igual que la
  columna derecha de la de Visión; las de operación y teach entran enteras
  en la ventana a propósito, porque se miran de reojo con el robot andando.
- **Pestaña de Visión** (`pc/kuko/calibracion.py` y los métodos `_vis_*` de
  `ui.py`, más `PROTOCOLO.md` §8). Existe por un problema concreto: el robot
  se muda de habitación, cambia la luz y los rangos HSV —elegidos midiendo
  bajo **otra** luz— dejan de contener a las piezas. El verde es el peor
  caso, y su modo de falla es total y silencioso: no detecta peor, no
  detecta **nada** (ya pasó, 0 de 120 círculos, medido en el comentario del
  `VERDE` de `vision_python/config.py`).

  Seis decisiones que conviene entender antes de tocarlo, y la primera es
  la única de todo este repositorio que puede lastimar a alguien:

  - **Calibrar frena el robot, no sólo la cinta** (`CAL1`/`CAL0`,
    `PROTOCOLO.md` §1.2.1). Parar la cinta no alcanza: una pieza apoyada a
    mano cruza la línea de detección igual —la cruza quien la apoya—, la
    visión la informa y el brazo sale a buscarla con alguien inclinado sobre
    la cinta. **Pasó, y por poco.** El enclavamiento está en
    `Robot::iniciarSiguientePieza()`, que es el único embudo por el que
    arranca una maniobra, y **no** frena un movimiento en curso: dejar el
    brazo colgado en el aire con una pieza en la ventosa es peor que dejarlo
    terminar. Por eso hay dos datos y no uno —`cal` (se pidió) y `rep` (el
    brazo ya está quieto en home)— y el cartel de la pestaña sólo se pone en
    verde con el segundo. Del lado de la PC, la visión deja de emitir
    mensajes de pieza en cuanto se pide la calibración; las dos defensas
    hacen falta, porque el firmware no puede confiar en una PC que puede
    estar vieja y la PC no puede confiar en una placa que puede no estar
    reflasheada.
  - **Un color es UN rango**, con el arco de tono pudiendo dar la vuelta
    (`h0 > h1`). El rojo vive en las dos puntas de la rueda y eso **no** son
    dos colores: se ve partido sólo porque `cv2.inRange()` no sabe que H=179
    y H=0 son vecinos. Esa limitación vive en `Rango.a_tramos()` y no sube
    hasta la pantalla —la primera versión la dejaba subir y el rojo aparecía
    con dos cuadraditos y dos juegos de sliders, o sea "dos rojos".
  - **`detection.py` y `camera.py` leen `config.X` en el punto de uso**, no
    `from config import X`. No es estilo: la segunda forma copia el número
    una sola vez al importar, así que con los sliders andando la pantalla se
    movería y la detección no —y eso falla en silencio, con el operador
    creyendo que ya probó un rango que nunca se aplicó. Lo que se le pide al
    sensor al abrirlo (resolución, backend, FOURCC) sí se sigue leyendo una
    sola vez: ningún backend de Windows lo cambia sin reabrir el
    dispositivo.
  - **Lo que se mide no pasa por la detección.** Cuando el verde se cae, la
    detección informa cero verdes, o sea que justo cuando hace falta saber
    el color de la pieza, el camino normal no dice nada.
    `calibracion.muestrear()` encuentra las piezas por **saturación** —la
    cinta es gris, las piezas son plástico de color— sin mirar los rangos
    configurados, y mide el **núcleo erosionado** de cada una; el borde es
    una franja mezclada con la cinta, ni pieza ni fondo. Eso es lo que
    permite que la pantalla diga «hay un verde en H=80 y tu rango llega a
    74». El tono se promedia **circularmente**: un rojo tiene píxeles en
    H=178 y en H=2, y la mediana aritmética de eso da cian.
  - **La interfaz no toca el `VideoCapture`.** `set()` y `read()` sobre el
    mismo dispositivo desde dos hilos cuelgan MSMF, así que la exposición
    viaja como pedido y la aplica el hilo de visión entre dos fotogramas. La
    corrección por software se aplica dentro de `Camera.read()`, o sea que
    el fotograma que se detecta y el que se muestra son el mismo.
  - **Los presets de luz parten siempre de fábrica**, nunca de lo que hay
    puesto: si no, apretar dos veces el mismo botón correría el tono dos
    veces. Sus corrimientos **no están medidos** contra las lámparas de la
    facultad —son la dirección correcta con una magnitud razonable—; el que
    da el número bueno es el ajuste automático con las piezas delante.
  - **El ajuste automático identifica las piezas por dónde están, no por
    el color que aparentan.** Se apoyan las tres sobre la línea de
    detección, una debajo de la otra y en el orden de
    `cal.ORDEN_CALIBRACION` (rojo, azul, verde de arriba hacia abajo), y
    recién al confirmar se mide. Antes se adivinaba el color por el tono, y
    eso sólo funciona con la calibración ya más o menos buena — o sea justo
    cuando no hace falta: bajo una luz nueva el verde mide H≈80-88, más
    cerca del azul de referencia (105) que del verde (60), y la pantalla
    contestaba «falta ver: Verde» con la pieza verde apoyada delante de la
    cámara, sin forma de salir de ahí. La posición la sabe el operador; el
    color es lo que se está tratando de averiguar.
  - **El ajuste automático no calibra a medias.** Si no ve exactamente tres
    piezas, o si no están en columna, no cambia nada y dice por qué: dejar
    dos colores buenos y uno con el rango de otra sala no se puede
    distinguir mirando la pantalla, y con los colores asignados por
    posición una pieza de más corre a las otras de lugar.

  Las habitaciones ("Pieza", "Aula facultad") viven en `pc/config/vision.json`
  y **sí** van al repositorio, igual que las secuencias de teach: una
  calibración medida es trabajo hecho. Lo que no va es el puerto COM ni el
  encuadre de la cámara (`local.json`), que son distintos en cada PC a
  propósito.

  `pc/tests/test_vision.py` prueba todo esto sin cámara y sin robot, sobre
  escenas sintéticas (una cinta gris y tres hexágonos): que mover un umbral
  cambie lo que detecta el detector, que se mida una pieza que la detección
  **no** ve, que el ajuste automático recupere una escena corrida, que el
  rojo siga siendo un solo color dando la vuelta, y que el cartel no se
  ponga en verde hasta que el brazo esté de verdad quieto.

  `CAL0`/`CAL1` **no tocan `cinta_pwm`**: bajar ese parámetro a cero dejaría
  la cinta parada para siempre, porque el firmware sólo reaplica el PWM si la
  cinta ya está andando.
- Las pestañas de proceso y servicio son una lista de ajustes estilo menú de
  opciones, armada **recorriendo lo que contesta `P?`** — no hay ninguna
  lista de parámetros del lado de Python. `pc/kuko/parametros.py` sólo
  aporta el nombre en castellano, el grupo y la explicación de cada uno, y
  un parámetro sin ficha ahí aparece igual, con su nombre corto. Agregar un
  ajuste sigue siendo una línea en `Robot::registrarParametros()`.
