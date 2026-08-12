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

`pruebas/` y `otros_codigos/` contienen archivos `.bak` de prueba/referencia
(sketches independientes viejos para motores, encoders, cinemática, etc.) —
no forman parte del build `main`, útiles como referencia histórica al
depurar un subsistema específico de forma aislada.

## Arquitectura

Todo corre en un único loop de core del ESP32 (`src/main.cpp`) más ISRs de
timer de hardware para la generación de pulsos de paso. Todavía no hay uso
de RTOS tasks (`src/tasks/TaskManager.*` y `src/motion/Trajectory.*` existen
como headers vacíos — placeholders para trabajo futuro, no conectados
actualmente).

**Máquina de estados del robot** (`src/robot/Robot.h/.cpp`) es el
orquestador. Antes del `switch`, cada `update()` corre `supervisarColision()`
(ver más abajo), que puede meter al robot en `COLLISION_STOP` desde
cualquier estado. `Robot::update()` es un `switch` sobre `RobotState` (HOMING →
GO_POSITION → GRAB → GO_UP → CONVEYOR_RUN → GO_DOWN → RELEASE → GO_ZERO2 →
CONVEYOR_STOP → READY), cada uno con un handler privado `updateX()` llamado
en cada tick del loop. Todas las esperas dentro de estos handlers son
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

`src/robot/StepperISR.h` / `SteppeprISR.cpp` (notar el typo en el nombre
del archivo) es una **implementación alternativa/legacy del stepper** que
usa escritura directa a registros GPIO y un único timer compartido para
todos los ejes. No está incluida por `Robot.cpp` ni `main.cpp` —
`Stepper.h/.cpp` es la que realmente está en uso. No asumas que ambas están
activas; revisá los `#include` antes de editar cualquiera de las dos.

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
traza en vivo a 20 Hz. Como el puerto serie suele estar tomado por la visión,
todos los parámetros viven en `GuardConfig` y cada 15 s sale sola una línea
`[GUARD]` con los mismos números.

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
pensado para que la interfaz de Python los parsee. Se vuelca con `D`.

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
tener distancias distintas. `Motors::MAX_SPEED`/`MAX_ACCELERATION` son el
único techo de velocidad global para todo el sistema.

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
`include/Config.h`, `include/Types.h` e `include/Constants.h` existen
actualmente pero están vacíos.

## Estado WIP conocido

- `src/vision/Vision.*`, `src/tasks/TaskManager.*`, `src/motion/Trajectory.*`
  son archivos stub vacíos — todavía sin implementación.
- `ConveyorIntercept` está implementado pero todavía no llamado desde `Robot`.
- Los ángulos de homing (`HOME_ANGLE_M1/2/3` en `Robot.h`) y las coordenadas
  del target de pick (`TARGET_X/Y/Z` en `Robot::updateGoPosition()`) se
  ajustan a mano contra el robot físico — se espera que sigan cambiando a
  medida que se calibra el hardware.
