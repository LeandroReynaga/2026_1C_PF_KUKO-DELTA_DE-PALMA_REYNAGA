#include "Robot.h"
#include "Pinout.h"
#include "hardware/Pneumatics.h"
#include "hardware/Conveyor.h"
#include "hardware/Encoders.h"
#include "kinematics/DeltaKinematics.h"
#include "motion/ConveyorIntercept.h"

#include <string.h>
#include <ctype.h>
#include <stdlib.h>

Conveyor conveyor(CINTAPWM);
Pneumatics pneumatics;

// ============================================================
//  CINTA TRANSPORTADORA
//  v = pi * d * (N/60), con d = 2,4 cm y N = 60 rpm  ->  7,54 cm/s
//  Se toma como constante y conocida: toda la intercepcion depende de este
//  numero, asi que si se cambia la polea o las rpm hay que recalcularlo
//  aca (y recalibrar el PWM de Conveyor::begin(), que hoy arranca al 60%
//  sin relacion medida con las rpm reales).
// ============================================================
static float BELT_VELOCITY_CMS = 7.2f; //antes 7.54, despues 6.75

// Cuanto se le manda al driver de la cinta, en porcentaje del PWM. OJO:
// esto NO cambia solo a BELT_VELOCITY_CMS, que es la velocidad MEDIDA y es
// la que usa la planificacion. Tocar uno sin el otro hace que el robot le
// erre a todas las piezas, asi que despues de mover el PWM hay que volver a
// medir la cinta y actualizar el otro numero (la interfaz muestra los dos
// juntos y la vision mide el real, para poder compararlos).
static float CONVEYOR_PWM = 60.0f;
static float DETECTION_LINE_X  = -23.0f; // donde la camara detecta las piezas

// Ancho util de la cinta (Y). Fuera de esto el dato de vision es erroneo.
static const float BELT_MIN_Y = -1.95f;
static const float BELT_MAX_Y = 12.05f;

// ============================================================
//  LATENCIA DE LA VISION
// ============================================================
// Entre el instante en que la pieza cruza FISICAMENTE la linea de
// deteccion y el instante en que el mensaje llega al ESP32 pasa un
// tiempo muerto: exposicion de la camara, buffer del driver, el
// fotograma que hay que esperar para ver el cruce, la deteccion en
// si y el envio por Serial. Durante todo eso la pieza SIGUE
// AVANZANDO sobre la cinta.
//
// Sin compensarlo, el robot cree que la pieza esta mas atras de
// donde realmente esta, apunta detras de ella, y la pieza le queda
// adelantada: aparece a la derecha de la ventosa.
//
// Se modela como TIEMPO y no como una distancia fija a proposito.
// El error en cm es latencia * velocidad de cinta, asi que
// expresarlo en segundos lo deja valido aunque se cambien las rpm
// de la cinta; una constante en cm habria que recalibrarla.
//
// PARA CALIBRARLO: medir cuanto le erra el gripper en X y dividir
// por la velocidad de la cinta.
//
//     latencia_s = error_cm / BELT_VELOCITY_CMS
//
//   pieza ADELANTADA (a la derecha de la ventosa) -> falta, subirlo
//   pieza ATRASADA   (a la izquierda)             -> sobra, bajarlo
//
// Valor actual: se midio 1,5 cm de atraso con la cinta a 7,54 cm/s
//     1,5 / 7,54 = 0,199 s
static float VISION_LATENCY_S = 0.0f; // antes 0.2

// ============================================================
//  GEOMETRIA DE AGARRE (coordenadas de la PUNTA del gripper)
//  DeltaKinematics ya descuenta el offset de herramienta (0,0,-2.8).
// ============================================================
static float GRAB_Z      = -32.6f; // -32.3 antes, -32.9 despues. cara superior de la pieza (1 cm de alto)
static float APPROACH_DX = -2.0f;  // 2 cm por detras: rampa a favor de la cinta
static float APPROACH_DZ = 2.7f;   // 0.4 antes, despues 0.7. 4 mm por arriba

// Cuanto baja el tramo 2 por DEBAJO de la cara de la pieza. Es lo que hace
// que el contacto ocurra a mitad del movimiento y no al final, o sea con el
// gripper todavia andando a la velocidad de la cinta en vez de frenado
// (ver la explicacion completa en ConveyorIntercept.h). De paso comprime la
// ventosa, que ayuda al sellado.
//
// Es EL parametro para ajustar la suavidad del encuentro:
//   0,25 mm -> toca a 7,54 cm/s = velocidad de cinta (velocity-matched)
//   menos    -> toca mas lento que la cinta (la pieza se le adelanta)
//   mas      -> toca mas rapido que la cinta (el gripper la alcanza)
static float PRESS_DZ = 0.025f;

static float LIFT_DZ = 3.0f;       // despegue de la pieza de la cinta (antes 2)

// Area donde es seguro agarrar, validada a mano sobre el robot real
// (inspeccion visual de las rotulas en todas las posiciones). El punto de
// aproximacion cae en X = -12 cuando se agarra en el borde de entrada.
static const float WORK_AREA_MIN_X = -10.0f;
static const float WORK_AREA_MAX_X = 10.0f;

// ============================================================
//  TACHOS (posicion de la punta del gripper para soltar la pieza)
//  Modo COLOR: 1 rojo, 2 verde, 3 azul
//  Modo FORMA: 1 cuadrado, 2 hexagono, 3 circulo
// ============================================================
static float BIN_X[3] = {-12.0f, 0.0f, 12.0f};
static float BIN_Y    = -9.55f;
static float BIN_Z    = -26.5f; //antes -29.3cm

// ============================================================
//  CAJA DE ALFAJORES (modo SORT_ALFAJORES)
// ============================================================
//  Sobre los tachos se apoya una tapa con la forma de una caja de 6, y el
//  robot la llena con circulos ordenados por color. Vista desde arriba:
//
//                        cinta (Y positivo)
//          +---------------------------------+
//          |    1        2        3          |   Y = -6,8   (fila de adelante)
//          |    4        5        6          |   Y = -12,8  (fila del fondo)
//          +---------------------------------+
//              X=-6     X=0     X=+6
//
//  El piso de la caja esta a la misma altura que la cinta, asi que el Z de
//  soltado es el mismo que el de agarre: la punta baja hasta BOX_Z y el
//  alfajor, que cuelga 1 cm por debajo, queda apoyado en el piso.
// ============================================================
static const float BOX_X[6] = {-6.0f, 0.0f, 6.0f, -6.0f, 0.0f, 6.0f};
static const float BOX_Y[6] = {-6.8f, -6.8f, -6.8f, -12.8f, -12.8f, -12.8f};

// Corrimiento de TODA la caja. La grilla de arriba es la geometria de la
// caja (6 cm entre celdas, y eso no cambia), pero la caja apoyada sobre la
// mesa nunca queda dos veces en el mismo lugar. Estos dos numeros son para
// centrarla mirando el robot, en vez de reflashear por medio centimetro.
static float BOX_DX = 0.0f;
static float BOX_DY = 0.0f;

// EL parametro de ajuste fino del modo: si los alfajores quedan colgados o
// la ventosa aplasta la caja, se corrige aca (y en ningun otro lado).
static float BOX_Z = -32.6f;

// Altura desde la que se hace la bajada lenta sobre la celda.
static float BOX_APPROACH_DZ = 3.0f;

// Altura de CRUCE sobre la caja: a la que se entra, a la que se sale y a la
// que se vuela de una celda a otra.
//
// No es un numero elegido por comodidad. moveSynchronized escala velocidad
// y aceleracion de cada eje en proporcion a su recorrido, asi que los tres
// motores siguen el mismo perfil normalizado: el movimiento es una RECTA EN
// EL ESPACIO DE LAS ARTICULACIONES y en cartesiano se curva hacia abajo.
// Yendo desde el extremo lejano de la cinta hasta la celda 4 con los 3 cm
// de aproximacion como unica altura, esa curva hunde la punta hasta Z=-33,0
// con la pieza colgando en -34,0: por debajo del piso de la caja (-33,6).
// El brazo araria la caja antes de llegar a la celda.
//
// Con 6 cm de cruce el peor caso deja 1,4 cm de luz entre el alfajor que
// lleva y la cara superior de los que ya estan puestos, y la salida hacia
// la cinta deja 1,4 cm entre la punta y esos mismos alfajores.
//
// Si se cambia la geometria de la caja (celdas mas al fondo, piso mas bajo)
// hay que rehacer esa cuenta: es lo que separa "pasa por encima" de "choca".
static float BOX_TRANSIT_DZ = 6.0f;

// Disposicion por defecto de la caja: que color va en cada celda.
//
//      1 azul     2 rojo     3 verde
//      4 rojo     5 azul     6 verde
//
// Es provisoria hasta que la interfaz deje elegir los 6 colores a mano; el
// comando 'X' ya permite cambiarla sin recompilar.
static const char BOX_LAYOUT_DEFECTO[6] = {'B', 'R', 'G',
                                           'R', 'B', 'G'};

// Tope de alfajores del mismo color en una caja de 6.
static const uint8_t BOX_MAX_POR_COLOR = 3;

// Cuando el color que llega sirve para mas de una celda, se elige SIEMPRE
// la de numero mas grande (un rojo que puede ir en la 1 o en la 3 va a la
// 3; un verde que puede ir en la 4, 5 o 6 va a la 6). Ver
// piezaSirveParaCaja(), que recorre las celdas de la 6 a la 1.
//
// De paso, esto cumple solo la condicion que importa para no golpear nada:
// como las celdas 4-6 son la fila del fondo, quedan llenas antes que la
// fila de adelante (1-3). El brazo entra a la caja desde la cinta, o sea
// desde Y positivo, asi que para llegar al fondo cruza por encima de la
// fila de adelante; llenandola ultima, nunca sobrevuela un alfajor puesto.
//
// Esta eleccion NO decide a que pieza se va a buscar: eso lo fija el orden
// en que las detecto la vision (la cola es FIFO, ver iniciarSiguientePieza).

// Cuanto se espera con la bomba apagada antes de sacar el brazo de la caja.
// Es mas que RELEASE_DETACH_MS a proposito: en el tacho la pieza se suelta
// en el aire y se cae sola, pero en la caja queda APOYADA con la ventosa
// todavia tocandola, asi que tiene menos ayuda para despegarse. Si igual
// sale pegada al gripper, este es el numero a subir (y la solucion de
// fondo, la misma que en el tacho: la electrovalvula que falta).
static float BOX_RELEASE_DETACH_MS = 0;

// Ventana para confirmar un cambio de modo que cruza el limite de la tapa.
static float CONFIRMACION_TAPA_MS = 10000;

// ============================================================
//  ANGULOS DE HOMING
// ============================================================
// Angulo real de cada brazo cuando su final de carrera esta pisado. Es la
// referencia de TODO: con estos numeros se fija la posicion en pasos al
// terminar el homing y se calibra el cero de los encoders.
//
// Estaban en Robot.h como constantes de compilacion. Se mudaron aca porque
// ahora son ajustables desde la tabla de parametros (nivel servicio) y la
// tabla trabaja con punteros a float.
//
// Se miden a mano contra el robot: son de los pocos valores que no se
// pueden calcular, solo verificar.
static float HOME_ANGLE_M1 = -45.1f;
static float HOME_ANGLE_M2 = -44.3f;
static float HOME_ANGLE_M3 = -44.5f;

// ============================================================
//  TIEMPOS (todos no bloqueantes, con millis())
// ============================================================
static float HOMING_SETTLE_WAIT_MS = 2500; // ventana de promediado de encoders
static float BIN_SETTLE_MS         = 5;  // quieto sobre el tacho antes de soltar antes 200

// Cuanto se espera con la bomba apagada para que la pieza se despegue del
// gripper. Es un parche temporal: falta la electrovalvula que mete aire en
// la linea de vacio al apagar la bomba. Cuando este montada (misma senal
// que la bomba, no cambia el pinout) la pieza cae en el instante y este
// tiempo se puede bajar a ~100 ms.
static float RELEASE_DETACH_MS = 80;

// Cuanto antes del instante de bajada se prende la bomba de vacio. Ver el
// comentario largo en updatePickApproach(): tiene que alcanzar para que el
// vacio se forme, y de mas la deja soplando al aire esperando la pieza.
static float PUMP_LEAD_MS = 300;

// Margen de atraso tolerable al llegar al punto de aproximacion antes de
// dar por perdido el instante de encuentro y replanificar.
static float PICK_LATE_TOLERANCE_MS = 30;
static float MAX_REPLAN_ATTEMPTS    = 2;

// ============================================================
//  RECUPERACION DE COLISIONES
// ============================================================
// Cuanto se queda quieto el robot despues de detectar una colision, antes
// de arrancar la recalibracion. No es para que "se acomode" nada: es para
// que quien este mirando alcance a ver que paso y a sacar la mano o el
// obstaculo antes de que el brazo vuelva a moverse solo.
static float COLLISION_PAUSE_MS = 3000;

// Colisiones seguidas (sin completar ninguna pieza en el medio) despues de
// las cuales se deja de reintentar. Si el brazo choca contra lo mismo tres
// veces, rehomear una cuarta no lo va a arreglar: lo unico que se logra es
// seguir golpeando el robot.
static float MAX_COLISIONES_SEGUIDAS = 3;

// Cuanto se suspende la supervision al conmutar la bomba de vacio.
//
// La bomba es una carga fuerte que arranca justo cuando el brazo termina el
// tramo rapido y todavia no empezo el lento -- o sea, exactamente donde
// aparecian los falsos positivos. La salida del AS5600 es ratiometrica a su
// VCC y el ADC del ESP32 mide contra una referencia interna, asi que
// cualquier hundida del riel al arrancar la bomba corre las tres lecturas
// de golpe sin que se haya movido nada.
//
// Ademas, el tramo de bajada TOCA la pieza a proposito (y la comprime, ver
// PRESS_DZ): ahi hay una perturbacion mecanica real y esperada, que no es
// una colision. Esta ventana cubre las dos cosas.
static float BLANQUEO_NEUMATICA_MS = 300;

// Cada cuanto se vuelca una linea con los numeros de la supervision. Existe
// porque cuando la vision de Python tiene tomado el puerto serie no se le
// pueden mandar comandos al robot ('S', 'M'), asi que los datos para
// calibrar tienen que salir solos. La interfaz de Python los acumula como
// una linea mas, no le molesta.
//
// 0 = apagado.
static float DIAGNOSTICO_PERIODICO_MS = 15000;

// Tiempo maximo que puede tardar el homing en encontrar los 3 finales de
// carrera. Si se pasa, es que un eje no llega (trabado contra algo, o el
// obstaculo que causo la colision sigue ahi): se corta todo, se para la
// cinta y se espera intervencion manual. Sin esto, un brazo trabado deja
// al robot empujando contra el obstaculo para siempre.
static float HOMING_TIMEOUT_MS = 20000;

namespace {

// Recorta espacios/tabs al principio y al final, in situ. Los mensajes los
// puede tipear una persona a mano en el monitor serie mientras todavia no
// esta el programa de Python, asi que "3.5, B, S" tiene que valer igual que
// "3.5,B,S".
char *recortar(char *s)
{
    while (*s == ' ' || *s == '\t') s++;

    char *fin = s + strlen(s);
    while (fin > s && (fin[-1] == ' ' || fin[-1] == '\t')) fin--;
    *fin = '\0';

    return s;
}

} // namespace

Robot::Robot() :

motor1(PUL1, DIR1, ENA, 0),
motor2(PUL2, DIR2, ENA, 1),
motor3(PUL3, DIR3, ENA, 2)

{
    state = IDLE;

    axis1Homed = false;
    axis2Homed = false;
    axis3Homed = false;

    memcpy(boxLayout, BOX_LAYOUT_DEFECTO, BOX_CELLS);

    for (uint8_t i = 0; i < BOX_CELLS; i++)
    {
        boxFilled[i] = false;
    }
}

void Robot::begin()
{
    pneumatics.begin();

    motor1.begin();
    motor2.begin();
    motor3.begin();

    motor1.setSpeed(2000);
    motor2.setSpeed(2000);
    motor3.setSpeed(2000);

    endstops.begin();

    guard.begin();
    guard.setObservar(!supervisionHabilitada);
    fallos.begin();

    // Parametros: primero se registran con los valores compilados (que
    // quedan como los "de fabrica") y recien despues se carga lo guardado
    // en la NVS. El orden importa: al reves, lo guardado se tomaria por
    // valor de fabrica y 'P0' no volveria a ningun lado conocido.
    params.begin();
    registrarParametros();
    params.cargarGuardados();
    sincronizarParametros();
    generacionParams = params.generacion();

    // Una disposicion imposible de completar (mas de 3 alfajores de un mismo
    // color) se detecta al arrancar y bloquea la entrada al modo: mejor eso
    // que un robot esperando para siempre una pieza que no va a poder ubicar.
    boxLayoutValido = layoutValido(boxLayout);

    Serial.println();
    Serial.println("=== KUKO DELTA CARBON ===");
    Serial.println("Comandos por Serial (uno por linea):");
    Serial.println("  Y,color,forma   pieza detectada. Ej: 3.5,B,S");
    Serial.println("                  color = R/G/B, forma = S/H/C");
    Serial.println("  C               clasificar por COLOR");
    Serial.println("  F               clasificar por FORMA");
    Serial.println("  A               modo ALFAJORES (llenar la caja de 6)");
    Serial.println("  N               caja nueva (marca las 6 celdas como vacias)");
    Serial.println("  X<6 colores>    disposicion de la caja, celda 1 a 6. Ej: XBRGRBG");
    Serial.println("  R               parada de emergencia / reinicio");
    Serial.println("  D               historial de fallos");
    Serial.println("  S               estado de la supervision por encoders");
    Serial.println("  G               prender/apagar la supervision");
    Serial.println("  M               traza en vivo del encoder vs los pasos");
    Serial.println("  U<grados>       umbral fijo de colision. Ej: U12.5");
    Serial.println("  T<ms>           tiempo de confirmacion. Ej: T80");
    Serial.println("  K<ms>           margen por velocidad. Ej: K80");
    Serial.println("  L<ms>           atraso del encoder a compensar. Ej: L70");
    Serial.println("  Q<grados>       umbral con el robot quieto en home. Ej: Q5");
    Serial.println("  V1 / V0 / V?    telemetria: encender / apagar / una foto");
    Serial.println("  P?              listar parametros ajustables");
    Serial.println("  P<nombre>=<val> fijar un parametro. Ej: Pvis_lat=0.18");
    Serial.println("  P* / P0         guardar en la NVS / volver a fabrica");

    telemetria.anunciar((uint8_t)(ERROR + 1), params.cantidad());

    if (!boxLayoutValido)
    {
        Serial.print("[CAJA] disposicion invalida (maximo ");
        Serial.print(BOX_MAX_POR_COLOR);
        Serial.println(" alfajores por color): el modo ALFAJORES queda bloqueado");
    }
}

void Robot::update()
{
    procesarSerial();

    // Supervision de colisiones: corre ANTES de la maquina de estados, en
    // cada vuelta del loop, sin importar en que estado este el robot. Si
    // detecta una colision cambia el estado a COLLISION_STOP y el switch de
    // abajo ya entra por ahi.
    supervisarColision();

    vencerConfirmacion();

    if (trazaActiva && (uint32_t)(millis() - ultimaTraza_ms) >= TRAZA_INTERVALO_MS)
    {
        ultimaTraza_ms = millis();
        imprimirTraza();
    }

    if (DIAGNOSTICO_PERIODICO_MS > 0 &&
        state != IDLE && state != ERROR &&
        (uint32_t)(millis() - ultimoDiagnostico_ms) >= (uint32_t)DIAGNOSTICO_PERIODICO_MS)
    {
        ultimoDiagnostico_ms = millis();
        imprimirDiagnosticoCorto();
    }

    // Un parametro que cambio (por 'P', por 'U'/'T'/'K'/'L'/'Q', o al
    // cargar la NVS) puede tener que bajarse a un objeto que guarda copia.
    if (params.generacion() != generacionParams)
    {
        generacionParams = params.generacion();
        sincronizarParametros();
    }

    emitirTelemetria(millis());

    switch (state)
    {
        case HOMING:         updateHoming();        break;
        case WAIT_PIECE:     updateWaitPiece();     break;
        case GO_HOME_IDLE:   updateGoHomeIdle();    break;
        case PICK_APPROACH:  updatePickApproach();  break;
        case PICK_DESCEND:   updatePickDescend();   break;
        case PICK_LIFT:      updatePickLift();      break;
        case GO_BIN:         updateGoBin();         break;
        case BIN_SETTLE:     updateBinSettle();     break;
        case RELEASE_WAIT:   updateReleaseWait();   break;
        case BOX_TRANSIT:    updateBoxTransit();    break;
        case BOX_APPROACH:   updateBoxApproach();   break;
        case BOX_DESCEND:    updateBoxDescend();    break;
        case BOX_LIFT:       updateBoxLift();       break;
        case COLLISION_STOP: updateCollisionStop(); break;

        default:                                    break;
    }

    motor1.update();
    motor2.update();
    motor3.update();
}

// ============================================================
//  CONSOLA SERIE
// ============================================================

void Robot::procesarSerial()
{
    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r')
        {
            if (cmdOverflow)
            {
                Serial.println("[SERIAL] linea demasiado larga, descartada");
                cmdOverflow = false;
            }
            else if (cmdLen > 0)
            {
                cmdBuffer[cmdLen] = '\0';
                procesarComando(cmdBuffer, cmdLen);
            }
            cmdLen = 0;
        }
        else if (cmdOverflow)
        {
            // Se descarta hasta el fin de linea: si se reiniciara el buffer
            // aca, la COLA de una linea larga terminaria ejecutandose como
            // si fuera un comando nuevo y valido.
        }
        else if (cmdLen < sizeof(cmdBuffer) - 1)
        {
            cmdBuffer[cmdLen++] = c;
        }
        else
        {
            cmdOverflow = true;
            cmdLen = 0;
        }
    }
}

void Robot::procesarComando(char *cmd, uint8_t len)
{
    (void)len; // se recalcula despues de recortar espacios

    cmd = recortar(cmd);
    const size_t n = strlen(cmd);

    if (n == 0)
    {
        return; // linea vacia (o solo espacios): se ignora sin ruido
    }

    // --- Comandos de un solo caracter: modo de clasificacion y emergencia ---
    // Un mensaje de pieza SIEMPRE tiene 2 comas, asi que no hay ambiguedad
    // con 'C' (modo color) ni con 'R' (reset), aunque esas mismas letras se
    // usen como forma/color adentro de un mensaje de pieza.
    if (n == 1)
    {
        const char c = toupper(cmd[0]);

        if (c == 'R')
        {
            if (state == ERROR)
            {
                Serial.println("[RESET] Rehomeando...");
                startHoming();
            }
            else
            {
                emergencyStop();
            }
            return;
        }

        if (c == 'D')
        {
            fallos.imprimirHistorial();
            return;
        }

        if (c == 'S')
        {
            imprimirEstadoSupervision();
            return;
        }

        if (c == 'M')
        {
            trazaActiva = !trazaActiva;

            if (trazaActiva)
            {
                Serial.println("[TRAZA] on. columnas por eje: raw (0-4095), enc y cmd en grados");
                Serial.println("[TRAZA] desde el homing, err = enc - cmd (el que mira el guard)");
                ultimaTraza_ms = 0;
            }
            else
            {
                Serial.println("[TRAZA] off");
            }
            return;
        }

        if (c == 'G')
        {
            supervisionHabilitada = !supervisionHabilitada;

            // El guard NO se desarma: sigue midiendo, marcando picos y
            // avisando por Serial lo que habria hecho. Lo unico que cambia
            // es si eso frena o no el robot. Asi se puede seguir
            // trabajando y calibrando al mismo tiempo, sin quedarse sin
            // datos justo cuando hacen falta.
            guard.setObservar(!supervisionHabilitada);

            if (!supervisionHabilitada)
            {
                Serial.println("[GUARD] paradas APAGADAS: sigue midiendo y avisando, pero no frena");
            }
            else
            {
                Serial.println("[GUARD] paradas ACTIVAS");
            }
            return;
        }

        if (c == 'C' || c == 'F' || c == 'A')
        {
            const SortMode nuevo = (c == 'C') ? SORT_BY_COLOR :
                                   (c == 'F') ? SORT_BY_SHAPE :
                                                SORT_ALFAJORES;
            pedirModo(nuevo, c);
            return;
        }

        if (c == 'N')
        {
            // Caja nueva: se pide cuando la caja llena ya se retiro y se
            // puso otra vacia. Es lo unico que borra el mapa de celdas.
            reiniciarCaja(true);
            return;
        }

        Serial.print("[SERIAL] comando invalido: '");
        Serial.print(cmd);
        Serial.println("'. Validos: 'C', 'F', 'A', 'N', 'R', 'D', 'S', 'G' o 'Y,color,forma'");
        return;
    }

    // --- Telemetria ('V1', 'V0', 'V?') y parametros ('P?', 'P*', 'P0',
    //     'Pnombre=valor') ---
    // Van antes que todo lo demas de varios caracteres porque son los
    // unicos que puede mandar la interfaz sin intervencion de una persona:
    // conviene que ni siquiera pasen por el resto del parser.
    if (procesarComandoTelemetria(cmd) || procesarComandoParametro(cmd))
    {
        return;
    }

    // --- Disposicion de la caja: 'X' + los 6 colores, de la celda 1 a la 6 ---
    // Es el reemplazo provisorio de la interfaz que todavia no existe: deja
    // elegir que color va en cada celda sin recompilar. Ej: XBRGRBG.
    if (toupper(cmd[0]) == 'X' && strchr(cmd, ',') == NULL)
    {
        if (n != BOX_CELLS + 1)
        {
            Serial.print("[CAJA] se esperan ");
            Serial.print(BOX_CELLS);
            Serial.println(" colores (celda 1 a 6). Ej: XBRGRBG");
            return;
        }

        char nuevo[BOX_CELLS];

        for (uint8_t i = 0; i < BOX_CELLS; i++)
        {
            nuevo[i] = toupper(cmd[i + 1]);
        }

        if (!layoutValido(nuevo))
        {
            Serial.print("[CAJA] disposicion invalida: colores R/G/B y como maximo ");
            Serial.print(BOX_MAX_POR_COLOR);
            Serial.println(" de cada uno");
            return;
        }

        // Cambiar la disposicion con una pieza en vuelo la mandaria a una
        // celda que ya no significa lo mismo.
        if (currentCell != CELDA_NINGUNA)
        {
            Serial.println("[CAJA] hay una pieza en camino a una celda: probar de nuevo en un momento");
            return;
        }

        aplicarLayout(nuevo);
        return;
    }

    // --- Ajuste en caliente de la supervision: 'U<grados>', 'T<ms>',
    //     'K<ms>' (margen por velocidad), 'L<ms>' (atraso a compensar) y
    //     'Q<grados>' (umbral con el robot quieto en home) ---
    // No llevan coma, asi que no se confunden nunca con un mensaje de pieza
    // (que siempre tiene dos). Se conservan porque son cortos de tipear con
    // el robot delante, pero NO tocan el guard directamente: escriben en la
    // tabla de parametros igual que 'P'. Si escribieran el guard por su
    // cuenta, la tabla quedaria diciendo el valor viejo y el proximo
    // sincronizarGuard() pisaria el cambio hecho a mano.
    {
        const char letra = toupper(cmd[0]);

        if ((letra == 'U' || letra == 'T' || letra == 'K' || letra == 'L' ||
             letra == 'Q') &&
            strchr(cmd, ',') == NULL)
        {
            char *finValor = NULL;
            const float valor = strtof(cmd + 1, &finValor);

            if (finValor == cmd + 1 || *finValor != '\0' || valor < 0.0f)
            {
                Serial.print("[SERIAL] valor invalido: '");
                Serial.print(cmd);
                Serial.println("'. Se espera 'U12.5' (grados) o 'T80'/'K80'/'L70' (ms)");
                return;
            }

            switch (letra)
            {
                case 'U':
                    aplicarParametro("g_umbral", valor);
                    break;

                case 'T':
                    aplicarParametro("g_conf", valor);
                    break;

                case 'K':
                    aplicarParametro("g_margen", valor);
                    break;

                case 'Q':
                    aplicarParametro("g_reposo", valor);
                    break;

                default: // 'L'
                    aplicarParametro("g_retardo", valor);
                    break;
            }
            return;
        }
    }

    // --- Mensaje de pieza: "Y,color,forma" (exactamente 3 campos) ---
    char *coma1 = strchr(cmd, ',');
    if (coma1 == NULL)
    {
        Serial.print("[SERIAL] comando invalido: '");
        Serial.print(cmd);
        Serial.println("'. Validos: 'C', 'F', 'A', 'N', 'R', 'D', 'S', 'G' o 'Y,color,forma'");
        return;
    }

    char *coma2 = strchr(coma1 + 1, ',');
    if (coma2 == NULL)
    {
        Serial.println("[SERIAL] faltan campos, se espera 'Y,color,forma' (ej: 3.5,B,S)");
        return;
    }

    if (strchr(coma2 + 1, ',') != NULL)
    {
        Serial.println("[SERIAL] sobran campos, se espera 'Y,color,forma' (ej: 3.5,B,S)");
        return;
    }

    *coma1 = '\0';
    *coma2 = '\0';

    char *campoY     = recortar(cmd);
    char *campoColor = recortar(coma1 + 1);
    char *campoForma = recortar(coma2 + 1);

    // El campo Y tiene que ser un numero COMPLETO. Con atof() no alcanzaba:
    // devuelve 0.0 ante cualquier texto que no sea numerico, y 0.0 cae
    // dentro de la cinta, asi que un "hola,B,S" se habria aceptado en
    // silencio como una pieza en Y=0.
    char *fin = NULL;
    const float y = strtof(campoY, &fin);

    if (*campoY == '\0' || fin == campoY || *fin != '\0')
    {
        Serial.print("[SERIAL] Y invalida: '");
        Serial.print(campoY);
        Serial.println("'. Tiene que ser un numero en cm (ej: 3.5)");
        return;
    }

    if (strlen(campoColor) != 1)
    {
        Serial.print("[SERIAL] color invalido: '");
        Serial.print(campoColor);
        Serial.println("'. Tiene que ser un solo caracter: R, G o B");
        return;
    }

    if (strlen(campoForma) != 1)
    {
        Serial.print("[SERIAL] forma invalida: '");
        Serial.print(campoForma);
        Serial.println("'. Tiene que ser un solo caracter: S, H o C");
        return;
    }

    const char color = toupper(campoColor[0]);
    const char shape = toupper(campoForma[0]);

    if (color != 'R' && color != 'G' && color != 'B')
    {
        Serial.print("[SERIAL] color invalido: '");
        Serial.print(campoColor);
        Serial.println("'. Validos: R (rojo), G (verde), B (azul)");
        return;
    }

    if (shape != 'S' && shape != 'H' && shape != 'C')
    {
        Serial.print("[SERIAL] forma invalida: '");
        Serial.print(campoForma);
        Serial.println("'. Validas: S (cuadrado), H (hexagono), C (circulo)");
        return;
    }

    if (y < BELT_MIN_Y || y > BELT_MAX_Y)
    {
        Serial.print("[SERIAL] Y=");
        Serial.print(y);
        Serial.print(" fuera de la cinta (");
        Serial.print(BELT_MIN_Y);
        Serial.print(" a ");
        Serial.print(BELT_MAX_Y);
        Serial.println(" cm)");
        return;
    }

    Piece p;
    p.y = y;
    p.color = color;
    p.shape = shape;
    // Instante en que LLEGO el mensaje, no en que la pieza cruzo la
    // linea: el desfasaje entre los dos se corrige en
    // planificarPieza() con VISION_LATENCY_S.
    p.detectedAt_ms = millis();

    if (!queuePush(p))
    {
        Serial.println("[COLA] llena, se descarta la pieza mas nueva");
        piezasDescartadas++;
        return;
    }

    // Se cuenta al ENTRAR a la cola, no al detectarla la vision: son las
    // piezas que el robot efectivamente tuvo la oportunidad de agarrar, que
    // es contra lo que tiene sentido medir cuantas termino depositando.
    piezasDetectadas++;

    Serial.print("[PIEZA] Y=");
    Serial.print(p.y);
    Serial.print(" color=");
    Serial.print(p.color);
    Serial.print(" forma=");
    Serial.print(p.shape);
    Serial.print("  en cola: ");
    Serial.println(queueCount);
}

// ============================================================
//  COMANDOS DE TELEMETRIA Y PARAMETROS
// ============================================================
//  Los dos devuelven true si consumieron el comando (aunque haya sido para
//  contestar un error): un comando que empieza con 'V' o con 'P' no puede
//  ser ninguna otra cosa, asi que dejarlo seguir al resto del parser solo
//  lograria un segundo mensaje de error mas confuso que el primero.

bool Robot::procesarComandoTelemetria(const char *cmd)
{
    if (toupper(cmd[0]) != 'V' || strchr(cmd, ',') != NULL)
    {
        return false;
    }

    if (strlen(cmd) == 2)
    {
        switch (cmd[1])
        {
            case '1':
                telemetria.setActiva(true);
                return true;

            case '0':
                telemetria.setActiva(false);
                return true;

            case '?':
                // Una foto de [E] y [H] sin encender el stream: es lo que
                // pide la interfaz al conectarse, para pintar la pantalla
                // antes de decidir nada.
                telemetria.pedirFoto();
                return true;

            default:
                break;
        }
    }

    Serial.print("[SERIAL] comando de telemetria invalido: '");
    Serial.print(cmd);
    Serial.println("'. Validos: 'V1' (on), 'V0' (off), 'V?' (una foto)");

    return true;
}

// ------------------------------------------------------------------
void Robot::aplicarParametro(const char *nombre, float valor)
{
    const TablaParametros::Resultado r = params.fijar(nombre, valor, hayPiezaEnMano());

    // La respuesta sale SIEMPRE, salga bien o mal, y con el mismo formato:
    // es lo que mantiene sincronizada a la interfaz sin que tenga que
    // volver a pedir la tabla entera despues de cada cambio.
    Serial.print("[P] set n=");
    Serial.print(nombre);

    if (r == TablaParametros::FIJADO)
    {
        Serial.print(" v=");
        Serial.print(params.leer(nombre), 4);
        Serial.println(" ok");
        return;
    }

    Serial.print(" v=");
    Serial.print(valor, 4);

    switch (r)
    {
        case TablaParametros::DESCONOCIDO:
            Serial.println(" err=desconocido");
            break;

        case TablaParametros::FUERA_RANGO:
            Serial.println(" err=rango");
            break;

        default:
            // Cambiar la altura de un tacho con la pieza ya en el aire la
            // mandaria a un lugar distinto del que se planifico.
            Serial.println(" err=bloqueado");
            break;
    }
}

// ------------------------------------------------------------------
bool Robot::procesarComandoParametro(const char *cmd)
{
    if (toupper(cmd[0]) != 'P')
    {
        return false;
    }

    const char *igual = strchr(cmd, '=');

    if (igual == NULL)
    {
        if (strlen(cmd) == 2 && cmd[1] == '?')
        {
            params.volcar();
            return true;
        }

        if (strlen(cmd) == 2 && cmd[1] == '*')
        {
            params.guardar();
            return true;
        }

        if (strlen(cmd) == 2 && cmd[1] == '0')
        {
            params.restaurarDeFabrica();
            return true;
        }

        Serial.print("[SERIAL] comando de parametros invalido: '");
        Serial.print(cmd);
        Serial.println("'. Validos: 'P?', 'P*', 'P0' o 'Pnombre=valor'");
        return true;
    }

    // El nombre se copia en vez de partir la linea en el lugar: el buffer
    // de comandos se reutiliza y prefiero no dejarlo modificado a mitad de
    // camino por si mas adelante alguien agrega algo despues de esto.
    char nombre[16];

    const size_t largo = (size_t)(igual - (cmd + 1));

    if (largo == 0 || largo >= sizeof(nombre))
    {
        Serial.println("[SERIAL] nombre de parametro vacio o demasiado largo");
        return true;
    }

    memcpy(nombre, cmd + 1, largo);
    nombre[largo] = '\0';

    char       *fin   = NULL;
    const float valor = strtof(igual + 1, &fin);

    // Se exige que el numero se consuma ENTERO. Con atof() un "0.18cm"
    // pasaria como 0.18 y un "hola" como 0.0, que en un parametro de
    // altura significa clavar la ventosa contra la cinta.
    if (fin == igual + 1 || *fin != '\0')
    {
        Serial.print("[SERIAL] valor invalido para '");
        Serial.print(nombre);
        Serial.print("': '");
        Serial.print(igual + 1);
        Serial.println("'. Se espera un numero. Ej: Pvis_lat=0.18");
        return true;
    }

    aplicarParametro(nombre, valor);

    return true;
}

const char *Robot::nombreModo(SortMode m) const
{
    switch (m)
    {
        case SORT_BY_COLOR:  return "por COLOR";
        case SORT_BY_SHAPE:  return "por FORMA";
        case SORT_ALFAJORES: return "ALFAJORES";
        default:             return "?";
    }
}

void Robot::aplicarModoPendiente()
{
    if (!sortModePending)
    {
        return;
    }

    sortMode = pendingSortMode;
    sortModePending = false;

    Serial.print("[MODO] aplicado -> ");
    Serial.println(nombreModo(sortMode));

    if (esAlfajores(sortMode))
    {
        // La caja arranca SIEMPRE vacia al entrar al modo. El mapa de celdas
        // que hubiera quedado de una tanda anterior no dice nada de la caja
        // que hay puesta ahora: entre una y otra alguien la retiro y puso
        // otra, y creer que ya hay alfajores puestos hace que el robot
        // saltee celdas que estan vacias.
        reiniciarCaja(false);
        imprimirCaja();
    }
}

// ============================================================
//  CAMBIO DE MODO Y CONFIRMACION DE LA TAPA
// ============================================================

void Robot::pedirModo(SortMode nuevo, char letra)
{
    if (esAlfajores(nuevo) && !boxLayoutValido)
    {
        Serial.println("[MODO] la disposicion de la caja es invalida: no se entra a ALFAJORES");
        return;
    }

    if (nuevo == modoObjetivo())
    {
        Serial.print("[MODO] ya esta ");
        Serial.println(nombreModo(nuevo));

        // Pedir el modo en el que ya se esta sirve de consulta: es la unica
        // forma de ver como viene la caja sin borrarla con 'N'.
        if (esAlfajores(nuevo))
        {
            imprimirCaja();
        }
        return;
    }

    // Un cambio entre COLOR y FORMA no toca nada fisico y se aplica derecho.
    // Entrar o salir de ALFAJORES, en cambio, significa poner o sacar la
    // tapa: hasta que alguien confirme que ya esta asi, no se cambia nada.
    if (esAlfajores(nuevo) == esAlfajores(modoObjetivo()))
    {
        aplicarModo(nuevo);
        return;
    }

    const bool confirmando = esperandoConfirmacion &&
                             modoAConfirmar == nuevo &&
                             (uint32_t)(millis() - confirmacionPedida_ms) < (uint32_t)CONFIRMACION_TAPA_MS;

    if (!confirmando)
    {
        modoAConfirmar        = nuevo;
        esperandoConfirmacion = true;
        confirmacionPedida_ms = millis();

        if (esAlfajores(nuevo))
        {
            Serial.println("[TAPA] para entrar al modo ALFAJORES hay que PONER la tapa sobre los tachos.");
        }
        else
        {
            Serial.println("[TAPA] para salir del modo ALFAJORES hay que SACAR la tapa de los tachos.");
        }

        Serial.print("[TAPA] si ya esta asi, confirmar con '");
        Serial.print(letra);
        Serial.print("' otra vez dentro de ");
        Serial.print((uint32_t)(CONFIRMACION_TAPA_MS / 1000));
        Serial.print(" s. Queda: ");
        Serial.println(nombreModo(nuevo));
        return;
    }

    esperandoConfirmacion = false;
    aplicarModo(nuevo);
}

void Robot::aplicarModo(SortMode nuevo)
{
    // Si el robot tiene una pieza en la mano, el cambio queda pendiente
    // hasta que la suelte: cambiarle el destino a una pieza en vuelo la
    // mandaria al lugar equivocado.
    if (hayPiezaEnMano())
    {
        pendingSortMode = nuevo;
        sortModePending = true;

        Serial.print("[MODO] pendiente -> ");
        Serial.println(nombreModo(nuevo));
        return;
    }

    sortMode = nuevo;
    sortModePending = false;

    Serial.print("[MODO] ");
    Serial.println(nombreModo(sortMode));

    if (esAlfajores(sortMode))
    {
        // La caja arranca SIEMPRE vacia al entrar al modo. El mapa de celdas
        // que hubiera quedado de una tanda anterior no dice nada de la caja
        // que hay puesta ahora: entre una y otra alguien la retiro y puso
        // otra, y creer que ya hay alfajores puestos hace que el robot
        // saltee celdas que estan vacias.
        reiniciarCaja(false);
        imprimirCaja();
    }
}

void Robot::vencerConfirmacion()
{
    if (!esperandoConfirmacion ||
        (uint32_t)(millis() - confirmacionPedida_ms) < (uint32_t)CONFIRMACION_TAPA_MS)
    {
        return;
    }

    esperandoConfirmacion = false;

    Serial.print("[TAPA] sin confirmacion: el modo sigue ");
    Serial.println(nombreModo(modoObjetivo()));
}

// ============================================================
//  CAJA DE ALFAJORES
// ============================================================

bool Robot::layoutValido(const char *layout)
{
    uint8_t porColor[3] = {0, 0, 0}; // R, G, B

    for (uint8_t i = 0; i < BOX_CELLS; i++)
    {
        switch (layout[i])
        {
            case 'R': porColor[0]++; break;
            case 'G': porColor[1]++; break;
            case 'B': porColor[2]++; break;
            default:  return false;
        }
    }

    // Con mas de 3 de un color la caja no se puede terminar nunca (4 rojos y
    // 2 verdes, o 6 verdes, son los ejemplos tipicos): el robot se quedaria
    // esperando indefinidamente una pieza que no tiene donde ir.
    for (uint8_t i = 0; i < 3; i++)
    {
        if (porColor[i] > BOX_MAX_POR_COLOR)
        {
            return false;
        }
    }

    return true;
}

void Robot::aplicarLayout(const char *layout)
{
    const uint8_t habia = celdasLlenas();

    memcpy(boxLayout, layout, BOX_CELLS);
    boxLayoutValido = true;

    // La disposicion nueva no dice nada de los alfajores que ya estaban: el
    // mapa arranca de cero y la caja tiene que estar vacia de verdad.
    reiniciarCaja(false);

    Serial.println("[CAJA] disposicion nueva");
    imprimirCaja();

    if (habia > 0)
    {
        Serial.print("[CAJA] ojo: habia ");
        Serial.print(habia);
        Serial.println(" alfajores puestos. Sacarlos antes de seguir.");
    }
}

uint8_t Robot::celdasLlenas() const
{
    uint8_t n = 0;

    for (uint8_t i = 0; i < BOX_CELLS; i++)
    {
        if (boxFilled[i])
        {
            n++;
        }
    }

    return n;
}

bool Robot::piezaSirveParaCaja(const Piece &p, uint8_t &celda) const
{
    celda = CELDA_NINGUNA;

    if (boxComplete || p.shape != 'C')
    {
        return false;
    }

    // De la celda 6 hacia la 1: entre varias del color que llega, gana
    // siempre la de numero mas grande.
    for (int8_t c = BOX_CELLS - 1; c >= 0; c--)
    {
        if (!boxFilled[c] && boxLayout[c] == p.color)
        {
            celda = (uint8_t)c;
            return true;
        }
    }

    return false;
}

void Robot::marcarCeldaLlena(uint8_t celda)
{
    boxFilled[celda] = true;

    Serial.print("[CAJA] celda ");
    Serial.print(celda + 1);
    Serial.print(" (");
    Serial.print(boxLayout[celda]);
    Serial.print(") lista: ");
    Serial.print(celdasLlenas());
    Serial.print(" de ");
    Serial.println(BOX_CELLS);

    if (celdasLlenas() < BOX_CELLS)
    {
        return;
    }

    boxComplete = true;

    imprimirCaja();
    Serial.println("[CAJA] COMPLETA. Desde aca se deja pasar todo.");
    Serial.println("[CAJA] Sacar la caja, poner una vacia y mandar 'N'.");
}

void Robot::reiniciarCaja(bool avisar)
{
    for (uint8_t i = 0; i < BOX_CELLS; i++)
    {
        boxFilled[i] = false;
    }

    boxComplete = false;

    if (avisar)
    {
        Serial.println("[CAJA] caja nueva: las 6 celdas quedan libres");
        imprimirCaja();
    }
}

void Robot::imprimirCaja() const
{
    // Dos lineas con la forma fisica de la caja: el color que pide cada
    // celda y si ya esta puesta. La primera fila es la mas cercana a la
    // cinta.
    for (uint8_t fila = 0; fila < 2; fila++)
    {
        Serial.print("[CAJA] ");

        for (uint8_t col = 0; col < 3; col++)
        {
            const uint8_t i = fila * 3 + col;

            Serial.print(i + 1);
            Serial.print(':');
            Serial.print(boxLayout[i]);
            Serial.print(boxFilled[i] ? "[x] " : "[ ] ");
        }

        Serial.println(fila == 0 ? "  (Y=-6.8, fila de adelante)"
                                 : "  (Y=-12.8, fila del fondo)");
    }

    Serial.print("[CAJA] ");
    Serial.print(celdasLlenas());
    Serial.print(" de ");
    Serial.print(BOX_CELLS);
    Serial.println(boxComplete ? " -- COMPLETA, mandar 'N' para empezar otra" : "");
}

// ============================================================
//  COLA DE PIEZAS
// ============================================================

bool Robot::queuePush(const Piece &p)
{
    if (queueCount >= QUEUE_CAPACITY)
    {
        return false;
    }

    pieceQueue[(queueHead + queueCount) % QUEUE_CAPACITY] = p;
    queueCount++;
    return true;
}

bool Robot::queuePop(Piece &out)
{
    if (queueCount == 0)
    {
        return false;
    }

    out = pieceQueue[queueHead];
    queueHead = (queueHead + 1) % QUEUE_CAPACITY;
    queueCount--;
    return true;
}

// ============================================================
//  HOMING
// ============================================================

void Robot::startHoming(bool conservarContexto)
{
    axis1Homed = false;
    axis2Homed = false;
    axis3Homed = false;
    homed = false;

    state = HOMING;

    // Mientras dura el homing no se supervisa nada: los ejes van contra los
    // finales de carrera a proposito, y ademas setPosition() les redefine la
    // cuenta de pasos de golpe, asi que cualquier comparacion contra el
    // encoder en el medio no significa nada. Se vuelve a armar al final,
    // cuando se calibra.
    guard.desarmar();

    homingStart_ms = millis();

    motor1.setPosition(999999);  // fuerza que nunca este en home al principio
    motor2.setPosition(999999);
    motor3.setPosition(999999);

    motor1.moveContinuous(false);
    motor2.moveContinuous(false);
    motor3.moveContinuous(false);

    motor1.setSpeed(1000);
    motor2.setSpeed(1000);
    motor3.setSpeed(1000);

    // Las piezas encoladas traen timestamps de antes del corte: sus
    // posiciones ya no son confiables (la cinta estuvo parada mientras
    // tanto), asi que se descartan todas.
    //
    // La excepcion es la recalibracion despues de una colision: ahi la cinta
    // NUNCA se detuvo, por lo que los timestamps siguen siendo validos y las
    // piezas siguieron avanzando de forma perfectamente conocida. Se conserva
    // la cola entera y al terminar el homing se vuelve a mirar cual sigue
    // siendo alcanzable: las que se pasaron de largo durante los 3 segundos
    // de pausa mas el homing las descarta planificarPieza() sola, una por
    // una, avisando por Serial.
    if (!conservarContexto)
    {
        queueHead = 0;
        queueCount = 0;

        // Un cambio de modo que habia quedado pendiente pertenecia al ciclo
        // que se corto: no se arrastra al ciclo nuevo. El modo ACTIVO si se
        // mantiene (el operador ya lo eligio y no lo dio de baja).
        sortModePending = false;

        colisionesSeguidas = 0;
    }

    // Se suelta lo que hubiera quedado agarrado antes del corte, para
    // arrancar el ciclo con el gripper vacio y en estado conocido.
    pneumatics.release();
    pumpOn = false;

    // La celda que tenia reservada la pieza perdida vuelve a quedar libre.
    // El MAPA de la caja, en cambio, no se toca ni con el reset manual: los
    // alfajores que ya estan puestos siguen fisicamente ahi. Solo 'N' lo
    // borra, que es cuando alguien saco la caja y puso otra.
    currentCell = CELDA_NINGUNA;

    // Un cambio de modo a medio confirmar pertenecia al ciclo que se corto.
    esperandoConfirmacion = false;

    moveIssued = false;
    replanCount = 0;
    homingSettleStart_ms = 0;
}

void Robot::updateHoming()
{
    // Un eje que no llega nunca a su final de carrera es un eje trabado
    // (tipicamente: el obstaculo que provoco la colision sigue ahi). Seguir
    // empujando contra el no arregla nada y castiga la mecanica.
    if (!(axis1Homed && axis2Homed && axis3Homed) &&
        (uint32_t)(millis() - homingStart_ms) > (uint32_t)HOMING_TIMEOUT_MS)
    {
        motor1.stop();
        motor2.stop();
        motor3.stop();

        // Aca si se para la cinta: el robot no puede recuperarse solo, y
        // dejarla andando solo acumula piezas sin clasificar.
        conveyor.stop();

        registrarFallo(FALLO_HOMING, 0);

        Serial.print("[HOMING] no encontro los finales de carrera en ");
        Serial.print((uint32_t)(HOMING_TIMEOUT_MS / 1000));
        Serial.println(" s: hay un eje trabado. Revisar y mandar 'R'.");

        state = ERROR;
        return;
    }

    // Motor 1

    if (!axis1Homed)
    {
        if (endstops.readMotor1())
        {
            motor1.stop();
            motor1.setPosition(angleToSteps(HOME_ANGLE_M1));
            axis1Homed = true;
        }
    }

    // Motor 2

    if (!axis2Homed)
    {
        if (endstops.readMotor2())
        {
            motor2.stop();
            motor2.setPosition(angleToSteps(HOME_ANGLE_M2));
            axis2Homed = true;
        }
    }

    // Motor 3

    if (!axis3Homed)
    {
        if (endstops.readMotor3())
        {
            motor3.stop();
            motor3.setPosition(angleToSteps(HOME_ANGLE_M3));
            axis3Homed = true;
        }
    }

    // ¿Todos llegaron?

    if (axis1Homed && axis2Homed && axis3Homed)
    {
        if (homingSettleStart_ms == 0)
        {
            // Arranca la ventana de acumulación de media móvil por canal.
            encoders.iniciarAsentamientoHoming();

            // La misma ventana le sirve a la supervision para promediar SU
            // referencia: son 2,5 s con los 3 ejes frenados contra el
            // endstop, la unica oportunidad de tomar un cero de encoder sin
            // el ruido de +-1 grado encima. Si la referencia saliera de una
            // lectura puntual, ese error se sumaria a todas las
            // comparaciones posteriores.
            // Se arma siempre, incluso con las paradas apagadas: en ese caso
            // el guard queda observando (mide y avisa, pero no frena).
            guard.iniciarReferencia(motor1.getPosition(),
                                    motor2.getPosition(),
                                    motor3.getPosition());

            homingSettleStart_ms = millis();
            return;
        }

        if (millis() - homingSettleStart_ms < (uint32_t)HOMING_SETTLE_WAIT_MS)
        {
            return; // seguimos acumulando muestras, sin bloquear el resto del sistema
        }

        homingSettleStart_ms = 0;

        // Calibra usando el PROMEDIO de todas las muestras de la ventana de
        // espera (no una lectura puntual).
        encoders.calibrarHoming(HOME_ANGLE_M1, HOME_ANGLE_M2, HOME_ANGLE_M3);

        // Recien aca queda armada la supervision: con la referencia ya
        // promediada y los pasos de los 3 ejes en su valor de home.
        guard.fijarReferencia();

        homed = true;

        // La cinta arranca recien con el robot ya calibrado: antes de eso
        // no tendria sentido aceptar piezas.
        conveyor.begin();
        conveyor.setSpeedPercent(CONVEYOR_PWM);

        Serial.println("[HOMING] OK. Robot listo.");
        Serial.print("[MODO] ");
        Serial.println(nombreModo(sortMode));

        if (esAlfajores(sortMode))
        {
            imprimirCaja();
        }

        Serial.print("[GUARD] ");
        Serial.print(guard.nombreEstado());
        Serial.print("  umbral=");
        Serial.print(guard.umbral(), 1);
        Serial.print(" grados  confirmacion=");
        Serial.print(guard.confirmacion());
        Serial.println(" ms");

        moveIssued = false;
        state = GO_HOME_IDLE;
    }
}

bool Robot::homingFinished() const
{
    return homed;
}

Robot::RobotState Robot::getState() const
{
    return state;
}

// ============================================================
//  MOVIMIENTO
// ============================================================

bool Robot::goToPositionIK(float x, float y, float z, const Motors::MotionLimits &limits)
{
    DeltaKinematics::DeltaAngles pose = DeltaKinematics::solveIK(x, y, z);
    if (!pose.success)
    {
        return false;
    }
    Motors::moveSynchronized(motor1, motor2, motor3, pose.steps1, pose.steps2, pose.steps3, limits);
    return true;
}

// ============================================================
//  ESPERA / VUELTA A HOME
// ============================================================

void Robot::updateGoHomeIdle()
{
    // Si aparece una pieza mientras vuelve a home, se sale a buscarla --
    // pero SOLO con el brazo ya frenado (el enPosicion()).
    //
    // POR QUE: Stepper::moveTo() fija el pin de direccion y reinicia la
    // rampa sin mirar si el eje ya venia andando (ver el comentario en
    // Stepper.cpp). Si el destino nuevo queda del otro lado, el driver
    // invierte el sentido con el rotor girando, cosa que ningun paso a paso
    // puede seguir. Medido en el banco de pruebas: cada interrupcion del
    // regreso invertia un eje a hasta 36.000 pasos/s.
    //
    // HONESTIDAD SOBRE LO QUE ESTO ARREGLA: se sospechaba que era la causa
    // de la descalibracion que aparecia sola cada tantas piezas, y NO LO
    // ES. Probado en el robot, con 9 de cada 10 ciclos interrumpidos a
    // proposito (disparando la pieza justo cuando arranca el regreso):
    //
    //     sin esta espera   deriva max por eje  19,1 / 21,0 / 19,1
    //     con esta espera   deriva max por eje  22,1 / 19,7 / 16,5
    //
    // O sea lo mismo. Se deja igual porque la inversion en movimiento es
    // real y no cuesta nada evitarla, pero la descalibracion hay que
    // buscarla en otro lado.
    //
    // Esperar cuesta muy poco: el regreso a home dura menos de medio
    // segundo, y la pieza recien detectada necesita ~1,7 s para llegar al
    // area de agarre, asi que el brazo la va a esperar igual parado en el
    // punto de aproximacion. Medido: 18 de 18 piezas agarradas, ninguna
    // perdida por esperar.
    //
    // De paso el plan sale mejor: ConveyorIntercept estima el tiempo de
    // viaje suponiendo que el brazo ARRANCA DETENIDO, cosa que solo es
    // cierta si efectivamente lo esta.
    //
    // Ojo: en la primera vuelta de este estado el brazo todavia esta parado
    // sobre el tacho (el movimiento a home ni se lanzo), asi que ahi la
    // pieza se toma en el acto y no se pierde nada de productividad.
    if (queueCount > 0 && enPosicion() && iniciarSiguientePieza())
    {
        return;
    }

    if (!moveIssued)
    {
        // Home = brazos horizontales (0 grados en los 3 ejes).
        Motors::moveSynchronized(motor1, motor2, motor3, 0, 0, 0, Motors::FAST_LIMITS);
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        state = WAIT_PIECE;
    }
}

void Robot::updateWaitPiece()
{
    if (queueCount > 0)
    {
        iniciarSiguientePieza();
    }
}

// ============================================================
//  PLANIFICACION DE LA MANIOBRA
// ============================================================

bool Robot::planificarPieza(const Piece &p)
{
    ConveyorIntercept::BeltConfig belt;
    belt.velocityX = BELT_VELOCITY_CMS;
    belt.detectionLineX = DETECTION_LINE_X;

    ConveyorIntercept::PickGeometry geom;
    geom.grabZ = GRAB_Z;
    geom.approachDX = APPROACH_DX;
    geom.approachDZ = APPROACH_DZ;
    geom.pressDZ = PRESS_DZ;

    // En modo alfajores el brazo despega mas alto: desde la cinta tiene que
    // cruzar por encima de la pared de la caja y de los alfajores ya
    // colocados, y el camino entre dos puntos se curva hacia abajo (ver
    // BOX_TRANSIT_DZ). Con los 3 cm de siempre, ese hundimiento mete la
    // pieza por debajo del piso de la caja antes de llegar a la celda.
    //
    // ConveyorIntercept valida el punto de despegue con la cinematica, asi
    // que si algun agarre no admitiera la subida mas alta, esa pieza queda
    // marcada como no alcanzable y se deja pasar en vez de forzarla.
    geom.liftDZ = esAlfajores(sortMode) ? BOX_TRANSIT_DZ : LIFT_DZ;

    geom.workAreaMinX = WORK_AREA_MIN_X;
    geom.workAreaMaxX = WORK_AREA_MAX_X;

    // Al tiempo que paso desde que llego el mensaje se le suma la
    // latencia de la vision: cuando ese mensaje llego, la pieza ya
    // no estaba sobre la linea de deteccion, ya habia avanzado.
    const float tSinceDetection = VISION_LATENCY_S +
                                  (millis() - p.detectedAt_ms) / 1000.0f;

    ConveyorIntercept::InterceptResult r = ConveyorIntercept::solve(
        p.y, tSinceDetection, belt, geom,
        motor1.getPosition(), motor2.getPosition(), motor3.getPosition(),
        Motors::VEL_MAX, Motors::ACC_RAPIDA, Motors::ACC_SUAVE);

    if (!r.reachable)
    {
        return false;
    }

    approachX    = r.approachX;    approachY    = r.approachY;    approachZ    = r.approachZ;
    descendEndX  = r.descendEndX;  descendEndY  = r.descendEndY;  descendEndZ  = r.descendEndZ;
    liftX        = r.liftX;        liftY        = r.liftY;        liftZ        = r.liftZ;

    // Punto de contacto, solo para el log de diagnostico.
    lastGrabX = r.grabX;
    lastContactSpeedX = r.contactSpeedX;

    descendStart_ms = millis() + (uint32_t)(r.descendStartDelay * 1000.0f);

    return true;
}

bool Robot::iniciarSiguientePieza()
{
    // Aca el robot no tiene ninguna pieza en la mano: es el momento seguro
    // para aplicar un cambio de modo pendiente, ANTES de decidir el tacho
    // de la proxima pieza.
    aplicarModoPendiente();

    Piece p;

    // ORDEN DE AGARRE: el de deteccion, siempre. La cola es FIFO, asi que
    // queuePop() devuelve la pieza mas vieja, que es la MAS ADELANTADA sobre
    // la cinta (todas avanzan a la misma velocidad). Esa es la que menos
    // tiempo queda antes de pasarse de largo, asi que es la que hay que
    // atender primero.
    //
    // Lo unico que puede saltear una pieza es que ya no se llegue a agarrar
    // (planificarPieza) o que no sirva para la caja (piezaSirveParaCaja). Ni
    // el color, ni la forma, ni la celda de destino adelantan a una pieza
    // sobre otra: el modo alfajores decide DONDE va cada pieza, nunca en que
    // orden se van a buscar.
    while (queuePop(p))
    {
        // En modo alfajores el destino se decide ANTES de planificar el
        // agarre: la mayoria de las piezas no se agarran (cuadrados,
        // hexagonos, y los circulos de un color que ya no falta) y no tiene
        // sentido calcularles la intercepcion.
        uint8_t celda = CELDA_NINGUNA;

        if (esAlfajores(sortMode) && !piezaSirveParaCaja(p, celda))
        {
            // Con la caja ya completa no se avisa pieza por pieza: pasarian
            // todas y el aviso de caja llena ya explico lo que va a pasar.
            if (!boxComplete)
            {
                Serial.print("[CAJA] no hace falta (color=");
                Serial.print(p.color);
                Serial.print(" forma=");
                Serial.print(p.shape);
                Serial.println("), se deja pasar");
            }
            continue;
        }

        if (!planificarPieza(p))
        {
            Serial.print("[PIEZA] no alcanzable (Y=");
            Serial.print(p.y);
            Serial.println("), se deja pasar");

            // Esta SI es una perdida: la pieza estaba para agarrar y no se
            // llego. Las que se dejan pasar en modo caja por no hacer falta
            // no se cuentan aca, que esas son el modo funcionando bien.
            piezasDescartadas++;

            continue; // se prueba con la siguiente de la cola
        }

        currentPiece = p;
        currentCell = celda;
        currentBin = binIndexFor(p);
        replanCount = 0;
        moveIssued = false;
        state = PICK_APPROACH;

        Serial.print("[PIEZA] contacto en X=");
        Serial.print(lastGrabX);
        Serial.print(" Y=");
        Serial.print(p.y);
        Serial.print(" a ");
        Serial.print(lastContactSpeedX);
        Serial.print(" cm/s (cinta ");
        Serial.print(BELT_VELOCITY_CMS);

        if (currentCell != CELDA_NINGUNA)
        {
            Serial.print(") -> celda ");
            Serial.println(currentCell + 1);
        }
        else
        {
            Serial.print(") -> tacho ");
            Serial.println(currentBin + 1);
        }

        return true;
    }

    return false;
}

uint8_t Robot::binIndexFor(const Piece &p) const
{
    if (sortMode == SORT_BY_COLOR)
    {
        switch (p.color)
        {
            case 'R': return 0;
            case 'G': return 1;
            case 'B': return 2;
            default:  return 0;
        }
    }

    switch (p.shape)
    {
        case 'S': return 0;
        case 'H': return 1;
        case 'C': return 2;
        default:  return 0;
    }
}

// ============================================================
//  TRAMO 1: aproximacion (aceleracion maxima) + espera del instante justo
// ============================================================

void Robot::updatePickApproach()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(approachX, approachY, approachZ, Motors::FAST_LIMITS))
        {
            // No deberia pasar: ConveyorIntercept ya valido este punto.
            Serial.println("[PIEZA] punto de aproximacion invalido, se descarta");
            currentCell = CELDA_NINGUNA; // la celda vuelve a quedar libre
            moveIssued = false;
            state = GO_HOME_IDLE;
            return;
        }
        moveIssued = true;
    }

    if (!enPosicion())
    {
        return;
    }

    // Se espera al instante calculado para lanzar la bajada. La resta con
    // signo aguanta el desborde de millis().
    const int32_t atraso_ms = (int32_t)(millis() - descendStart_ms);

    // La bomba se prende PUMP_LEAD_MS antes de la bajada, no al llegar aca.
    //
    // Antes se prendia al llegar al punto de aproximacion. Con la pieza
    // viniendo atras del brazo eso es casi lo mismo (se baja enseguida),
    // pero con una sola pieza en la cola el brazo llega a la aproximacion y
    // se queda esperandola: ahi la bomba quedaba soplando todo el viaje de
    // la pieza por la cinta, que puede ser mas de un segundo por pieza.
    //
    // El adelanto existe porque el vacio SI necesita un rato para formarse:
    // prenderla justo al arrancar la bajada la haria tocar la pieza con la
    // ventosa a medio hacer. Con el adelanto en 300 ms -- la misma ventana
    // en que se silencia el guard por el hundimiento del riel de los
    // encoders, asi que la conmutacion sigue quedando tapada -- el
    // comportamiento no cambia para las piezas que se agarran enseguida, y
    // solo cambia para las que hay que esperar.
    if (!pumpOn && atraso_ms >= -(int32_t)PUMP_LEAD_MS)
    {
        pneumatics.grab();
        pumpOn = true;

        // El arranque de la bomba le pega a la alimentacion de los encoders
        // (ver BLANQUEO_NEUMATICA_MS): se suspende la deteccion mientras
        // dura, y el guard informa cuanto se corrio cada eje.
        guard.silenciar((uint32_t)BLANQUEO_NEUMATICA_MS);
    }

    if (atraso_ms < 0)
    {
        return; // todavia no es momento: la pieza no llego
    }

    if (atraso_ms > (int32_t)PICK_LATE_TOLERANCE_MS)
    {
        // Se perdio la ventana (el tramo 1 tardo mas de lo estimado). Se
        // replanifica con la pieza donde este ahora, si todavia da.
        if (replanCount < (uint8_t)MAX_REPLAN_ATTEMPTS && planificarPieza(currentPiece))
        {
            replanCount++;
            moveIssued = false;
            Serial.println("[PIEZA] replanificando agarre");
            return;
        }

        Serial.println("[PIEZA] se perdio la ventana de agarre, se deja pasar");
        pneumatics.release();
        pumpOn = false;
        currentCell = CELDA_NINGUNA; // la celda vuelve a quedar libre
        moveIssued = false;
        state = GO_HOME_IDLE;
        return;
    }

    moveIssued = false;
    state = PICK_DESCEND;
}

// ============================================================
//  TRAMO 2: bajada a la pieza (aceleracion minima, a favor de la cinta)
//  El destino SOBREPASA a la pieza, asi el contacto ocurre a mitad del
//  movimiento y a la misma velocidad que la cinta (ver ConveyorIntercept.h).
// ============================================================

void Robot::updatePickDescend()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(descendEndX, descendEndY, descendEndZ, Motors::SOFT_LIMITS))
        {
            Serial.println("[PIEZA] punto de agarre invalido, se descarta");
            pneumatics.release();
            pumpOn = false;
            currentCell = CELDA_NINGUNA; // la celda vuelve a quedar libre
            moveIssued = false;
            state = GO_HOME_IDLE;
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        state = PICK_LIFT;
    }
}

// ============================================================
//  TRAMO 3: despegue de la pieza de la cinta (aceleracion maxima)
// ============================================================

void Robot::updatePickLift()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(liftX, liftY, liftZ, Motors::FAST_LIMITS))
        {
            Serial.println("[PIEZA] punto de despegue invalido");
            currentCell = CELDA_NINGUNA; // la celda vuelve a quedar libre
            moveIssued = false;
            state = GO_HOME_IDLE;
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;

        // La celda reservada es lo que decide el destino, no el modo: si el
        // modo cambiara justo ahora, esta pieza ya salio con una celda
        // asignada y tiene que terminar ahi.
        state = (currentCell != CELDA_NINGUNA) ? BOX_TRANSIT : GO_BIN;
    }
}

// ============================================================
//  TRAMO 4: traslado al tacho (aceleracion maxima)
// ============================================================

void Robot::updateGoBin()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(BIN_X[currentBin], BIN_Y, BIN_Z, Motors::FAST_LIMITS))
        {
            Serial.println("[TACHO] posicion invalida");
            emergencyStop();
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        binSettleStart_ms = millis();
        state = BIN_SETTLE;
    }
}

// ============================================================
//  ASENTAMIENTO Y SOLTADO
// ============================================================

void Robot::updateBinSettle()
{
    // Quieto sobre el tacho: si se soltara apenas frena, la pieza saldria
    // disparada con la inercia del brazo en vez de caer vertical.
    if (millis() - binSettleStart_ms < BIN_SETTLE_MS)
    {
        return;
    }

    pneumatics.release();
    pumpOn = false;
    guard.silenciar((uint32_t)BLANQUEO_NEUMATICA_MS); // apagarla tambien mueve el riel

    releaseStart_ms = millis();
    state = RELEASE_WAIT;
}

void Robot::updateReleaseWait()
{
    // Espera a que la pieza se despegue sola del gripper (ver
    // RELEASE_DETACH_MS: es un parche hasta montar la electrovalvula). En la
    // caja la espera es mas larga porque el alfajor queda apoyado y la
    // ventosa sigue tocandolo (ver BOX_RELEASE_DETACH_MS).
    const uint32_t espera = (currentCell != CELDA_NINGUNA) ? BOX_RELEASE_DETACH_MS
                                                           : RELEASE_DETACH_MS;

    if (millis() - releaseStart_ms < espera)
    {
        return;
    }

    // Pieza entregada: el ciclo cerro bien, asi que la racha de colisiones
    // seguidas se corta aca (lo que hubiera chocado antes, ya se resolvio).
    colisionesSeguidas = 0;

    // Y recien aca se cuenta como depositada: soltada de verdad, en su
    // destino. Contarla al agarrarla habria inflado el numero con las que
    // se pierden a mitad de camino, que son justamente las que interesa
    // que aparezcan en la diferencia.
    contarDepositada(currentPiece);

    // En la caja el brazo esta METIDO ENTRE LAS PAREDES: antes de decidir
    // nada tiene que salir derecho para arriba. Ir desde adentro de la caja
    // a cualquier otro lado hunde el camino contra los alfajores puestos.
    if (currentCell != CELDA_NINGUNA)
    {
        // La celda pasa a llena, pero currentCell se conserva hasta haber
        // salido: es de donde el tramo de salida tiene que subir.
        marcarCeldaLlena(currentCell);

        moveIssued = false;
        state = BOX_LIFT;
        return;
    }

    // Recien ahora, con la pieza ya soltada, se puede cambiar de modo.
    aplicarModoPendiente();

    // Si hay otra pieza, sale a buscarla directo desde arriba del tacho.
    if (queueCount > 0 && iniciarSiguientePieza())
    {
        return;
    }

    moveIssued = false;
    state = GO_HOME_IDLE;
}

// ============================================================
//  CAJA DE ALFAJORES: los cuatro tramos hasta dejar el alfajor
// ============================================================
//
//  1. TRANSIT   accel MAX, hasta 6 cm sobre la celda. Es el tramo que cruza
//               por encima de la pared de la caja y de los alfajores ya
//               puestos (ver BOX_TRANSIT_DZ: con menos altura, la curva del
//               movimiento los baaria).
//  2. APPROACH  accel MAX, baja en vertical hasta 3 cm sobre la celda.
//  3. DESCEND   accel MIN, apoya el alfajor. Se suelta abajo, no antes.
//  4. LIFT      accel MAX, sale en vertical a la altura de cruce.
//
//  Los tramos 2 y 4 son verticales sobre la misma celda: ahi el camino no
//  se desvia (menos de 1,3 mm de deriva lateral medida en toda la bajada).
// ============================================================

void Robot::updateBoxTransit()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(BOX_X[currentCell] + BOX_DX, BOX_Y[currentCell] + BOX_DY,
                            BOX_Z + BOX_TRANSIT_DZ, Motors::FAST_LIMITS))
        {
            Serial.println("[CAJA] altura de cruce invalida");
            emergencyStop();
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        state = BOX_APPROACH;
    }
}

void Robot::updateBoxApproach()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(BOX_X[currentCell] + BOX_DX, BOX_Y[currentCell] + BOX_DY,
                            BOX_Z + BOX_APPROACH_DZ, Motors::FAST_LIMITS))
        {
            Serial.println("[CAJA] punto de aproximacion invalido");
            emergencyStop();
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        state = BOX_DESCEND;
    }
}

void Robot::updateBoxDescend()
{
    if (!moveIssued)
    {
        // Aceleracion minima, igual que al entrar a la pieza sobre la cinta:
        // el alfajor se APOYA en el piso de la caja, no se deja caer.
        //
        // A diferencia del agarre, aca NO se silencia la supervision: no hay
        // conmutacion de la bomba ni un contacto buscado a proposito (la
        // punta baja justo hasta donde la pieza queda apoyada), y en cambio
        // la caja es un obstaculo rigido. Si esta corrida de lugar o la tapa
        // no esta donde deberia, esto es exactamente lo que hay que detectar.
        if (!goToPositionIK(BOX_X[currentCell] + BOX_DX, BOX_Y[currentCell] + BOX_DY,
                            BOX_Z, Motors::SOFT_LIMITS))
        {
            Serial.println("[CAJA] punto de soltado invalido");
            emergencyStop();
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        binSettleStart_ms = millis();
        state = BIN_SETTLE; // suelta la pieza, igual que en el tacho
    }
}

void Robot::updateBoxLift()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(BOX_X[currentCell] + BOX_DX, BOX_Y[currentCell] + BOX_DY,
                            BOX_Z + BOX_TRANSIT_DZ, Motors::FAST_LIMITS))
        {
            Serial.println("[CAJA] salida invalida");
            emergencyStop();
            return;
        }
        moveIssued = true;
    }

    if (!enPosicion())
    {
        return;
    }

    moveIssued = false;

    // Recien aca la pieza queda del todo entregada y el brazo libre.
    currentCell = CELDA_NINGUNA;

    // Ya fuera de la caja: recien ahora se puede cambiar de modo o salir a
    // buscar la pieza siguiente.
    aplicarModoPendiente();

    if (queueCount > 0 && iniciarSiguientePieza())
    {
        return;
    }

    state = GO_HOME_IDLE;
}

// ============================================================
//  SUPERVISION POR ENCODERS (deteccion de colisiones)
// ============================================================
//
//  Los encoders NO cierran el lazo de posicion: el posicionamiento sigue
//  siendo a lazo abierto por micropasos, que da muy buen resultado. Lo que
//  cierran es un lazo de SEGURIDAD: miran si el brazo esta donde los pasos
//  dicen que deberia estar.
//
//  Mientras el robot anda bien, los pasos emitidos y el angulo medido se
//  mueven juntos y la diferencia queda chica (ruido del encoder, retardo de
//  sus filtros, atraso mecanico de la rampa). Cuando el brazo choca contra
//  algo, los pasos siguen saliendo pero el eje no gira: la diferencia se
//  abre rapido y no vuelve. Eso es lo que se detecta.
//
//  Dos condiciones, no una: la diferencia tiene que pasar el umbral Y
//  mantenerse. Una lectura suelta fuera de rango no frena nada.
//
//  Ver CollisionGuard.h para el detalle del calculo y de los parametros.
//
// ============================================================

void Robot::supervisarColision()
{
    // Quieto en home esperando piezas: la unica situacion en la que el guard
    // puede exigir un umbral chico (ver el chequeo en reposo en
    // CollisionGuard.h). WAIT_PIECE es exactamente eso -- se entra recien
    // cuando el movimiento a home termino -- asi que no hace falta mirar
    // nada mas.
    guard.setEnHome(state == WAIT_PIECE);

    // Se llama SIEMPRE, aunque las paradas esten apagadas: con 'G' el guard
    // pasa a modo observador (mide y avisa, no frena), asi la traza, los
    // picos y la ganancia medida siguen sirviendo mientras se calibra.

    // Un encoder que dejo de ser confiable no frena el robot (seria peor el
    // remedio: se pararia por no poder ver, no por haber chocado), pero si
    // queda registrado: mientras dure, ese eje esta sin vigilancia.
    uint8_t ejeCaido = 0;
    if (guard.consumirAvisoSensor(ejeCaido))
    {
        registrarFallo(FALLO_ENCODER, (uint8_t)(ejeCaido + 1));
        Serial.print("[GUARD] encoder del eje ");
        Serial.print(ejeCaido + 1);
        Serial.println(" no confiable: ese eje queda sin supervisar");
    }

    if (!guard.actualizar(motor1.getPosition(),
                          motor2.getPosition(),
                          motor3.getPosition()))
    {
        return;
    }

    dispararColision(guard.ejeDelFallo(),
                     guard.errorDelFallo(),
                     guard.cmdDeltaDelFallo(),
                     guard.encDeltaDelFallo());
}

void Robot::dispararColision(uint8_t eje, float errorDeg, float cmdDelta, float encDelta)
{
    // Lo primero, antes de cualquier print: frenar. Los tres ejes, no solo
    // el que disparo -- el brazo delta es un mecanismo cerrado, si uno se
    // trabo los otros dos estan forzando contra el.
    motor1.stop();
    motor2.stop();
    motor3.stop();

    // La pieza que estuviera agarrada se suelta. Despues del rehoming no hay
    // forma de saber si sigue pegada a la ventosa ni donde quedo, asi que
    // arrastrarla seria peor: el ciclo tiene que arrancar con el gripper
    // vacio y en estado conocido. La pieza perdida queda en el registro.
    pneumatics.release();
    pumpOn = false;

    // La cinta tambien para: si siguiera andando durante la pausa y el
    // rehoming, las piezas se acumularian y pasarian de largo sin
    // clasificar. Arranca sola de nuevo al final del homing
    // (conveyor.begin() en updateHoming), o sea recien con el robot ya
    // recalibrado y listo para trabajar.
    conveyor.stop();

    // Los dos motivos terminan igual (frenar y recalibrar), pero no son lo
    // mismo para quien despues lee el registro: uno es un golpe, el otro es
    // el robot perdiendo la calibracion solo, sin que nadie lo toque.
    const bool descalibracion =
        (guard.motivoDelFallo() == CollisionGuard::MOTIVO_DESCALIBRACION);

    registrarFallo(descalibracion ? FALLO_DESCALIBRACION : FALLO_COLISION,
                   (uint8_t)(eje + 1), errorDeg, cmdDelta, encDelta);

    if (descalibracion)
    {
        Serial.print("[DESCALIBRACION] eje ");
        Serial.print(eje + 1);
        Serial.print(": quieto en home el encoder marca ");
        Serial.print(errorDeg, 1);
        Serial.println(" grados de diferencia con los pasos.");
        Serial.println("[DESCALIBRACION] No es un choque: se perdieron pasos, o alguien");
        Serial.println("        movio el brazo, o hay algo flojo. Se recalibra.");
    }
    else
    {
        Serial.print("[COLISION] eje ");
        Serial.print(eje + 1);
        Serial.print(": el encoder marca ");
        Serial.print(errorDeg, 1);
        Serial.print(" grados de diferencia con los pasos (pasos ");
        Serial.print(cmdDelta, 1);
        Serial.print(" / encoder ");
        Serial.print(encDelta, 1);
        Serial.println(")");
    }

    colisionesSeguidas++;

    if (colisionesSeguidas >= (uint8_t)MAX_COLISIONES_SEGUIDAS)
    {
        // Chocar, rehomear y volver a chocar contra lo mismo no lo va a
        // resolver: hay algo fisico que sacar antes de seguir.
        conveyor.stop();

        Serial.print("[COLISION] ");
        Serial.print(colisionesSeguidas);
        Serial.println(" colisiones seguidas sin completar una pieza: hay algo trabado.");
        Serial.println("[COLISION] Robot detenido. Revisar y mandar 'R'.");

        moveIssued = false;
        state = ERROR;
        return;
    }

    Serial.print("[COLISION] parado ");
    Serial.print(COLLISION_PAUSE_MS / 1000.0f, 1);
    Serial.println(" s y despues recalibra");

    moveIssued = false;
    collisionStart_ms = millis();
    state = COLLISION_STOP;
}

void Robot::updateCollisionStop()
{
    // Pausa no bloqueante: el loop sigue girando entero (Serial, encoders,
    // cola de piezas). Los motores estan frenados, no hay nada que mover.
    if ((uint32_t)(millis() - collisionStart_ms) < COLLISION_PAUSE_MS)
    {
        return;
    }

    Serial.println("[COLISION] recalibrando...");

    // La cola se descarta: con la cinta detenida durante la pausa y el
    // homing, las piezas ya no estan donde su timestamp dice que estarian.
    // Las que queden sobre la cinta las vuelve a detectar la vision cuando
    // arranque de nuevo.
    startHoming(false);
}

// ============================================================
//  REGISTRO DE FALLOS
// ============================================================

void Robot::registrarFallo(uint8_t tipo, uint8_t eje,
                            float errorDeg, float cmdDelta, float encDelta)
{
    RegistroFallo r;

    r.tipo     = tipo;
    r.eje      = eje;
    r.errorDeg = errorDeg;
    r.cmdDelta = cmdDelta;
    r.encDelta = encDelta;
    r.estado   = nombreEstado(state);

    // Con que pieza fallo: color, forma, donde estaba sobre la cinta y a que
    // tacho iba. Es lo que despues necesita la interfaz para mostrar el
    // fallo con contexto y no como un numero suelto.
    if (hayManiobraEnCurso())
    {
        r.conPieza = true;
        r.enMano   = hayPiezaEnMano();
        r.color    = currentPiece.color;
        r.forma    = currentPiece.shape;
        r.piezaY   = currentPiece.y;
        r.piezaX   = piezaXEstimada(currentPiece);

        // En modo alfajores el destino es una celda de la caja (1-6); en los
        // otros dos, uno de los tres tachos.
        r.tacho    = (currentCell != CELDA_NINGUNA) ? (uint8_t)(currentCell + 1)
                                                    : (uint8_t)(currentBin + 1);
    }

    fallos.registrar(r);
}

float Robot::piezaXEstimada(const Piece &p) const
{
    // Misma cuenta que usa planificarPieza(): desde que se detecto, la pieza
    // avanzo velocidad * tiempo, corregido por la latencia de la vision.
    const float t = VISION_LATENCY_S + (millis() - p.detectedAt_ms) / 1000.0f;
    return DETECTION_LINE_X + BELT_VELOCITY_CMS * t;
}

bool Robot::hayManiobraEnCurso() const
{
    return state == PICK_APPROACH ||
           state == PICK_DESCEND  ||
           state == PICK_LIFT     ||
           state == GO_BIN        ||
           state == BIN_SETTLE    ||
           state == RELEASE_WAIT  ||
           state == BOX_TRANSIT   ||
           state == BOX_APPROACH  ||
           state == BOX_DESCEND   ||
           state == BOX_LIFT;
}

bool Robot::hayPiezaEnMano() const
{
    // Desde que empieza la bajada de agarre ya se cuenta como "en la mano":
    // el contacto ocurre a mitad del tramo 2, no al final. BOX_LIFT queda
    // afuera a proposito: ahi el alfajor ya esta apoyado en la caja.
    return state == PICK_DESCEND ||
           state == PICK_LIFT    ||
           state == GO_BIN       ||
           state == BIN_SETTLE   ||
           state == BOX_TRANSIT  ||
           state == BOX_APPROACH ||
           state == BOX_DESCEND;
}

const char *Robot::nombreEstado(RobotState s) const
{
    switch (s)
    {
        case IDLE:           return "IDLE";
        case HOMING:         return "HOMING";
        case WAIT_PIECE:     return "WAIT_PIECE";
        case GO_HOME_IDLE:   return "GO_HOME_IDLE";
        case PICK_APPROACH:  return "PICK_APPROACH";
        case PICK_DESCEND:   return "PICK_DESCEND";
        case PICK_LIFT:      return "PICK_LIFT";
        case GO_BIN:         return "GO_BIN";
        case BIN_SETTLE:     return "BIN_SETTLE";
        case RELEASE_WAIT:   return "RELEASE_WAIT";
        case BOX_TRANSIT:    return "BOX_TRANSIT";
        case BOX_APPROACH:   return "BOX_APPROACH";
        case BOX_DESCEND:    return "BOX_DESCEND";
        case BOX_LIFT:       return "BOX_LIFT";
        case COLLISION_STOP: return "COLLISION_STOP";
        case ERROR:          return "ERROR";
        default:             return "?";
    }
}

void Robot::imprimirEstadoSupervision() const
{
    Serial.print("[GUARD] ");
    Serial.print(guard.nombreEstado());
    Serial.print(guard.armado() ? "" : (supervisionHabilitada ? " (habilitada)" : " (apagada por el operador)"));
    Serial.print("  umbral=");
    Serial.print(guard.umbral(), 1);
    Serial.print("+");
    Serial.print(guard.margenVelocidad());
    Serial.print("ms*vel  confirmacion=");
    Serial.print(guard.confirmacion());
    Serial.print(" ms  compensacion=");
    Serial.print(guard.retardo());
    Serial.println(" ms");

    Serial.print("[GUARD] en reposo (home): umbral=");
    Serial.print(guard.umbralReposo(), 1);
    Serial.print(" grados  confirmacion=");
    Serial.print(GuardConfig::CONFIRMACION_REPOSO_MS);
    Serial.println(" ms");

    for (uint8_t i = 0; i < 3; i++)
    {
        // err     lo que ve el guard ahora mismo
        // pico    el peor error desde el homing: con esto se elige el umbral
        // gan     encoder/pasos con el eje frenado. TIENE que dar ~1.00; si
        //         da menos, se estan perdiendo cuentas (y ningun margen lo
        //         arregla, hay que ir al sensor)
        // atraso  error/velocidad en marcha. Si es estable y parecido en los
        //         3 ejes, es atraso de medicion y se compensa con 'L'
        // raw     extremos de lectura cruda: dicen cuanto margen queda hasta
        //         la zona donde el ADC ya no mide (60 a 3950)
        Serial.print("[GUARD] eje ");
        Serial.print(i + 1);
        Serial.print("  err=");
        Serial.print(guard.errorActual(i), 2);
        Serial.print("  pico=");
        Serial.print(guard.picoDesdeHoming(i), 2);
        Serial.print("  pico_total=");
        Serial.print(guard.picoHistorico(i), 2);
        // pico_reposo  el peor error visto QUIETO EN HOME: es el piso de
        //              ruido real de esa pose, y con el se elige
        //              UMBRAL_REPOSO_DEG (que tiene que quedarle 2 o 3
        //              veces por encima)
        Serial.print("  pico_reposo=");
        Serial.print(guard.picoEnReposo(i), 2);
        Serial.print("  umbral_ef=");
        Serial.print(guard.umbralEfectivo(i), 1);
        Serial.print("  gan=");
        Serial.print(guard.gananciaMedida(i), 3);
        Serial.print("  atraso=");
        Serial.print(guard.atrasoMedido_ms(i), 1);
        Serial.print("ms  fuga=");
        Serial.print(guard.derivaAbsorbida(i), 2);
        Serial.print("  raw=");
        Serial.print(guard.rawMinimo(i));
        Serial.print("..");
        Serial.print(guard.rawMaximo(i));
        Serial.print("  encoder=");
        Serial.println(guard.fueraDeRango(i) ? "FUERA DE RANGO" :
                       (guard.sensorCaido(i) ? "CAIDO" : "ok"));
    }

    fallos.imprimirResumen();
}

// ============================================================
//  TRAZA EN VIVO ('M')
// ============================================================
//  Vuelca a 20 Hz lo que ve la supervision, para poder mirar un movimiento
//  entero y entender que pasa cuando salta un falso positivo. Con esto se
//  distinguen los tres casos que se parecen entre si:
//
//    ATRASO      enc va detras de cmd durante todo el movimiento y lo
//                alcanza al frenar: al final err vuelve a ~0 solo
//    PERDIDA DE  enc no alcanza a cmd nunca: al frenar queda un escalon
//    CUENTAS     que no se va, y se suma al del movimiento siguiente
//    ZONA CIEGA  raw se queda clavado en un extremo mientras cmd avanza
//
//  Formato pensado para copiar y pegar en una planilla o en Python.
// ============================================================

// Una sola linea, pensada para leerla en el log de la interfaz de Python
// cuando no se puede pedir 'S' a mano:
//
//   pico  peor error desde el homing -> con esto se elige el umbral fijo
//   prep  peor error QUIETO EN HOME -> con esto se elige UMBRAL_REPOSO_DEG
//   gan   encoder/pasos con el eje frenado; tiene que dar ~1.00
//   atr   atraso de la medicion en ms (error/velocidad en marcha)
//   fuga  grados de deriva que la fuga en reposo lleva absorbidos: si crece
//         sin parar y siempre para el mismo lado, se pierden pasos de verdad
void Robot::imprimirDiagnosticoCorto() const
{
    Serial.print("[GUARD]");

    for (uint8_t i = 0; i < 3; i++)
    {
        Serial.print(" e");
        Serial.print(i + 1);
        Serial.print(" err=");
        Serial.print(guard.errorActual(i), 1);
        Serial.print(" pico=");
        Serial.print(guard.picoDesdeHoming(i), 1);
        Serial.print(" prep=");
        Serial.print(guard.picoEnReposo(i), 1);
        Serial.print(" gan=");
        Serial.print(guard.gananciaMedida(i), 3);
        Serial.print(" atr=");
        Serial.print(guard.atrasoMedido_ms(i), 0);
        Serial.print(" fuga=");
        Serial.print(guard.derivaAbsorbida(i), 1);
    }

    Serial.print(" fallos=");
    Serial.println(fallos.total());
}

void Robot::imprimirTraza()
{
    Serial.print("[TRAZA] t=");
    Serial.print(millis());

    for (uint8_t i = 0; i < 3; i++)
    {
        Serial.print(" e");
        Serial.print(i + 1);
        Serial.print(" raw=");
        Serial.print(encoders.leerRaw(i));
        Serial.print(" enc=");
        Serial.print(guard.encDeltaActual(i), 2);
        Serial.print(" cmd=");
        Serial.print(guard.cmdDeltaActual(i), 2);
        Serial.print(" err=");
        Serial.print(guard.errorActual(i), 2);
        Serial.print(" vel=");
        Serial.print(guard.velocidadCmd(i), 0);
    }

    Serial.println();
}

// ============================================================
//  TABLA DE PARAMETROS
// ============================================================
//  Lo que la interfaz puede ajustar sin recompilar. El nivel decide en que
//  pestana aparece; el rango es lo que el firmware hace cumplir.
//
//  QUE NO ESTA ACA, Y POR QUE
//  --------------------------
//  La geometria del delta (DeltaKinematics.h), los micropasos, el ancho
//  util de la cinta y el area de trabajo NO son parametros: son limites de
//  seguridad y calibraciones de las que depende que el robot no se clave
//  contra la mesa. Un slider para eso no es flexibilidad, es una trampa.
//  Se cambian en el codigo, recompilando, que es lo que corresponde a algo
//  que hay que volver a validar despues de tocarlo.
// ============================================================

void Robot::registrarParametros()
{
    // --- Nivel 1: operacion ---
    // La latencia de vision es EL ajuste fino de cada arranque: cuanto
    // avanza la pieza entre que cruza la linea y que llega el mensaje.
    // Admite negativos a proposito (ver PROTOCOLO.md): si alguna vez la
    // correccion tuviera que ir para el otro lado, el rango ya lo permite.
    params.registrar("vis_lat", &VISION_LATENCY_S, -0.20f, 0.30f, "s", NIVEL_OPERACION);

    // --- Nivel 2: proceso (afinado del agarre y de la supervision) ---
    params.registrar("press_dz", &PRESS_DZ,    0.0f,   0.30f, "cm", NIVEL_PROCESO);
    params.registrar("apr_dx",   &APPROACH_DX, -6.0f,  0.0f,  "cm", NIVEL_PROCESO);
    params.registrar("apr_dz",   &APPROACH_DZ, 0.5f,   6.0f,  "cm", NIVEL_PROCESO);
    params.registrar("lift_dz",  &LIFT_DZ,     1.0f,   8.0f,  "cm", NIVEL_PROCESO);
    params.registrar("rel_ms",   &RELEASE_DETACH_MS, 0.0f, 2000.0f, "ms", NIVEL_PROCESO, 'i');
    params.registrar("bin_ms",   &BIN_SETTLE_MS,     0.0f, 2000.0f, "ms", NIVEL_PROCESO, 'i');

    params.registrar("g_umbral",  &pGuardUmbral,   1.0f,  45.0f,  "deg", NIVEL_PROCESO);
    params.registrar("g_reposo",  &pGuardReposo,   2.0f,  20.0f,  "deg", NIVEL_PROCESO);
    params.registrar("g_conf",    &pGuardConfirma, 10.0f, 1000.0f, "ms", NIVEL_PROCESO, 'i');
    params.registrar("g_margen",  &pGuardMargen,   0.0f,  500.0f,  "ms", NIVEL_PROCESO, 'i');
    params.registrar("g_retardo", &pGuardRetardo,  0.0f,  500.0f,  "ms", NIVEL_PROCESO, 'i');
    params.registrar("g_salto",   &pGuardSalto,    1.0f,  20.0f,   "deg", NIVEL_PROCESO);
    params.registrar("g_salto_k", &pGuardSaltoPct, 0.0f,  20.0f,   "%", NIVEL_PROCESO);

    params.registrar("col_pausa", &COLLISION_PAUSE_MS, 0.0f, 15000.0f, "ms", NIVEL_PROCESO, 'i');
    params.registrar("col_max",   &MAX_COLISIONES_SEGUIDAS, 1.0f, 10.0f, "", NIVEL_PROCESO, 'i');
    params.registrar("g_neum",    &BLANQUEO_NEUMATICA_MS,   0.0f, 2000.0f, "ms", NIVEL_PROCESO, 'i');

    // Movimiento. La velocidad esta a proposito por encima de lo alcanzable
    // (ver Motors.h): el numero que de verdad cambia el ciclo y la vibracion
    // es la aceleracion, y por eso las dos van en proceso y la velocidad en
    // servicio.
    params.registrar("acc_rap",   &Motors::ACC_RAPIDA, 20000.0f, 150000.0f, "pas/s2", NIVEL_PROCESO, 'i');
    params.registrar("acc_suave", &Motors::ACC_SUAVE,   5000.0f, 100000.0f, "pas/s2", NIVEL_PROCESO, 'i');

    // Cinta: el PWM es lo que se manda, cinta_cms es lo que se mide. Se
    // tocan de a dos (ver el comentario de CONVEYOR_PWM).
    params.registrar("cinta_pwm", &CONVEYOR_PWM, 0.0f, 100.0f, "%", NIVEL_PROCESO);

    params.registrar("pick_tol",  &PICK_LATE_TOLERANCE_MS, 0.0f, 500.0f, "ms", NIVEL_PROCESO, 'i');
    params.registrar("pump_lead", &PUMP_LEAD_MS,           0.0f, 2000.0f, "ms", NIVEL_PROCESO, 'i');
    params.registrar("replan",    &MAX_REPLAN_ATTEMPTS,    0.0f, 5.0f,   "",   NIVEL_PROCESO, 'i');
    params.registrar("diag_ms",   &DIAGNOSTICO_PERIODICO_MS, 0.0f, 120000.0f, "ms", NIVEL_PROCESO, 'i');

    params.registrar("tele_ms", &telemetria.periodoRapida_ms,  20.0f, 2000.0f,  "ms", NIVEL_PROCESO, 'i');
    params.registrar("est_ms",  &telemetria.periodoProceso_ms, 100.0f, 10000.0f, "ms", NIVEL_PROCESO, 'i');
    params.registrar("sal_ms",  &telemetria.periodoSalud_ms,   500.0f, 30000.0f, "ms", NIVEL_PROCESO, 'i');

    // --- Nivel 3: servicio ---
    // Los Z son los que aplastan cosas si se equivoca el signo o la coma,
    // y ademas no se pueden cambiar con una pieza en la mano: le cambiarian
    // el destino a una maniobra ya planificada.
    params.registrar("grab_z", &GRAB_Z, -36.0f, -28.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("box_z",  &BOX_Z,  -36.0f, -28.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("box_apr", &BOX_APPROACH_DZ, 1.0f, 8.0f,  "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("box_tr",  &BOX_TRANSIT_DZ,  3.0f, 12.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("box_rel", &BOX_RELEASE_DETACH_MS, 0.0f, 2000.0f, "ms", NIVEL_SERVICIO, 'i');

    params.registrar("bin_x1", &BIN_X[0], -20.0f, 20.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("bin_x2", &BIN_X[1], -20.0f, 20.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("bin_x3", &BIN_X[2], -20.0f, 20.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("bin_y",  &BIN_Y,    -20.0f, 0.0f,  "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("bin_z",  &BIN_Z,    -34.0f, -20.0f, "cm", NIVEL_SERVICIO, 'f', true);

    params.registrar("home_a1", &HOME_ANGLE_M1, -60.0f, -30.0f, "deg", NIVEL_SERVICIO);
    params.registrar("home_a2", &HOME_ANGLE_M2, -60.0f, -30.0f, "deg", NIVEL_SERVICIO);
    params.registrar("home_a3", &HOME_ANGLE_M3, -60.0f, -30.0f, "deg", NIVEL_SERVICIO);

    // Estos dos existen TAMBIEN del lado de Python (config.py). Si se
    // cambian de un solo lado, el robot le empieza a errar a las piezas sin
    // que nada avise: el nucleo compara los dos al conectarse.
    params.registrar("cinta_cms", &BELT_VELOCITY_CMS, 1.0f, 30.0f, "cm/s", NIVEL_SERVICIO);
    params.registrar("linea_x",   &DETECTION_LINE_X, -40.0f, 0.0f, "cm", NIVEL_SERVICIO);

    // La caja entera se corre con estos dos. La grilla de 6 cm entre celdas
    // es la geometria de la caja y no se toca; lo que se mueve es donde
    // quedo apoyada sobre la mesa.
    params.registrar("box_dx", &BOX_DX, -4.0f, 4.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("box_dy", &BOX_DY, -4.0f, 4.0f, "cm", NIVEL_SERVICIO, 'f', true);

    // El piso baja a 5000 porque el rango util quedo ABAJO del que tenia:
    // el pico real del movimiento mas largo son ~16.400 pasos/s, asi que
    // cualquier tope por encima de eso no limita nada. Los valores que
    // sirven para recortar los tramos largos van de 6.000 a 14.000.
    params.registrar("vel_max", &Motors::VEL_MAX, 5000.0f, 120000.0f, "pas/s", NIVEL_SERVICIO, 'i');

    params.registrar("home_set", &HOMING_SETTLE_WAIT_MS, 500.0f,  10000.0f, "ms", NIVEL_SERVICIO, 'i');
    params.registrar("home_to",  &HOMING_TIMEOUT_MS,     5000.0f, 60000.0f, "ms", NIVEL_SERVICIO, 'i');
    params.registrar("tapa_ms",  &CONFIRMACION_TAPA_MS,  2000.0f, 60000.0f, "ms", NIVEL_SERVICIO, 'i');
}

// ------------------------------------------------------------------
// Baja los parametros a los objetos que guardan una COPIA adentro en vez de
// leer la variable en cada uso. Son los unicos que no se enteran solos de un
// cambio; todo lo demas lee su float directamente en el punto de uso.
void Robot::sincronizarParametros()
{
    guard.setUmbral(pGuardUmbral);
    guard.setUmbralReposo(pGuardReposo);
    guard.setSaltoParada(pGuardSalto, pGuardSaltoPct / 100.0f);
    guard.setConfirmacion((uint32_t)pGuardConfirma);
    guard.setMargenVelocidad((uint32_t)pGuardMargen);
    guard.setRetardo((uint32_t)pGuardRetardo);

    Motors::aplicarLimites();

    // El PWM se reaplica solo si la cinta ESTA andando: mandarselo con la
    // cinta parada la arrancaria sola al tocar cualquier parametro, con el
    // robot posiblemente en ERROR y nadie mirando.
    if (conveyor.getSpeed() > 0)
    {
        conveyor.setSpeedPercent(CONVEYOR_PWM);
    }
}

// ============================================================
//  TELEMETRIA
// ============================================================

// Modo -> la letra con la que viaja por serie. Es la MISMA letra que se usa
// para pedirlo ('C', 'F', 'A'), asi que la interfaz no necesita una segunda
// tabla de traduccion para leer lo que informa el firmware.
static char letraModo(Robot::SortMode m)
{
    switch (m)
    {
        case Robot::SORT_BY_COLOR:  return 'C';
        case Robot::SORT_BY_SHAPE:  return 'F';
        case Robot::SORT_ALFAJORES: return 'A';
        default:                    return '?';
    }
}

void Robot::emitirTelemetria(uint32_t ahora)
{
    if (telemetria.tocaRapida(ahora))
    {
        TeleRapida d;

        d.t_ms   = ahora;
        d.estado = (uint8_t)state;
        d.bomba  = pneumatics.isActive();

        // Los finales se leen aca y no se guardan de la ultima vez que los
        // miro el homing: la idea es justamente poder verificar uno
        // empujandolo con el dedo y ver si se pinta en la pantalla.
        d.finales[0] = endstops.readMotor1();
        d.finales[1] = endstops.readMotor2();
        d.finales[2] = endstops.readMotor3();

        const long pasos[3] = {motor1.getPosition(),
                               motor2.getPosition(),
                               motor3.getPosition()};

        for (uint8_t i = 0; i < 3; i++)
        {
            d.ang[i] = encoders.leerGrados(i);
            d.cmd[i] = stepsToAngle(pasos[i]);
            d.err[i] = guard.errorActual(i);
            d.umb[i] = guard.umbralEfectivo(i);
            d.vel[i] = guard.velocidadCmd(i);
        }

        telemetria.emitirRapida(d);
    }

    if (telemetria.tocaProceso(ahora))
    {
        TeleProceso d;

        d.t_ms         = ahora;
        d.estado       = (uint8_t)state;
        d.estadoNombre = nombreEstado(state);

        d.modo          = letraModo(sortMode);
        d.modoPendiente = sortModePending ? letraModo(pendingSortMode) : '-';

        d.esperandoTapa = esperandoConfirmacion;
        d.tapaRestante_ms = 0;

        if (esperandoConfirmacion)
        {
            const uint32_t pasado = ahora - confirmacionPedida_ms;

            d.tapaRestante_ms = (pasado < (uint32_t)CONFIRMACION_TAPA_MS)
                                    ? (uint32_t)(CONFIRMACION_TAPA_MS - pasado)
                                    : 0;
        }

        d.cola              = queueCount;
        d.colaAntiguedad_ms = antiguedadCola();

        d.homed          = homed;
        d.guard          = (uint8_t)guard.estado();
        d.observando     = guard.observando();
        d.paradasActivas = supervisionHabilitada;

        const int pwm = conveyor.getSpeed();

        d.cinta    = (pwm > 0);
        d.cintaPwm = (uint8_t)((pwm * 100) / 255);

        // La caja solo tiene sentido en el modo que la usa: fuera de ahi se
        // manda '-' para que la interfaz no dibuje una grilla que no
        // corresponde a nada.
        const bool enCaja = esAlfajores(sortMode);

        d.layout         = enCaja ? boxLayout : NULL;
        d.llenas         = enCaja ? boxFilled : NULL;
        d.celdaReservada = (currentCell == CELDA_NINGUNA) ? 0 : (uint8_t)(currentCell + 1);

        if (hayManiobraEnCurso())
        {
            d.piezaColor = currentPiece.color;
            d.piezaForma = currentPiece.shape;
            d.piezaY     = currentPiece.y;
            d.tacho      = (uint8_t)(currentBin + 1);
        }
        else
        {
            d.piezaColor = '-';
            d.piezaForma = '-';
            d.piezaY     = 0.0f;
            d.tacho      = 0;
        }

        d.detectadas  = piezasDetectadas;
        d.depositadas = piezasDepositadas;
        d.descartadas = piezasDescartadas;
        d.fallos      = fallos.total();

        for (uint8_t i = 0; i < 3; i++)
        {
            d.porColor[i] = porColorOk[i];
            d.porForma[i] = porFormaOk[i];
        }

        telemetria.emitirProceso(d);
    }

    if (telemetria.tocaSalud(ahora))
    {
        TeleSalud d;

        d.t_ms      = ahora;
        d.uptime_s  = ahora / 1000UL;
        d.loopHz    = telemetria.loopHz();
        d.heapLibre = ESP.getFreeHeap();

        for (uint8_t i = 0; i < 3; i++)
        {
            TeleSalud::Eje &e = d.ejes[i];

            // El orden importa: un encoder fuera de rango tambien aparece
            // como caido, y lo primero es mas especifico (dice DONDE esta
            // el problema, no solo que lo hay).
            e.encoder = guard.fueraDeRango(i) ? "rango"
                        : (guard.sensorCaido(i) ? "caido" : "ok");

            e.ganancia   = guard.gananciaMedida(i);
            e.atraso_ms  = guard.atrasoMedido_ms(i);
            e.pico       = guard.picoDesdeHoming(i);
            e.picoReposo = guard.picoEnReposo(i);
            e.fuga       = guard.derivaAbsorbida(i);
            e.rawMin     = guard.rawMinimo(i);
            e.rawMax     = guard.rawMaximo(i);
            e.resincronizaciones = encoders.cuentaResincronizaciones(i);
        }

        telemetria.emitirSalud(d);
    }
}

// ------------------------------------------------------------------
uint32_t Robot::antiguedadCola() const
{
    if (queueCount == 0)
    {
        return 0;
    }

    // queueHead es la mas VIEJA (la cola es FIFO), que es tambien la mas
    // adelantada sobre la cinta: la que menos tiempo tiene antes de
    // pasarse de largo.
    return (uint32_t)(millis() - pieceQueue[queueHead].detectedAt_ms);
}

// ------------------------------------------------------------------
void Robot::contarDepositada(const Piece &p)
{
    piezasDepositadas++;

    switch (p.color)
    {
        case 'R': porColorOk[0]++; break;
        case 'G': porColorOk[1]++; break;
        case 'B': porColorOk[2]++; break;
        default:                   break;
    }

    switch (p.shape)
    {
        case 'S': porFormaOk[0]++; break;
        case 'H': porFormaOk[1]++; break;
        case 'C': porFormaOk[2]++; break;
        default:                   break;
    }
}

// ============================================================
//  EMERGENCIA
// ============================================================

void Robot::emergencyStop()
{
    motor1.stop();
    motor2.stop();
    motor3.stop();

    conveyor.stop();

    // Con el robot frenado a mitad de camino, la posicion real deja de
    // corresponderse con los pasos comandados: no tiene sentido seguir
    // comparando hasta que se rehomee.
    guard.desarmar();

    registrarFallo(FALLO_MANUAL, 0);

    Serial.println("[EMERGENCIA] Parada manual. Presiona 'R' para rehomear.");

    moveIssued = false;
    state = ERROR;
}
