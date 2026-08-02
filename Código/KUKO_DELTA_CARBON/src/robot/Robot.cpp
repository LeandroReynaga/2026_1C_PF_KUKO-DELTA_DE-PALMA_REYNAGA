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
static const float BELT_VELOCITY_CMS = 7.54f;
static const float DETECTION_LINE_X  = -23.0f; // donde la camara detecta las piezas

// Ancho util de la cinta (Y). Fuera de esto el dato de vision es erroneo.
static const float BELT_MIN_Y = -2.8f;
static const float BELT_MAX_Y = 11.2f;

// ============================================================
//  GEOMETRIA DE AGARRE (coordenadas de la PUNTA del gripper)
//  DeltaKinematics ya descuenta el offset de herramienta (0,0,-2.8).
// ============================================================
static const float GRAB_Z      = -32.3f; // cara superior de la pieza (1 cm de alto)
static const float APPROACH_DX = -2.0f;  // 2 cm por detras: rampa a favor de la cinta
static const float APPROACH_DZ = 0.4f;   // 4 mm por arriba

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
static const float PRESS_DZ = 0.025f;

static const float LIFT_DZ = 2.0f;       // despegue de la pieza de la cinta

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
static const float BIN_X[3] = {-12.0f, 0.0f, 12.0f};
static const float BIN_Y    = -9.55f;
static const float BIN_Z    = -29.3f;

// ============================================================
//  TIEMPOS (todos no bloqueantes, con millis())
// ============================================================
static const uint32_t HOMING_SETTLE_WAIT_MS = 2500; // ventana de promediado de encoders
static const uint32_t BIN_SETTLE_MS         = 200;  // quieto sobre el tacho antes de soltar

// Cuanto se espera con la bomba apagada para que la pieza se despegue del
// gripper. Es un parche temporal: falta la electrovalvula que mete aire en
// la linea de vacio al apagar la bomba. Cuando este montada (misma senal
// que la bomba, no cambia el pinout) la pieza cae en el instante y este
// tiempo se puede bajar a ~100 ms.
static const uint32_t RELEASE_DETACH_MS = 10000;

// Margen de atraso tolerable al llegar al punto de aproximacion antes de
// dar por perdido el instante de encuentro y replanificar.
static const uint32_t PICK_LATE_TOLERANCE_MS = 30;
static const uint8_t  MAX_REPLAN_ATTEMPTS    = 2;

Robot::Robot() :

motor1(PUL1, DIR1, ENA, 0),
motor2(PUL2, DIR2, ENA, 1),
motor3(PUL3, DIR3, ENA, 2)

{
    state = IDLE;

    axis1Homed = false;
    axis2Homed = false;
    axis3Homed = false;
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

    Serial.println();
    Serial.println("=== KUKO DELTA CARBON ===");
    Serial.println("Comandos por Serial (uno por linea):");
    Serial.println("  Y,color,forma   pieza detectada. Ej: 3.5,B,S");
    Serial.println("                  color = R/G/B, forma = S/H/C");
    Serial.println("  C               clasificar por COLOR");
    Serial.println("  F               clasificar por FORMA");
    Serial.println("  R               parada de emergencia / reinicio");
}

void Robot::update()
{
    procesarSerial();

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
            if (cmdLen > 0)
            {
                cmdBuffer[cmdLen] = '\0';
                procesarComando(cmdBuffer, cmdLen);
            }
            cmdLen = 0;
        }
        else if (cmdLen < sizeof(cmdBuffer) - 1)
        {
            cmdBuffer[cmdLen++] = c;
        }
        else
        {
            // Linea mas larga que el buffer: se descarta entera, para no
            // interpretar un pedazo suelto como si fuera un comando valido.
            cmdLen = 0;
        }
    }
}

void Robot::procesarComando(char *cmd, uint8_t len)
{
    // --- Comandos de un solo caracter: modo de clasificacion y emergencia ---
    // Un mensaje de pieza SIEMPRE tiene 2 comas, asi que no hay ambiguedad
    // con 'C' (modo color) ni con 'R' (reset), aunque esas mismas letras se
    // usen como forma/color adentro de un mensaje de pieza.
    if (len == 1)
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

        if (c == 'C' || c == 'F')
        {
            const SortMode nuevo = (c == 'C') ? SORT_BY_COLOR : SORT_BY_SHAPE;

            // Si el robot tiene una pieza en la mano, el cambio queda
            // pendiente hasta que la suelte: cambiarle el tacho de destino
            // a una pieza en vuelo la mandaria al lugar equivocado.
            const bool conPiezaEnMano = (state == PICK_DESCEND ||
                                          state == PICK_LIFT   ||
                                          state == GO_BIN      ||
                                          state == BIN_SETTLE);

            if (conPiezaEnMano)
            {
                pendingSortMode = nuevo;
                sortModePending = true;
                Serial.print("[MODO] pendiente -> ");
                Serial.println(nombreModo(nuevo));
            }
            else
            {
                sortMode = nuevo;
                sortModePending = false;
                Serial.print("[MODO] ");
                Serial.println(nombreModo(sortMode));
            }
            return;
        }

        Serial.println("[SERIAL] comando desconocido");
        return;
    }

    // --- Mensaje de pieza: "Y,color,forma" ---
    char *coma1 = strchr(cmd, ',');
    if (coma1 == NULL)
    {
        Serial.println("[SERIAL] comando desconocido");
        return;
    }

    char *coma2 = strchr(coma1 + 1, ',');
    if (coma2 == NULL)
    {
        Serial.println("[SERIAL] mensaje de pieza incompleto");
        return;
    }

    const char color = toupper(coma1[1]);
    const char shape = toupper(coma2[1]);

    *coma1 = '\0'; // corta el campo Y para poder convertirlo
    const float y = atof(cmd);

    if (color != 'R' && color != 'G' && color != 'B')
    {
        Serial.println("[SERIAL] color invalido (R/G/B)");
        return;
    }
    if (shape != 'S' && shape != 'H' && shape != 'C')
    {
        Serial.println("[SERIAL] forma invalida (S/H/C)");
        return;
    }
    if (y < BELT_MIN_Y || y > BELT_MAX_Y)
    {
        Serial.print("[SERIAL] Y fuera de la cinta: ");
        Serial.println(y);
        return;
    }

    Piece p;
    p.y = y;
    p.color = color;
    p.shape = shape;
    p.detectedAt_ms = millis(); // el retardo vision->serial se toma despreciable

    if (!queuePush(p))
    {
        Serial.println("[COLA] llena, se descarta la pieza mas nueva");
        return;
    }

    Serial.print("[PIEZA] Y=");
    Serial.print(p.y);
    Serial.print(" color=");
    Serial.print(p.color);
    Serial.print(" forma=");
    Serial.print(p.shape);
    Serial.print("  en cola: ");
    Serial.println(queueCount);
}

const char *Robot::nombreModo(SortMode m) const
{
    return (m == SORT_BY_COLOR) ? "por COLOR" : "por FORMA";
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

void Robot::startHoming()
{
    axis1Homed = false;
    axis2Homed = false;
    axis3Homed = false;
    homed = false;

    state = HOMING;

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
    queueHead = 0;
    queueCount = 0;

    // Se suelta lo que hubiera quedado agarrado antes del corte, para
    // arrancar el ciclo con el gripper vacio y en estado conocido.
    pneumatics.release();
    pumpOn = false;

    // Un cambio de modo que habia quedado pendiente pertenecia al ciclo que
    // se corto: no se arrastra al ciclo nuevo. El modo ACTIVO si se
    // mantiene (el operador ya lo eligio y no lo dio de baja).
    sortModePending = false;

    moveIssued = false;
    replanCount = 0;
    homingSettleStart_ms = 0;
}

void Robot::updateHoming()
{
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
            homingSettleStart_ms = millis();
            return;
        }

        if (millis() - homingSettleStart_ms < HOMING_SETTLE_WAIT_MS)
        {
            return; // seguimos acumulando muestras, sin bloquear el resto del sistema
        }

        homingSettleStart_ms = 0;

        // Calibra usando el PROMEDIO de todas las muestras de la ventana de
        // espera (no una lectura puntual).
        encoders.calibrarHoming(HOME_ANGLE_M1, HOME_ANGLE_M2, HOME_ANGLE_M3);

        homed = true;

        // La cinta arranca recien con el robot ya calibrado: antes de eso
        // no tendria sentido aceptar piezas.
        conveyor.begin();

        Serial.println("[HOMING] OK. Robot listo.");
        Serial.print("[MODO] ");
        Serial.println(nombreModo(sortMode));

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
    // Si aparece una pieza mientras vuelve a home, se interrumpe el regreso
    // y sale a buscarla desde donde este: el proximo moveSynchronized
    // recalcula todo desde la posicion actual.
    if (queueCount > 0 && iniciarSiguientePieza())
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
    geom.liftDZ = LIFT_DZ;
    geom.workAreaMinX = WORK_AREA_MIN_X;
    geom.workAreaMaxX = WORK_AREA_MAX_X;

    const float tSinceDetection = (millis() - p.detectedAt_ms) / 1000.0f;

    ConveyorIntercept::InterceptResult r = ConveyorIntercept::solve(
        p.y, tSinceDetection, belt, geom,
        motor1.getPosition(), motor2.getPosition(), motor3.getPosition(),
        Motors::MAX_SPEED, Motors::MAX_ACCELERATION, Motors::MIN_ACCELERATION);

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

    while (queuePop(p))
    {
        if (!planificarPieza(p))
        {
            Serial.print("[PIEZA] no alcanzable (Y=");
            Serial.print(p.y);
            Serial.println("), se deja pasar");
            continue; // se prueba con la siguiente de la cola
        }

        currentPiece = p;
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
        Serial.print(") -> tacho ");
        Serial.println(currentBin + 1);

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

    // Ya en el punto de aproximacion: se prende la bomba para que el vacio
    // este bien formado antes de tocar la pieza.
    if (!pumpOn)
    {
        pneumatics.grab();
        pumpOn = true;
    }

    // Se espera al instante calculado para lanzar la bajada. La resta con
    // signo aguanta el desborde de millis().
    const int32_t atraso_ms = (int32_t)(millis() - descendStart_ms);

    if (atraso_ms < 0)
    {
        return; // todavia no es momento: la pieza no llego
    }

    if (atraso_ms > (int32_t)PICK_LATE_TOLERANCE_MS)
    {
        // Se perdio la ventana (el tramo 1 tardo mas de lo estimado). Se
        // replanifica con la pieza donde este ahora, si todavia da.
        if (replanCount < MAX_REPLAN_ATTEMPTS && planificarPieza(currentPiece))
        {
            replanCount++;
            moveIssued = false;
            Serial.println("[PIEZA] replanificando agarre");
            return;
        }

        Serial.println("[PIEZA] se perdio la ventana de agarre, se deja pasar");
        pneumatics.release();
        pumpOn = false;
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
            moveIssued = false;
            state = GO_HOME_IDLE;
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        state = GO_BIN;
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

    releaseStart_ms = millis();
    state = RELEASE_WAIT;
}

void Robot::updateReleaseWait()
{
    // Espera a que la pieza se despegue sola del gripper (ver
    // RELEASE_DETACH_MS: es un parche hasta montar la electrovalvula).
    if (millis() - releaseStart_ms < RELEASE_DETACH_MS)
    {
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
//  EMERGENCIA
// ============================================================

void Robot::emergencyStop()
{
    motor1.stop();
    motor2.stop();
    motor3.stop();

    conveyor.stop();

    Serial.println("[EMERGENCIA] Parada manual. Presiona 'R' para rehomear.");

    moveIssued = false;
    state = ERROR;
}
