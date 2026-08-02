#include "Robot.h"
#include "Pinout.h"
#include "hardware/Pneumatics.h"
#include "hardware/Conveyor.h"
#include "hardware/Encoders.h"
#include "kinematics/DeltaKinematics.h"
#include "hardware/Conveyor.h"

Conveyor conveyor(CINTAPWM);
Pneumatics pneumatics;

// Duraciones de espera no bloqueante (antes eran delay(2500), delay(10000) y delay(3000))
static const uint32_t HOMING_SETTLE_WAIT_MS = 2500;
static const uint32_t RELEASE_WAIT_MS        = 10000;
static const uint32_t CONVEYOR_STOP_WAIT_MS  = 3000;

// ============================================================
// Barrido de velocidad/aceleracion (bring-up de motores)
// ------------------------------------------------------------
// Objetivo: encontrar a ojo/oido el limite real de los NEMA23 con los
// drivers DM556 ajustados a 2.7A, antes de fijar velocidades definitivas.
// El robot va y vuelve entre dos puntos cartesianos (punto A: cerca de la
// cinta; punto B: al otro lado del rango), subiendo un escalon de
// velocidad/aceleracion en cada vuelta (o quedando fijo si el step es 0,
// para probar un valor puntual como 80mil/120mil de aceleracion). Todavia
// no hay deteccion automatica de paso perdido (eso requiere calibrar el
// filtro de los encoders), asi que el corte es manual: apenas se note
// que un motor patina/pierde pasos, presionar 'R' por el monitor serie.
// ============================================================
static const float    SPEED_TEST_A_X = 0.0f;
static const float    SPEED_TEST_A_Y = 0.0f;
static const float    SPEED_TEST_A_Z = -30.5f;

static const float    SPEED_TEST_B_X = 0.0f;
static const float    SPEED_TEST_B_Y = 0.0f;
static const float    SPEED_TEST_B_Z = -29.5f;

static const float    SPEED_TEST_START_SPEED = 70000.0f;//5000.0f;  // pasos/seg, arranque conservador
static const float    SPEED_TEST_START_ACCEL = 17000.0f;//130000.0f;  // pasos/seg^2
static const float    SPEED_TEST_SPEED_STEP  = 000.0f;  // incremento por vuelta completa (ida+vuelta)
static const float    SPEED_TEST_ACCEL_STEP  = 000.0f;
static const uint32_t SPEED_TEST_PAUSE_MS    = 00;      // pausa entre tramos, para observar/escuchar


Robot::Robot() :

motor1(PUL1, DIR1, ENA, 0),
motor2(PUL2, DIR2, ENA, 1),
motor3(PUL3, DIR3, ENA, 2)

{
    state = IDLE;

    axis1Homed = false;
    axis2Homed = false;
    axis3Homed = false;

    speedTestLimits = {SPEED_TEST_START_SPEED, SPEED_TEST_START_ACCEL};
}

void Robot::begin()
{
    pneumatics.begin();
    //conveyor.begin();

    motor1.begin();
    motor2.begin();
    motor3.begin();

    motor1.setSpeed(2000);
    motor2.setSpeed(2000);
    motor3.setSpeed(2000);

    endstops.begin();
}

void Robot::update()
{
    // Consola de ajuste fino por el monitor serie, se revisa en cualquier
    // estado. Comandos (terminados con Enter/nueva linea):
    //   R           -> en ERROR rehomea; en cualquier otro estado es la
    //                  parada de emergencia manual (todavia no hay
    //                  deteccion automatica de paso perdido via encoders)
    //   a<numero>   -> setea speedTestLimits.maxAcceleration (ej: a32000)
    //   v<numero>   -> setea speedTestLimits.maxSpeed        (ej: v40000)
    // El cambio se aplica al PROXIMO tramo (A->B o B->A) que se comande,
    // no al que esta en curso: el Stepper ya arranco ese movimiento con
    // la velocidad/aceleracion que tenia configurada en ese momento.
    static char cmdBuffer[16];
    static uint8_t cmdLen = 0;

    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r')
        {
            if (cmdLen > 0)
            {
                cmdBuffer[cmdLen] = '\0';

                if (cmdLen == 1 && (cmdBuffer[0] == 'R' || cmdBuffer[0] == 'r'))
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
                }
                else if (cmdLen > 1 && (cmdBuffer[0] == 'a' || cmdBuffer[0] == 'A'))
                {
                    float valor = atof(cmdBuffer + 1);
                    valor = constrain(valor, 0.0f, Motors::MAX_ACCELERATION);
                    speedTestLimits.maxAcceleration = valor;

                    Serial.print("[SPEED TEST] aceleracion -> ");
                    Serial.println(speedTestLimits.maxAcceleration);
                }
                else if (cmdLen > 1 && (cmdBuffer[0] == 'v' || cmdBuffer[0] == 'V'))
                {
                    float valor = atof(cmdBuffer + 1);
                    valor = constrain(valor, 0.0f, Motors::MAX_SPEED);
                    speedTestLimits.maxSpeed = valor;

                    Serial.print("[SPEED TEST] velocidad -> ");
                    Serial.println(speedTestLimits.maxSpeed);
                }
                else
                {
                    Serial.println("[SPEED TEST] comando no reconocido (usar 'aNUM', 'vNUM' o 'R')");
                }
            }

            cmdLen = 0;
        }
        else if (cmdLen < sizeof(cmdBuffer) - 1)
        {
            cmdBuffer[cmdLen++] = c;
        }
    }

    switch(state)
    {
        case HOMING:

            updateHoming();

            break;

        case SPEED_TEST_TO_POINT_A:

            updateSpeedTestToPointA();

            break;

        case SPEED_TEST_TO_POINT_B:

            updateSpeedTestToPointB();

            break;

        case GO_ZERO:

            updateGoZero();

            break;
        
        case GO_POSITION:
        
            updateGoPosition();

            break;

        case GRAB:
            updateGrab();

            break;

        case GO_UP:
            updateGoUp();

            break;

        case CONVEYOR_RUN:
            updateConveyorRun();

            break;
        
        case GO_DOWN:
            updateGoDown();

            break;

        case RELEASE:
            updateRelease();

            break;

        case GO_ZERO2:
            updateGoZero2();

            break;

        case CONVEYOR_STOP:
            updateConveyorStop();

            break;
        

        default:

            break;
    }
        
    motor1.update();
    motor2.update();
    motor3.update();
}

void Robot::startHoming()
{
    axis1Homed = false;
    axis2Homed = false;
    axis3Homed = false;

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

    // Cada rehomeo reinicia el barrido de velocidad/aceleracion desde el
    // valor conservador (no desde el valor que veniamos probando cuando
    // se disparo la parada de emergencia).
    speedTestLimits = {SPEED_TEST_START_SPEED, SPEED_TEST_START_ACCEL};
    speedTestMoveIssued = false;
    speedTestPauseStart_ms = 0;
}

void Robot::updateHoming()
{
    // Motor 1

    if(!axis1Homed)
    {
        if(endstops.readMotor1())
        {
            motor1.stop();

            motor1.setPosition(angleToSteps(HOME_ANGLE_M1));

            axis1Homed = true;
        }
    }

    // Motor 2

    if(!axis2Homed)
    {
        if(endstops.readMotor2())
        {
            motor2.stop();

            motor2.setPosition(angleToSteps(HOME_ANGLE_M2));

            axis2Homed = true;
        }
    }

    // Motor 3

    if(!axis3Homed)
    {
        if(endstops.readMotor3())
        {
            motor3.stop();

            motor3.setPosition(angleToSteps(HOME_ANGLE_M3));

            axis3Homed = true;
        }
    }

    // ¿Todos llegaron?

    if(axis1Homed &&
       axis2Homed &&
       axis3Homed
       )
    {
        
                if (homingSettleStart_ms == 0)
        {
            // Arranca la ventana de acumulación de media móvil por canal,
            // durante todo el segundo de espera.
            encoders.iniciarAsentamientoHoming();
            homingSettleStart_ms = millis();
            return;
        }

        if (millis() - homingSettleStart_ms < HOMING_SETTLE_WAIT_MS)
        {
            return; // seguimos acumulando muestras, sin bloquear el resto del sistema
        }

        homingSettleStart_ms = 0;

        // Calibra usando el PROMEDIO de todas las muestras del segundo
        // de espera (no una lectura puntual).
        encoders.calibrarHoming(HOME_ANGLE_M1, HOME_ANGLE_M2, HOME_ANGLE_M3);

        Serial.println("[SPEED TEST] Homing OK. Arranca barrido de velocidad/aceleracion.");
        Serial.print("[SPEED TEST] velocidad=");
        Serial.print(speedTestLimits.maxSpeed);
        Serial.print(" pasos/seg  aceleracion=");
        Serial.print(speedTestLimits.maxAcceleration);
        Serial.println(" pasos/seg^2");

        state = SPEED_TEST_TO_POINT_A; // bring-up: barrido de velocidad en vez del ciclo de recogida
    }
}

void Robot::updateSpeedTestToPointA()
{
    // Pequeña pausa antes de arrancar cada tramo, para poder observar el
    // resultado del tramo anterior antes de que arranque el siguiente.
    if (speedTestPauseStart_ms == 0)
    {
        speedTestPauseStart_ms = millis();
        return;
    }

    if (millis() - speedTestPauseStart_ms < SPEED_TEST_PAUSE_MS)
    {
        return;
    }

    if (!speedTestMoveIssued)
    {
        if (!goToPositionIK(SPEED_TEST_A_X, SPEED_TEST_A_Y, SPEED_TEST_A_Z, speedTestLimits))
        {
            return; // punto invalido para esta geometria: no se comanda nada, se reintenta el siguiente tick
        }
        speedTestMoveIssued = true;
    }

    if (motor1.targetReached() &&
        motor2.targetReached() &&
        motor3.targetReached())
    {
        speedTestMoveIssued = false;
        speedTestPauseStart_ms = 0;
        //state = SPEED_TEST_TO_POINT_B;
        state = READY; // Que quede en el punto A y termine la secuencia
    }
}

void Robot::updateSpeedTestToPointB()
{
    if (speedTestPauseStart_ms == 0)
    {
        speedTestPauseStart_ms = millis();
        return;
    }

    if (millis() - speedTestPauseStart_ms < SPEED_TEST_PAUSE_MS)
    {
        return;
    }

    if (!speedTestMoveIssued)
    {
        if (!goToPositionIK(SPEED_TEST_B_X, SPEED_TEST_B_Y, SPEED_TEST_B_Z, speedTestLimits))
        {
            return; // punto invalido para esta geometria: no se comanda nada, se reintenta el siguiente tick
        }
        speedTestMoveIssued = true;
    }

    if (motor1.targetReached() &&
        motor2.targetReached() &&
        motor3.targetReached())
    {
        speedTestMoveIssued = false;
        speedTestPauseStart_ms = 0;

        // Escalon de velocidad/aceleracion para el proximo ciclo, topeado
        // al maximo global del sistema (Motors::MAX_SPEED/MAX_ACCELERATION).
        speedTestLimits.maxSpeed += SPEED_TEST_SPEED_STEP;
        if (speedTestLimits.maxSpeed > Motors::MAX_SPEED)
        {
            speedTestLimits.maxSpeed = Motors::MAX_SPEED;
        }

        speedTestLimits.maxAcceleration += SPEED_TEST_ACCEL_STEP;
        if (speedTestLimits.maxAcceleration > Motors::MAX_ACCELERATION)
        {
            speedTestLimits.maxAcceleration = Motors::MAX_ACCELERATION;
        }

        Serial.print("[SPEED TEST] velocidad=");
        Serial.print(speedTestLimits.maxSpeed);
        Serial.print(" pasos/seg  aceleracion=");
        Serial.print(speedTestLimits.maxAcceleration);
        Serial.println(" pasos/seg^2");

        state = SPEED_TEST_TO_POINT_A;
    }
}

void Robot::emergencyStop()
{
    motor1.stop();
    motor2.stop();
    motor3.stop();

    Serial.println("[EMERGENCIA] Parada manual solicitada por teclado.");
    Serial.print("[EMERGENCIA] Ultima velocidad probada: ");
    Serial.print(speedTestLimits.maxSpeed);
    Serial.print(" pasos/seg, aceleracion: ");
    Serial.print(speedTestLimits.maxAcceleration);
    Serial.println(" pasos/seg^2");
    Serial.println("[EMERGENCIA] Presiona 'R' de nuevo para rehomear y reiniciar el barrido.");

    speedTestMoveIssued = false;
    speedTestPauseStart_ms = 0;

    state = ERROR;
}

bool Robot::homingFinished() const
{
    return state == READY;
}

Robot::RobotState Robot::getState() const
{
    return state;
}

void Robot::updateGoZero()
{

        motor1.moveTo(0);
        motor2.moveTo(0);
        motor3.moveTo(0);

      if(motor1.targetReached() &&
        motor2.targetReached() &&
        motor3.targetReached())
        {

        positionMoveIssued = false; // el proximo GO_POSITION tiene que volver a comandar el movimiento
        state = GO_POSITION;
        //state = READY; // Que quede en 0 y termine la secuencia
        }

}

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

void Robot::updateGoPosition()
{
    // Coordenada objetivo del efector (cm), mismo sistema que DeltaKinematics.
    // Ajustar acá el punto de recogida.
    constexpr float TARGET_X = 0.0f;
    constexpr float TARGET_Y = 0.0f;
    constexpr float TARGET_Z = -30.0f;

    // El movimiento se comanda UNA sola vez al entrar al estado (no en cada
    // vuelta de loop): Motors::moveSynchronized calcula velocidad segun la
    // distancia restante, asi que llamarlo repetidas veces reconfiguraria
    // el timer de cada Stepper en cada tick sin necesidad.
    if (!positionMoveIssued)
    {
        if (!goToPositionIK(TARGET_X, TARGET_Y, TARGET_Z))
        {
            return; // punto invalido: no se comanda nada, se reintenta el siguiente tick
        }
        positionMoveIssued = true;
    }

    if(motor1.targetReached() &&
        motor2.targetReached() &&
        motor3.targetReached())
        {
        state = GRAB;
        //state = READY; // Que termine la secuencia en la posicion indicada
        }

}

void Robot::updateGrab()
{
        // Activar la bomba para agarrar el objeto
        pneumatics.grab();
        {
            state = GO_UP;
        }

}

void Robot::updateGoUp()
{
        motor1.moveTo(-1300);
        motor2.moveTo(-1300);
        motor3.moveTo(-1300);

      if(motor1.targetReached() &&
        motor2.targetReached() &&
        motor3.targetReached())
        {
            state = CONVEYOR_RUN;
        }

}

void Robot::updateConveyorRun()
{
        conveyor.begin();
        {
            state = GO_DOWN;
        }

}

void Robot::updateGoDown()
{

        motor1.moveTo(200);
        motor2.moveTo(200);
        motor3.moveTo(200);

      if(motor1.targetReached() &&
        motor2.targetReached() &&
        motor3.targetReached())
        {
            state = RELEASE;
        }

}

void Robot::updateRelease()
{
    // Antes: pneumatics.release(); delay(10000);
    // El delay() congelaba TODO el sistema durante 10s: los motores no
    // podían actualizarse, los encoders no podían leer, y el homing/loop
    // completo quedaba bloqueado. Reemplazado por una espera no bloqueante
    // basada en millis().

    if (releaseWaitStart_ms == 0)
    {
        // Primera vuelta en este estado: soltamos la pieza y arrancamos el conteo
        pneumatics.release();
        releaseWaitStart_ms = millis();
        return;
    }

    if (millis() - releaseWaitStart_ms >= RELEASE_WAIT_MS)
    {
        releaseWaitStart_ms = 0; // listo para la próxima vez que se entre a este estado
        state = GO_ZERO2;
    }
}

void Robot::updateGoZero2()
{
        motor1.moveTo(-500);
        motor2.moveTo(-500);
        motor3.moveTo(-500);

      if(motor1.targetReached() &&
        motor2.targetReached() &&
        motor3.targetReached())
        {
            state = CONVEYOR_STOP;
        }

}

void Robot::updateConveyorStop()
{
    // Antes: delay(3000); conveyor.stop();
    // Mismo problema que en updateRelease(): bloqueaba todo el sistema.

    if (conveyorStopWaitStart_ms == 0)
    {
        conveyorStopWaitStart_ms = millis();
        return;
    }

    if (millis() - conveyorStopWaitStart_ms >= CONVEYOR_STOP_WAIT_MS)
    {
        conveyor.stop();
        conveyorStopWaitStart_ms = 0;
        state = READY;
    }
}