#include "Robot.h"
#include "Pinout.h"
#include "hardware/Pneumatics.h"
#include "hardware/Conveyor.h"
#include "hardware/Encoders.h"
#include "kinematics/DeltaKinematics.h"

Pneumatics pneumatics;
Conveyor conveyor(CINTAPWM);

// Duraciones de espera no bloqueante (antes eran delay(2500), delay(10000) y delay(3000))
static const uint32_t HOMING_SETTLE_WAIT_MS = 2500;
static const uint32_t RELEASE_WAIT_MS        = 10000;
static const uint32_t CONVEYOR_STOP_WAIT_MS  = 3000;

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
}

void Robot::update()
{

    switch(state)
    {
        case HOMING:

            updateHoming();

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


void Robot::testMotor1()
{
    motor1.setSpeed(1200);

    motor1.moveContinuous(false);
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

        motor1.setSpeed(15000);
        motor2.setSpeed(15000);
        motor3.setSpeed(15000);

        state = GO_POSITION; // vamos a la posición objetivo de recogida
    }
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
    constexpr float TARGET_X = 5.0f;
    constexpr float TARGET_Y = -10.0f;
    constexpr float TARGET_Z = -29.0f;

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
        //state = GRAB;
            state = READY; // Que termine la secuencia en la posicion indicada
            
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