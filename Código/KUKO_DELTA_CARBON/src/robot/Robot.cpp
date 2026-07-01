#include "Robot.h"
#include "Pinout.h"
#include "hardware/Pneumatics.h"
#include "hardware/Conveyor.h"

Pneumatics pneumatics;
Conveyor conveyor(CINTAPWM);
Robot::Robot() :

motor1(PUL1, DIR1, ENA),
motor2(PUL2, DIR2, ENA),
motor3(PUL3, DIR3, ENA)

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
        state = GO_ZERO;
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

            state = GO_POSITION;
        }

}

void Robot::updateGoPosition()
{

        motor1.moveTo(350); //cuanto mas bajo el valor, mas arriba va a estar el brazo
        motor2.moveTo(350);
        motor3.moveTo(350);

      if(motor1.targetReached() &&
        motor2.targetReached() &&
        motor3.targetReached())
        {
            state = GRAB;
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
        motor1.moveTo(-1000);
        motor2.moveTo(-1000);
        motor3.moveTo(-1000);

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
        pneumatics.release();
        delay(10000); //delay 10 segundos
        {
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
        delay(3000); //delay 5 segundos    
        conveyor.stop();

        {
            state = READY;
        }

}
