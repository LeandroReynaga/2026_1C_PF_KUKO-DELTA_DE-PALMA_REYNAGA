#include "Robot.h"

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
    motor1.begin();
    motor2.begin();
    motor3.begin();

    motor1.setSpeed(1200);
    motor2.setSpeed(1200);
    motor3.setSpeed(1200);

    endstops.begin();
}

void Robot::update()
{
    motor1.update();
    motor2.update();
    motor3.update();

    switch(state)
    {
        case HOMING:

            updateHoming();

            break;

        default:

            break;
    }
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

            motor1.setPosition(0);

            axis1Homed = true;
        }
    }

    // Motor 2

    if(!axis2Homed)
    {
        if(endstops.readMotor2())
        {
            motor2.stop();

            motor2.setPosition(0);

            axis2Homed = true;
        }
    }

    // Motor 3

    if(!axis3Homed)
    {
        if(endstops.readMotor3())
        {
            motor3.stop();

            motor3.setPosition(0);

            axis3Homed = true;
        }
    }

    // ¿Todos llegaron?

    if(axis1Homed &&
       axis2Homed &&
       axis3Homed
       )
    {
        state = READY;
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