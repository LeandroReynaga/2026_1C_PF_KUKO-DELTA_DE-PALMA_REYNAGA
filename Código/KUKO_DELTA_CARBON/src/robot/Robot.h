#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>

#include "Stepper.h"

#include "../hardware/Endstops.h"

#include "Pinout.h"

class Robot
{
public:

    enum RobotState
    {
        IDLE,
        HOMING,
        READY,
        ERROR
    };

    Robot();

    void begin();

    void update();

    void testMotor1();
    
    void startHoming();

    bool homingFinished() const;

    RobotState getState() const;

private:

    // Motores
    Stepper motor1;
    Stepper motor2;
    Stepper motor3;

    // Finales de carrera
    Endstops endstops;

    // Estado del robot
    RobotState state;

    // Estado de cada eje
    bool axis1Homed;
    bool axis2Homed;
    bool axis3Homed;

    // Rutina privada
    void updateHoming();
};

#endif