#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Stepper.h"
#include "../hardware/Endstops.h"
#include "Pinout.h"
#include "../hardware/Motors.h"

class Robot
{
public:

    enum RobotState
    {
        IDLE,
        HOMING,
        GO_ZERO,
        GO_POSITION,
        GRAB,
        GO_UP,
        CONVEYOR_RUN,
        GO_DOWN,
        RELEASE,
        GO_ZERO2,
        CONVEYOR_STOP,
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

    bool goToPositionIK(float x, float y, float z, const Motors::MotionLimits &limits = Motors::DEFAULT_LIMITS);

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
    void updateGoZero();
    void updateGoPosition();
    void updateGrab();
    void updateGoUp();
    void updateConveyorRun();
    void updateGoDown();
    void updateRelease();
    void updateGoZero2();
    void updateConveyorStop();

    uint32_t homingSettleStart_ms = 0;
    uint32_t releaseWaitStart_ms      = 0;
    uint32_t conveyorStopWaitStart_ms = 0;
    
    bool positionMoveIssued = false;

    static constexpr long MICROPASOS = 20000;

    //static constexpr float HOME_ANGLE_M1 = -46.08f; //-5;//-46.08f;
    //static constexpr float HOME_ANGLE_M2 = -46.26f; //-3;//-46.26f;
    //static constexpr float HOME_ANGLE_M3 = -46.40f; //-2.5;//-46.44f;

    static constexpr float HOME_ANGLE_M1 = -45.1f;//-44.1f; //-2.35f;
    static constexpr float HOME_ANGLE_M2 = -44.3f;//-43.3f; //-2.35f;
    static constexpr float HOME_ANGLE_M3 = -44.5f;//-46.5f; //-2.35f;

    static long angleToSteps(float angle)
{
    return lround(angle * MICROPASOS / 360.0f);
}


};

#endif