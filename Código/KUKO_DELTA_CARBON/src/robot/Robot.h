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
        SPEED_TEST_TO_POINT_A,
        SPEED_TEST_TO_POINT_B,
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

    // Parada de emergencia manual: detiene los 3 motores donde esten y pasa
    // a ERROR. Pensada para dispararse por teclado (tecla 'R') apenas se
    // note a ojo/oido que un motor perdio pasos durante el barrido de
    // velocidad, ya que todavia no hay deteccion automatica por encoder.
    void emergencyStop();

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
    void updateSpeedTestToPointA();
    void updateSpeedTestToPointB();
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

    // Barrido de velocidad/aceleracion (bring-up de motores): va y vuelve
    // al punto de prueba subiendo speedTestLimits en cada vuelta, hasta
    // que el usuario detecte perdida de pasos a ojo/oido y presione 'R'.
    Motors::MotionLimits speedTestLimits;
    bool speedTestMoveIssued = false;
    uint32_t speedTestPauseStart_ms = 0;

    static constexpr long MICROPASOS = 10000;


    static constexpr float HOME_ANGLE_M1 = -45.1f;
    static constexpr float HOME_ANGLE_M2 = -44.3f;
    static constexpr float HOME_ANGLE_M3 = -44.5f;

    static long angleToSteps(float angle)
{
    return lround(angle * MICROPASOS / 360.0f);
}


};

#endif