#include "Motors.h"
#include "Pinout.h"

Motors::Motors() :
    motor1(AccelStepper::DRIVER, PUL1, DIR1),
    motor2(AccelStepper::DRIVER, PUL2, DIR2),
    motor3(AccelStepper::DRIVER, PUL3, DIR3)
{
}

void Motors::begin()
{
    pinMode(ENA, OUTPUT);

    disable();

    motor1.setMaxSpeed(50.0);
    motor2.setMaxSpeed(50.0);
    motor3.setMaxSpeed(50.0);

    motor1.setAcceleration(20.0);
    motor2.setAcceleration(20.0);
    motor3.setAcceleration(20.0);
}

void Motors::enable()
{
    digitalWrite(ENA, LOW);
}

void Motors::disable()
{
    digitalWrite(ENA, HIGH);
}

void Motors::run()
{
    motor1.run();
    motor2.run();
    motor3.run();
}

void Motors::stopAll()
{
    motor1.stop();
    motor2.stop();
    motor3.stop();
}