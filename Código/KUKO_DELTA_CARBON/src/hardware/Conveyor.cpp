#include "conveyor.h"

Conveyor::Conveyor(int pwmPin)
{
    pinPWM = pwmPin;
    currentSpeed = 0;
}

void Conveyor::begin()
{
    pinMode(pinPWM, OUTPUT);
    setSpeedPercent(60.0); // Iniciamos la cinta al 60% de su velocidad
}

void Conveyor::setSpeedPercent(float percent)
{
    if(percent < 0) percent = 0;
    if(percent > 100) percent = 100;

    currentSpeed = (percent / 100.0) * 255.0;

    analogWrite(pinPWM, currentSpeed);
}

void Conveyor::stop()
{
    currentSpeed = 0;

    analogWrite(pinPWM,0);
}

int Conveyor::getSpeed()
{
    return currentSpeed;
}