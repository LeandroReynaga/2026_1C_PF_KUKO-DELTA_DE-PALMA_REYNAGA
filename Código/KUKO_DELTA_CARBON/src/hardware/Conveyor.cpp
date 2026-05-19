#include "conveyor.h"

Conveyor::Conveyor(int pwmPin)
{
    pinPWM = pwmPin;

    pwmFreq = 20000;
    pwmResolution = 8;

    currentSpeed = 0;
}

void Conveyor::begin()
{
    // Nueva API ESP32
    ledcAttach(pinPWM, pwmFreq, pwmResolution);

    // Arranca al 50%
    setSpeedPercent(50.0);
}

void Conveyor::setSpeedPercent(float percent)
{
    if(percent < 0) percent = 0;
    if(percent > 100) percent = 100;

    currentSpeed = (percent / 100.0) * 255.0;

    // Nueva API
    ledcWrite(pinPWM, currentSpeed);
}

void Conveyor::stop()
{
    currentSpeed = 0;

    ledcWrite(pinPWM, 0);
}

int Conveyor::getSpeed()
{
    return currentSpeed;
}