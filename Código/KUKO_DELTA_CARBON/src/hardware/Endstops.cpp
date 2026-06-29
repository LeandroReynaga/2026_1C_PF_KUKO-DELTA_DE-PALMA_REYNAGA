#include "Endstops.h"
#include "Pinout.h"

void Endstops::begin()
{
    pinMode(FC1, INPUT_PULLUP);
    pinMode(FC2, INPUT_PULLUP);
    pinMode(FC3, INPUT_PULLUP);
}

bool Endstops::readMotor1()
{
    bool state = !digitalRead(FC1);

    Serial.print("FC1 = ");
    Serial.println(state);

    return state;
}

bool Endstops::readMotor2()
{
    return !digitalRead(FC2);
}

bool Endstops::readMotor3()
{
    return !digitalRead(FC3);
}

bool Endstops::allPressed()
{
    return(
        readMotor1() &&
        readMotor2() &&
        readMotor3()
    );
}