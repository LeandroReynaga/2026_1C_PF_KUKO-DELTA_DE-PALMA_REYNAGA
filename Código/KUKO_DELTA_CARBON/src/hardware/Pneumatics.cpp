#include "Pneumatics.h"
#include "Pinout.h"

void Pneumatics::begin()
{
    pinMode(BOMBA, OUTPUT);

    digitalWrite(BOMBA, LOW);

    state = false;
}

void Pneumatics::grab()
{
    digitalWrite(BOMBA, HIGH);

    state = true;
}

void Pneumatics::release()
{
    digitalWrite(BOMBA, LOW);

    state = false;
}

bool Pneumatics::isActive()
{
    return state;
}