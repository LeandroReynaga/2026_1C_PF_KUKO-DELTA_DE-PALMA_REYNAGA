#include "Pneumatics.h"
#include "Pinout.h"

void Pneumatics::begin()
{
    pinMode(BOMBA, OUTPUT);

    digitalWrite(BOMBA, LOW);

    pumpState = false;

    previousMillis = millis();

}

void Pneumatics::update()
{
    unsigned long currentMillis = millis();

    if(currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;

        pumpState = !pumpState;

        digitalWrite(BOMBA, pumpState);


    }
}

void Pneumatics::grab()
{
    digitalWrite(BOMBA, HIGH);

    pumpState = true;
}

void Pneumatics::release()
{
    digitalWrite(BOMBA, LOW);

    pumpState = false;
}


bool Pneumatics::isActive()
{
    return pumpState;
}

