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

        Serial.print("Bomba: ");

        if(pumpState)
        {
            Serial.println("ON");
        }
        else
        {
            Serial.println("OFF");
        }
    }
}

/*

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
*/
