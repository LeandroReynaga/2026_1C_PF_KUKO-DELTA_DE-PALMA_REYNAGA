#include "Endstops.h"
#include "Pinout.h"

void Endstops::begin()
{
    pinMode(FC1, INPUT_PULLUP);
    pinMode(FC2, INPUT_PULLUP);
    pinMode(FC3, INPUT_PULLUP);
}

/*
bool Endstops::readMotor1()
{
    bool state = !digitalRead(FC1);

    Serial.print("FC1 = ");
    Serial.println(state);

    return state;
}
*/

bool Endstops::readMotor1()
{
    bool state = !digitalRead(FC1);

    /*
    // Detectar cuando pasa de desactivado a activado
    if(state && !previousStateMotor1)
    {
        counterMotor1++;
    }

    previousStateMotor1 = state;

    Serial.print("FC1 = ");
    Serial.print(state);
    Serial.print(" | Contador = ");
    Serial.println(counterMotor1);
    */
    return state;
}


bool Endstops::readMotor2()
{
     bool state = !digitalRead(FC2);

    /*
    // Detectar cuando pasa de desactivado a activado
    if(state && !previousStateMotor2)
    {
        counterMotor2++;
    }

    previousStateMotor2 = state;

    Serial.print("FC2 = ");
    Serial.print(state);
    Serial.print(" | Contador = ");
    Serial.println(counterMotor2);
*/
    return state;
}

bool Endstops::readMotor3()
{
    bool state = !digitalRead(FC3);
    /*
    // Detectar cuando pasa de desactivado a activado
    if(state && !previousStateMotor3)
    {
        counterMotor3++;
    }

    previousStateMotor3 = state;

    Serial.print("FC3 = ");
    Serial.print(state);
    Serial.print(" | Contador = ");
    Serial.println(counterMotor3);
*/
    return state;
}


bool Endstops::allPressed()
{
    return(
        readMotor1() &&
        readMotor2() &&
        readMotor3()
    );
}