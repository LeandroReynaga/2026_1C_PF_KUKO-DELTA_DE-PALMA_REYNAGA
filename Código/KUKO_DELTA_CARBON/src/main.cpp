#include <Arduino.h>
#include "Pinout.h"
#include "hardware/Conveyor.h"

// Crear objeto cint
Conveyor conveyor(CINTAPWM);

void setup()
{
    Serial.begin(115200);

    conveyor.begin();
}

void loop()
{
}