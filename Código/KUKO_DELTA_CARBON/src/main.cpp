#include <Arduino.h>
#include "Pinout.h"
#include "hardware/Conveyor.h"


Conveyor conveyor(CINTAPWM); // Le pasamos el pin de la cinta transportadora definido en Pinout.h

void setup()
{
    Serial.begin(115200);

    conveyor.begin();
}

void loop()
{
}