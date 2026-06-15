#include <Arduino.h>
#include "Pinout.h"
#include "hardware/Conveyor.h"
#include "hardware/Encoders.h"


Conveyor conveyor(CINTAPWM); // Le pasamos el pin de la cinta transportadora definido en Pinout.h
Encoders encoders;
void setup()
{
    Serial.begin(115200);

    conveyor.begin();
    encoders.begin();
}

void loop(){

    Serial.print("E1: ");
    Serial.print(encoders.leerGrados(4));

    Serial.print(" | E2: ");
    Serial.print(encoders.leerGrados(5));

    Serial.print(" | E3: ");
    Serial.println(encoders.leerGrados(6));

    delay(200);

}