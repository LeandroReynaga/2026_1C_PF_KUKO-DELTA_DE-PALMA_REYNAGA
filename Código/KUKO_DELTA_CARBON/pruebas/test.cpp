// Entorno para pruebas de la clase KUKO_DELTA_CARBON
// Sirve para probar códigos que difieran del main principal.
// Ayuda a no modificar el main y hacer pruebas de manera más rápida y sencilla.
//
// Se compila y flashea con:  pio run -e test -t upload
// Ese entorno cambia el filtro de fuentes y compila SOLO este archivo, así
// que nada de src/ entra acá: lo que se quiera probar se escribe abajo.
//
// setup() y loop() tienen que existir aunque estén vacíos: el framework
// Arduino los llama desde su propio main(), y sin ellos no linkea.

#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
}

void loop()
{
}
