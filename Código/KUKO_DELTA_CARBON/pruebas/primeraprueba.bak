#include <Arduino.h>
#include <AccelStepper.h>

/*
====================================================
CONFIGURACION DE PINES
====================================================
*/

#define PIN_ENA   13
#define PIN_STEP  14
#define PIN_DIR   25

/*
====================================================
CONFIGURACION DEL DRIVER
====================================================

Modo DRIVER:
STEP + DIR

AccelStepper::DRIVER
usa:
- 1 pin STEP
- 1 pin DIR
*/

AccelStepper motor(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

/*
====================================================
VARIABLES DE CONTROL
====================================================
*/

// Velocidad maxima [steps/s]
float velocidadMaxima = 800.0;

// Aceleracion [steps/s²]
float aceleracion = 400.0;

// Movimiento objetivo
long posicionObjetivo = 4000;

// Sentido
bool sentidoHorario = true;

/*
====================================================
SETUP
====================================================
*/

void setup()
{
    Serial.begin(115200);

    /*
    --------------------------------------------
    CONFIGURACION DE PINES
    --------------------------------------------
    */

    pinMode(PIN_ENA, OUTPUT);

    /*
    ENABLE DEL DRIVER
    --------------------------------------------
    En nuestro DM556:
    LOW = deshabilitado
    HIGH = habilitado
    */
    
    digitalWrite(PIN_ENA, HIGH);
    /*
    --------------------------------------------
    CONFIGURACION DEL MOTOR
    --------------------------------------------
    */

    motor.setMaxSpeed(velocidadMaxima);

    motor.setAcceleration(aceleracion);

    /*
    --------------------------------------------
    PRIMER MOVIMIENTO
    --------------------------------------------
    */

    if(sentidoHorario)
    {
        motor.moveTo(posicionObjetivo);
    }
    else
    {
        motor.moveTo(-posicionObjetivo);
    }

    Serial.println("Sistema iniciado");
}

/*
====================================================
LOOP PRINCIPAL
====================================================
*/

void loop()
{
    /*
    run()
    --------------------------------------------
    Debe ejecutarse SIEMPRE.
    Genera los pulsos STEP.
    */

    motor.run();

    /*
    --------------------------------------------
    CUANDO LLEGA AL DESTINO
    invierte sentido
    --------------------------------------------
    */

    if(motor.distanceToGo() == 0)
    {
        delay(1000);

        posicionObjetivo = -posicionObjetivo;

        motor.moveTo(posicionObjetivo);

        Serial.print("Nueva posicion objetivo: ");
        Serial.println(posicionObjetivo);
    }
}