#include <Arduino.h>
#include <AccelStepper.h>

/*
====================================================
PINES
====================================================
*/

#define PIN_ENA 13

#define PIN_STEP1 14
#define PIN_DIR1  25

#define PIN_STEP2 27
#define PIN_DIR2  33

#define PIN_STEP3 26
#define PIN_DIR3  32

/*
====================================================
MOTORES
====================================================
*/

AccelStepper motor1(AccelStepper::DRIVER, PIN_STEP1, PIN_DIR1);
AccelStepper motor2(AccelStepper::DRIVER, PIN_STEP2, PIN_DIR2);
AccelStepper motor3(AccelStepper::DRIVER, PIN_STEP3, PIN_DIR3);

/*
====================================================
PARAMETROS
====================================================
*/

const float velocidadMaxima = 100.0;
const float aceleracion = 200.0;

long posicionObjetivo = 3000;

/*
====================================================
SETUP
====================================================
*/

void setup()
{
    Serial.begin(115200);

    pinMode(PIN_ENA, OUTPUT);

    // DM556 habilitado
    digitalWrite(PIN_ENA, HIGH);

    motor1.setMaxSpeed(velocidadMaxima);
    motor2.setMaxSpeed(velocidadMaxima);
    motor3.setMaxSpeed(velocidadMaxima);

    motor1.setAcceleration(aceleracion);
    motor2.setAcceleration(aceleracion);
    motor3.setAcceleration(aceleracion);

    motor1.moveTo(posicionObjetivo);
    motor2.moveTo(posicionObjetivo);
    motor3.moveTo(posicionObjetivo);

    Serial.println("Prueba de 3 motores iniciada");
}

/*
====================================================
LOOP
====================================================
*/

void loop()
{
    motor1.run();
    motor2.run();
    motor3.run();

    if (
        motor1.distanceToGo() == 0 &&
        motor2.distanceToGo() == 0 &&
        motor3.distanceToGo() == 0
    )
    {
        delay(1000);

        posicionObjetivo = -posicionObjetivo;

        motor1.moveTo(posicionObjetivo);
        motor2.moveTo(posicionObjetivo);
        motor3.moveTo(posicionObjetivo);

        Serial.print("Nueva posicion objetivo: ");
        Serial.println(posicionObjetivo);
    }
}