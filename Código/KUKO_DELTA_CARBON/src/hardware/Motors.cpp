#include "Motors.h"
#include "Pinout.h"

Motors::Motors()
    : motor1(AccelStepper::DRIVER, PUL1, DIR1),   // Motor 1 controlado por PUL1 y DIR1
      motor2(AccelStepper::DRIVER, PUL2, DIR2),   // Motor 2 controlado por PUL2 y DIR2
      motor3(AccelStepper::DRIVER, PUL3, DIR3)    // Motor 3 controlado por PUL3 y DIR3
{
}

void Motors::begin()
{
    pinMode(ENA, OUTPUT);        // Configura el pin ENABLE como salida

    habilitar();                 // Habilita los drivers DM556

    motor1.setMinPulseWidth(5); // Evita que el DM556 pierda pulsos demasiado cortos
    motor2.setMinPulseWidth(5); // Establece 5 µs para el motor 2
    motor3.setMinPulseWidth(5); // Establece 5 µs para el motor 3

    motor1.setMaxSpeed(2000);    // Velocidad máxima del motor 1 en pulsos/segundo
    motor1.setAcceleration(1000); // Aceleración del motor 1 en pulsos/segundo²

    motor2.setMaxSpeed(2000);    // Velocidad máxima del motor 2 en pulsos/segundo
    motor2.setAcceleration(100); // Aceleración del motor 2 en pulsos/segundo²

    motor3.setMaxSpeed(2000);    // Velocidad máxima del motor 3 en pulsos/segundo
    motor3.setAcceleration(1000); // Aceleración del motor 3 en pulsos/segundo²

    motor1.setCurrentPosition(0); // Define la posición inicial del motor 1 como 0 pulsos
    motor2.setCurrentPosition(0); // Define la posición inicial del motor 2 como 0 pulsos
    motor3.setCurrentPosition(0); // Define la posición inicial del motor 3 como 0 pulsos
}

void Motors::run()
{
    motor1.run(); // Ejecuta el movimiento del motor 1 si tiene pulsos pendientes
    motor2.run(); // Ejecuta el movimiento del motor 2 si tiene pulsos pendientes
    motor3.run(); // Ejecuta el movimiento del motor 3 si tiene pulsos pendientes
}

void Motors::habilitar()
{
    digitalWrite(ENA, HIGH); // Habilita los drivers; en muchos DM556 el ENABLE es activo en LOW
}

void Motors::deshabilitar()
{
    digitalWrite(ENA, LOW); // Deshabilita los drivers
}

void Motors::moverMotorAPulsos(uint8_t motor, long pulsos)
{
    if (motor == 1) motor1.moveTo(pulsos); // Mueve motor 1 a una posición absoluta
    if (motor == 2) motor2.moveTo(pulsos); // Mueve motor 2 a una posición absoluta
    if (motor == 3) motor3.moveTo(pulsos); // Mueve motor 3 a una posición absoluta
}

void Motors::moverMotorRelativo(uint8_t motor, long pulsos)
{
    if (motor == 1) motor1.move(pulsos); // Mueve motor 1 una cantidad relativa de pulsos
    if (motor == 2) motor2.move(pulsos); // Mueve motor 2 una cantidad relativa de pulsos
    if (motor == 3) motor3.move(pulsos); // Mueve motor 3 una cantidad relativa de pulsos
}

void Motors::moverMotorAGrados(uint8_t motor, float grados)
{
    long pulsos = (grados * PULSOS_POR_VUELTA) / 360.0; // Convierte grados a pulsos

    moverMotorAPulsos(motor, pulsos); // Mueve el motor seleccionado a esa posición
}

long Motors::posicionPulsos(uint8_t motor)
{
    if (motor == 1) return motor1.currentPosition(); // Devuelve posición actual del motor 1 en pulsos
    if (motor == 2) return motor2.currentPosition(); // Devuelve posición actual del motor 2 en pulsos
    if (motor == 3) return motor3.currentPosition(); // Devuelve posición actual del motor 3 en pulsos

    return 0; // Si el motor no existe, devuelve 0
}

float Motors::posicionGrados(uint8_t motor)
{
    return (posicionPulsos(motor) * 360.0) / PULSOS_POR_VUELTA; // Convierte pulsos a grados
}

bool Motors::motorEnMovimiento(uint8_t motor)
{
    if (motor == 1) return motor1.distanceToGo() != 0; // Indica si motor 1 sigue moviéndose
    if (motor == 2) return motor2.distanceToGo() != 0; // Indica si motor 2 sigue moviéndose
    if (motor == 3) return motor3.distanceToGo() != 0; // Indica si motor 3 sigue moviéndose

    return false; // Si el motor no existe, devuelve falso
}