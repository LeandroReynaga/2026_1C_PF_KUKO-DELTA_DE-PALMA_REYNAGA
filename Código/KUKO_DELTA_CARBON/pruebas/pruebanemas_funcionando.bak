#include <Arduino.h>
#include "Pinout.h"
#include "hardware/Conveyor.h"
#include "hardware/Encoders.h"
#include "hardware/Pneumatics.h"
#include "hardware/Endstops.h"
#include "hardware/Motors.h"

Conveyor conveyor(CINTAPWM); // Objeto encargado de controlar la cinta transportadora
Encoders encoders;           // Objeto encargado de leer los encoders AS5600
Pneumatics pneumatics;       // Objeto encargado de controlar la bomba neumática
Endstops endstops;           // Objeto encargado de leer los finales de carrera
Motors motors;               // Objeto encargado de controlar los tres motores

bool movimientoPendiente = false; // Indica si debe mostrarse el resultado final del movimiento

// Muestra una sola vez la posición teórica actual del motor 1
void mostrarEstadoMotor1()
{
    Serial.print("Motor 1 | Pulsos: ");              // Texto identificatorio
    Serial.print(motors.posicionPulsos(1));          // Posición teórica en pulsos

    Serial.print(" | Grados teoricos: ");            // Texto identificatorio
    Serial.print(motors.posicionGrados(1), 2);       // Posición teórica en grados

    Serial.print(" | En movimiento: ");              // Texto identificatorio
    Serial.println(
        motors.motorEnMovimiento(1) ? "SI" : "NO"); // Indica si todavía se mueve
}


void setup()
{
    Serial.begin(115200); // Inicializa la comunicación con el monitor serie

    // conveyor.begin();
    // encoders.begin();
    // pneumatics.begin();
    // endstops.begin();

    motors.begin(); // Inicializa los motores y habilita los drivers DM556

    Serial.println();
    Serial.println("Prueba Motor 1 - DM556 + AccelStepper");
    Serial.println("Configuracion actual del DM556:");
    Serial.println("SW1 ON, SW2 ON, SW3 ON, SW4 OFF");
    Serial.println("SW5 OFF, SW6 ON, SW7 OFF, SW8 OFF");
    Serial.println("Resolucion: 10000 pulsos/vuelta");
    Serial.println();
    Serial.println("Comandos disponibles:");
    Serial.println("p 10000 -> mover Motor 1 a la posicion absoluta 10000");
    Serial.println("r 1000  -> mover Motor 1 1000 pulsos desde su posicion actual");
    Serial.println("g 90    -> mover Motor 1 a la posicion teorica de 90 grados");
    Serial.println("s       -> mostrar una vez la posicion actual");
    Serial.println();

    mostrarEstadoMotor1(); // Muestra una sola vez la posición inicial

}

void loop()
{
     motors.run(); // Debe ejecutarse continuamente para generar los pulsos STEP

    if (Serial.available() > 0) // Comprueba si se recibió un comando
    {
        char comando = Serial.read(); // Lee la primera letra enviada

        if (comando == 'p') // Movimiento absoluto expresado en pulsos
        {
            long pulsos = Serial.parseInt();        // Lee el número ingresado
            motors.moverMotorAPulsos(1, pulsos);    // Define el destino absoluto
            movimientoPendiente = true;             // Espera informar el final

            Serial.print("Movimiento absoluto solicitado: ");
            Serial.print(pulsos);
            Serial.println(" pulsos");
        }
        else if (comando == 'r') // Movimiento relativo desde la posición actual
        {
            long pulsos = Serial.parseInt();        // Lee la cantidad de pulsos
            motors.moverMotorRelativo(1, pulsos);   // Ordena el movimiento relativo
            movimientoPendiente = true;             // Espera informar el final

            Serial.print("Movimiento relativo solicitado: ");
            Serial.print(pulsos);
            Serial.println(" pulsos");
        }
        else if (comando == 'g') // Movimiento absoluto expresado en grados
        {
            float grados = Serial.parseFloat();     // Lee el valor en grados
            motors.moverMotorAGrados(1, grados);    // Convierte a pulsos y mueve
            movimientoPendiente = true;             // Espera informar el final

            Serial.print("Movimiento solicitado: ");
            Serial.print(grados, 2);
            Serial.println(" grados");
        }
        else if (comando == 's') // Solicita mostrar el estado actual
        {
            mostrarEstadoMotor1(); // Imprime la posición una sola vez
        }
    }

    // Cuando el motor termina, informa una sola vez su posición final
    if (movimientoPendiente && !motors.motorEnMovimiento(1))
    {
        movimientoPendiente = false; // Evita repetir continuamente el mensaje

        Serial.println("Movimiento finalizado.");
        mostrarEstadoMotor1();        // Muestra la posición final una sola vez
        Serial.println("Ingrese un nuevo comando:");
    }


    /*
    Serial.print("FC1: ");
    Serial.print(endstops.readMotor1());

    Serial.print(" | FC2: ");
    Serial.print(endstops.readMotor2());

    Serial.print(" | FC3: ");
    Serial.print(endstops.readMotor3());

    Serial.print(" | ALL: ");
    Serial.println(endstops.allPressed());

    delay(200);
    */

    /*
    //la bomba se activa durante 2 segundos y luego se libera
    pneumatics.grab();
    delay(2000);
    pneumatics.release();
    delay(2000);
    */

    /*
    Serial.print("E1: ");
    Serial.print(encoders.leerGrados(4));

    Serial.print(" | E2: ");
    Serial.print(encoders.leerGrados(5));

    Serial.print(" | E3: ");
    Serial.println(encoders.leerGrados(6));
*/

}