#include <Arduino.h>
#include "Pinout.h"
#include "hardware/Conveyor.h"
#include "hardware/Encoders.h"
#include "hardware/Pneumatics.h"
#include "hardware/Endstops.h"
#include "hardware/Motors.h"
#include "robot/Robot.h"
#include "kinematics/DeltaKinematics.h"

Robot robot;
Endstops endstops;

// Impresión de diagnóstico no bloqueante: se muestra a intervalos fijos
// en lugar de en cada vuelta del loop (que sería miles de veces por
// segundo, ilegible y con costo real de tiempo de CPU/UART).
uint32_t ultimoPrint_ms = 0;
const uint32_t INTERVALO_PRINT_MS = 200; // 5 Hz, suficiente para ver la evolución en vivo


void setup()
{
    Serial.begin(115200);

    encoders.begin();
    endstops.begin();
    robot.begin();
    robot.startHoming();
}

void loop()
{
    // Debe llamarse SIEMPRE, en cada vuelta del loop: es lo que hace avanzar
    // la máquina de estados no bloqueante de los encoders.
    encoders.update();

    robot.update();

    uint32_t ahora = millis();
    if (ahora - ultimoPrint_ms >= INTERVALO_PRINT_MS)
    {
        ultimoPrint_ms = ahora;
        
        Serial.print("M1: ");
        Serial.print(encoders.esValido(0) ? String(encoders.leerGrados(0), 1) : "ERR");

        Serial.print(" | M2: ");
        Serial.print(encoders.esValido(1) ? String(encoders.leerGrados(1), 1) : "ERR");

        Serial.print(" | M3: ");
        Serial.println(encoders.esValido(2) ? String(encoders.leerGrados(2), 1) : "ERR");
        


    }

}