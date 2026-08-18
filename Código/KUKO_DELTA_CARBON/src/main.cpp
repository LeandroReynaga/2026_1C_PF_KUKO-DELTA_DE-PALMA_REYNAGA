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
    // Buffer de transmision grande ANTES de abrir el puerto (despues de
    // Serial.begin() no tiene efecto). Sin esto, un mensaje largo -- el
    // volcado de fallos del comando 'D', por ejemplo -- llena la FIFO del
    // UART y Serial.print() BLOQUEA hasta que se vacie: a 115200 baudios,
    // 2 kB son ~180 ms con el loop entero congelado. Y si en ese rato el
    // brazo se estuvo moviendo, el encoder ve un salto enorme entre lectura
    // y lectura, el filtro de plausibilidad lo rechaza por implausible y el
    // canal se cae. Con el buffer en RAM, el mensaje se encola y se
    // transmite por interrupcion mientras el loop sigue girando.
    Serial.setTxBufferSize(2048);
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

    // Cuenta las vueltas para poder informar el ritmo del loop en la línea
    // de salud. Es el primer síntoma de que algo empezó a bloquear: si las
    // vueltas por segundo se desploman, hay un delay(), un Serial.print()
    // que llenó la FIFO o un encoder reintentando lecturas.
    robot.contarVuelta();

    uint32_t ahora = millis();
    if (ahora - ultimoPrint_ms >= INTERVALO_PRINT_MS)
    {
        ultimoPrint_ms = ahora;

    }

}