#ifndef MOTORS_H
#define MOTORS_H

#include <AccelStepper.h>

#define PULSOS_POR_VUELTA 20000  // SW5 OFF, SW6 ON, SW7 OFF, SW8 OFF = 10000 pulsos/vuelta

class Motors
{
private:
    AccelStepper motor1;     // Objeto para controlar el motor 1
    AccelStepper motor2;     // Objeto para controlar el motor 2
    AccelStepper motor3;     // Objeto para controlar el motor 3

public:
    Motors();                // Constructor de la clase Motors

    void begin();            // Inicializa motores y drivers
    void run();              // Ejecuta los movimientos pendientes

    void habilitar();        // Habilita los drivers DM556
    void deshabilitar();     // Deshabilita los drivers DM556

    void moverMotorAPulsos(uint8_t motor, long pulsos);   // Mueve un motor a una posición absoluta en pulsos
    void moverMotorRelativo(uint8_t motor, long pulsos);  // Mueve un motor cierta cantidad de pulsos desde su posición actual
    void moverMotorAGrados(uint8_t motor, float grados);  // Mueve un motor a una posición expresada en grados

    long posicionPulsos(uint8_t motor);     // Devuelve la posición teórica actual en pulsos
    float posicionGrados(uint8_t motor);    // Devuelve la posición teórica actual en grados

    bool motorEnMovimiento(uint8_t motor);  // Indica si el motor todavía tiene movimiento pendiente
};

#endif