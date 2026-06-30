#ifndef ENDSTOPS_H
#define ENDSTOPS_H

#include <Arduino.h>

class Endstops
{
private:

    bool stateMotor1;
    bool stateMotor2;
    bool stateMotor3;

    //
    bool previousStateMotor1 = false;
    uint32_t counterMotor1 = 0;

    bool previousStateMotor2 = false;
    uint32_t counterMotor2 = 0;

    
    bool previousStateMotor3 = false;
    uint32_t counterMotor3 = 0;
    //
    
public:

    void begin();

    bool readMotor1();
    bool readMotor2();
    bool readMotor3();

    bool allPressed();
};

#endif