#ifndef ENDSTOPS_H
#define ENDSTOPS_H

#include <Arduino.h>

class Endstops
{
private:

    bool stateMotor1;
    bool stateMotor2;
    bool stateMotor3;

public:

    void begin();

    bool readMotor1();
    bool readMotor2();
    bool readMotor3();

    bool allPressed();
};

#endif