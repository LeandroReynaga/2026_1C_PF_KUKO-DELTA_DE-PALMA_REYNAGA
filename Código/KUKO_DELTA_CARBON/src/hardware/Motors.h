#ifndef MOTORS_H
#define MOTORS_H

#include <AccelStepper.h>

class Motors
{
public:

    Motors();
    
    void begin();

    void enable();
    void disable();

    void run(); 

    void stopAll();

    AccelStepper motor1;
    AccelStepper motor2;
    AccelStepper motor3;
};

#endif