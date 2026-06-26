#ifndef PNEUMATICS_H
#define PNEUMATICS_H

#include <Arduino.h>

class Pneumatics
{
private:

    bool pumpState;

    unsigned long previousMillis = 0;

    const unsigned long interval = 2000;

public:

    void begin();

    void grab();
    void release();

    bool isActive();

    void update();
};

#endif