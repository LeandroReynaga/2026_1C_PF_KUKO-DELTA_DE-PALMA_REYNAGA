#ifndef PNEUMATICS_H
#define PNEUMATICS_H

#include <Arduino.h>

class Pneumatics
{
private:

    bool state;

public:

    void begin();

    void grab();
    void release();

    bool isActive();
};

#endif