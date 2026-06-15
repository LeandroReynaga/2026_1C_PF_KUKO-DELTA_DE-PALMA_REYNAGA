#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>
#include <Wire.h>

class Encoders
{
private:

    static const uint8_t TCA_ADDR = 0x70;
    static const uint8_t AS5600_ADDR = 0x36;

    static const uint8_t AS5600_ANGLE_H = 0x0E;
    static const uint8_t AS5600_ANGLE_L = 0x0F;

    void seleccionarCanal(uint8_t canal);

public:

    void begin();

    uint16_t leerRaw(uint8_t canal);

    float leerGrados(uint8_t canal);
};

#endif