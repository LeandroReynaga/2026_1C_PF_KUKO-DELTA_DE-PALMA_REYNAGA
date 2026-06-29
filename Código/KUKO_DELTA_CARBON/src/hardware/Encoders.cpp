#include "Encoders.h"

#define SDA_PIN 21
#define SCL_PIN 22

void Encoders::begin()
{
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(30000);
} 

void Encoders::seleccionarCanal(uint8_t canal)
{
    if(canal > 7)
    {
        return;  
    }

    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << canal);
    Wire.endTransmission();
}

uint16_t Encoders::leerRaw(uint8_t canal)
{
    seleccionarCanal(canal);

    delayMicroseconds(300);

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_ANGLE_H);

    if(Wire.endTransmission(false) != 0)
    {
        return 0xFFFF;
    }

    Wire.requestFrom(AS5600_ADDR, (uint8_t)2);

    if(Wire.available() < 2)
    {
        return 0xFFFF;
    }

    uint8_t highByte = Wire.read();
    uint8_t lowByte  = Wire.read();

    return ((highByte & 0x0F) << 8) | lowByte;
}

float Encoders::leerGrados(uint8_t canal)
{
    uint16_t raw = leerRaw(canal);

    if(raw == 0xFFFF)
    {
        return -1.0;
    }

    return (raw * 360.0f) / 4096.0f;
}