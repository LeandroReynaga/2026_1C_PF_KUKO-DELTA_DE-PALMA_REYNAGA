#ifndef CONVEYOR_H
#define CONVEYOR_H

class Conveyor
{
private:

    int pinPWM; // Pin para la cinta transportadora

    int pwmChannel;
    int pwmFreq;
    int pwmResolution;

    int currentSpeed; // 0-255

public:

    Conveyor(int pwmPin);

    void begin();

    void setSpeedPercent(float percent);

    void stop();

    int getSpeed();
};

#endif