#ifndef STEPPER_ISR_H
#define STEPPER_ISR_H

#include <Arduino.h>
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

class StepperISR
{
public:

    enum MotionMode
    {
        IDLE,
        CONTINUOUS,
        POSITION
    };

    StepperISR(uint8_t stepPin,
               uint8_t dirPin,
               uint8_t enablePin);

    void begin();

    void enable();
    void disable();

    bool isEnabled() const;

    void setSpeed(float stepsPerSecond);
    void setAcceleration(float acceleration);

    void setDirection(bool direction);

    void moveContinuous(bool direction);

    void moveSteps(long steps);

    void moveTo(long position);

    void stop();

    bool isMoving() const;

    bool targetReached() const;

    long getPosition() const;

    void setPosition(long position);

    //--------------------------

    static void beginTimer();

    static void endTimer();

private:

    friend void IRAM_ATTR onStepperTimer();

    void IRAM_ATTR updateISR();


    //---------------- GPIO ----------------

    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;

    bool stepHighGPIO;
    bool dirHighGPIO;
    bool enaHighGPIO;

    uint32_t stepMask;
    uint32_t dirMask;
    uint32_t enaMask;

    //---------------- Estado ----------------

    volatile bool enabled;

    volatile bool direction;

    volatile MotionMode motionMode;

    volatile long currentPosition;

    volatile long targetPosition;

    //---------------- Movimiento ----------------

    volatile float speed;

    volatile float acceleration;

    // cantidad de ticks del timer entre pasos
    volatile uint32_t ticksPerStep;

    volatile uint32_t tickCounter;

    //---------------- Pulso STEP ----------------

    enum PulseState
    {
        STEP_IDLE,
        STEP_HIGH
    };

    volatile PulseState pulseState;

    volatile uint8_t pulseWidthTicks;

    //---------------- Registro ----------------

    static constexpr uint8_t MAX_STEPPERS = 6;

    static StepperISR* steppers[MAX_STEPPERS];

    static volatile uint8_t stepperCount;

    //---------------- Timer ----------------

    static hw_timer_t* timer;

    static portMUX_TYPE timerMux;
};

#endif