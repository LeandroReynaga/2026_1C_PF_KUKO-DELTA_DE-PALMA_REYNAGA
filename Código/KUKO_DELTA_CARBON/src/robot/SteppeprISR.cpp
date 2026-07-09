#include "StepperISR.h"

//==========================================================
// VARIABLES ESTÁTICAS
//==========================================================

StepperISR* StepperISR::steppers[MAX_STEPPERS];

volatile uint8_t StepperISR::stepperCount = 0;

hw_timer_t* StepperISR::timer = nullptr;

portMUX_TYPE StepperISR::timerMux = portMUX_INITIALIZER_UNLOCKED;

//==========================================================
// ISR GLOBAL
//==========================================================

void IRAM_ATTR onStepperTimer()
{
    for(uint8_t i = 0; i < StepperISR::stepperCount; i++)
    {
        StepperISR::steppers[i]->updateISR();
    }
}

//==========================================================
// CONSTRUCTOR
//==========================================================

StepperISR::StepperISR(uint8_t stepPin,
                       uint8_t dirPin,
                       uint8_t enablePin)
{
    this->stepPin = stepPin;
    this->dirPin = dirPin;
    this->enablePin = enablePin;

    stepHighGPIO = (stepPin >= 32);
    dirHighGPIO  = (dirPin  >= 32);
    enaHighGPIO  = (enablePin >= 32);

    stepMask = 1UL << (stepPin & 31);
    dirMask  = 1UL << (dirPin  & 31);
    enaMask  = 1UL << (enablePin & 31);

    enabled = false;
    direction = true;

    motionMode = IDLE;

    currentPosition = 0;
    targetPosition = 0;

    speed = 1000.0f;
    acceleration = 0;

    tickCounter = 0;

    ticksPerStep = 100;

    pulseState = STEP_IDLE;

    pulseWidthTicks = 0;
}

inline void IRAM_ATTR gpioSet(uint32_t mask, bool highGPIO)
{
    if(highGPIO)
        GPIO.out1_w1ts.val = mask;
    else
        GPIO.out_w1ts = mask;
}

inline void IRAM_ATTR gpioClear(uint32_t mask, bool highGPIO)
{
    if(highGPIO)
        GPIO.out1_w1tc.val = mask;
    else
        GPIO.out_w1tc = mask;
}

//==========================================================
// BEGIN
//==========================================================

void StepperISR::begin()
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);

    GPIO.out_w1tc = stepMask;
    GPIO.out_w1tc = dirMask;
    GPIO.out_w1tc = enaMask;

    if(stepperCount < MAX_STEPPERS)
    {
        steppers[stepperCount++] = this;
    }

    enable();
}

//==========================================================

void StepperISR::enable()
{
    GPIO.out_w1ts = enaMask;

    enabled = true;
}

//==========================================================

void StepperISR::disable()
{
    GPIO.out_w1tc = enaMask;

    enabled = false;

    motionMode = IDLE;
}

//==========================================================

bool StepperISR::isEnabled() const
{
    return enabled;
}

//==========================================================
// TIMER
//==========================================================

void StepperISR::beginTimer()
{
    if(timer != nullptr)
        return;

    // Timer a 1 MHz
    // 80 MHz / 80 = 1 MHz
    timer = timerBegin(0, 80, true);

    timerAttachInterrupt(timer, &onStepperTimer, true);

    // Interrupción cada 5 us
    timerAlarmWrite(timer, 5, true);

    timerAlarmEnable(timer);
}

//==========================================================

void StepperISR::endTimer()
{
    if(timer == nullptr)
        return;

    timerAlarmDisable(timer);

    timerDetachInterrupt(timer);

    timerEnd(timer);

    timer = nullptr;
}

//==========================================================
// VELOCIDAD
//==========================================================

void StepperISR::setSpeed(float stepsPerSecond)
{
    if(stepsPerSecond <= 0.0f)
        return;

    portENTER_CRITICAL(&timerMux);

    speed = stepsPerSecond;

    /*
        Timer = 5 us

        ticksPerStep =
        periodo_paso / periodo_timer

        ejemplo

        1000 pasos/s

        periodo = 1000 us

        ticks = 1000/5 = 200
    */

    ticksPerStep =
        (uint32_t)((1000000.0f / speed) / 5.0f);

    if(ticksPerStep == 0)
        ticksPerStep = 1;

    portEXIT_CRITICAL(&timerMux);
}

//==========================================================

void StepperISR::setAcceleration(float acceleration)
{
    this->acceleration = acceleration;
}

//==========================================================

void StepperISR::setDirection(bool dir)
{
    portENTER_CRITICAL(&timerMux);

    direction = dir;

    if(dir)
        GPIO.out_w1ts = dirMask;
    else
        GPIO.out_w1tc = dirMask;

    portEXIT_CRITICAL(&timerMux);
}