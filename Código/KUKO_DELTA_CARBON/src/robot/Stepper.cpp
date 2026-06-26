#include "Stepper.h"

Stepper::Stepper(uint8_t stepPin,
                 uint8_t dirPin,
                 uint8_t enablePin)
{
    this->stepPin = stepPin;
    this->dirPin = dirPin;
    this->enablePin = enablePin;

    enabled = false;
    direction = true;

    motionMode = IDLE;

    currentPosition = 0;
    targetPosition = 0;

    speed = 500.0f;
    acceleration = 0.0f;

    stepInterval = 1000000UL / speed;

    pulseState = STEP_LOW;

    lastStepTime = 0;
    pulseStartTime = 0;
}

void Stepper::begin()
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);

    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin, LOW);

    enable();
}

void Stepper::enable()
{
    digitalWrite(enablePin, HIGH);
    enabled = true;
}

void Stepper::disable()
{
    digitalWrite(enablePin, LOW);

    enabled = false;
    motionMode = IDLE;
}

bool Stepper::isEnabled() const
{
    return enabled;
}

void Stepper::setDirection(bool direction)
{
    this->direction = direction;

    digitalWrite(dirPin, direction);
}

void Stepper::setSpeed(float stepsPerSecond)
{
    if(stepsPerSecond <= 0.0f)
        return;

    speed = stepsPerSecond;

    stepInterval = (uint32_t)(1000000.0f / speed);
}

void Stepper::setAcceleration(float acceleration)
{
    this->acceleration = acceleration;
}

void Stepper::moveContinuous(bool direction)
{
    enable();

    setDirection(direction);

    motionMode = CONTINUOUS;
}

void Stepper::moveSteps(long steps)
{
    moveTo(currentPosition + steps);
}

void Stepper::moveTo(long position)
{
    targetPosition = position;

    if(targetPosition > currentPosition)
        setDirection(true);

    else if(targetPosition < currentPosition)
        setDirection(false);

    else
    {
        motionMode = IDLE;
        return;
    }

    enable();

    motionMode = POSITION;
}

void Stepper::stop()
{
    motionMode = IDLE;
    pulseState = STEP_LOW;
}

bool Stepper::isMoving() const
{
    return motionMode != IDLE;
}

bool Stepper::targetReached() const
{
    return currentPosition == targetPosition;
}

long Stepper::getPosition() const
{
    return currentPosition;
}

void Stepper::setPosition(long position)
{
    currentPosition = position;
}

void Stepper::update()
{
    if(!enabled)
        return;

    if(motionMode == IDLE)
        return;
        
    uint32_t now = micros();

    switch(pulseState)
    {
        case STEP_LOW:

            if(now - lastStepTime >= stepInterval)
            {
                if(motionMode == POSITION)
                {
                    if(currentPosition == targetPosition)
                    {
                        motionMode = IDLE;
                        return;
                    }
                }

                startPulse();
            }

            break;

        case STEP_HIGH:

            if(now - pulseStartTime >= STEP_PULSE_US)
            {
                finishPulse();
            }

            break;
    }
}

void Stepper::startPulse()
{
    digitalWrite(stepPin, HIGH);

    pulseState = STEP_HIGH;

    pulseStartTime = micros();
}

void Stepper::finishPulse()
{
    digitalWrite(stepPin, LOW);

    pulseState = STEP_LOW;

    lastStepTime = micros();

    if(direction)
        currentPosition++;
    else
        currentPosition--;

    if(motionMode == POSITION)
    {
        if(currentPosition == targetPosition)
            motionMode = IDLE;
    }
}