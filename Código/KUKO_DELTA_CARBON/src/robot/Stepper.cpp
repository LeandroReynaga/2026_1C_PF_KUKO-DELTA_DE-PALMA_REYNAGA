#include "Stepper.h"

Stepper *Stepper::instancias[4] = {nullptr, nullptr, nullptr, nullptr};

Stepper::Stepper(uint8_t stepPin,
                  uint8_t dirPin,
                  uint8_t enablePin,
                  uint8_t timerIndex)
{
    this->stepPin = stepPin;
    this->dirPin = dirPin;
    this->enablePin = enablePin;
    this->timerIndex = timerIndex;

    enabled = false;
    direction = true;

    motionMode = IDLE;

    currentPosition = 0;
    targetPosition = 0;

    speed = 500.0f;
    acceleration = 0.0f;

    stepInterval = 1000000UL / (uint32_t)speed;

    timer = nullptr;

    if (timerIndex < 4)
    {
        instancias[timerIndex] = this;
    }
}

void Stepper::begin()
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);

    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin, LOW);

    // Timer con prescaler 80 sobre el reloj de 80MHz del APB -> 1 tick = 1us.
    // Así stepInterval (calculado en microsegundos, igual que antes) se puede
    // usar directamente como valor de alarma.
    timer = timerBegin(timerIndex, 80, true);

    switch (timerIndex)
    {
        case 0: timerAttachInterrupt(timer, &Stepper::isrTimer0, true); break;
        case 1: timerAttachInterrupt(timer, &Stepper::isrTimer1, true); break;
        case 2: timerAttachInterrupt(timer, &Stepper::isrTimer2, true); break;
        case 3: timerAttachInterrupt(timer, &Stepper::isrTimer3, true); break;
        default: break;
    }

    aplicarFrecuenciaTimer();
    timerAlarmEnable(timer);

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
    if (stepsPerSecond <= 0.0f)
        return;

    speed = stepsPerSecond;
    stepInterval = (uint32_t)(1000000.0f / speed);

    aplicarFrecuenciaTimer();
}

void Stepper::setAcceleration(float acceleration)
{
    this->acceleration = acceleration; // reservado para una futura rampa de velocidad
}

void Stepper::aplicarFrecuenciaTimer()
{
    if (timer == nullptr) return;

    // autoreload = true: la ISR se sigue disparando sola cada stepInterval us
    // sin que el loop() tenga que hacer nada.
    timerAlarmWrite(timer, stepInterval, true);
}

void Stepper::moveContinuous(bool dir)
{
    enable();
    setDirection(dir);

    portENTER_CRITICAL(&mux);
    motionMode = CONTINUOUS;
    portEXIT_CRITICAL(&mux);
}

void Stepper::moveSteps(long steps)
{
    moveTo(currentPosition + steps);
}

void Stepper::moveTo(long position)
{
    portENTER_CRITICAL(&mux);
    targetPosition = position;
    long actual = currentPosition;
    portEXIT_CRITICAL(&mux);

    if (position > actual)
        setDirection(true);
    else if (position < actual)
        setDirection(false);
    else
    {
        portENTER_CRITICAL(&mux);
        motionMode = IDLE;
        portEXIT_CRITICAL(&mux);
        return;
    }

    enable();

    portENTER_CRITICAL(&mux);
    motionMode = POSITION;
    portEXIT_CRITICAL(&mux);
}

void Stepper::stop()
{
    portENTER_CRITICAL(&mux);
    motionMode = IDLE;
    portEXIT_CRITICAL(&mux);
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
    portENTER_CRITICAL(&mux);
    currentPosition = position;
    portEXIT_CRITICAL(&mux);
}

// Ya no hace falta que loop() llame a esto para que el motor se mueva; se
// deja vacía para no romper el Robot.cpp existente, que todavía la invoca.
void Stepper::update()
{
}

// ------------------------------------------------------------------
// A partir de acá corre en contexto de INTERRUPCIÓN. Nada de Serial, nada
// de I2C, nada de float pesado, nada que pueda bloquear.
// ------------------------------------------------------------------
void IRAM_ATTR Stepper::onTimerTick()
{
    if (!enabled) return;
    if (motionMode == IDLE) return;

    portENTER_CRITICAL_ISR(&mux);

    if (motionMode == POSITION && currentPosition == targetPosition)
    {
        motionMode = IDLE;
        portEXIT_CRITICAL_ISR(&mux);
        return;
    }

    portEXIT_CRITICAL_ISR(&mux);

    // Pulso: alto, esperar el ancho mínimo, bajo.
    // El bloqueo es de pocos microsegundos, aceptable dentro de una ISR.
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(stepPin, LOW);

    portENTER_CRITICAL_ISR(&mux);
    if (direction)
        currentPosition++;
    else
        currentPosition--;

    if (motionMode == POSITION && currentPosition == targetPosition)
    {
        motionMode = IDLE;
    }
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR Stepper::isrTimer0() { if (instancias[0]) instancias[0]->onTimerTick(); }
void IRAM_ATTR Stepper::isrTimer1() { if (instancias[1]) instancias[1]->onTimerTick(); }
void IRAM_ATTR Stepper::isrTimer2() { if (instancias[2]) instancias[2]->onTimerTick(); }
void IRAM_ATTR Stepper::isrTimer3() { if (instancias[3]) instancias[3]->onTimerTick(); }