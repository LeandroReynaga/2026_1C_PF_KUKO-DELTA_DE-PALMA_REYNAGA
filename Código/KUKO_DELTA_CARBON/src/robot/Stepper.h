#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>

class Stepper
{
public:

    // ===========================
    // Estados del motor
    // ===========================

    enum MotionMode
    {
        IDLE,           // Motor detenido
        CONTINUOUS,     // Movimiento continuo
        POSITION        // Movimiento hacia una posición
    };

    // ===========================
    // Constructor
    // ===========================

    Stepper(uint8_t stepPin,
            uint8_t dirPin,
            uint8_t enablePin);

    // ===========================
    // Inicialización
    // ===========================

    void begin();

    // ===========================
    // Driver
    // ===========================

    void enable();

    void disable();

    bool isEnabled() const;

    // ===========================
    // Configuración
    // ===========================

    void setDirection(bool direction);

    void setSpeed(float stepsPerSecond);

    void setAcceleration(float acceleration);      // Se implementará más adelante

    // ===========================
    // Movimiento
    // ===========================

    void moveContinuous(bool direction);

    void moveSteps(long steps);

    void moveTo(long targetPosition);

    void stop();

    bool isMoving() const;

    bool targetReached() const;

    // ===========================
    // Posición
    // ===========================

    long getPosition() const;

    void setPosition(long position);

    // ===========================
    // Actualización (NO bloqueante)
    // ===========================

    void update();

private:

    // ===========================
    // Pines
    // ===========================

    uint8_t stepPin;

    uint8_t dirPin;

    uint8_t enablePin;

    // ===========================
    // Estado del driver
    // ===========================

    bool enabled;

    bool direction;

    MotionMode motionMode;

    // ===========================
    // Posición
    // ===========================

    volatile long currentPosition;

    volatile long targetPosition;

    // ===========================
    // Velocidad
    // ===========================

    float speed;

    float acceleration;

    uint32_t stepInterval;

    // ===========================
    // Generador de pulsos
    // ===========================

    enum PulseState
    {
        STEP_LOW,
        STEP_HIGH
    };

    PulseState pulseState;

    uint32_t lastStepTime;

    uint32_t pulseStartTime;

    static constexpr uint8_t STEP_PULSE_US = 5;

    // ===========================
    // Funciones privadas
    // ===========================

    void startPulse();

    void finishPulse();
};

#endif