#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>

// Modo de movimiento (idéntico al original)
enum MotionMode : uint8_t
{
    IDLE,
    CONTINUOUS,
    POSITION
};

class Stepper
{
public:
    // timerIndex: 0, 1 o 2 (uno distinto por motor). El ESP32 tiene 4
    // timers de hardware disponibles (0-3), así que alcanzan para los 3 ejes.
    Stepper(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t timerIndex);

    void begin();

    void enable();
    void disable();
    bool isEnabled() const;

    void setDirection(bool direction);
    void setSpeed(float stepsPerSecond);
    void setAcceleration(float acceleration);

    void moveContinuous(bool direction);
    void moveSteps(long steps);
    void moveTo(long position);
    void stop();

    bool isMoving() const;
    bool targetReached() const;

    long getPosition() const;
    void setPosition(long position);

    // Ya NO genera los pulsos (eso lo hace la interrupción de hardware).
    // Se conserva únicamente por compatibilidad con el Robot.cpp existente
    // (que la sigue llamando en cada vuelta de loop); no hace nada crítico.
    void update();

private:
    static const uint16_t STEP_PULSE_US = 5; // ancho de pulso; ajustar si el DM556 necesita otro valor

    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;
    uint8_t timerIndex;

    // Compartidas entre el loop() (que las lee/escribe) y la ISR (que las
    // escribe). volatile + sección crítica -> acceso seguro entre núcleos.
    volatile bool        enabled;
    volatile bool        direction;
    volatile MotionMode  motionMode;
    volatile long        currentPosition;
    volatile long        targetPosition;

    float speed;
    float acceleration;
    uint32_t stepInterval; // microsegundos entre pasos, calculado a partir de speed

    hw_timer_t *timer;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    void aplicarFrecuenciaTimer();

    // Lógica real de generación de pulso; corre en contexto de interrupción.
    void IRAM_ATTR onTimerTick();

    // Punteros estáticos para que las ISR (funciones libres, requeridas por
    // el API de timers de hardware) encuentren la instancia correspondiente.
    static Stepper *instancias[4];
    static void IRAM_ATTR isrTimer0();
    static void IRAM_ATTR isrTimer1();
    static void IRAM_ATTR isrTimer2();
    static void IRAM_ATTR isrTimer3();
};

#endif