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
    void setSpeed(float stepsPerSecond);         // techo de velocidad (velocidad de crucero)
    void setAcceleration(float stepsPerSecond2); // pasos/seg^2, define que tan rapido rampea

    void moveContinuous(bool direction);
    void moveSteps(long steps);
    void moveTo(long position);
    void stop();

    // ------------------------------------------------------------------
    //  Encadenado de tramos (redirigir SIN soltar la rampa)
    // ------------------------------------------------------------------
    // moveTo() reinicia la rampa (n = 0, cn = 0): el eje arranca desde el
    // intervalo de partida, o sea desde parado. Encadenando tramos con eso,
    // cada punto intermedio es una frenada entera mas un arranque, y una
    // trayectoria de veinte puntos son veinte frenadas -- que con la
    // aceleracion alta es exactamente el traqueteo que se siente en el brazo.
    //
    // redirigir() cambia el destino CONSERVANDO cn, o sea la velocidad
    // instantanea: el tren de pulsos no se interrumpe ni pega un salto. Lo
    // unico que recalcula es en que paso hay que empezar a frenar para el
    // destino nuevo.
    //
    // Solo se puede si el eje va en el mismo sentido que el destino nuevo y
    // si lo que queda alcanza para frenar desde la velocidad actual. Cuando
    // no se puede devuelve false SIN TOCAR NADA, y quien llama vuelve al
    // camino de siempre (esperar a que llegue y hacer moveTo). Nunca deja el
    // eje a medio configurar.
    //
    // Ojo con la aceleracion: el largo de la rampa depende de ella, asi que
    // al cambiarla hay que volver a derivar n de la velocidad real (la que
    // dice cn). Eso se hace aca, con float y FUERA de la interrupcion.
    bool puedeRedirigir(long position, float acceleration) const;
    bool redirigir(long position, float maxSpeed, float acceleration);

    // Pasos que faltan para el destino actual. Es con lo que se decide
    // cuando arrancar la mezcla de una esquina.
    long pasosRestantes() const;

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

    // ------------------------------------------------------------------
    // Rampa trapezoidal (algoritmo de D. Austin 2004 / el mismo que usa
    // AccelStepper), pero en punto fijo ENTERO -- CERO floats dentro de la
    // interrupcion. Motivo: el ESP32/Arduino no garantiza guardar el
    // contexto de la FPU al entrar a una ISR de timer; usar float ahi
    // adentro corrompe registros y crashea (LoadProhibited), como paso.
    //
    // "cn" se guarda escalado (cn_real_us = cn / RAMP_SCALE) para que la
    // division entera de la formula de Austin no trunque a 0 antes de
    // tiempo y la rampa se quede "pegada" sin llegar a la velocidad real.
    //
    // Los limites de la rampa (accelSteps/decelStart) SI se calculan con
    // float (sqrt, division) pero solo en moveTo()/moveContinuous(),
    // fuera de la ISR -> ahi el float es 100% seguro.
    // ------------------------------------------------------------------
    static constexpr long RAMP_SCALE = 256;      // precision extra en punto fijo
    static constexpr uint32_t BASE_TICK_US = 20; // ritmo fijo del timer de hardware

    volatile long cn;             // intervalo actual, ESCALADO por RAMP_SCALE
    volatile long n;              // indice de paso en la rampa (+ acelerando, - frenando)
    volatile long stepIndex;      // pasos dados en el movimiento actual (0-based, cuenta hacia adelante)
    volatile long ticksUntilStep; // cuenta regresiva de ticks base hasta el proximo paso

    long c0;          // intervalo inicial ESCALADO (arranque desde parado)
    long cmin;        // intervalo minimo ESCALADO (velocidad crucero)
    long accelSteps;  // pasos de aceleracion calculados para el movimiento actual
    long decelStart;  // stepIndex (pasos ya dados) a partir del cual hay que frenar

    volatile bool rampEnabled; // true si hay aceleracion configurada (evita comparar
                                // el float 'acceleration' dentro de la ISR)

    float maxSpeed;     // techo configurado con setSpeed() (pasos/seg)
    float acceleration; // configurado con setAcceleration() (pasos/seg^2)

    hw_timer_t *timer;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    // Cuantos pasos cuesta frenar desde donde esta el eje ahora, si se
    // pasara a usar `acelNueva`.
    //
    // NO se calcula con v^2/(2a): esa formula es la del movimiento continuo
    // y la rampa de Austin en punto fijo se aparta bastante de ella --
    // medido con la aritmetica de esta misma ISR, hasta un 190 % de mas en
    // rampas largas, porque la division entera de la recurrencia trunca cada
    // vez mas a medida que n crece. Usarla dejaba al eje llegando al destino
    // a 7.600 pasos/s en el peor caso, o sea con los pulsos cortados de
    // golpe y pasos perdidos.
    //
    // El dato exacto ya lo lleva la rampa: `n` baja de a uno por paso
    // mientras frena, asi que frenar cuesta exactamente |n| pasos. Lo unico
    // que hay que corregir es el cambio de aceleracion, y eso si escala
    // limpio (la distancia de frenado va con 1/a).
    long pasosDeFrenado(float acelNueva) const;

    // Margen sobre ese calculo. Pasarse es inofensivo -- el eje frena un
    // poco antes y llega mas despacio --; quedarse corto es perder pasos.
    static constexpr float MARGEN_FRENADO = 1.10f;
    static const long      FRENADO_EXTRA  = 4;

    // Decide el intervalo (cn) para el PROXIMO paso. Solo aritmetica
    // entera -> segura para llamar desde la ISR. También se usa para
    // "sembrar" el primer intervalo desde moveTo()/moveContinuous().
    void IRAM_ATTR computeNextInterval();

    // Genera el pulso y avanza la posición; corre en contexto de interrupción.
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