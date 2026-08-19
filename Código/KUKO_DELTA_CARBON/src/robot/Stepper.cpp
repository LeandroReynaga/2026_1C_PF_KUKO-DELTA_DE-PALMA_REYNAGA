#include "Stepper.h"
#include <math.h>

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

    cn = 0;
    n = 0;
    stepIndex = 0;
    ticksUntilStep = 0;

    maxSpeed = 500.0f;
    acceleration = 0.0f;
    rampEnabled = false; // sin rampa hasta que se llame setAcceleration()
    c0 = 0;
    cmin = (long)((1000000.0f / maxSpeed) * RAMP_SCALE);
    accelSteps = 0;
    decelStart = 0;

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
    timer = timerBegin(timerIndex, 80, true);

    switch (timerIndex)
    {
        case 0: timerAttachInterrupt(timer, &Stepper::isrTimer0, true); break;
        case 1: timerAttachInterrupt(timer, &Stepper::isrTimer1, true); break;
        case 2: timerAttachInterrupt(timer, &Stepper::isrTimer2, true); break;
        case 3: timerAttachInterrupt(timer, &Stepper::isrTimer3, true); break;
        default: break;
    }

    // El timer corre a un ritmo FIJO y chico (BASE_TICK_US), configurado
    // UNA sola vez aca y nunca mas tocado desde la ISR. El ritmo real de
    // los pasos lo controla ticksUntilStep, no la frecuencia del timer.
    timerAlarmWrite(timer, BASE_TICK_US, true);
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

    long newCmin = (long)((1000000.0f / stepsPerSecond) * RAMP_SCALE);

    portENTER_CRITICAL(&mux);
    maxSpeed = stepsPerSecond;
    cmin = newCmin;
    portEXIT_CRITICAL(&mux);
}

void Stepper::setAcceleration(float stepsPerSecond2)
{
    if (stepsPerSecond2 <= 0.0f)
        return;

    // Formula de Austin (2004): intervalo (en us) del primer paso al
    // arrancar desde parado, dada la aceleracion deseada. Se calcula UNA
    // vez aca (float, fuera de la ISR: seguro) y se guarda escalado.
    long newC0 = (long)(0.676f * sqrtf(2.0f / stepsPerSecond2) * 1000000.0f * RAMP_SCALE);

    portENTER_CRITICAL(&mux);
    acceleration = stepsPerSecond2;
    c0 = newC0;
    rampEnabled = true;
    portEXIT_CRITICAL(&mux);
}

void Stepper::moveContinuous(bool dir)
{
    enable();
    setDirection(dir);

    // Sin objetivo fijo: acelera hasta crucero y se queda ahi (no hay
    // frenado automatico, solo via stop()).
    long steps = rampEnabled
        ? (long)((maxSpeed * maxSpeed) / (2.0f * acceleration))
        : 0;

    portENTER_CRITICAL(&mux);
    accelSteps = steps;
    decelStart = 0x7FFFFFFFL; // "nunca": no hay objetivo, no se frena solo
    n = 0;
    cn = 0;
    stepIndex = 0;
    motionMode = CONTINUOUS;
    computeNextInterval(); // intervalo del primer paso
    ticksUntilStep = cn / (RAMP_SCALE * (long)BASE_TICK_US);
    if (ticksUntilStep < 1) ticksUntilStep = 1;
    portEXIT_CRITICAL(&mux);
}

void Stepper::moveSteps(long steps)
{
    moveTo(currentPosition + steps);
}

// OJO: ESTO SE LLAMA CON EL EJE DETENIDO.
//
// No hay ninguna transicion suave. Se fija el pin de direccion segun el
// destino nuevo y se reinicia la rampa (n = 0, o sea que el tren de pulsos
// vuelve al intervalo de arranque). Si el eje YA VENIA ANDANDO y el destino
// nuevo queda del otro lado, el driver invierte el sentido con el rotor
// girando: eso no lo puede seguir ningun paso a paso y se pierden pasos, sin
// que el robot choque contra nada ni se entere.
//
// Paso de verdad: cuando una pieza entraba justo mientras el brazo volvia a
// home, Robot le cambiaba el destino a mitad de camino y un eje se invertia
// a 36.000 pasos/s. El sintoma era una descalibracion que aparecia "sola"
// cada tantas piezas. Se arreglo en updateGoHomeIdle() esperando a
// enPosicion() antes de salir a buscar la pieza.
//
// Si alguna vez hace falta redirigir el brazo en movimiento de verdad (para
// no perder productividad), no alcanza con llamar aca: hay que frenar
// primero con rampa y recien despues fijar el destino nuevo. stop() TAMPOCO
// sirve para eso, porque corta los pulsos de golpe.
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

    long D = labs(position - actual);
    long steps = rampEnabled
        ? (long)((maxSpeed * maxSpeed) / (2.0f * acceleration))
        : 0;
    if (2 * steps > D) steps = D / 2; // distancia corta: perfil triangular

    portENTER_CRITICAL(&mux);
    accelSteps = steps;
    decelStart = D - steps;
    n = 0;
    cn = 0;
    stepIndex = 0;
    motionMode = POSITION;
    computeNextInterval(); // decide el intervalo del primer paso (arranca en c0)
    ticksUntilStep = cn / (RAMP_SCALE * (long)BASE_TICK_US);
    if (ticksUntilStep < 1) ticksUntilStep = 1;
    portEXIT_CRITICAL(&mux);
}

void Stepper::stop()
{
    portENTER_CRITICAL(&mux);
    motionMode = IDLE;
    n = 0;
    cn = 0;
    stepIndex = 0;
    ticksUntilStep = 0;
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
// de I2C, nada de float/double, nada que llame a timerAlarmWrite ni a
// ninguna otra función del API de timers (esas viven en flash, no
// garantizado en IRAM). Todo lo de acá para abajo es aritmética entera y
// GPIO puro.
// ------------------------------------------------------------------
void IRAM_ATTR Stepper::computeNextInterval()
{
    // Sin aceleracion configurada, no rampea: crucero directo al maximo
    // (comportamiento anterior, por si alguien no llama setAcceleration()).
    // rampEnabled es bool, no float: nada de FPU en esta comparacion.
    if (!rampEnabled)
    {
        cn = cmin;
        n = 0;
        return;
    }

    // El umbral de frenado (decelStart) ya viene precalculado desde
    // moveTo()/moveContinuous() -> aca no hace falta ni sqrt ni division
    // para decidir cuando frenar, solo una comparacion entera.
    if (n > 0 && stepIndex >= decelStart)
    {
        n = -n; // arrancar a frenar (mismo truco que el algoritmo de Austin)
    }

    if (n == 0)
    {
        cn = c0;
        n  = 1;
        return;
    }

    // --- CRUCERO (meseta) ---
    // Una vez que el intervalo llego a cmin, el eje va a velocidad maxima y
    // no hay nada que recalcular. Lo importante es que n NO avanza aca: n
    // vale el largo de la rampa de aceleracion, y es lo que hace que el
    // frenado (n = -n) sea su espejo exacto.
    //
    // Antes n se incrementaba tambien durante la meseta. Con este robot eso
    // nunca se noto porque la meseta era inalcanzable a proposito
    // (Motors::VEL_MAX estaba muy por encima del pico real, ~16.400
    // pasos/s), pero apenas se baja VEL_MAX para limitar los tramos largos
    // el camino se activa y el bug aparece: al invertirlo, n valdria
    // aproximadamente TODOS los pasos recorridos en vez del largo de la
    // rampa, asi que la formula repartiria el frenado en muchos mas pasos
    // de los que quedan. El eje llegaria al destino todavia a media
    // velocidad y los pulsos se cortarian de golpe -- pasos perdidos y un
    // golpe seco, que es justo lo contrario de lo que se busca al limitar
    // la velocidad.
    if (n > 0 && cn <= cmin)
    {
        cn = cmin;

        // Y en la PRIMERA vuelta de la meseta se corrige el punto de
        // frenado con el largo REAL de la rampa. decelStart se precalculo
        // en moveTo() con la formula continua (v^2 / 2a), pero la rampa de
        // Austin es discreta y sale ~2,5 % mas larga, asi que frenando en
        // el punto teorico faltan pasos y el eje llega al destino a ~16 %
        // de la velocidad de crucero. Con esta correccion llega a ~3 %, que
        // es exactamente lo mismo que consigue hoy el perfil triangular.
        //
        // D no se guarda, pero sale de los dos que si: D = decelStart +
        // accelSteps. Es aritmetica entera, se ejecuta una sola vez por
        // movimiento y la condicion se apaga sola (despues accelSteps == n
        // y n queda congelado).
        //
        // SOLO para movimientos con destino. En CONTINUOUS (el barrido del
        // homing) no hay destino ni frenado automatico: decelStart vale
        // 0x7FFFFFFF a proposito, o sea "nunca", y sumarle algo lo DESBORDA
        // y lo deja negativo. Con eso, stepIndex >= decelStart da verdadero
        // enseguida, el eje se pone a frenar, llega a n = 0, rearranca la
        // rampa y vuelve a frenar: el barrido avanzaba oscilando entre 195 y
        // 1000 pasos/s en vez de sostener el crucero, y tardaba 4 veces mas
        // (5,3 s contra 1,3 s, simulado con la aritmetica de esta ISR).
        //
        // Y pasa con CUALQUIER VEL_MAX: el homing llama setSpeed(1000)
        // inmediatamente despues de moveContinuous(), asi que el crucero
        // siempre se alcanza. No es un caso raro que destape bajar la
        // velocidad, es todo homing.
        if (motionMode == POSITION && accelSteps != n)
        {
            decelStart = (decelStart + accelSteps) - n;
            accelSteps = n;
        }

        return;
    }

    cn = cn - (2 * cn) / (4 * n + 1); // formula de Austin, en punto fijo entero
    if (cn < cmin) cn = cmin;
    n++;
}

void IRAM_ATTR Stepper::onTimerTick()
{
    if (!enabled) return;
    if (motionMode == IDLE) return;

    portENTER_CRITICAL_ISR(&mux);

    if (motionMode == POSITION && currentPosition == targetPosition)
    {
        motionMode = IDLE;
        n = 0;
        cn = 0;
        stepIndex = 0;
        ticksUntilStep = 0;
        portEXIT_CRITICAL_ISR(&mux);
        return;
    }

    ticksUntilStep--;
    if (ticksUntilStep > 0)
    {
        // Todavia no toca el proximo paso: no se genera pulso este tick.
        portEXIT_CRITICAL_ISR(&mux);
        return;
    }

    portEXIT_CRITICAL_ISR(&mux);

    // Pulso: alto, esperar el ancho mínimo, bajo.
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(stepPin, LOW);

    portENTER_CRITICAL_ISR(&mux);
    if (direction)
        currentPosition++;
    else
        currentPosition--;
    stepIndex++;

    if (motionMode == POSITION && currentPosition == targetPosition)
    {
        motionMode = IDLE;
        n = 0;
        cn = 0;
        stepIndex = 0;
        ticksUntilStep = 0;
    }
    else
    {
        computeNextInterval(); // decide el intervalo para el SIGUIENTE paso
        ticksUntilStep = cn / (RAMP_SCALE * (long)BASE_TICK_US);
        if (ticksUntilStep < 1) ticksUntilStep = 1;
    }
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR Stepper::isrTimer0() { if (instancias[0]) instancias[0]->onTimerTick(); }
void IRAM_ATTR Stepper::isrTimer1() { if (instancias[1]) instancias[1]->onTimerTick(); }
void IRAM_ATTR Stepper::isrTimer2() { if (instancias[2]) instancias[2]->onTimerTick(); }
void IRAM_ATTR Stepper::isrTimer3() { if (instancias[3]) instancias[3]->onTimerTick(); }