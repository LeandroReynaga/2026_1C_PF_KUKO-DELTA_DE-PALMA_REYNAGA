#include "Motors.h"
#include "robot/Stepper.h"

namespace Motors {

// Valores de fabrica. La tabla de parametros los toma como referencia al
// registrarlos, asi que 'P0' vuelve exactamente aca.
float VEL_MAX    = 12000.0f;
float ACC_SUAVE  = 17000.0f;
float ACC_RAPIDA = 97000.0f;

MotionLimits DEFAULT_LIMITS = {VEL_MAX, ACC_RAPIDA};
MotionLimits FAST_LIMITS    = {VEL_MAX, ACC_RAPIDA};
MotionLimits SOFT_LIMITS    = {VEL_MAX, ACC_SUAVE};

void aplicarLimites()
{
    DEFAULT_LIMITS = {VEL_MAX, ACC_RAPIDA};
    FAST_LIMITS    = {VEL_MAX, ACC_RAPIDA};
    SOFT_LIMITS    = {VEL_MAX, ACC_SUAVE};
}

bool redirigirSincronizado(Stepper &m1, Stepper &m2, Stepper &m3,
                            long target1, long target2, long target3,
                            const MotionLimits &limits, float pisoEscala)
{
    const long d1 = labs(target1 - m1.getPosition());
    const long d2 = labs(target2 - m2.getPosition());
    const long d3 = labs(target3 - m3.getPosition());
    const long maxDist = max(d1, max(d2, d3));

    if (maxDist == 0) {
        return false;
    }

    // Mismo reparto que moveSynchronized: velocidad y aceleracion en
    // proporcion al recorrido de cada eje, para que los tres lleguen juntos.
    float k1 = (float)d1 / (float)maxDist;
    float k2 = (float)d2 / (float)maxDist;
    float k3 = (float)d3 / (float)maxDist;

    // Piso del reparto (ver el header). Solo sube, y al eje dominante --
    // que siempre vale 1 -- no lo toca nunca.
    if (pisoEscala > 0.0f)
    {
        if (k1 < pisoEscala) k1 = pisoEscala;
        if (k2 < pisoEscala) k2 = pisoEscala;
        if (k3 < pisoEscala) k3 = pisoEscala;
    }

    // Un eje que no se mueve en el tramo nuevo tendria velocidad cero y no se
    // puede encadenar: hay que frenarlo de verdad.
    if (k1 <= 0.0f || k2 <= 0.0f || k3 <= 0.0f) {
        return false;
    }

    const float a1 = limits.maxAcceleration * k1;
    const float a2 = limits.maxAcceleration * k2;
    const float a3 = limits.maxAcceleration * k3;

    // LOS TRES O NINGUNO.
    //
    // Hubo una version que dejaba encadenar a los que podian y rearrancaba
    // desde cero al que no (tipicamente el eje que invierte el sentido en un
    // vertice). Suena razonable y en el robot se ve pesimo: el eje que sale
    // de cero tarda mucho mas que los otros dos, que ya venian a velocidad
    // de crucero, asi que el brazo se queda dando vueltas alrededor del
    // punto esperandolo. Peor que la frenada que trataba de evitar.
    //
    // Si alguno no puede seguir, frenan los tres juntos: es una frenada
    // limpia y, sobre todo, siempre la misma.
    if (!m1.puedeRedirigir(target1, a1) ||
        !m2.puedeRedirigir(target2, a2) ||
        !m3.puedeRedirigir(target3, a3)) {
        return false;
    }

    m1.redirigir(target1, limits.maxSpeed * k1, a1);
    m2.redirigir(target2, limits.maxSpeed * k2, a2);
    m3.redirigir(target3, limits.maxSpeed * k3, a3);

    return true;
}

void moveSynchronized(Stepper &m1, Stepper &m2, Stepper &m3,
                       long target1, long target2, long target3,
                       const MotionLimits &limits)
{
    const long d1 = labs(target1 - m1.getPosition());
    const long d2 = labs(target2 - m2.getPosition());
    const long d3 = labs(target3 - m3.getPosition());
    const long maxDist = max(d1, max(d2, d3));

    if (maxDist == 0) {
        return; // ya estan en destino, nada que sincronizar
    }

    const float k1 = (float)d1 / (float)maxDist;
    const float k2 = (float)d2 / (float)maxDist;
    const float k3 = (float)d3 / (float)maxDist;

    m1.setSpeed(limits.maxSpeed * k1);
    m1.setAcceleration(limits.maxAcceleration * k1);
    m2.setSpeed(limits.maxSpeed * k2);
    m2.setAcceleration(limits.maxAcceleration * k2);
    m3.setSpeed(limits.maxSpeed * k3);
    m3.setAcceleration(limits.maxAcceleration * k3);

    m1.moveTo(target1);
    m2.moveTo(target2);
    m3.moveTo(target3);
}

}