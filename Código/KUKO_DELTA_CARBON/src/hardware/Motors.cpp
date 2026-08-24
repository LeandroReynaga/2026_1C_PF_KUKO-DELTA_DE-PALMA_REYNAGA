#include "Motors.h"
#include "robot/Stepper.h"

namespace Motors {

// Valores de fabrica. La tabla de parametros los toma como referencia al
// registrarlos, asi que 'P0' vuelve exactamente aca.
float VEL_MAX    = 12000.0f;
float ACC_AGARRE = 180000.0f; // antes 17000 (una sola "suave") y 110000
float ACC_CAJA   = 17000.0f;
float ACC_RAPIDA = 95000.0f;  // antes 97000

MotionLimits DEFAULT_LIMITS = {VEL_MAX, ACC_RAPIDA};
MotionLimits FAST_LIMITS    = {VEL_MAX, ACC_RAPIDA};
MotionLimits AGARRE_LIMITS  = {VEL_MAX, ACC_AGARRE};
MotionLimits CAJA_LIMITS    = {VEL_MAX, ACC_CAJA};

void aplicarLimites()
{
    DEFAULT_LIMITS = {VEL_MAX, ACC_RAPIDA};
    FAST_LIMITS    = {VEL_MAX, ACC_RAPIDA};
    AGARRE_LIMITS  = {VEL_MAX, ACC_AGARRE};
    CAJA_LIMITS    = {VEL_MAX, ACC_CAJA};
}

bool redirigirLineal(Stepper &m1, Stepper &m2, Stepper &m3,
                     long target1, long target2, long target3,
                     const MotionLimits &limits)
{
    Stepper *eje[3]    = { &m1, &m2, &m3 };
    const long meta[3] = { target1, target2, target3 };

    long d[3];
    long maxDist = 0;

    for (uint8_t i = 0; i < 3; i++)
    {
        d[i] = labs(meta[i] - eje[i]->getPosition());

        if (d[i] > maxDist) maxDist = d[i];
    }

    if (maxDist == 0)
    {
        return false;
    }

    bool alguno = false;

    for (uint8_t i = 0; i < 3; i++)
    {
        // Proporcional y sin piso: ver el header. La k se cancela entre la
        // distancia de frenado del eje y lo que tiene para recorrer.
        const float k = (float)d[i] / (float)maxDist;

        const float acel = limits.maxAcceleration * k;
        const float vel  = limits.maxSpeed * k;

        if (eje[i]->puedeRedirigir(meta[i], acel))
        {
            eje[i]->redirigir(meta[i], vel, acel);
            alguno = true;
        }
        else if (!eje[i]->isMoving())
        {
            // Parado: arrancar de cero es seguro y no interrumpe nada.
            eje[i]->setSpeed(vel);
            eje[i]->setAcceleration(acel);
            eje[i]->moveTo(meta[i]);
            alguno = true;
        }
        // Andando y sin poder redirigir: se lo deja llegar. Es el eje que
        // esta invirtiendo el sentido, y tiene que pasar por cero igual.
    }

    return alguno;
}

void empujarDetenidos(Stepper &m1, Stepper &m2, Stepper &m3,
                      long target1, long target2, long target3,
                      const MotionLimits &limits)
{
    Stepper *eje[3]    = { &m1, &m2, &m3 };
    const long meta[3] = { target1, target2, target3 };

    long d[3];
    long maxDist = 0;

    for (uint8_t i = 0; i < 3; i++)
    {
        d[i] = labs(meta[i] - eje[i]->getPosition());

        if (d[i] > maxDist) maxDist = d[i];
    }

    if (maxDist == 0)
    {
        return;
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        if (d[i] == 0 || eje[i]->isMoving())
        {
            continue;
        }

        // Reparto proporcional a lo que le FALTA. Se probo sacarlo a la
        // velocidad del tramo entero para que se pusiera a la par mas
        // rapido, y en el robot se sintio peor: un eje que arranca a fondo
        // para dar unos pocos pasos es un tiron, y son varios por recta.
        const float k = (float)d[i] / (float)maxDist;

        eje[i]->setSpeed(limits.maxSpeed * k);
        eje[i]->setAcceleration(limits.maxAcceleration * k);
        eje[i]->moveTo(meta[i]);
    }
}

bool redirigirSincronizado(Stepper &m1, Stepper &m2, Stepper &m3,
                            long target1, long target2, long target3,
                            const MotionLimits &limits)
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
    const float k1 = (float)d1 / (float)maxDist;
    const float k2 = (float)d2 / (float)maxDist;
    const float k3 = (float)d3 / (float)maxDist;

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