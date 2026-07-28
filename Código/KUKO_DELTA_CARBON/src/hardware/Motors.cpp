#include "Motors.h"
#include "robot/Stepper.h"

namespace Motors {

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

} // namespace Motors