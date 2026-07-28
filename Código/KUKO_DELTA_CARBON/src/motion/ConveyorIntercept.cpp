#include "ConveyorIntercept.h"
#include "kinematics/DeltaKinematics.h"
#include <math.h>

namespace ConveyorIntercept {

namespace {

/**
 * Tiempo (segundos) que tarda un movimiento trapezoidal de distancia D
 * (en pasos) con velocidad y aceleracion maximas dadas. Misma formula
 * usada para validar la sincronizacion de los 3 motores.
 */
float moveTimeSeconds(long distanceSteps, float maxSpeed, float maxAcceleration)
{
    float D = (float)labs(distanceSteps);
    if (D <= 0.0f || maxSpeed <= 0.0f || maxAcceleration <= 0.0f)
    {
        return 0.0f;
    }

    float dAccel = (maxSpeed * maxSpeed) / (2.0f * maxAcceleration);
    if (2.0f * dAccel >= D)
    {
        // Perfil triangular: la distancia es corta, nunca llega a maxSpeed.
        return 2.0f * sqrtf(D / maxAcceleration);
    }

    float tAccel = maxSpeed / maxAcceleration;
    float dCruise = D - 2.0f * dAccel;
    float tCruise = dCruise / maxSpeed;
    return 2.0f * tAccel + tCruise;
}

} // namespace

InterceptResult solve(float pieceY, float pieceZ,
                       float timeSinceDetection,
                       const BeltConfig &belt,
                       long currentSteps1, long currentSteps2, long currentSteps3,
                       float maxSpeed, float maxAcceleration)
{
    InterceptResult result;
    result.reachable = false;

    // Posicion de la pieza en el instante en que arranca este calculo
    // (ya se movio un poco desde que cruzo la linea, por el retardo de
    // comunicacion/procesamiento).
    const float pieceXNow = belt.detectionLineX + belt.velocityX * timeSinceDetection;

    // Semilla: arrancar asumiendo que se puede llegar "ya" (tiempo de viaje = 0)
    float travelTime = 0.0f;

    for (int i = 0; i < MAX_ITERATIONS; i++)
    {
        const float candidateX = pieceXNow + belt.velocityX * travelTime;

        DeltaKinematics::DeltaAngles pose = DeltaKinematics::solveIK(candidateX, pieceY, pieceZ);
        if (!pose.success)
        {
            // Ese punto de la trayectoria futura no es alcanzable. No tiene
            // sentido seguir iterando: si no se puede llegar ahi, tampoco
            // se va a poder llegar a un punto todavia mas lejos (la pieza
            // se sigue moviendo en la misma direccion).
            result.reachable = false;
            return result;
        }

        const long d1 = labs(pose.steps1 - currentSteps1);
        const long d2 = labs(pose.steps2 - currentSteps2);
        const long d3 = labs(pose.steps3 - currentSteps3);
        long maxDist = d1;
        if (d2 > maxDist) maxDist = d2;
        if (d3 > maxDist) maxDist = d3;

        const float newTravelTime = moveTimeSeconds(maxDist, maxSpeed, maxAcceleration);

        if (fabsf(newTravelTime - travelTime) < CONVERGENCE_TOLERANCE_S)
        {
            result.x = candidateX;
            result.y = pieceY;
            result.z = pieceZ;
            result.etaSeconds = newTravelTime;
            result.reachable = true;
            return result;
        }

        travelTime = newTravelTime;
    }

    // No convergio en MAX_ITERATIONS pasadas: no confiar en el resultado.
    result.reachable = false;
    return result;
}

} // namespace ConveyorIntercept