#include "ConveyorIntercept.h"
#include "kinematics/DeltaKinematics.h"
#include <math.h>

namespace ConveyorIntercept {

namespace {

/**
 * Tiempo (segundos) que tarda un movimiento de distancia D (en pasos) con
 * la velocidad y aceleracion dadas.
 *
 * Tiene que dar EXACTAMENTE lo mismo que hace Motors::moveSynchronized: ahi
 * se escalan velocidad Y aceleracion de cada eje por la misma fraccion k,
 * y en ambos regimenes (triangular y trapezoidal) los k se cancelan, asi
 * que el tiempo del movimiento coordinado es el del eje que mas recorre.
 *
 * Nota: asume arranque desde parado. Si el robot ya venia moviendose (por
 * ejemplo, se interrumpio la vuelta a home), el tramo 1 va a tardar un
 * poco menos que lo estimado. No importa para la precision del agarre,
 * porque el instante del encuentro se fija con descendStartDelay y no con
 * la llegada del tramo 1.
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
        // Con los limites del robot este es SIEMPRE el caso (ver Motors.h).
        return 2.0f * sqrtf(D / maxAcceleration);
    }

    float tAccel = maxSpeed / maxAcceleration;
    float dCruise = D - 2.0f * dAccel;
    float tCruise = dCruise / maxSpeed;
    return 2.0f * tAccel + tCruise;
}

// Distancia del eje que mas recorre: es la que fija el tiempo del
// movimiento coordinado de los 3 motores.
long maxAxisDistance(long fromS1, long fromS2, long fromS3,
                      long toS1, long toS2, long toS3)
{
    long d1 = labs(toS1 - fromS1);
    long d2 = labs(toS2 - fromS2);
    long d3 = labs(toS3 - fromS3);

    long maxDist = d1;
    if (d2 > maxDist) maxDist = d2;
    if (d3 > maxDist) maxDist = d3;
    return maxDist;
}

/**
 * Perfil triangular: en que INSTANTE del movimiento se alcanzo la fraccion
 * de recorrido s (0..1), y a que velocidad (en fraccion por segundo) se
 * esta yendo en ese momento.
 *
 * Con perfil triangular de duracion T, la velocidad de la fraccion es
 *      sPunto(s) = (4/T) * sqrt(min(s, 1-s) / 2)
 * y el tiempo hasta esa fraccion sale de integrar la rampa. Como todos los
 * ejes comparten la misma s (Motors::moveSynchronized escala velocidad y
 * aceleracion por igual), esto vale para el movimiento coordinado entero.
 *
 * Nota: la posicion cartesiana no es exactamente lineal en s (el delta es
 * no lineal), pero sobre los ~2 cm del tramo 2 el desvio medido es de 0,1
 * grados de articulacion, despreciable frente a la repetibilidad mecanica.
 */
void triangularAt(float s, float T, float &tAtFraction, float &fractionSpeed)
{
    if (T <= 0.0f)
    {
        tAtFraction = 0.0f;
        fractionSpeed = 0.0f;
        return;
    }

    if (s <= 0.5f)
    {
        // Todavia acelerando: s = 2*(t/T)^2
        tAtFraction = T * sqrtf(s * 0.5f);
    }
    else
    {
        // Ya frenando: el tiempo que FALTA sale de la fraccion restante.
        tAtFraction = T * (1.0f - sqrtf((1.0f - s) * 0.5f));
    }

    const float f = (s <= 0.5f) ? s : (1.0f - s);
    fractionSpeed = (4.0f / T) * sqrtf(f * 0.5f);
}

} // namespace

InterceptResult solve(float pieceY,
                       float timeSinceDetection,
                       const BeltConfig &belt,
                       const PickGeometry &geom,
                       long currentSteps1, long currentSteps2, long currentSteps3,
                       float maxSpeed, float accelFast, float accelSlow)
{
    InterceptResult result;
    result.reachable = false;

    if (belt.velocityX <= 0.0f)
    {
        return result; // sin cinta en movimiento no hay intercepcion que resolver
    }

    // Donde esta el centro de la pieza AHORA (ya avanzo desde que cruzo la
    // linea de deteccion).
    const float pieceXNow = belt.detectionLineX + belt.velocityX * timeSinceDetection;

    // ---- Geometria del encuentro velocity-matched (ver ConveyorIntercept.h) ----
    // Fraccion del tramo 2 recorrida cuando el gripper cruza la cara
    // superior de la pieza, y cuanto hay que sobrepasarla en X para que ese
    // cruce caiga exactamente sobre su centro.
    const float sContact = geom.approachDZ / (geom.approachDZ + geom.pressDZ);
    const float overshootDX = fabsf(geom.approachDX) * (geom.pressDZ / geom.approachDZ);

    // Semilla: el punto mas temprano posible dentro del area de trabajo.
    // Si la pieza ya entro al area, arrancamos desde donde esta.
    float grabX = geom.workAreaMinX;
    if (pieceXNow > grabX) grabX = pieceXNow;

    float t1 = 0.0f;
    float t2 = 0.0f;
    float tContact = 0.0f;
    float contactSpeedX = 0.0f;

    for (int i = 0; i < MAX_ITERATIONS; i++)
    {
        if (grabX > geom.workAreaMaxX)
        {
            return result; // la pieza sale del area antes de que el robot llegue
        }

        // Los puntos de la maniobra, para este candidato de agarre.
        DeltaKinematics::DeltaAngles approach = DeltaKinematics::solveIK(
            grabX + geom.approachDX, pieceY, geom.grabZ + geom.approachDZ);
        DeltaKinematics::DeltaAngles descendEnd = DeltaKinematics::solveIK(
            grabX + overshootDX, pieceY, geom.grabZ - geom.pressDZ);
        DeltaKinematics::DeltaAngles lift = DeltaKinematics::solveIK(
            grabX + overshootDX, pieceY, geom.grabZ + geom.liftDZ);

        // Se validan todos ANTES de comprometerse con la pieza: no sirve
        // descubrir a mitad de la maniobra, con la pieza ya pegada al
        // gripper, que el punto siguiente no era alcanzable.
        if (!approach.success || !descendEnd.success || !lift.success)
        {
            return result;
        }

        t1 = moveTimeSeconds(maxAxisDistance(currentSteps1, currentSteps2, currentSteps3,
                                              approach.steps1, approach.steps2, approach.steps3),
                              maxSpeed, accelFast);

        t2 = moveTimeSeconds(maxAxisDistance(approach.steps1, approach.steps2, approach.steps3,
                                              descendEnd.steps1, descendEnd.steps2, descendEnd.steps3),
                              maxSpeed, accelSlow);

        // Instante del contacto dentro del tramo 2, y a que velocidad en X
        // va el gripper justo ahi (que es lo que queremos igual a la cinta).
        float fractionSpeed = 0.0f;
        triangularAt(sContact, t2, tContact, fractionSpeed);
        contactSpeedX = (fabsf(geom.approachDX) + overshootDX) * fractionSpeed;

        // X mas temprano donde el robot podria TOCAR la pieza, si saliera
        // AHORA mismo y encadenara los dos tramos sin pausa. Ojo que lo que
        // cuenta es tContact, no t2: el encuentro pasa a mitad del tramo 2.
        const float earliestX = pieceXNow + belt.velocityX * (t1 + tContact);

        // Si ese punto cae antes del area de trabajo, el robot llega
        // sobrado: se agarra en el borde de entrada (lo antes posible
        // dentro del area) y se espera a la pieza ahi.
        float newGrabX = (earliestX > geom.workAreaMinX) ? earliestX : geom.workAreaMinX;

        const bool converged = (fabsf(newGrabX - grabX) < CONVERGENCE_TOLERANCE_CM);
        grabX = newGrabX;

        if (converged)
        {
            if (grabX > geom.workAreaMaxX)
            {
                return result;
            }

            result.approachX = grabX + geom.approachDX;
            result.approachY = pieceY;
            result.approachZ = geom.grabZ + geom.approachDZ;

            result.grabX = grabX;
            result.grabY = pieceY;
            result.grabZ = geom.grabZ;

            result.descendEndX = grabX + overshootDX;
            result.descendEndY = pieceY;
            result.descendEndZ = geom.grabZ - geom.pressDZ;

            // Se sube en vertical desde donde termina el tramo 2, no desde
            // el centro de la pieza: cualquier corrimiento en X con la
            // pieza ya pegada solo la arrastraria.
            result.liftX = grabX + overshootDX;
            result.liftY = pieceY;
            result.liftZ = geom.grabZ + geom.liftDZ;

            result.t1 = t1;
            result.t2 = t2;
            result.tContact = tContact;
            result.contactSpeedX = contactSpeedX;

            // Cuando hay que lanzar el tramo 2 para que el CONTACTO (no el
            // fin del tramo) caiga justo cuando el centro de la pieza pasa
            // por grabX.
            const float timeToMeet = (grabX - pieceXNow) / belt.velocityX;
            result.descendStartDelay = timeToMeet - tContact;
            if (result.descendStartDelay < 0.0f)
            {
                result.descendStartDelay = 0.0f; // residuo de la tolerancia de convergencia
            }

            result.reachable = true;
            return result;
        }
    }

    // No convergio: no confiar en el resultado, mejor dejar pasar la pieza.
    return result;
}

} // namespace ConveyorIntercept
