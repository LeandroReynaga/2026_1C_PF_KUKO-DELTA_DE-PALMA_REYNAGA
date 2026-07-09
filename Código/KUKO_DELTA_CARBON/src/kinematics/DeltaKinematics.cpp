#include "DeltaKinematics.h"
#include <math.h>

namespace DeltaKinematics {

namespace {

// --- Constantes derivadas, calculadas una única vez en compilación ---
constexpr float REFF = BASE_RADIUS - EFFECTOR_RADIUS;
constexpr float SQRT3_2 = 0.8660254037844386f; // sin(120°) == -sin(240°)
constexpr float EPSILON = 1e-6f;

/**
 * Resuelve el ángulo del motor de UNA pierna, mediante la intersección
 * de dos esferas (radio L1 centrada en el motor, radio L2 centrada en
 * el punto del efector). Requiere que (x0, y0, z0) ya esté expresado
 * en el sistema de coordenadas rotado de esa pierna.
 */
IKStatus solveLeg(float x0, float y0, float z0, float &theta) {
    if (fabsf(z0) < EPSILON) {
        return IKStatus::INVALID_INPUT; // evita división por cero
    }

    y0 += REFF; // traslada el eje al radio efectivo del sistema

    const float a = (x0 * x0 + y0 * y0 + z0 * z0
                      + BICEP_LENGTH * BICEP_LENGTH
                      - FOREARM_LENGTH * FOREARM_LENGTH) / (2.0f * z0);
    const float b = y0 / z0;

    // Discriminante de la ecuación cuadrática de la intersección
    const float discriminant = -(a - b * REFF) * (a - b * REFF)
                                + BICEP_LENGTH * BICEP_LENGTH * (b * b + 1.0f);
    if (discriminant < 0.0f) {
        return IKStatus::UNREACHABLE;
    }

    // Raíz correspondiente a la configuración física del brazo (codo afuera)
    const float yj = (-REFF - a * b - sqrtf(discriminant)) / (b * b + 1.0f);
    const float zj = a + b * yj;

    theta = atan2f(-zj, -REFF - yj) * RAD_TO_DEG;

    if (theta < THETA_MIN || theta > THETA_MAX) {
        return IKStatus::JOINT_LIMIT;
    }
    return IKStatus::OK;
}

} // namespace (anónimo, uso interno del archivo)

const char *toString(IKStatus status) {
    switch (status) {
        case IKStatus::OK:            return "OK";
        case IKStatus::UNREACHABLE:   return "UNREACHABLE";
        case IKStatus::JOINT_LIMIT:   return "JOINT_LIMIT";
        case IKStatus::INVALID_INPUT: return "INVALID_INPUT";
        default:                      return "UNKNOWN";
    }
}

DeltaAngles solveIK(float x, float y, float z) {
    DeltaAngles result;

    // --- Pierna 1: motor de referencia, sin rotación ---
    const IKStatus s1 = solveLeg(x, y, z, result.theta1);

    // --- Pierna 2: motor a 120°, coordenadas rotadas -120° ---
    const float x2 = -0.5f * x + SQRT3_2 * y;
    const float y2 = -SQRT3_2 * x - 0.5f * y;
    const IKStatus s2 = solveLeg(x2, y2, z, result.theta2);

    // --- Pierna 3: motor a 240°, coordenadas rotadas -240° ---
    const float x3 = -0.5f * x - SQRT3_2 * y;
    const float y3 = SQRT3_2 * x - 0.5f * y;
    const IKStatus s3 = solveLeg(x3, y3, z, result.theta3);

    // Se reporta el primer fallo encontrado (alcanza para diagnóstico)
    if (s1 != IKStatus::OK)      result.status = s1;
    else if (s2 != IKStatus::OK) result.status = s2;
    else if (s3 != IKStatus::OK) result.status = s3;
    else                          result.status = IKStatus::OK;

    result.success = (result.status == IKStatus::OK);
    return result;
}

} // namespace DeltaKinematics