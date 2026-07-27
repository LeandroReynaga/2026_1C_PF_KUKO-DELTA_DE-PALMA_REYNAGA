#include "DeltaKinematics.h"
#include <math.h>

namespace DeltaKinematics {

namespace {

// --- Constantes derivadas, calculadas una única vez en compilación ---
// PIVOT_Y: posición del pivote del motor en el eje local Y (el brazo superior
// gira siempre en el plano X=0 de esta pierna, con el pivote fijo aquí).
// EFFECTOR_RADIUS se aplica aparte, corriendo el punto objetivo (no el pivote).
constexpr float PIVOT_Y = -BASE_RADIUS;
constexpr float PIVOT_Y_SQ = PIVOT_Y * PIVOT_Y;
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

    const float y0p = y0 - EFFECTOR_RADIUS; // desplaza el OBJETIVO por el radio del efector

    const float a = (x0 * x0 + y0p * y0p + z0 * z0
                      + BICEP_LENGTH * BICEP_LENGTH
                      - FOREARM_LENGTH * FOREARM_LENGTH
                      - PIVOT_Y_SQ) / (2.0f * z0);
    const float b = (PIVOT_Y - y0p) / z0;

    // Discriminante de la ecuación cuadrática de la intersección
    const float discriminant = -(a + b * PIVOT_Y) * (a + b * PIVOT_Y)
                                + BICEP_LENGTH * BICEP_LENGTH * (b * b + 1.0f);
    if (discriminant < 0.0f) {
        return IKStatus::UNREACHABLE;
    }

    // Raíz correspondiente a la configuración física del brazo (codo afuera)
    const float yj = (PIVOT_Y - a * b - sqrtf(discriminant)) / (b * b + 1.0f);
    const float zj = a + b * yj;

    theta = atan2f(-zj, PIVOT_Y - yj) * RAD_TO_DEG;

    if (theta < THETA_MIN || theta > THETA_MAX) {
        return IKStatus::JOINT_LIMIT;
    }
    return IKStatus::OK;
}

// Convierte grados a micropasos absolutos, listos para AccelStepper::moveTo().
long angleToSteps(float degrees, bool invert) {
    const long steps = lroundf(degrees * STEPS_PER_DEGREE);
    return invert ? -steps : steps;
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

    x = x * 1.02f; // Ganancia para ajustar coordenada X, medida en el robot real
    y = y * 0.98f; // Ganancia para ajustar coordenada Y, medida en el robot real

    // Sistema de coordenadas del robot (vista superior):
    //
    //            Y+
    //             |
    //     M1 ----------- M2      M1 a 150°, M2 a 30° (respecto a X+)
    //       \           /
    //         \       /
    //           \   /
    //            M3               M3 a 270° (sobre el eje -Y)
    //
    // X+ hacia la derecha, origen (0,0) en el centro de la base.

    // --- Pierna 1 (M1, arriba-izquierda, 150°) ---
    const float x1 = -0.5f * x - SQRT3_2 * y;
    const float y1 = SQRT3_2 * x - 0.5f * y;
    const IKStatus s1 = solveLeg(x1, y1, z, result.theta1);

    // --- Pierna 2 (M2, arriba-derecha, 30°) ---
    const float x2 = -0.5f * x + SQRT3_2 * y;
    const float y2 = -SQRT3_2 * x - 0.5f * y;
    const IKStatus s2 = solveLeg(x2, y2, z, result.theta2);

    // --- Pierna 3 (M3, abajo, 270°, sobre -Y) ---
    const IKStatus s3 = solveLeg(x, y, z, result.theta3);

    // Se reporta el primer fallo encontrado (alcanza para diagnóstico)
    if (s1 != IKStatus::OK)      result.status = s1;
    else if (s2 != IKStatus::OK) result.status = s2;
    else if (s3 != IKStatus::OK) result.status = s3;
    else                          result.status = IKStatus::OK;

    result.success = (result.status == IKStatus::OK);

    // Solo se calculan pasos si la solución es válida; si falló, quedan en 0
    // para no arriesgarse a mandar un target sin sentido a los motores.
    if (result.success) {
        result.steps1 = angleToSteps(result.theta1, INVERT_MOTOR1);
        result.steps2 = angleToSteps(result.theta2, INVERT_MOTOR2);
        result.steps3 = angleToSteps(result.theta3, INVERT_MOTOR3);
    }

    return result;
}

} // namespace DeltaKinematics