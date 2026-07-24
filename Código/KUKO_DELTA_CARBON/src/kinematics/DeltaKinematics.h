#pragma once
#include <Arduino.h>

/**
 * DeltaKinematics
 * ----------------
 * Cinemática inversa para un robot Delta de 3 grados de libertad
 * (efector puntual, sin rotación de la plataforma).
 *
 * Uso típico desde el sketch principal:
 *
 *   DeltaKinematics::DeltaAngles pose = DeltaKinematics::solveIK(x, y, z);
 *   if (pose.success) {
 *       moverMotores(pose.theta1, pose.theta2, pose.theta3);
 *   } else {
 *       // pose.status indica el motivo del fallo
 *   }
 */
namespace DeltaKinematics {

// ============================================================
//  PARÁMETROS GEOMÉTRICOS DEL ROBOT (unidades: cm)
//  Único lugar que hay que tocar si se vuelve a medir el robot.
// ============================================================
constexpr float BICEP_LENGTH    = 18.0f;   // L1: brazo superior, motor -> codo
constexpr float FOREARM_LENGTH  = 35.2f;   // L2: brazo inferior, varilla libre codo -> efector
constexpr float BASE_RADIUS     = 8.275f; //7.075f;  // Centro de la base al eje del motor (141.5 mm / 2)
constexpr float EFFECTOR_RADIUS = 3.83f;   // Centro del efector al eje de articulación (76.6 mm / 2)

// Límites articulares de seguridad (grados), medidos desde la horizontal del brazo.
// Ajustar según los topes mecánicos reales una vez que se midan sobre el robot.
// Mientras tanto quedan amplios para no bloquear pruebas iniciales.
constexpr float THETA_MIN = -90.0f;
constexpr float THETA_MAX = 90.0f;

// ============================================================
//  CONFIGURACIÓN DEL DRIVER (conversión grados -> pasos)
//  Driver de los NEMA23 configurado a 20000 micropasos por vuelta.
//  El robot ya está calibrado con 0 pasos = 0°, no se aplica offset.
// ============================================================
constexpr long STEPS_PER_REV    = 20000;                    // micropasos por vuelta completa (360°)
constexpr float STEPS_PER_DEGREE = STEPS_PER_REV / 360.0f;

// Si al probar un motor gira al revés de lo esperado, invertir acá
// (mucho más simple que recablear o invertir el driver).
constexpr bool INVERT_MOTOR1 = false;
constexpr bool INVERT_MOTOR2 = false;
constexpr bool INVERT_MOTOR3 = false;

// ============================================================
//  Resultado de la cinemática inversa
// ============================================================
enum class IKStatus : uint8_t {
    OK = 0,
    UNREACHABLE,    // La coordenada cae fuera del espacio de trabajo físico
    JOINT_LIMIT,    // Se resolvió, pero un ángulo excede THETA_MIN/THETA_MAX
    INVALID_INPUT   // Entrada geométricamente inválida (p. ej. z = 0)
};

struct DeltaAngles {
    float theta1 = 0.0f;
    float theta2 = 0.0f;
    float theta3 = 0.0f;
    long steps1 = 0;   // Posición objetivo del motor 1, en micropasos (para AccelStepper::moveTo)
    long steps2 = 0;
    long steps3 = 0;
    IKStatus status = IKStatus::INVALID_INPUT;
    bool success = false; // Atajo equivalente a (status == IKStatus::OK)
};

// Convierte un status en texto, útil para logs de diagnóstico por Serial.
const char *toString(IKStatus status);

/**
 * Cinemática inversa completa del robot.
 * Entrada: posición (x, y, z) del efector en cm, respecto al centro de la base.
 * Salida: ángulos de los 3 motores en grados + estado de la solución.
 */
DeltaAngles solveIK(float x, float y, float z);

} // namespace DeltaKinematics