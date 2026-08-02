#pragma once
#include <Arduino.h>

class Stepper; 

/**
 * Motors
 * ------
 * Utilidades para coordinar VARIOS Stepper a la vez. No dependen de la cinematica,
 * solo de posiciones en pasos.
 *
 * La clase que controla cada eje individual sigue siendo Stepper
 * (Stepper.h/.cpp, generacion de pulsos por interrupcion de hardware).
 * Este archivo NO redefine esa clase, solo agrega logica para coordinar los
 * 3 motores juntos.
 */
namespace Motors {

struct MotionLimits {
    float maxSpeed;        // pasos/seg
    float maxAcceleration; // pasos/seg^2
};

// ============================================================
//  LIMITES FISICOS DEL SISTEMA (driver + microstepping + mecanica)
//  Unico lugar a tocar para subir/bajar el techo de velocidad global.
// ============================================================
constexpr float MAX_SPEED = 70000.0f;         // pasos/seg
constexpr float MIN_ACCELERATION = 17000.0f; // pasos/seg^2
constexpr float MAX_ACCELERATION = 97000.0f;  // pasos/seg^2
constexpr MotionLimits DEFAULT_LIMITS = {MAX_SPEED, MAX_ACCELERATION};

/**
 * Mueve 3 motores en simultaneo, escalando la velocidad de cada uno segun
 * cuanto tiene que recorrer respecto al que mas recorre.Esto sincroniza la
 * llegada de forma EXACTA: tiempo = distancia/velocidad, y al escalar
 * velocidad proporcional a distancia, ese tiempo queda igual para los 3.
 */
void moveSynchronized(Stepper &m1, Stepper &m2, Stepper &m3,
                       long target1, long target2, long target3,
                       const MotionLimits &limits = DEFAULT_LIMITS);

}