#pragma once
#include <Arduino.h>

class Stepper; // forward declaration: Motors.h solo necesita referencias,
                // no la definicion completa (eso lo incluye Motors.cpp)

/**
 * Motors
 * ------
 * Utilidades para coordinar VARIOS Stepper a la vez. No dependen de ninguna
 * cinematica particular (delta, cartesiana, etc.), solo de posiciones en pasos.
 *
 * La clase que controla cada eje individual sigue siendo Stepper
 * (Stepper.h/.cpp, generacion de pulsos por interrupcion de hardware).
 * Este archivo NO redefine esa clase, solo agrega logica para coordinar
 * 3 instancias juntas.
 */
namespace Motors {

struct MotionLimits {
    float maxSpeed;        // pasos/seg
    float maxAcceleration; // pasos/seg^2 (Stepper::setAcceleration aun no implementa rampa real)
};

// ============================================================
//  LIMITES FISICOS DEL SISTEMA (driver + microstepping + mecanica)
//  Unico lugar a tocar para subir/bajar el techo de velocidad global.
// ============================================================
constexpr float MAX_SPEED = 4000.0f;         // pasos/seg
constexpr float MAX_ACCELERATION = 500.0f;  // pasos/seg^2 (reservado a futuro), antes 3000
constexpr MotionLimits DEFAULT_LIMITS = {MAX_SPEED, MAX_ACCELERATION};

/**
 * Mueve 3 motores en simultaneo, escalando la velocidad de cada uno segun
 * cuanto tiene que recorrer respecto al que mas recorre. Sin rampa de
 * aceleracion (Stepper hoy es velocidad constante), esto sincroniza la
 * llegada de forma EXACTA: tiempo = distancia/velocidad, y al escalar
 * velocidad proporcional a distancia, ese tiempo queda igual para los 3.
 *
 * limits: techo de velocidad/aceleracion para el movimiento (por defecto,
 * el maximo del sistema). Pasar un MotionLimits mas chico para probar
 * a velocidad reducida sin tocar MAX_SPEED/MAX_ACCELERATION globales.
 */
void moveSynchronized(Stepper &m1, Stepper &m2, Stepper &m3,
                       long target1, long target2, long target3,
                       const MotionLimits &limits = DEFAULT_LIMITS);

} // namespace Motors