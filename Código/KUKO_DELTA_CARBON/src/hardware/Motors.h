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
//  Medidos sobre el robot real con el gripper montado, drivers DM556 a
//  2.7A y 10000 micropasos/vuelta.
//
//  MAX_SPEED esta puesta A PROPOSITO por encima de lo alcanzable: asi
//  ningun movimiento llega nunca a velocidad de crucero y el perfil queda
//  siempre TRIANGULAR (acelerando o frenando, nunca meseta), que es lo que
//  menos vibracion produce.
//
//  Eso se verifica solo: para que aparezca meseta hace falta recorrer mas
//  de v^2/a pasos (~50.500 con la aceleracion maxima), y el recorrido
//  articular completo que permite DeltaKinematics::THETA_MIN/MAX (+-50
//  grados) es de apenas ~2.800 pasos. La meseta es geometricamente
//  inalcanzable: no depende de elegir bien los puntos.
// ============================================================
constexpr float MAX_SPEED = 70000.0f;         // pasos/seg
constexpr float MIN_ACCELERATION = 17000.0f; // pasos/seg^2
constexpr float MAX_ACCELERATION = 97000.0f;  // pasos/seg^2
constexpr MotionLimits DEFAULT_LIMITS = {MAX_SPEED, MAX_ACCELERATION};

// Movimientos normales (traslados, levantar la pieza, ir al tacho).
constexpr MotionLimits FAST_LIMITS = {MAX_SPEED, MAX_ACCELERATION};

// Unico tramo donde el gripper toca la pieza: se baja la aceleracion para
// que el encuentro sea suave y no la desplace ni la haga rebotar.
constexpr MotionLimits SOFT_LIMITS = {MAX_SPEED, MIN_ACCELERATION};

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