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
//  VEL_MAX ES UN TOPE QUE AHORA SI ACTUA. Estuvo mucho tiempo en 70.000, o
//  sea a proposito por encima de lo alcanzable, para que el perfil quedara
//  siempre TRIANGULAR. Con la aceleracion maxima, el pico de un movimiento
//  es sqrt(a * D), y el recorrido articular completo (+-50 grados con
//  DeltaKinematics::THETA_MIN/MAX) son ~2.778 pasos: el pico real nunca
//  pasaba de ~16.400 pasos/s. Cualquier tope por encima de eso no hacia
//  absolutamente nada.
//
//  Se bajo a 12.000 para lo unico que un tope puede hacer aca: recortar el
//  pico de los tramos LARGOS sin tocar la aceleracion. Un movimiento que
//  antes llegaba a 16.400 ahora corta en 12.000; los cortos (menos de
//  v^2/a = 1.485 pasos, o sea 53 grados de articulacion) siguen siendo
//  exactamente los mismos de antes, porque nunca llegan al tope.
//
//  El motivo es fisico: un eje que perdio pasos frenando de un tramo largo
//  y rapido. Bajar la aceleracion no servia -- probado en el robot, con
//  algunos valores vibraba PEOR, como si entrara en la frecuencia natural
//  del brazo -- y ademas habria hecho mas lento TODO el ciclo, no solo el
//  caso malo.
//
//  Costo medido sobre el movimiento mas largo posible: 338 ms -> 355 ms
//  (+4,9 %). Sobre los cortos, cero.
//
//  OJO al tocar esto: con un tope alcanzable se activa el tramo de crucero
//  de Stepper::computeNextInterval(), que antes de este cambio nunca se
//  habia ejecutado en este robot y tenia dos errores en el frenado (ver el
//  comentario ahi). Si se vuelve a subir VEL_MAX por encima de ~16.400, el
//  perfil vuelve a ser triangular y ese tramo deja de correr.
// ============================================================
//  Dejaron de ser constexpr para poder barrerlos desde la interfaz sin
//  recompilar: son los tres numeros con los que se busca el compromiso
//  entre ciclo rapido y vibracion, y encontrarlo a fuerza de reflashear es
//  media tarde de trabajo. Los limita el rango declarado en la tabla de
//  parametros, no el tipo.
extern float VEL_MAX;    // pasos/seg
extern float ACC_SUAVE;  // pasos/seg^2 -- tramo que toca la pieza
extern float ACC_RAPIDA; // pasos/seg^2 -- todo lo demas

// Los tres juegos de limites que usan los puntos de llamada. NO son const:
// aplicarLimites() los rearma cuando cambia alguno de los tres numeros de
// arriba. Se leen igual que antes (Motors::FAST_LIMITS), asi que ningun
// punto de uso cambio.
extern MotionLimits DEFAULT_LIMITS;

// Movimientos normales (traslados, levantar la pieza, ir al tacho).
extern MotionLimits FAST_LIMITS;

// Unico tramo donde el gripper toca la pieza: se baja la aceleracion para
// que el encuentro sea suave y no la desplace ni la haga rebotar.
extern MotionLimits SOFT_LIMITS;

// Rearma los tres juegos a partir de VEL_MAX/ACC_*. La llama Robot cuando
// cambia la generacion de la tabla de parametros; sin esto, cambiar un
// valor no tendria efecto hasta el proximo arranque.
void aplicarLimites();

/**
 * Mueve 3 motores en simultaneo, escalando la velocidad de cada uno segun
 * cuanto tiene que recorrer respecto al que mas recorre.Esto sincroniza la
 * llegada de forma EXACTA: tiempo = distancia/velocidad, y al escalar
 * velocidad proporcional a distancia, ese tiempo queda igual para los 3.
 */
void moveSynchronized(Stepper &m1, Stepper &m2, Stepper &m3,
                       long target1, long target2, long target3,
                       const MotionLimits &limits = DEFAULT_LIMITS);

/**
 * Igual que moveSynchronized, pero SIN frenar: cambia el destino de los tres
 * ejes conservando la velocidad que traen (ver Stepper::redirigir).
 *
 * Es lo que permite pasar de un tramo al siguiente sin detenerse en el punto
 * intermedio -- el brazo redondea la esquina en vez de clavarse y arrancar.
 *
 * Los tres o ninguno: primero se pregunta si los tres pueden, y recien
 * despues se aplica. Redirigir dos y dejar el tercero yendo al punto viejo
 * partiria el movimiento en dos direcciones distintas.
 *
 * Devuelve false si alguno no puede (cambio de sentido, o lo que le queda no
 * alcanza para frenar desde la velocidad que trae). Ahi quien llama tiene
 * que hacer lo de siempre: esperar a que lleguen y despues moveSynchronized.
 */
// `pisoEscala` es el minimo que puede recibir un eje del reparto, en vez de
// lo estrictamente proporcional. Existe por movL: partiendo una recta en
// tramos cortos, el eje que menos recorre se lleva a veces el 1 % de la
// aceleracion (medido: 455 pas/s2 de 40.000), y con esa aceleracion su
// distancia de frenado se dispara y puedeRedirigir() se niega. Con el
// reparto estricto eso pasa en la MITAD de los tramos de una diagonal, y
// cada negativa es una frenada: es exactamente el "va a los tirones".
//
// Un eje con el piso puesto llega antes que los otros, no despues, y lo que
// se aparta de la recta por eso esta acotado por lo poco que ese eje
// recorria en el tramo -- que es chico por definicion, porque es justamente
// el eje que casi no se mueve. 0 = reparto estricto (el de siempre).
bool redirigirSincronizado(Stepper &m1, Stepper &m2, Stepper &m3,
                            long target1, long target2, long target3,
                            const MotionLimits &limits = DEFAULT_LIMITS,
                            float pisoEscala = 0.0f);

}