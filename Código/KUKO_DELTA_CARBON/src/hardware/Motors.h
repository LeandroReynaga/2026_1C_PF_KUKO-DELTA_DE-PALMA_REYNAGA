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
extern float ACC_AGARRE; // pasos/seg^2 -- bajada sobre la pieza en la cinta
extern float ACC_CAJA;   // pasos/seg^2 -- apoyo de la pieza en la celda
extern float ACC_RAPIDA; // pasos/seg^2 -- todo lo demas

// ============================================================
//  POR QUE EL AGARRE Y LA CAJA TIENEN ACELERACIONES DISTINTAS
// ============================================================
//  Eran una sola ("ACC_SUAVE", 17.000) y se separaron porque los dos tramos
//  quieren cosas OPUESTAS, aunque los dos "toquen" una pieza:
//
//  AGARRE (PICK_DESCEND). La pieza va montada en la cinta a 7,1 cm/s y el
//      gripper la tiene que alcanzar EN MOVIMIENTO: el contacto pasa a
//      mitad del tramo y tiene que ocurrir a la misma velocidad que la
//      cinta (ver ConveyorIntercept.h). Ir despacio no lo hace mas suave,
//      lo hace PEOR -- con 17.000 el gripper tocaba a 1,86 cm/s y la pieza
//      se le deslizaba por debajo a 5,2 cm/s. Aca alto es mejor.
//
//  CAJA (BOX_DESCEND). La celda esta quieta y la pieza se APOYA en el piso.
//      No hay nada que alcanzar, y toda velocidad de llegada es un golpe.
//      Aca bajo es mejor, y por eso se queda en los 17.000 de siempre.
//
//  Subir ACC_AGARRE por encima de los 100.000 que aceptaba la tabla es
//  seguro porque el tramo es CORTO. Lo que puede perder pasos, y que esta
//  explicado arriba en VEL_MAX, es el frenado de un tramo LARGO y rapido.
//  Medido: la bajada de agarre son 368 pasos con apr_dx = -2 cm y 526 con
//  -5 cm, contra los v^2/a = 1.309 pasos que harian falta para siquiera
//  llegar a VEL_MAX con 110.000. O sea que el perfil sigue siendo
//  TRIANGULAR y el tramo de crucero de Stepper::computeNextInterval() --el
//  que tenia los dos errores de frenado-- no se ejecuta nunca aca.
// ============================================================

// Los tres juegos de limites que usan los puntos de llamada. NO son const:
// aplicarLimites() los rearma cuando cambia alguno de los tres numeros de
// arriba. Se leen igual que antes (Motors::FAST_LIMITS), asi que ningun
// punto de uso cambio.
extern MotionLimits DEFAULT_LIMITS;

// Movimientos normales (traslados, levantar la pieza, ir al tacho).
extern MotionLimits FAST_LIMITS;

// Bajada sobre la pieza que viene por la cinta (PICK_DESCEND). Ver arriba:
// aca la aceleracion es ALTA, para llegar a la velocidad de la cinta.
extern MotionLimits AGARRE_LIMITS;

// Apoyo de la pieza en la celda de la caja (BOX_DESCEND). Aca la
// aceleracion es baja, que es lo que hace suave un apoyo contra algo quieto.
extern MotionLimits CAJA_LIMITS;

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
/**
 * Encadenado para una RECTA (movL), donde los tramos son colineales.
 *
 * Se diferencia de redirigirSincronizado en una sola cosa, y es toda la
 * diferencia: aca NO es "los tres o ninguno". Cada eje se resuelve solo.
 *
 * POR QUE. En un vertice de una trayectoria ensenada los tres ejes cambian
 * de direccion a la vez, y ahi frenar a los tres juntos es lo correcto. En
 * una recta cartesiana, en cambio, un eje puede invertir el sentido EN
 * MEDIO del tramo mientras los otros dos siguen derecho -- pasa siempre, y
 * es geometria, no un caso raro: yendo de home a (10, -9, -30) el eje 3
 * invierte a mitad de camino. Ese eje esta en su punto de retorno, o sea a
 * velocidad casi cero, asi que dejarlo frenar no cuesta nada; frenar a los
 * otros dos por acompanarlo es lo que se siente como un tiron.
 *
 * Tres caminos por eje:
 *   - se puede redirigir  -> se redirige, sin soltar la velocidad;
 *   - no se puede y esta andando -> se lo deja terminar el tramo que tiene
 *     (esta llegando a su punto de retorno, que es justo donde queria ir);
 *   - no se puede y esta parado -> moveTo, que en un eje detenido es seguro.
 *
 * Lo que se paga es que los ejes dejan de llegar exactamente juntos, y por
 * eso esto NO sirve para un vertice: el desfasaje esta acotado por lo que
 * ese eje recorre en UN tramo, que es chico justamente porque el eje que se
 * queda atras es el que casi no se mueve.
 *
 * El reparto es ESTRICTAMENTE proporcional, y eso no es un detalle: con
 * velocidad y aceleracion en proporcion a k, la distancia de frenado de un
 * eje vale k * v^2/(2a) y lo que ese eje tiene para recorrer en el tramo
 * vale k * (pasos del eje dominante). La k se cancela, o sea que si el
 * tramo le alcanza al eje que mas recorre, le alcanza a los tres.
 *
 * Darle un piso al reparto rompe justamente eso: le sube la velocidad al
 * eje flaco sin darle mas distancia. Hubo una version con piso y es lo que
 * dejaba trabado un movimiento con un eje al 20 % del recorrido de otro
 * (por ejemplo ir a Y = -9, donde un eje recorre 51 grados y otro 11).
 *
 * Devuelve false solo si no habia nada que hacer.
 */
bool redirigirLineal(Stepper &m1, Stepper &m2, Stepper &m3,
                     long target1, long target2, long target3,
                     const MotionLimits &limits);

/**
 * Vuelve a poner en marcha a los ejes que estan DETENIDOS y todavia no
 * llegaron al destino. A los que estan andando no los toca.
 *
 * Es el complemento de redirigirLineal: ahi un eje que no se puede
 * encadenar se deja llegar, y queda parado hasta el tramo siguiente. Un
 * tramo de espera no se nota, pero varios seguidos si -- el eje que
 * invierte el sentido en medio de una recta se atrasa un poco en cada uno,
 * el atraso se acumula, y la punta se va yendo de la recta cada vez mas.
 * Eso es el zigzag del tramo final.
 *
 * Llamandola en cada vuelta del loop, ningun eje se queda quieto mientras
 * los otros avanzan: en cuanto frena, arranca de nuevo hacia el destino
 * vigente. Como recibe velocidad proporcional a lo que le FALTA, el que
 * viene atrasado se lleva mas y se pone a la par solo.
 */
void empujarDetenidos(Stepper &m1, Stepper &m2, Stepper &m3,
                      long target1, long target2, long target3,
                      const MotionLimits &limits);

bool redirigirSincronizado(Stepper &m1, Stepper &m2, Stepper &m3,
                            long target1, long target2, long target3,
                            const MotionLimits &limits = DEFAULT_LIMITS);

}