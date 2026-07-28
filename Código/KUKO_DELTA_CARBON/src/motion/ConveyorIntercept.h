#pragma once
#include <Arduino.h>

/**
 * ConveyorIntercept
 * ------------------
 * Resuelve DONDE y CUANDO el robot tiene que ir a buscar una pieza que se
 * mueve sobre la cinta a velocidad constante, para que el brazo llegue
 * exactamente cuando la pieza pasa por ese punto.
 *
 * El problema es circular: la posicion futura de la pieza depende de
 * cuanto tarda el robot en llegar, y cuanto tarda el robot depende de a
 * donde va. Se resuelve con iteracion de punto fijo: se arranca con una
 * suposicion del tiempo de viaje, se calcula donde estaria la pieza en ese
 * momento, se calcula cuanto tardaria REALMENTE el robot en llegar ahi, y
 * se repite hasta que el tiempo deja de cambiar. Convergio en 1-5
 * iteraciones en todas las pruebas hechas (validado antes en Python).
 *
 * No toca motores ni hace I/O: solo calcula. Quien llama decide que hacer
 * con el resultado (llamar a Motors::moveSynchronized, etc).
 */
namespace ConveyorIntercept {

struct BeltConfig {
    float velocityX;      // cm/s, CON SIGNO (direccion de avance de la pieza en X)
    float detectionLineX; // coordenada X (cm, sistema del robot) de la linea de deteccion
};

struct InterceptResult {
    float x, y, z;      // punto (cm) donde el robot tiene que ir a buscar la pieza
    float etaSeconds;   // cuanto va a tardar el robot en llegar ahi, desde AHORA
    bool reachable;      // false si no convergio o el punto queda fuera del volumen de trabajo
};

// ============================================================
//  LIMITES DE BUSQUEDA (ajustar si hace falta)
// ============================================================
constexpr int MAX_ITERATIONS = 15;
constexpr float CONVERGENCE_TOLERANCE_S = 0.001f; // 1ms: suficiente, no hace falta mas precision

/**
 * Calcula donde y cuando interceptar una pieza.
 *
 * pieceY, pieceZ: coordenadas de la pieza (cm). Z es siempre la misma
 *   (altura fija de la pieza sobre la cinta).
 * timeSinceDetection: segundos transcurridos desde que la pieza cruzo la
 *   linea de deteccion hasta AHORA (tipicamente el retardo de comunicacion
 *   + procesamiento; chico, pero se tiene en cuenta).
 * belt: velocidad de la cinta y posicion de la linea de deteccion.
 * currentSteps1/2/3: posicion ACTUAL de los 3 motores (para saber cuanto
 *   tardaria el robot en llegar a cada candidato durante la iteracion).
 * maxSpeed, maxAcceleration: los mismos limites que usa Motors::moveSynchronized
 *   para el movimiento real (tienen que coincidir, si no la prediccion de
 *   tiempo no va a ser precisa).
 */
InterceptResult solve(float pieceY, float pieceZ,
                       float timeSinceDetection,
                       const BeltConfig &belt,
                       long currentSteps1, long currentSteps2, long currentSteps3,
                       float maxSpeed, float maxAcceleration);

} // namespace ConveyorIntercept