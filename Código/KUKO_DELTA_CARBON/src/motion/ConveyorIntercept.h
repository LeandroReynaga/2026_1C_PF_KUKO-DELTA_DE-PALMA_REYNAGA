#pragma once
#include <Arduino.h>

/**
 * ConveyorIntercept
 * ------------------
 * Resuelve DONDE y CUANDO el robot tiene que ir a buscar una pieza que se
 * mueve sobre la cinta a velocidad constante.
 *
 * La maniobra de agarre son DOS tramos encadenados con aceleraciones
 * distintas (por eso este modulo no puede razonar sobre un solo movimiento):
 *
 *   Tramo 1 - aceleracion MAXIMA, desde donde este el robot hasta el punto
 *             de aproximacion, que esta (approachDX, 0, approachDZ) respecto
 *             del centro de la pieza EN EL INSTANTE DEL CONTACTO.
 *   Tramo 2 - aceleracion MINIMA, hacia +X (a favor de la cinta) y hacia
 *             abajo, hasta tocar la pieza con la MISMA velocidad que la
 *             cinta (velocity-matched).
 *
 * ENCUENTRO VELOCITY-MATCHED
 * ---------------------------
 * Un movimiento punto-a-punto SIEMPRE termina con velocidad cero. Si el
 * tramo 2 terminara justo en el centro de la pieza, el gripper la tocaria
 * estando frenado mientras la pieza sigue a la velocidad de la cinta: el
 * mismo golpe que si el robot esperara quieto.
 *
 * Para que el contacto ocurra con el gripper todavia en movimiento, el
 * tramo 2 apunta a un punto que SOBREPASA a la pieza en X y queda pressDZ
 * por debajo de su cara superior. Asi el gripper cruza la altura de la
 * pieza a mitad del movimiento (no al final), justo cuando su velocidad en
 * X vale lo mismo que la cinta. Ese pressDZ ademas comprime un poco la
 * ventosa, que es lo que se busca para que selle bien.
 *
 * La condicion geometrica que hace que el contacto caiga EXACTAMENTE en el
 * centro de la pieza sale sola:
 *
 *      s_contacto = approachDZ / (approachDZ + pressDZ)
 *      overshootDX = |approachDX| * pressDZ / approachDZ
 *
 * (s_contacto es la fraccion del tramo recorrida al tocar). Con los valores
 * del robot -approachDZ = 4 mm, pressDZ = 0,25 mm- el contacto cae al 94 %
 * del recorrido y el sobrepaso es de 1,25 mm.
 *
 * pressDZ es el unico parametro a tocar para ajustar la velocidad de
 * contacto: mas presion -> contacto mas temprano -> mas rapido.
 *
 * El problema es circular: donde va a estar la pieza depende de cuanto
 * tarda el robot, y cuanto tarda el robot depende de a donde va. Se
 * resuelve con iteracion de punto fijo sobre la coordenada X de agarre.
 *
 * OJO con una asimetria importante: el robot es MUCHO mas rapido que la
 * cinta (los dos tramos juntos tardan ~0,4 s y la pieza tarda ~1,7 s en
 * cruzar desde la linea de deteccion hasta el area de trabajo). O sea que
 * casi siempre el robot llega ANTES y lo que hace falta no es apurarlo
 * sino decirle CUANDO arrancar. Por eso el resultado no es solo un punto:
 * incluye descendStartDelay, el momento en que hay que lanzar el tramo 2.
 *
 * El patron de uso es:
 *   1) mandar el robot al punto de aproximacion (tramo 1) apenas se puede;
 *   2) esperar ahi hasta descendStartDelay;
 *   3) lanzar el tramo 2, que toca la pieza justo cuando pasa por ahi.
 * Asi la precision del encuentro depende SOLO del tramo 2 (corto y
 * repetible), no de cuanto tardo el robot en llegar desde donde estaba.
 *
 * No toca motores ni hace I/O: solo calcula.
 */
namespace ConveyorIntercept {

struct BeltConfig {
    float velocityX;      // cm/s, CON SIGNO (direccion de avance de la pieza en X)
    float detectionLineX; // coordenada X (cm, sistema del robot) de la linea de deteccion
};

/**
 * Geometria de la maniobra de agarre. Todas las coordenadas Z son de la
 * PUNTA del gripper (DeltaKinematics ya descuenta el offset de herramienta).
 */
struct PickGeometry {
    float grabZ;        // altura de la cara superior de la pieza
    float approachDX;   // desplazamiento X del punto de aproximacion (negativo: por detras)
    float approachDZ;   // desplazamiento Z del punto de aproximacion (positivo: por arriba)
    float pressDZ;      // cuanto se hunde por debajo de la cara de la pieza (comprime la ventosa)
    float liftDZ;       // cuanto sube despues de agarrar, para despegarla de la cinta
    float workAreaMinX; // area donde es seguro agarrar (validada a mano sobre el robot)
    float workAreaMaxX;
};

struct InterceptResult {
    // Tramo 1: punto de aproximacion (aceleracion maxima)
    float approachX, approachY, approachZ;

    // Punto de CONTACTO: centro de la pieza en el instante del agarre.
    // No es el destino de ningun movimiento, es donde ocurre el encuentro
    // (a mitad del tramo 2). Sirve para diagnostico.
    float grabX, grabY, grabZ;

    // Tramo 2: destino comandado (aceleracion minima). Sobrepasa a la pieza
    // en X y baja pressDZ debajo de su cara, para que el contacto ocurra en
    // movimiento y no al final del recorrido.
    float descendEndX, descendEndY, descendEndZ;

    // Tramo 3: punto al que se levanta la pieza (aceleracion maxima)
    float liftX, liftY, liftZ;

    float t1;                 // duracion estimada del tramo 1 (s)
    float t2;                 // duracion TOTAL del tramo 2 (s)
    float tContact;           // desde que arranca el tramo 2 hasta tocar la pieza (s)
    float contactSpeedX;      // velocidad X del gripper al tocar (cm/s), para diagnostico
    float descendStartDelay;  // segundos DESDE AHORA hasta lanzar el tramo 2

    bool reachable;           // false: no llega dentro del area de trabajo -> dejar pasar la pieza
};

// ============================================================
//  LIMITES DE BUSQUEDA
// ============================================================
constexpr int MAX_ITERATIONS = 15;
constexpr float CONVERGENCE_TOLERANCE_CM = 0.01f; // 0,1 mm: mas fino que la repetibilidad mecanica

/**
 * Planifica la maniobra completa de agarre de una pieza.
 *
 * pieceY: coordenada Y del centro de la pieza (cm). No cambia: la cinta
 *   mueve la pieza solo en X.
 * timeSinceDetection: segundos desde que la pieza cruzo la linea de
 *   deteccion hasta AHORA.
 * currentSteps1/2/3: posicion ACTUAL de los 3 motores, para estimar el
 *   tramo 1 desde donde realmente esta el robot.
 * maxSpeed, accelFast, accelSlow: los MISMOS limites con los que se van a
 *   comandar los movimientos reales (si no coinciden, la prediccion de
 *   tiempo no sirve).
 *
 * Devuelve reachable=false si la pieza ya no se puede agarrar dentro del
 * area de trabajo (llego tarde, o el punto no es alcanzable): en ese caso
 * hay que descartarla y pasar a la siguiente de la cola.
 */
InterceptResult solve(float pieceY,
                       float timeSinceDetection,
                       const BeltConfig &belt,
                       const PickGeometry &geom,
                       long currentSteps1, long currentSteps2, long currentSteps3,
                       float maxSpeed, float accelFast, float accelSlow);

} // namespace ConveyorIntercept
