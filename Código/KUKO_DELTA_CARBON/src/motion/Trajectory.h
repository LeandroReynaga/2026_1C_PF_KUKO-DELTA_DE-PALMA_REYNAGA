#pragma once

#include <Arduino.h>

#include "../hardware/Motors.h"
#include "../robot/Stepper.h"

/**
 * Movimiento LINEAL de la punta (movL), contra el movimiento articular
 * (movJ) que hace Motors::moveSynchronized.
 *
 * ------------------------------------------------------------------
 *  QUE PROBLEMA RESUELVE
 * ------------------------------------------------------------------
 * moveSynchronized reparte velocidad y aceleracion en proporcion al
 * recorrido de cada eje, asi que los tres motores arrancan y llegan juntos:
 * eso es una recta en el espacio de LOS ANGULOS, no en el espacio de la
 * punta. En un delta las dos cosas no se parecen. Medido con la cinematica
 * de este mismo robot (pc/tests/test_lineal.py, que es el espejo en Python
 * de este archivo):
 *
 *     tramo                              largo     se aparta de la recta
 *     cruzar la cinta en X               24,0 cm         27,3 mm
 *     de un tacho al otro                16,0 cm         10,2 mm
 *     bajada vertical en el centro        6,0 cm          0,0 mm
 *     bajada vertical en una esquina      6,0 cm          1,2 mm
 *     diagonal larga                     27,4 cm         33,4 mm
 *
 * O sea: yendo de un tacho al otro, el brazo pasa un centimetro mas abajo
 * (o mas arriba) de lo que uno cree. Al aire libre no importa y movJ es mas
 * rapido; metiendo una pieza entre las paredes de una celda, o bajando
 * sobre una pieza, si importa.
 *
 * ------------------------------------------------------------------
 *  COMO SE HACE
 * ------------------------------------------------------------------
 * Partiendo la recta en tramos cortos y resolviendo la cinematica en cada
 * punto intermedio: adentro de un tramo sigue siendo movJ, pero el error de
 * movJ cae con el CUADRADO del largo del tramo. Medido igual que arriba,
 * sobre la diagonal de 27,4 cm:
 *
 *     tramo de 0,5 cm  ->  0,013 mm        tramo de 3 cm  ->  0,38 mm
 *     tramo de 1,0 cm  ->  0,049 mm        tramo de 5 cm  ->  1,06 mm
 *     tramo de 2,0 cm  ->  0,194 mm        tramo de 8 cm  ->  2,35 mm
 *
 * Los tramos NO se frenan uno por uno: se encadenan con
 * Motors::redirigirSincronizado, que cambia el destino conservando la
 * velocidad. Un movimiento de veinte frenadas y veinte arranques seria mas
 * lento y mas brusco que el movJ que se quiere mejorar.
 *
 * ------------------------------------------------------------------
 *  QUIEN COORDINA: LAS VELOCIDADES, NO LOS DESTINOS
 * ------------------------------------------------------------------
 * A los motores se les da SIEMPRE el punto final de la recta como destino.
 * Los puntos intermedios no son destinos: sirven para recalcular, tramo a
 * tramo, EN QUE PROPORCION tiene que ir la velocidad de cada eje.
 *
 * Es la diferencia con la primera version, que le daba a los motores el
 * punto intermedio. Stepper planifica una rampa trapezoidal hasta el
 * destino que recibe, asi que con el punto intermedio de destino cada tramo
 * terminaba en una frenada planificada y encadenar era pelearle a esa
 * frenada. De ahi salian los tres sintomas que se vieron en el robot:
 *
 *   - el movimiento a los tirones;
 *   - la regla de que el tramo tenia que medir mas que la distancia de
 *     frenado (que ataba el paso a la velocidad);
 *   - y que ir MAS RAPIDO se viera mejor -- porque el paso crecia con la
 *     velocidad y habia menos frenadas.
 *
 * Con el destino en el final hay UNA sola rampa para toda la recta: acelera
 * al principio, frena al final, y en el medio no planifica ninguna parada.
 * Lo que mantiene la punta sobre la recta es que las velocidades esten en
 * la proporcion correcta, y eso se recalcula al cruzar cada punto
 * intermedio. Un control industrial hace exactamente esto; la unica
 * diferencia es que alla el reparto se recalcula a 1 kHz y aca una vez por
 * tramo.
 *
 * Se recalcula cuando los TRES ejes cruzaron el punto intermedio, no cuando
 * lo cruzo el que mas recorre: hacerlo con uno todavia atras es, por
 * definicion, salirse de la recta.
 *
 * Y el reparto sale de LO QUE LE FALTA a cada eje para el punto intermedio,
 * no de lo que ese tramo mide. Es lo que hace que la recta se corrija sola:
 * el eje atrasado se lleva la velocidad mas alta y el adelantado queda al
 * ralenti, asi que los tres cruzan el punto JUNTOS. Repartiendo por la
 * geometria del tramo nadie corrige nada -- el desvio se arrastra y el
 * movimiento se termina pareciendo un movJ, que es lo que se veia.
 *
 * ------------------------------------------------------------------
 *  LO QUE PUEDE SALIR MAL, Y COMO SE AVISA
 * ------------------------------------------------------------------
 * - **La recta puede salirse del alcance aunque las dos puntas esten
 *   adentro.** El volumen alcanzable de un delta no es convexo: buscando al
 *   azar sobre todo el alcance, 342 de 60.000 rectas entre dos puntos
 *   alcanzables se salen (medido en pc/tests/test_lineal.py). Adentro del
 *   cajon de teach no se encontro ninguna en 40.000, asi que hoy el chequeo
 *   casi nunca salta -- pero cuesta unos pocos cientos de microsegundos y el
 *   dia que movL se use en el ciclo normal, con la caja puesta, es la
 *   diferencia entre un mensaje y un brazo trabado. Por eso `comenzar`
 *   recorre la recta entera resolviendo la cinematica ANTES de mover un
 *   paso, y devuelve false sin haber movido nada. movJ no tiene este
 *   problema, y es de las pocas cosas en las que movL es peor.
 *
 * - **Un eje puede invertir el sentido en medio de la recta**, y no es un
 *   caso raro: yendo de home a (10, -9, -30) el eje 3 invierte a mitad de
 *   camino. Invertir con el rotor girando no lo sigue ningun paso a paso,
 *   asi que ese eje tiene que pasar por velocidad cero si o si.
 *
 *   Lo que NO tiene que pasar es que frenen los otros dos por acompanarlo:
 *   eso se veia como dos frenadas secas en mitad del recorrido. Por eso el
 *   encadenado usa Motors::redirigirLineal, que resuelve eje por eje en vez
 *   de "los tres o ninguno" -- el que invierte esta en su punto de retorno,
 *   o sea a velocidad casi cero, y dejarlo frenar solo no cuesta nada.
 */
class MovimientoLineal
{
public:
    enum class Estado : uint8_t
    {
        QUIETO,        // no hay nada en curso
        EN_CURSO,
        TERMINADO,     // llego al punto final
        SIN_SOLUCION   // un tramo se quedo sin cinematica (se corto)
    };

    struct Punto
    {
        float x, y, z;
    };

    struct Config
    {
        float pasoCm = 0.01f;   // largo nominal; se sube solo si no alcanza
        float velCms = 40.0f;   // velocidad de la PUNTA, no de los motores
        float acel   = 97000.0f; // pasos/s2 del eje que mas recorre
    };

    // Se llama una sola vez, con los motores del robot. Guardar punteros y
    // no recibirlos en cada llamada es lo que deja que `actualizar()` se
    // llame desde el loop sin arrastrar tres argumentos.
    void begin(Stepper &m1, Stepper &m2, Stepper &m3);

    // Prepara y lanza el primer tramo. false = no se movio nada (la recta
    // no tiene solucion en algun punto, o el largo es despreciable).
    bool comenzar(const Punto &desde, const Punto &hasta, const Config &cfg);

    // Una vuelta del loop. Encadena tramos y detecta el final.
    Estado actualizar();

    void cancelar();

    bool  enCurso() const { return activo; }
    Punto comandado() const { return { cmdX, cmdY, cmdZ }; }
    Punto destino() const { return { finX, finY, finZ }; }

    // Diagnostico del ultimo movimiento: cuantos tramos salieron y en
    // cuantos hubo que frenar porque no se pudo encadenar.
    uint16_t tramos() const { return tramosTotales; }
    uint16_t frenadas() const { return frenadasTotales; }
    float    pasoEfectivoCm() const { return pasoCm; }

private:
    Stepper *mot[3] = { nullptr, nullptr, nullptr };

    bool activo = false;

    // Punta de la recta y su largo. El origen se guarda para poder calcular
    // cada punto intermedio por interpolacion pura, sin ir acumulando.
    float iniX = 0.0f, iniY = 0.0f, iniZ = 0.0f;
    float finX = 0.0f, finY = 0.0f, finZ = 0.0f;
    float largoCm = 0.0f;

    // Ultimo punto EMITIDO (el destino del tramo en curso).
    float cmdX = 0.0f, cmdY = 0.0f, cmdZ = 0.0f;

    float pasoCm = 1.0f;
    float velCms = 20.0f;
    float acel   = 40000.0f;

    uint16_t totalTramos = 0;   // en cuantos se partio la recta
    uint16_t tramoActual = 0;   // cual se esta recorriendo (1..totalTramos)

    uint16_t tramosTotales   = 0;
    uint16_t frenadasTotales = 0;

    // Pasos de cada eje en el ultimo punto EMITIDO. Con esto se saca la
    // relacion pasos/cm local de cada tramo, que es lo que convierte la
    // velocidad pedida en cm/s a la velocidad de los motores.
    long ultSteps[3] = { 0, 0, 0 };

    // Limites con los que salio el tramo en curso. Los necesita el empujon
    // a los ejes que se quedaron parados a mitad de camino.
    Motors::MotionLimits ultLimites;

    // Pasos del eje dominante que tiene el tramo en curso. Es contra esto
    // que se decide cuando adelantar el destino al tramo siguiente.
    long pasosTramo = 0;

    // Punto k de la recta, con k en [0, totalTramos]. El ultimo se devuelve
    // exacto y no interpolado: acumular en float dejaria el destino final a
    // unas micras del que se pidio, y ese es justo el numero que despues
    // aparece escrito en la pantalla.
    Punto puntoDe(uint16_t k) const;

    // Emite el tramo `k`: cinematica, limites y salida a los motores.
    // `encadenar` intenta primero redirigir sin frenar.
    bool emitir(uint16_t k, bool encadenar);

    bool llegaron() const;
};
