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
 * rapido; metiendo un alfajor entre las paredes de una celda, o bajando
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
 *  EL LIMITE QUE IMPONE ESTA ARQUITECTURA (leer antes de tocar el paso)
 * ------------------------------------------------------------------
 * Stepper::redirigir solo acepta un destino que este a MAS de la distancia
 * de frenado desde la velocidad actual: si no, el eje llegaria andando y se
 * cortarian los pulsos. Como no hay un planificador con look-ahead que
 * reparta la frenada entre varios tramos, cada tramo tiene que ser mas
 * largo que la frenada. Con la aceleracion de teach (40.000 pasos/s2) y
 * ~90 pasos/cm eso da:
 *
 *     a 17 cm/s   frenada 0,4 cm      a 50 cm/s   frenada 3,8 cm
 *     a 33 cm/s   frenada 1,7 cm      a 100 cm/s  frenada 15 cm
 *
 * Por eso `comenzar` sube el paso pedido hasta esa cota si hace falta, y
 * por eso movL a toda velocidad no tiene sentido: a 100 cm/s los tramos
 * tendrian que medir 15 cm y el resultado seria movJ otra vez. El punto
 * dulce, y el valor de fabrica, es 1 cm de paso a 20 cm/s.
 *
 * Consecuencia util del mismo mecanismo: la velocidad se autolimita en
 * v = sqrt(2 * a * mira), donde `mira` es lo que se adelanta el destino. Es
 * decir que pedir mas velocidad que esa no la acelera; lo que la sube es
 * agrandar el paso, y eso se paga en precision.
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
 * - **Un eje puede invertir el sentido en medio de la recta.** Ahi
 *   redirigirSincronizado se niega (y hace bien: invertir con el rotor
 *   girando no lo sigue ningun paso a paso). Se frena, se espera y se
 *   arranca el tramo siguiente de cero. Se cuentan esas frenadas y se
 *   informan al terminar: si un movimiento trae muchas, el brazo se va a
 *   ver a los tirones y la explicacion esta en ese numero.
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
        float pasoCm = 1.0f;    // largo nominal de cada tramo
        float velCms = 20.0f;   // velocidad de la PUNTA, no de los motores
        float acel   = 40000.0f; // pasos/s2 del eje que mas recorre
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
    long restanteDominante() const;
};
