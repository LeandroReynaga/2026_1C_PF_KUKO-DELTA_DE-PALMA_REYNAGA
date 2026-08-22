#include "Trajectory.h"

#include "../kinematics/DeltaKinematics.h"

// Cota de tramos por movimiento. La recta mas larga que entra en el volumen
// de trabajo mide ~30 cm, asi que con el paso minimo razonable (0,2 cm) son
// 150; el tope esta para que un paso mal cargado no deje al brazo emitiendo
// tramos de micras durante un minuto.
static const uint16_t MAX_TRAMOS = 300;

// Recta mas corta que vale la pena partir. Por debajo de esto, movJ y movL
// son el mismo movimiento (el error de movJ cae con el cuadrado del largo).
static const float LARGO_MINIMO_CM = 0.05f;

// Margen sobre la distancia de frenado estimada al elegir el largo del
// tramo. La estimacion es v^2/(2a), que la rampa de Austin en punto fijo
// supera bastante (ver Stepper::pasosDeFrenado), asi que se toma al doble.
// Quedarse corto no rompe nada -- redirigirSincronizado se niega y el tramo
// se hace frenando --, pero se siente en el brazo.
static const float MARGEN_FRENADO = 2.0f;

// Piso del reparto de aceleracion entre ejes al encadenar (ver Motors.h).
// Sin el, en una diagonal la MITAD de los tramos termina en frenada porque
// el eje que menos recorre se queda sin aceleracion para poder frenar: es
// el "va a los tirones" que se veia en el robot.
static const float PISO_ESCALA = 0.25f;

// ------------------------------------------------------------------
void MovimientoLineal::begin(Stepper &m1, Stepper &m2, Stepper &m3)
{
    mot[0] = &m1;
    mot[1] = &m2;
    mot[2] = &m3;
}

// ------------------------------------------------------------------
MovimientoLineal::Punto MovimientoLineal::puntoDe(uint16_t k) const
{
    if (k >= totalTramos)
    {
        return { finX, finY, finZ };
    }

    const float s = (float)k / (float)totalTramos;

    return { iniX + s * (finX - iniX),
             iniY + s * (finY - iniY),
             iniZ + s * (finZ - iniZ) };
}

// ------------------------------------------------------------------
bool MovimientoLineal::comenzar(const Punto &desde, const Punto &hasta,
                                const Config &cfg)
{
    if (mot[0] == nullptr)
    {
        return false;
    }

    cancelar();

    iniX = desde.x;  iniY = desde.y;  iniZ = desde.z;
    finX = hasta.x;  finY = hasta.y;  finZ = hasta.z;

    const float dx = finX - iniX;
    const float dy = finY - iniY;
    const float dz = finZ - iniZ;

    largoCm = sqrtf(dx * dx + dy * dy + dz * dz);

    if (largoCm < LARGO_MINIMO_CM)
    {
        return false;
    }

    velCms = (cfg.velCms > 0.1f) ? cfg.velCms : 0.1f;
    acel   = (cfg.acel   > 1.0f) ? cfg.acel   : 1.0f;

    // --- largo del tramo ---
    // El pedido, pero nunca por debajo de lo que el eje necesita para
    // frenar: un destino mas cerca que eso lo rechaza Stepper::redirigir y
    // el movimiento se convierte en una sucesion de frenadas. Ver el
    // comentario largo del header.
    //
    // La cuenta va en centimetros de punta usando la relacion pasos/cm
    // MEDIA de esta recta, que es lo unico que se sabe antes de arrancar.
    // Adentro del volumen esa relacion va de ~69 a ~113 pasos/cm, asi que
    // usar la media se equivoca a lo sumo en un tercio; el margen de 2x que
    // se aplica encima cubre esa diferencia con comodidad.
    const DeltaKinematics::DeltaAngles a0 = DeltaKinematics::solveIK(iniX, iniY, iniZ);
    const DeltaKinematics::DeltaAngles a1 = DeltaKinematics::solveIK(finX, finY, finZ);

    if (!a0.success || !a1.success)
    {
        return false;
    }

    const long d1 = labs(a1.steps1 - a0.steps1);
    const long d2 = labs(a1.steps2 - a0.steps2);
    const long d3 = labs(a1.steps3 - a0.steps3);

    const float pasosPorCm = (float)max(d1, max(d2, d3)) / largoCm;

    pasoCm = (cfg.pasoCm > 0.05f) ? cfg.pasoCm : 0.05f;

    if (pasosPorCm > 1.0f)
    {
        const float velPasos  = velCms * pasosPorCm;
        const float frenadoCm = (velPasos * velPasos) / (2.0f * acel) / pasosPorCm;

        if (pasoCm < frenadoCm * MARGEN_FRENADO)
        {
            pasoCm = frenadoCm * MARGEN_FRENADO;
        }
    }

    long cuantos = (long)ceilf(largoCm / pasoCm);

    if (cuantos < 1)             cuantos = 1;
    if (cuantos > MAX_TRAMOS)    cuantos = MAX_TRAMOS;

    totalTramos = (uint16_t)cuantos;
    pasoCm      = largoCm / (float)totalTramos;

    // --- validacion de la recta ENTERA, antes de mover nada ---
    // El volumen alcanzable de un delta no es convexo: los dos extremos
    // pueden tener solucion y un punto del medio no. Descubrirlo a mitad de
    // camino significa frenar el brazo en cualquier lado; descubrirlo aca
    // significa un mensaje y nada mas. Son ~30 solveIK (unos pocos cientos
    // de microsegundos), una sola vez por movimiento.
    for (uint16_t k = 1; k <= totalTramos; k++)
    {
        const Punto p = puntoDe(k);

        if (!DeltaKinematics::solveIK(p.x, p.y, p.z).success)
        {
            return false;
        }
    }

    cmdX = iniX;  cmdY = iniY;  cmdZ = iniZ;

    ultSteps[0] = a0.steps1;
    ultSteps[1] = a0.steps2;
    ultSteps[2] = a0.steps3;

    tramosTotales   = 0;
    frenadasTotales = 0;
    tramoActual     = 0;
    activo          = true;

    if (!emitir(1, false))
    {
        activo = false;
        return false;
    }

    return true;
}

// ------------------------------------------------------------------
bool MovimientoLineal::emitir(uint16_t k, bool encadenar)
{
    const Punto p = puntoDe(k);

    const DeltaKinematics::DeltaAngles pose = DeltaKinematics::solveIK(p.x, p.y, p.z);

    if (!pose.success)
    {
        return false; // la validacion de comenzar() deberia haberlo visto
    }

    // --- velocidad ---
    // La velocidad se pide en centimetros de PUNTA por segundo, pero los
    // limites de Motors hablan en pasos: la conversion es la relacion
    // pasos/cm de ESTE tramo, que es lo que mantiene la punta a velocidad
    // pareja aunque la geometria cambie a lo largo de la recta. Cerca del
    // borde del volumen hacen falta muchos mas pasos por centimetro, y ahi
    // el tope de VEL_MAX hace que la punta baje la velocidad sola, que es
    // exactamente lo que corresponde.
    //
    // La relacion se saca contra el punto EMITIDO anterior y no contra la
    // posicion actual de los motores: la posicion actual viene un tramo
    // atrasada (justamente porque se encadena antes de llegar), asi que
    // usarla daria el doble de pasos para el mismo centimetro y la punta
    // saldria al doble de la velocidad pedida.
    const long e1 = labs(pose.steps1 - ultSteps[0]);
    const long e2 = labs(pose.steps2 - ultSteps[1]);
    const long e3 = labs(pose.steps3 - ultSteps[2]);

    const long dominante = max(e1, max(e2, e3));

    const float sx = p.x - cmdX;
    const float sy = p.y - cmdY;
    const float sz = p.z - cmdZ;

    float tramoCm = sqrtf(sx * sx + sy * sy + sz * sz);

    if (tramoCm < 1e-4f)
    {
        tramoCm = pasoCm;
    }

    if (dominante == 0)
    {
        // Tramo tan corto que no llega a un micropaso. Se saltea entero:
        // dejarlo sin marcar seria pedirlo de nuevo en la vuelta siguiente
        // y quedarse ahi para siempre.
        cmdX = p.x;
        cmdY = p.y;
        cmdZ = p.z;

        pasosTramo  = 0;
        tramoActual = k;

        return true;
    }

    Motors::MotionLimits limites;

    limites.maxSpeed        = velCms * ((float)dominante / tramoCm);
    limites.maxAcceleration = acel;

    if (limites.maxSpeed > Motors::VEL_MAX)
    {
        limites.maxSpeed = Motors::VEL_MAX;
    }

    bool encadenado = false;

    if (encadenar)
    {
        encadenado = Motors::redirigirSincronizado(*mot[0], *mot[1], *mot[2],
                                                   pose.steps1, pose.steps2,
                                                   pose.steps3, limites,
                                                   PISO_ESCALA);

        if (!encadenado)
        {
            // No se pudo seguir sin frenar (tipicamente, un eje que invierte
            // el sentido). Se espera a que paren los tres y se arranca de
            // nuevo: es una frenada, pero es limpia y siempre la misma.
            if (!llegaron())
            {
                return true; // todavia frenando; se reintenta el proximo loop
            }

            frenadasTotales++;
        }
    }

    if (!encadenado)
    {
        Motors::moveSynchronized(*mot[0], *mot[1], *mot[2],
                                 pose.steps1, pose.steps2, pose.steps3, limites);
    }

    cmdX = p.x;
    cmdY = p.y;
    cmdZ = p.z;

    ultSteps[0] = pose.steps1;
    ultSteps[1] = pose.steps2;
    ultSteps[2] = pose.steps3;

    pasosTramo  = dominante;
    tramoActual = k;
    tramosTotales++;

    return true;
}

// ------------------------------------------------------------------
long MovimientoLineal::restanteDominante() const
{
    return max(mot[0]->pasosRestantes(),
               max(mot[1]->pasosRestantes(), mot[2]->pasosRestantes()));
}

bool MovimientoLineal::llegaron() const
{
    return !mot[0]->isMoving() && !mot[1]->isMoving() && !mot[2]->isMoving();
}

// ------------------------------------------------------------------
MovimientoLineal::Estado MovimientoLineal::actualizar()
{
    if (!activo)
    {
        return Estado::QUIETO;
    }

    // --- final ---
    if (tramoActual >= totalTramos)
    {
        if (!llegaron())
        {
            return Estado::EN_CURSO;
        }

        activo = false;
        return Estado::TERMINADO;
    }

    // --- cuando adelantar el destino al tramo siguiente ---
    // Se adelanta cuando lo que falta del tramo en curso cabe en un tramo,
    // o sea manteniendo el destino entre uno y dos tramos por delante de la
    // punta. Esa distancia es la que gobierna las dos cosas que importan:
    // el error contra la recta (crece con ella) y la velocidad de crucero
    // que el eje puede sostener sin tener que frenar (tambien crece con
    // ella). Ver el header.
    if (restanteDominante() > pasosTramo)
    {
        return Estado::EN_CURSO;
    }

    if (!emitir(tramoActual + 1, true))
    {
        cancelar();
        return Estado::SIN_SOLUCION;
    }

    return Estado::EN_CURSO;
}

// ------------------------------------------------------------------
void MovimientoLineal::cancelar()
{
    activo      = false;
    tramoActual = 0;
    totalTramos = 0;
    pasosTramo  = 0;
}
