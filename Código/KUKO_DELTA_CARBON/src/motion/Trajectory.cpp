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

// Cuantas veces la distancia de frenado tiene que medir un tramo.
//
// NO es un margen de seguridad cualquiera: es lo que saca al sistema de un
// punto de equilibrio en el que se frena solo. Redirigir reinicia el indice
// de rampa en la distancia de frenado y despues el eje acelera; si se lo
// deja, la velocidad sube hasta que frenar cuesta TODO el tramo. Y ahi
// puedeRedirigir() pide 1,1 veces eso mas 4 pasos, o sea mas de lo que el
// tramo mide: se niega, y frena. Con el tramo en 3 veces la frenada, el eje
// llega a la esquina necesitando un tercio de lo que tiene.
//
// Es lo que se paga por no tener un planificador con look-ahead que reparta
// la frenada entre varios tramos: tramos mas largos, o sea menos parecido a
// una recta. A 3x sigue siendo 0,1 mm de desvio, contra los 27 mm de movJ.
static const float MARGEN_FRENADO = 3.0f;

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

    pasoCm = (cfg.pasoCm > 0.01f) ? cfg.pasoCm : 0.01f;

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

    const long meta[3] = { pose.steps1, pose.steps2, pose.steps3 };

    // Cuanto le toca a cada eje en ESTE tramo. Se mide contra el punto
    // emitido anterior y no contra donde estan los motores: la posicion real
    // viene atrasada a proposito (se recalcula antes de llegar), y usarla
    // daria un reparto y una velocidad que no son los del tramo.
    long d[3];
    long dominante = 0;

    for (uint8_t i = 0; i < 3; i++)
    {
        d[i] = labs(meta[i] - ultSteps[i]);

        if (d[i] > dominante) dominante = d[i];
    }

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
        // Tramo tan corto que no llega a un micropaso: se saltea entero.
        cmdX = p.x;  cmdY = p.y;  cmdZ = p.z;

        for (uint8_t i = 0; i < 3; i++)
        {
            ultSteps[i] = meta[i];
        }

        pasosTramo  = 0;
        tramoActual = k;

        return true;
    }

    // --- velocidad del tramo (del eje que mas recorre) ---
    float velTramo = velCms * ((float)dominante / tramoCm);

    if (velTramo > Motors::VEL_MAX) velTramo = Motors::VEL_MAX;

    const float techo = sqrtf(2.0f * acel * (float)dominante / MARGEN_FRENADO);

    if (velTramo > techo) velTramo = techo;

    // ------------------------------------------------------------------
    //  EL DESTINO ES EL PUNTO INTERMEDIO, Y TIENE QUE SERLO
    // ------------------------------------------------------------------
    // Se probo darles el punto FINAL y coordinar por velocidades, que es
    // como lo hace un control industrial. No funciona con este Stepper, y la
    // razon es una sola linea de redirigir(): "cn NO se toca". La velocidad
    // instantanea se conserva a proposito, asi que bajarle la velocidad al
    // eje que se adelanto NO lo frena -- solo le limita cuanto mas puede
    // acelerar. Sin poder frenar a nadie, el reparto no corrige nada y cada
    // eje termina acelerando hacia el final: eso es movJ, con panza y todo.
    //
    // Con el punto intermedio de destino, en cambio, la posicion queda
    // impuesta: el eje NO puede pasarse. Se paga en fluidez (cada punto es
    // una frenada planificada, y de ahi salen la regla del paso minimo y los
    // tirones), pero la recta sale recta.
    //
    // Salir de ese compromiso pide un reloj de pasos compartido con DDA
    // (un solo timer, los pasos repartidos por Bresenham). Es lo que hacen
    // GRBL y los controladores industriales, y es la reescritura pendiente.
    // ------------------------------------------------------------------
    // Y no el punto intermedio, que es la diferencia entre esto y lo que
    // habia antes. El punto intermedio no se usa como destino: se usa para
    // calcular EL REPARTO DE VELOCIDADES de este tramo, y nada mas.
    //
    // Por que. Stepper planifica una rampa trapezoidal hasta el destino que
    // se le da: si el destino es el punto intermedio, cada tramo termina en
    // una frenada planificada, y encadenar es pelearle a esa frenada -- de
    // ahi salian el "va a los tirones", el limite de que el tramo tenia que
    // medir mas que la distancia de frenado, y que ir MAS RAPIDO se viera
    // mejor (tramos mas largos = menos frenadas). Con el destino en el final
    // hay UNA sola rampa para toda la recta: acelera al principio, frena al
    // final y en el medio no planifica ninguna parada.
    //
    // Lo que hace que la punta vaya derecha, entonces, no es la sucesion de
    // destinos sino que las VELOCIDADES esten en la proporcion correcta en
    // cada tramo: los tres ejes avanzan como avanzarian yendo al punto
    // intermedio, pero sin frenar en el. Es como coordina un control
    // industrial, con la diferencia de que alla el reparto se recalcula a
    // 1 kHz y aca una vez por tramo.
    // --- el reparto sale de LO QUE LE FALTA A CADA EJE PARA EL PUNTO
    //     INTERMEDIO, no de lo que ese tramo mide ---
    //
    // Es lo que hace que la recta se corrija sola. Repartiendo por la
    // geometria del tramo, un eje que se quedo atras sigue yendo a la misma
    // velocidad que le tocaba: nadie lo hace alcanzar al resto, el desvio se
    // arrastra y el resultado se parece a un movJ. Repartiendo por lo que
    // falta, el que esta atrasado se lleva la velocidad mas alta y el que se
    // adelanto queda casi quieto, asi que los tres cruzan el punto
    // intermedio JUNTOS -- y tres ejes que pasan juntos por cada punto de la
    // recta es, exactamente, la recta.
    //
    // El destino sigue siendo el final del recorrido, o sea que esto no
    // vuelve a meter una frenada en cada punto: el punto intermedio se
    // apunta, no se toca.
    long falta[3];
    long faltaMax = 1;

    for (uint8_t i = 0; i < 3; i++)
    {
        falta[i] = labs(meta[i] - mot[i]->getPosition());

        if (falta[i] > faltaMax) faltaMax = falta[i];
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        // Piso chico y no cero: un eje que ya llego al punto intermedio no
        // puede frenar (su destino esta mas alla), asi que lo que se hace es
        // dejarlo al ralenti hasta que los otros lo alcancen.
        float parte = (float)falta[i] / (float)faltaMax;

        if (parte < 0.02f) parte = 0.02f;

        const float vel   = velTramo * parte;
        const float ac    = acel * parte;

        if (vel <= 0.0f || meta[i] == mot[i]->getPosition())
        {
            continue;
        }

        if (encadenar && mot[i]->puedeRedirigir(meta[i], ac))
        {
            mot[i]->redirigir(meta[i], vel, ac);
        }
        else if (!mot[i]->isMoving())
        {
            mot[i]->setSpeed(vel);
            mot[i]->setAcceleration(ac);
            mot[i]->moveTo(meta[i]);
        }
        // Andando y sin poder redirigir: es el eje que esta invirtiendo el
        // sentido. Se lo deja llegar y frenar -- tiene que pasar por cero
        // igual -- y lo vuelve a lanzar el empujon de actualizar().
    }

    cmdX = p.x;
    cmdY = p.y;
    cmdZ = p.z;

    for (uint8_t i = 0; i < 3; i++)
    {
        ultSteps[i] = meta[i];
    }

    ultLimites.maxSpeed        = velTramo;
    ultLimites.maxAcceleration = acel;

    pasosTramo  = dominante;
    tramoActual = k;
    tramosTotales++;

    return true;
}

// ------------------------------------------------------------------
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

    // Ningun eje se queda quieto mientras los otros avanzan. El que invierte
    // el sentido se deja frenar (tiene que pasar por cero), pero apenas paro
    // vuelve a salir hacia el destino vigente: sin esto se atrasa un poco en
    // cada tramo, el atraso se acumula y la punta se va yendo de la recta --
    // el zigzag del tramo final.
    Motors::empujarDetenidos(*mot[0], *mot[1], *mot[2],
                             ultSteps[0], ultSteps[1], ultSteps[2], ultLimites);

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

    // --- cuando emitir el punto siguiente ---
    // Cuando lo que le falta al eje que MAS le falta entra en un tramo. Se
    // encadena antes de llegar a proposito: llegar es frenar, y encadenar es
    // justamente evitar esa frenada. Se mira al que mas le falta -- y no al
    // que mas recorre -- porque adelantar el destino con un eje todavia
    // atras es, por definicion, salirse de la recta.
    if (max(mot[0]->pasosRestantes(),
            max(mot[1]->pasosRestantes(), mot[2]->pasosRestantes())) > pasosTramo)
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
