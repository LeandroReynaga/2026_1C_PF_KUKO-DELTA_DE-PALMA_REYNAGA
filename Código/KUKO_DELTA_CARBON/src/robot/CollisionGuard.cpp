#include "CollisionGuard.h"

#include "../hardware/Encoders.h"
#include "../kinematics/DeltaKinematics.h"

#include <math.h>

namespace {

// Cuantos grados de eje representa un micropaso.
constexpr float GRADOS_POR_PASO = 1.0f / DeltaKinematics::STEPS_PER_DEGREE;

// Espeja los INVERT_MOTORn de DeltaKinematics: si alla se invierte el
// sentido de un motor, los micropasos de ese eje cuentan al reves que el
// angulo articular y aca hay que interpretarlos igual, o la comparacion
// contra el encoder queda invertida.
constexpr float SIGNO_PASOS[3] = {
    DeltaKinematics::INVERT_MOTOR1 ? -1.0f : 1.0f,
    DeltaKinematics::INVERT_MOTOR2 ? -1.0f : 1.0f,
    DeltaKinematics::INVERT_MOTOR3 ? -1.0f : 1.0f
};

// Un eje invertido se distingue de un eje trabado por lo que hace el
// encoder: el trabado se queda donde esta (denc chico, del mismo signo o
// nulo), el invertido SIGUE al movimiento pero espejado, o sea denc ~ -dcmd.
//
// Por eso no alcanza con "denc va para el otro lado": un brazo que choca y
// rebota un poco tambien da un denc negativo chiquito, y confundirlo con un
// error de cableado apagaria la supervision justo cuando hace falta. Se
// exige que la relacion este cerca de -1 y sobre un recorrido grande (el
// primer movimiento despues del homing barre unos 45 grados, de sobra).
constexpr float RECORRIDO_MIN_PARA_JUZGAR_DEG = 10.0f;
constexpr float RATIO_INVERTIDO_MIN           = 0.6f;
constexpr float RATIO_INVERTIDO_MAX           = 1.6f;

} // namespace

constexpr float CollisionGuard::VEL_QUIETO_DEG_S;
constexpr float CollisionGuard::RECORRIDO_MIN_GANANCIA_DEG;
constexpr float CollisionGuard::VEL_MIN_PARA_ATRASO_DEG_S;

// Cuanto tiene que llevar frenado un eje para darlo por ASENTADO, o sea
// para que lo que marca el encoder ya se haya puesto al dia con los pasos.
// Con atrasos de la cadena de medicion de hasta ~100 ms, 400 ms son cuatro
// constantes de tiempo: queda menos del 2% del transitorio.
//
// Es la condicion tanto para medir la ganancia (si se midiera antes, el
// atraso se leeria como ganancia menor a 1) como para dejar actuar la fuga
// en reposo (si se fugara durante el transitorio, se estaria metiendo en la
// referencia un error que despues aparece con el signo contrario).
static const uint32_t ASENTAMIENTO_MS = 400;

// ------------------------------------------------------------------
void CollisionGuard::begin()
{
    est = DESARMADO;
    inhibidoPorConfig = false;

    umbralDeg         = GuardConfig::UMBRAL_DEG;
    confirmacionMs    = GuardConfig::CONFIRMACION_MS;
    margenVelocidadMs = GuardConfig::MARGEN_VELOCIDAD_MS;

    ultimoUs = micros();

    for (uint8_t i = 0; i < NUM_EJES; i++)
    {
        encDelta[i]        = 0.0f;
        cmdDelta[i]        = 0.0f;
        cmdDegPrev[i]      = 0.0f;
        velCmd[i]          = 0.0f;
        picoVel[i]         = 0.0f;
        ganancia[i]        = 0.0f;
        quietoDesde_ms[i]  = 0;
        atrasoSeg[i]       = 0.0f;
        fugaAcumulada[i]   = 0.0f;
        cmdCompensado[i]   = 0.0f;
        faltaMax[i]        = 0.0f;
        rawMin[i]          = 0xFFFF;
        rawMax[i]          = 0;
        ciego[i]           = false;
        refEnc[i]        = 0.0f;
        refPasos[i]      = 0;
        refValida[i]     = false;
        sumaEnc[i]       = 0.0;
        muestras[i]      = 0;
        errorDeg[i]      = 0.0f;
        enFalta[i]       = false;
        faltaDesde_ms[i] = 0;
        pico[i]          = 0.0f;
        picoHist[i]      = 0.0f;
        caido[i]         = false;
        avisoCaido[i]    = false;
        ultimoAvisoCaido_ms[i] = 0;
        enFaltaRep[i]       = false;
        faltaRepDesde_ms[i] = 0;
        picoRep[i]          = 0.0f;
        avisoDeriva[i]      = false;
        desvioRep[i]        = 0.0f;
        desvioRepListo[i]   = false;
        firmaRep[i]         = 0.0f;
        firmaLista[i]       = false;
        reposoDesde_ms[i]   = 0;
        firmaPrevia[i]         = 0.0f;
        hayFirmaPrevia[i]      = false;
        recorridoDesdeFirma[i] = 0.0f;
    }

    saltoParadaDeg       = GuardConfig::SALTO_PARADA_DEG;
    saltoParadaFrac      = GuardConfig::SALTO_PARADA_FRAC;
    umbralReposoDeg      = GuardConfig::UMBRAL_REPOSO_DEG;
    confirmacionReposoMs = GuardConfig::CONFIRMACION_REPOSO_MS;
    enHome               = false;

    motivoFallo   = MOTIVO_COLISION;
    ejeFallo      = 0;
    errorFallo    = 0.0f;
    cmdDeltaFallo = 0.0f;
    encDeltaFallo = 0.0f;
}

// ------------------------------------------------------------------
void CollisionGuard::setEnHome(bool estaEnHome)
{
    if (!estaEnHome)
    {
        // Al salir de home se descarta cualquier sospecha de reposo a medio
        // confirmar: desde que el brazo arranca a moverse, lo que se mida ya
        // no es comparable con la pose de referencia.
        for (uint8_t i = 0; i < NUM_EJES; i++)
        {
            enFaltaRep[i]     = false;
            desvioRepListo[i] = false;
            firmaLista[i]     = false;
        }
    }

    enHome = estaEnHome;
}

// ------------------------------------------------------------------
float CollisionGuard::leerEncoder(uint8_t eje) const
{
    // Se usa el angulo continuo CRUDO (sin el offset de calibracion de
    // home) a proposito: el offset cambia justo en el medio de la ventana
    // de promediado, cuando Robot llama a Encoders::calibrarHoming(). Si la
    // referencia se promediara con el offset viejo y las lecturas vivas
    // usaran el nuevo, la diferencia entre los dos apareceria como un error
    // fijo de decenas de grados. Al ser todo diferencial, el offset no hace
    // ninguna falta.
    return GuardConfig::ENCODER_SIGN[eje] * encoders.leerGradosContinuoCrudo(eje);
}

float CollisionGuard::pasosAGrados(uint8_t eje, long pasos)
{
    return SIGNO_PASOS[eje] * (float)pasos * GRADOS_POR_PASO;
}

// ------------------------------------------------------------------
void CollisionGuard::iniciarReferencia(long pasos1, long pasos2, long pasos3)
{
    if (inhibidoPorConfig)
    {
        return; // hay un encoder mal configurado: no se supervisa hasta arreglarlo
    }

    const long pasos[NUM_EJES] = {pasos1, pasos2, pasos3};

    for (uint8_t i = 0; i < NUM_EJES; i++)
    {
        sumaEnc[i]       = 0.0;
        muestras[i]      = 0;
        refPasos[i]      = pasos[i];
        refValida[i]     = false;
        errorDeg[i]      = 0.0f;
        enFalta[i]       = false;
        faltaDesde_ms[i] = 0;
        pico[i]          = 0.0f;
    }

    est = PROMEDIANDO;
}

// ------------------------------------------------------------------
void CollisionGuard::fijarReferencia()
{
    if (inhibidoPorConfig)
    {
        est = DESARMADO;
        return;
    }

    for (uint8_t i = 0; i < NUM_EJES; i++)
    {
        // Un canal que tuvo que reengancharse a la fuerza arrastra un salto
        // de origen desconocido (movimiento real que no se siguio, o una
        // falla del sensor). Si eso paso DURANTE la ventana de promediado,
        // la referencia que se acaba de juntar esta contaminada y no sirve.
        // Se limpia la bandera igual: el proximo homing arranca limpio.
        const bool huboResync = encoders.huboResincronizacion(i);
        encoders.limpiarResincronizacion(i);

        // Estado de "encoder caido" del ciclo anterior: se rehace en cada
        // homing, asi un canal que se recupero vuelve a supervisarse.
        caido[i]               = false;
        avisoCaido[i]          = false;
        ultimoAvisoCaido_ms[i] = 0;

        if (huboResync)
        {
            refValida[i] = false;
            Serial.print("[GUARD] eje ");
            Serial.print(i + 1);
            Serial.println(" se resincronizo durante el homing: queda SIN supervisar");
        }
        else if (muestras[i] > 0)
        {
            refEnc[i]    = (float)(sumaEnc[i] / (double)muestras[i]);
            refValida[i] = true;
        }
        else if (encoders.estaInicializado(i))
        {
            // Sin ventana de promediado (no deberia pasar): se cae a una
            // lectura puntual, que arrastra el ruido del encoder a la
            // referencia pero sigue siendo utilizable.
            refEnc[i]    = leerEncoder(i);
            refValida[i] = true;
        }
        else
        {
            // Encoder que nunca leyo: ese eje queda sin supervisar.
            refValida[i] = false;
            Serial.print("[GUARD] eje ");
            Serial.print(i + 1);
            Serial.println(" sin encoder: queda SIN supervisar");
        }

        enFalta[i]       = false;
        faltaDesde_ms[i] = 0;
        errorDeg[i]      = 0.0f;
        pico[i]          = 0.0f;

        // Estado dinamico: arranca desde el reposo real del robot, que es
        // donde estamos parados ahora mismo.
        encDelta[i]        = 0.0f;
        cmdDelta[i]        = 0.0f;
        cmdDegPrev[i]      = pasosAGrados(i, refPasos[i]);
        velCmd[i]          = 0.0f;
        picoVel[i]         = 0.0f;
        ganancia[i]        = 0.0f;
        quietoDesde_ms[i]  = 0;
        cmdCompensado[i]   = 0.0f;
        faltaMax[i]        = 0.0f;
        fugaAcumulada[i]   = 0.0f;

        // El chequeo en reposo se reevalua desde cero en cada homing: la
        // referencia es nueva, asi que el piso de ruido de la pose de home
        // hay que volver a medirlo.
        enFaltaRep[i]       = false;
        faltaRepDesde_ms[i] = 0;
        picoRep[i]          = 0.0f;
        avisoDeriva[i]      = false;
        desvioRep[i]        = 0.0f;
        desvioRepListo[i]   = false;
        firmaRep[i]         = 0.0f;
        firmaLista[i]       = false;
        reposoDesde_ms[i]   = 0;

        // El salto entre paradas tambien arranca de cero: la referencia es
        // nueva, asi que la firma de antes del homing no es comparable con
        // las de despues (justamente porque el homing acaba de corregir lo
        // que se hubiera perdido).
        firmaPrevia[i]         = 0.0f;
        hayFirmaPrevia[i]      = false;
        recorridoDesdeFirma[i] = 0.0f;

        // atrasoSeg NO se reinicia: es una propiedad de la cadena de
        // medicion (sensor + filtro + muestreo), no de esta corrida, y
        // conviene que siga promediando entre homings.

        // La zona ciega del ADC se vuelve a evaluar en cada homing: puede
        // haber quedado marcada por un recorrido del ciclo anterior.
        rawMin[i] = 0xFFFF;
        rawMax[i] = 0;
        ciego[i]  = false;
    }

    ultimoUs = micros();

    est = ARMADO;
}

// ------------------------------------------------------------------
void CollisionGuard::desarmar()
{
    est = DESARMADO;

    for (uint8_t i = 0; i < NUM_EJES; i++)
    {
        enFalta[i]       = false;
        faltaDesde_ms[i] = 0;
        faltaMax[i]      = 0.0f;
        errorDeg[i]      = 0.0f;
    }
}

// ------------------------------------------------------------------
void CollisionGuard::silenciar(uint32_t ms)
{
    silencioHasta_ms = millis() + ms;

    for (uint8_t i = 0; i < NUM_EJES; i++)
    {
        // La sospecha que hubiera en curso no se arrastra: el silencio
        // existe porque lo que pase durante el no es informacion valida.
        enFalta[i]       = false;
        faltaDesde_ms[i] = 0;
        faltaMax[i]      = 0.0f;
    }
}

// ------------------------------------------------------------------
bool CollisionGuard::actualizar(long pasos1, long pasos2, long pasos3)
{
    if (est == DESARMADO)
    {
        return false;
    }

    const long pasos[NUM_EJES] = {pasos1, pasos2, pasos3};

    // --- Ventana de promediado de la referencia (robot quieto) ---
    if (est == PROMEDIANDO)
    {
        for (uint8_t i = 0; i < NUM_EJES; i++)
        {
            refPasos[i] = pasos[i]; // los motores estan frenados: no cambia

            if (encoders.estaInicializado(i))
            {
                sumaEnc[i] += leerEncoder(i);
                muestras[i]++;
            }
        }
        return false;
    }

    // --- Supervision ---
    const uint32_t ahora = millis();

    // Paso de tiempo real de esta vuelta, para derivar la velocidad
    // comandada. Se usa micros() porque el loop corre en pocos ms y
    // millis() no tiene resolucion suficiente.
    const uint32_t ahoraUs = micros();
    float dt = (float)(uint32_t)(ahoraUs - ultimoUs) / 1000000.0f;
    ultimoUs = ahoraUs;
    if (dt <= 0.0f || dt > 0.25f)
    {
        dt = 0.001f; // primera vuelta, o el loop estuvo colgado: no derivar
    }

    // Silencio por conmutacion de la bomba: se sigue midiendo todo (la
    // traza y los diagnosticos no se interrumpen), pero no se declara nada.
    const bool enSilencio = (silencioHasta_ms != 0) &&
                            ((int32_t)(ahora - silencioHasta_ms) < 0);

    // El corrimiento que mete la bomba NO se compensa aca: lo cancela en
    // cada vuelta el rechazo de modo comun de abajo. Hubo una version que
    // ademas lo volcaba por Serial ("bomba: salto del error e1=..."), y se
    // saco: salia dos veces por pieza -- varias lineas por segundo con el
    // robot produciendo -- y no es algo que se lea en vivo. Lo que hace
    // falta para diagnosticar esto sigue estando en [T] (error contra
    // umbral, 10 Hz) y en [H] (picos y ganancia por eje).

    // --- Rechazo de modo comun ---
    // Cuando se hunde el riel de los encoders (la bomba), las TRES lecturas
    // se corren juntas y para el mismo lado: es un error que no corresponde
    // a movimiento de ningun eje. La mediana de los tres errores captura
    // justo eso y se descuenta antes de comparar contra el umbral.
    //
    // Un eje trabado no se cancela: si uno se va a -25 y los otros dos estan
    // en -1 y +2, la mediana es -1 y ese eje sigue mostrando -24.
    // (Se usa el error de la vuelta anterior, que a estos dt es lo mismo.)
    float comun;
    {
        const float a = errorDeg[0], b = errorDeg[1], c = errorDeg[2];
        comun = (a > b) ? ((b > c) ? b : ((a > c) ? c : a))
                        : ((a > c) ? a : ((b > c) ? c : b));
    }

    bool colision = false;

    for (uint8_t i = 0; i < NUM_EJES; i++)
    {
        if (!refValida[i])
        {
            continue; // eje sin encoder utilizable
        }

        const float cmdDegAhora = pasosAGrados(i, pasos[i]);

        // Los recorridos se calculan SIEMPRE, aunque despues el eje no se
        // supervise: son los numeros que muestra la traza, y sin ellos no
        // se podria diagnosticar justamente el caso en que el guard decide
        // que no puede mirar.
        cmdDelta[i] = cmdDegAhora - pasosAGrados(i, refPasos[i]);
        encDelta[i] = leerEncoder(i) - refEnc[i];

        // --- Velocidad comandada y su pico reciente ---
        // El margen del umbral se calcula sobre ESTO y no sobre la velocidad
        // medida por el encoder: si el brazo se traba, la velocidad real cae
        // a cero pero la comandada sigue alta, que es justo el caso que hay
        // que seguir vigilando.
        const float velInstantanea = (cmdDegAhora - cmdDegPrev[i]) / dt;

        // Odometro de camino comandado desde la ultima firma de reposo. Es
        // lo que hace comparable el salto entre dos paradas: la deriva de
        // medicion crece con el recorrido, un escalon de pasos perdidos no.
        recorridoDesdeFirma[i] += fabsf(cmdDegAhora - cmdDegPrev[i]);

        cmdDegPrev[i] = cmdDegAhora;

        velCmd[i] += 0.25f * (velInstantanea - velCmd[i]); // saca el escalon de la cuantizacion
        const float velAbs = fabsf(velCmd[i]);

        // Pico que decae: sube al instante con la velocidad y despues baja
        // despacio. Asi el margen sigue en pie durante todo el frenado y un
        // rato mas, que es cuando el encoder se esta poniendo al dia.
        const float caida = dt / (GuardConfig::DECAIMIENTO_MARGEN_MS / 1000.0f);
        picoVel[i] -= picoVel[i] * (caida < 1.0f ? caida : 1.0f);
        if (velAbs > picoVel[i])
        {
            picoVel[i] = velAbs;
        }

        // --- Zona ciega del ADC ---
        // Fuera de los limites utiles la lectura esta clavada: el eje sigue
        // girando y el encoder marca siempre lo mismo. Es indistinguible de
        // un brazo trabado, asi que hay que descartarlo ANTES de comparar.
        const uint16_t raw = encoders.leerRaw(i);
        if (raw != 0xFFFF)
        {
            if (raw < rawMin[i]) rawMin[i] = raw;
            if (raw > rawMax[i]) rawMax[i] = raw;

            if (!ciego[i] &&
                (raw <= GuardConfig::RAW_MIN_FIABLE || raw >= GuardConfig::RAW_MAX_FIABLE))
            {
                // Pegajoso hasta el proximo homing: mientras estuvo ahi
                // adentro, el angulo continuo perdio los grados que recorrio,
                // asi que la referencia quedo corrida aunque vuelva a salir.
                ciego[i] = true;
            }
        }

        // Motivos para dejar de mirar un eje, todos con la misma respuesta:
        // no se puede supervisar, pero tampoco se frena el robot por no
        // poder ver (seria peor el remedio). Se avisa una sola vez y el eje
        // vuelve a supervisarse en el proximo homing.
        //
        //   - el encoder esta caido: el filtro de plausibilidad viene
        //     rechazando lecturas seguidas
        //   - el canal tuvo que reengancharse a la fuerza: la referencia
        //     diferencial quedo corrida en una cantidad desconocida
        //   - el recorrido se metio en la zona ciega del ADC
        if (!encoders.esValido(i) || encoders.huboResincronizacion(i) || ciego[i])
        {
            if (!caido[i])
            {
                caido[i] = true;

                if (ultimoAvisoCaido_ms[i] == 0 ||
                    (uint32_t)(ahora - ultimoAvisoCaido_ms[i]) > REPETIR_AVISO_SENSOR_MS)
                {
                    avisoCaido[i]          = true;
                    ultimoAvisoCaido_ms[i] = ahora;
                }
            }
            errorDeg[i]      = 0.0f;
            enFalta[i]       = false;
            faltaDesde_ms[i] = 0;
            continue;
        }

        caido[i] = false;

        // --- Compensacion del atraso (si esta configurada) ---
        // Se le hace a la posicion comandada el mismo pasabajos de primer
        // orden que sufre la medicion, asi los dos llegan igual de tarde y
        // la resta queda limpia. Con retardoMs = 0 esto es exactamente el
        // valor comandado, sin tocar nada.
        float cmdParaComparar = cmdDelta[i];

        if (retardoMs > 0)
        {
            const float tau  = retardoMs / 1000.0f;
            const float alfa = dt / (tau + dt); // discretizacion del pasabajos
            cmdCompensado[i] += alfa * (cmdDelta[i] - cmdCompensado[i]);
            cmdParaComparar = cmdCompensado[i];
        }
        else
        {
            cmdCompensado[i] = cmdDelta[i];
        }

        const float error = encDelta[i] - cmdParaComparar;

        errorDeg[i] = error;

        // --- Atraso estimado: error / velocidad, en segundos ---
        // Solo tiene sentido con el eje andando: a baja velocidad el
        // cociente lo domina el ruido. Se promedia lento a proposito, es un
        // numero para leer despues de varios ciclos.
        if (velAbs > VEL_MIN_PARA_ATRASO_DEG_S)
        {
            const float atrasoInst = -error / velCmd[i];

            if (atrasoInst > -0.5f && atrasoInst < 0.5f)
            {
                atrasoSeg[i] += 0.02f * (atrasoInst - atrasoSeg[i]);
            }
        }

        // La deteccion mira el error SIN el corrimiento comun; errorDeg[]
        // queda crudo para los diagnosticos y la traza.
        const float magnitud = fabsf(error - comun);
        if (magnitud > pico[i])     pico[i]     = magnitud;
        if (magnitud > picoHist[i]) picoHist[i] = magnitud;

        // --- Eje quieto y ya asentado ---
        if (velAbs < VEL_QUIETO_DEG_S)
        {
            if (quietoDesde_ms[i] == 0)
            {
                quietoDesde_ms[i] = ahora;
            }
        }
        else
        {
            quietoDesde_ms[i] = 0;
        }

        const bool asentado = (quietoDesde_ms[i] != 0) &&
                              ((uint32_t)(ahora - quietoDesde_ms[i]) > ASENTAMIENTO_MS);

        // --- Ganancia: dice si el encoder ve TODO el recorrido (~1,00) ---
        if (asentado && fabsf(cmdDelta[i]) > RECORRIDO_MIN_GANANCIA_DEG)
        {
            // Se le devuelve lo que la fuga ya se llevo: si no, la ganancia
            // tenderia a 1,00 sola y taparia justamente lo que tiene que
            // delatar.
            ganancia[i] = (encDelta[i] + fugaAcumulada[i]) / cmdDelta[i];
        }

        // --- Umbral con margen por velocidad ---
        // La medicion del encoder llega atrasada un tiempo fijo, asi que el
        // error que eso provoca es proporcional a la velocidad. El margen se
        // expresa como ese tiempo y se desarma solo cuando el brazo frena.
        const float umbralEf = umbralDeg +
                               (margenVelocidadMs / 1000.0f) * picoVel[i];

        // --- Chequeo en reposo: quieto en home, el umbral es MUCHO menor ---
        //
        // Aca no hay atraso de medicion ni dinamica que tolerar: el brazo
        // esta frenado en la misma pose de siempre. Cualquier diferencia
        // sostenida es real (pasos perdidos, alguien movio el brazo, una
        // polea floja), y es justo lo que el umbral grande deja pasar.
        //
        // La confirmacion es larga a proposito: el robot esta parado, no hay
        // ninguna urgencia, y 1,5 s de error sostenido no lo produce el
        // ruido del encoder.
        bool sospechaReposo = false;

        if (enHome && asentado && !enSilencio)
        {
            // Se compara contra la calibracion DEL HOMING, no contra la
            // referencia de ahora: hay que devolverle lo que la fuga ya se
            // llevo. Sin esto el chequeo no sirve para nada en el caso mas
            // importante -- la descalibracion que entra de a poco, que es
            // justamente como se pierden pasos: la fuga la va absorbiendo a
            // medida que aparece y el error nunca llega a cruzar el umbral.
            //
            // (Medido en el banco: con un desvio real de 7 grados metido de
            // a poco, la fuga absorbio 5,9 y el error se quedo en 1,1.)
            const float crudo = error - comun + fugaAcumulada[i];

            // Y se promedia, porque el encoder tira picos sueltos de hasta
            // 5 grados con el brazo perfectamente quieto (ver la tabla en
            // CollisionGuard.h). Promediado un segundo ese ruido se cae a
            // 0,8 grados y un corrimiento real pasa entero.
            if (!desvioRepListo[i])
            {
                desvioRep[i]       = crudo;
                desvioRepListo[i]  = true;
                reposoDesde_ms[i]  = ahora;
            }
            else
            {
                const float alfa = dt / (GuardConfig::TAU_REPOSO_S + dt);
                desvioRep[i] += alfa * (crudo - desvioRep[i]);
            }

            // La firma se anota en CADA parada, cuando el promedio ya se
            // asento: es el error sistematico acumulado hasta este momento
            // (ganancia del encoder ciclo a ciclo), que no hay que detectar
            // sino descontar. A partir de ahi solo se mira lo que cambie
            // mientras el robot sigue quieto.
            if (!firmaLista[i] &&
                (uint32_t)(ahora - reposoDesde_ms[i]) >= GuardConfig::FIRMA_REPOSO_MS)
            {
                // ANTES de anotarla: cuanto cambio respecto de la parada
                // anterior. Anotarla sin mirar esto es exactamente lo que
                // hacia que un escalon de pasos perdidos quedara descontado
                // para siempre (ver SALTO_PARADA_DEG en CollisionGuard.h).
                const float salto = hayFirmaPrevia[i]
                                        ? fabsf(desvioRep[i] - firmaPrevia[i])
                                        : 0.0f;

                // La deriva legitima es proporcional al camino recorrido; un
                // escalon no. De ahi la parte proporcional de la tolerancia.
                const float tolerancia =
                    saltoParadaDeg + saltoParadaFrac * recorridoDesdeFirma[i];

                if (hayFirmaPrevia[i] && salto > tolerancia)
                {
                    Serial.print("[GUARD] eje ");
                    Serial.print(i + 1);
                    Serial.print(": la calibracion salto ");
                    Serial.print(salto, 1);
                    Serial.print(" grados entre dos paradas (tolerancia ");
                    Serial.print(tolerancia, 1);
                    Serial.print(" para ");
                    Serial.print(recorridoDesdeFirma[i], 0);
                    Serial.println(" grados de recorrido).");

                    if (observar)
                    {
                        Serial.println("        (observando) no se frena.");
                    }
                    else
                    {
                        motivoFallo   = MOTIVO_DESCALIBRACION;
                        ejeFallo      = i;
                        errorFallo    = desvioRep[i] - firmaPrevia[i];
                        cmdDeltaFallo = cmdDelta[i];
                        encDeltaFallo = encDelta[i] + fugaAcumulada[i];

                        desarmar();
                        return true;
                    }
                }

                firmaRep[i]   = desvioRep[i];
                firmaLista[i] = true;

                firmaPrevia[i]         = desvioRep[i];
                hayFirmaPrevia[i]      = true;
                recorridoDesdeFirma[i] = 0.0f;

                // Aca se imprimia la firma de cada parada. Se saco: se toma
                // una firma nueva por eje CADA VEZ que el brazo se asienta
                // en home, o sea tres lineas por pieza. El salto entre dos
                // firmas sigue avisando cuando se pasa de la tolerancia
                // (arriba), que es el unico caso que hay que mirar.
            }

            // Sin firma no hay contra que comparar: no se evalua nada.
            const float desvioReposo =
                firmaLista[i] ? fabsf(desvioRep[i] - firmaRep[i]) : 0.0f;

            if (desvioReposo > picoRep[i])
            {
                picoRep[i] = desvioReposo; // para elegir el umbral con datos
            }

            if (desvioReposo > umbralReposoDeg)
            {
                sospechaReposo = true;

                if (!enFaltaRep[i])
                {
                    enFaltaRep[i]       = true;
                    faltaRepDesde_ms[i] = ahora;
                }
                else if ((uint32_t)(ahora - faltaRepDesde_ms[i]) >= confirmacionReposoMs)
                {
                    if (observar)
                    {
                        if (ultimoAvisoObs_ms == 0 ||
                            (uint32_t)(ahora - ultimoAvisoObs_ms) > REPETIR_AVISO_OBS_MS)
                        {
                            ultimoAvisoObs_ms = ahora;
                            Serial.print("[GUARD] (observando) descalibracion en home: eje ");
                            Serial.print(i + 1);
                            Serial.print(" err=");
                            Serial.print(error, 1);
                            Serial.print(" umbral_reposo=");
                            Serial.println(umbralReposoDeg, 1);
                        }
                        enFaltaRep[i] = false;
                    }
                    else
                    {
                        motivoFallo   = MOTIVO_DESCALIBRACION;
                        ejeFallo      = i;

                        // Se informa el desvio TOTAL contra el homing (con
                        // lo que la fuga absorbio ya devuelto), que es el
                        // numero que significa algo: el error crudo puede
                        // ser mucho menor y confundir a quien lea el log.
                        errorFallo    = error - comun + fugaAcumulada[i];
                        cmdDeltaFallo = cmdDelta[i];
                        encDeltaFallo = encDelta[i] + fugaAcumulada[i];

                        desarmar();
                        return true;
                    }
                }
            }
            else
            {
                enFaltaRep[i] = false;
            }
        }
        else
        {
            // Fuera de home (o todavia asentandose): la sospecha se descarta,
            // y tanto el promedio como la firma se rehacen en la proxima
            // parada. Lo segundo es lo que hace al chequeo inmune a la
            // deriva de medicion que se acumula ciclo a ciclo (ver
            // CollisionGuard.h): cada parada empieza de cero.
            enFaltaRep[i]     = false;
            desvioRepListo[i] = false;
            firmaLista[i]     = false;
        }

        // --- Aviso por deriva absorbida (red lenta, no frena) ---
        if (!avisoDeriva[i] && fabsf(fugaAcumulada[i]) > GuardConfig::DERIVA_AVISO_DEG)
        {
            avisoDeriva[i] = true;

            Serial.print("[GUARD] eje ");
            Serial.print(i + 1);
            Serial.print(": la fuga lleva absorbidos ");
            Serial.print(fugaAcumulada[i], 1);
            Serial.println(" grados desde el homing. Si sigue creciendo para el");
            Serial.println("        mismo lado, se estan perdiendo pasos de verdad.");
        }

        // --- Fuga de la referencia con el eje en reposo ---
        // Borra despacio lo que haya quedado de cada tramo, para que no se
        // vaya acumulando ciclo a ciclo hasta llegar al umbral solo.
        //
        // Se suspende mientras hay una sospecha de reposo en curso: si no,
        // se comeria en 1,5 s justo el error que se esta tratando de
        // confirmar, y el chequeo de arriba no dispararia nunca.
        if (GuardConfig::FUGA_REPOSO_SEG > 0.0f && asentado && !sospechaReposo)
        {
            const float fraccion = dt / GuardConfig::FUGA_REPOSO_SEG;
            const float corrimiento = error * (fraccion < 1.0f ? fraccion : 1.0f);

            refEnc[i] += corrimiento;

            // Se lleva la cuenta de todo lo que se fugo: es el numero que
            // dice si de verdad se estan perdiendo pasos. Si crece siempre
            // para el mismo lado y sin parar, no es ruido ni atraso.
            fugaAcumulada[i] += corrimiento;
        }

        if (enSilencio || magnitud <= umbralEf)
        {
            // Dentro del margen (o en silencio): se cancela la sospecha.
            enFalta[i]       = false;
            faltaDesde_ms[i] = 0;
            faltaMax[i]      = 0.0f;
            continue;
        }

        // Pasado del umbral: arranca (o sigue) la confirmacion temporal.
        if (!enFalta[i])
        {
            enFalta[i]       = true;
            faltaDesde_ms[i] = ahora;
            faltaMax[i]      = magnitud;
            continue;
        }

        if (magnitud > faltaMax[i])
        {
            faltaMax[i] = magnitud;
        }

        if ((uint32_t)(ahora - faltaDesde_ms[i]) < confirmacionMs)
        {
            continue; // todavia no se sostuvo lo suficiente
        }

        // --- El error esta bajando: no es una colision ---
        // Es el encoder poniendose al dia despues de frenar. Un brazo
        // trabado sostiene el error: ni el brazo ni los pasos se mueven.
        // Se reinicia la sospecha y se vuelve a medir desde el valor actual;
        // mientras siga cayendo, nunca va a confirmar.
        if (magnitud < GuardConfig::CANCELA_SI_BAJA_A * faltaMax[i])
        {
            faltaDesde_ms[i] = ahora;
            faltaMax[i]      = magnitud;
            continue;
        }

        // --- Se sostuvo: antes de declarar colision, descartar que sea un
        //     encoder montado al reves (error de configuracion, no choque) ---
        if (fabsf(cmdDelta[i]) > RECORRIDO_MIN_PARA_JUZGAR_DEG &&
            encDelta[i] * cmdDelta[i] < 0.0f)
        {
            const float ratio = fabsf(encDelta[i]) / fabsf(cmdDelta[i]);

            if (ratio > RATIO_INVERTIDO_MIN && ratio < RATIO_INVERTIDO_MAX)
            {
                reportarSignoInvertido(i, cmdDelta[i], encDelta[i]);
                return false;
            }
        }

        // --- Modo observador: se avisa lo que habria pasado, no se frena ---
        if (observar)
        {
            if (ultimoAvisoObs_ms == 0 ||
                (uint32_t)(ahora - ultimoAvisoObs_ms) > REPETIR_AVISO_OBS_MS)
            {
                ultimoAvisoObs_ms = ahora;

                Serial.print("[GUARD] (observando) habria frenado: eje ");
                Serial.print(i + 1);
                Serial.print(" err=");
                Serial.print(error, 1);
                Serial.print(" umbral_ef=");
                Serial.print(umbralEf, 1);
                Serial.print(" dcmd=");
                Serial.print(cmdDelta[i], 1);
                Serial.print(" denc=");
                Serial.print(encDelta[i], 1);
                Serial.print(" vel=");
                Serial.println(velCmd[i], 0);
            }

            // Se rearma la confirmacion para que el proximo aviso sea de un
            // episodio nuevo y no del mismo sostenido.
            enFalta[i]       = false;
            faltaDesde_ms[i] = 0;
            continue;
        }

        colision      = true;
        motivoFallo   = MOTIVO_COLISION;
        ejeFallo      = i;
        errorFallo    = error;
        cmdDeltaFallo = cmdDelta[i];
        encDeltaFallo = encDelta[i];
        break;
    }

    if (colision)
    {
        desarmar(); // Robot vuelve a armarlo cuando termine de recalibrarse
        return true;
    }

    return false;
}

// ------------------------------------------------------------------
void CollisionGuard::reportarSignoInvertido(uint8_t eje, float cmdDelta, float encDelta)
{
    inhibidoPorConfig = true;
    desarmar();

    Serial.println();
    Serial.println("=========================================================");
    Serial.print("[GUARD] El encoder del eje ");
    Serial.print(eje + 1);
    Serial.println(" cuenta AL REVES que el motor.");
    Serial.print("        pasos: ");
    Serial.print(cmdDelta, 1);
    Serial.print(" grados   encoder: ");
    Serial.print(encDelta, 1);
    Serial.println(" grados");
    Serial.print("        Corregir ENCODER_SIGN[");
    Serial.print(eje);
    Serial.println("] en CollisionGuard.h y volver a compilar.");
    Serial.println("        Mientras tanto la supervision queda APAGADA: no");
    Serial.println("        se frena el robot por un error de configuracion.");
    Serial.println("=========================================================");
    Serial.println();
}

// ------------------------------------------------------------------
float CollisionGuard::errorActual(uint8_t eje) const
{
    if (eje >= NUM_EJES) return 0.0f;
    return errorDeg[eje];
}

float CollisionGuard::picoDesdeHoming(uint8_t eje) const
{
    if (eje >= NUM_EJES) return 0.0f;
    return pico[eje];
}

float CollisionGuard::picoHistorico(uint8_t eje) const
{
    if (eje >= NUM_EJES) return 0.0f;
    return picoHist[eje];
}

float CollisionGuard::picoEnReposo(uint8_t eje) const
{
    if (eje >= NUM_EJES) return 0.0f;
    return picoRep[eje];
}

float CollisionGuard::encDeltaActual(uint8_t eje) const
{
    if (eje >= NUM_EJES) return 0.0f;
    return encDelta[eje];
}

float CollisionGuard::cmdDeltaActual(uint8_t eje) const
{
    if (eje >= NUM_EJES) return 0.0f;
    return cmdDelta[eje];
}

float CollisionGuard::velocidadCmd(uint8_t eje) const
{
    if (eje >= NUM_EJES) return 0.0f;
    return velCmd[eje];
}

float CollisionGuard::umbralEfectivo(uint8_t eje) const
{
    if (eje >= NUM_EJES) return umbralDeg;
    return umbralDeg + (margenVelocidadMs / 1000.0f) * picoVel[eje];
}

float CollisionGuard::gananciaMedida(uint8_t eje) const
{
    if (eje >= NUM_EJES) return 0.0f;
    return ganancia[eje];
}

float CollisionGuard::atrasoMedido_ms(uint8_t eje) const
{
    if (eje >= NUM_EJES) return 0.0f;
    return atrasoSeg[eje] * 1000.0f;
}

float CollisionGuard::derivaAbsorbida(uint8_t eje) const
{
    if (eje >= NUM_EJES) return 0.0f;
    return fugaAcumulada[eje];
}

uint16_t CollisionGuard::rawMinimo(uint8_t eje) const
{
    if (eje >= NUM_EJES) return 0;
    return rawMin[eje];
}

uint16_t CollisionGuard::rawMaximo(uint8_t eje) const
{
    if (eje >= NUM_EJES) return 0;
    return rawMax[eje];
}

bool CollisionGuard::fueraDeRango(uint8_t eje) const
{
    if (eje >= NUM_EJES) return false;
    return ciego[eje];
}

bool CollisionGuard::sensorCaido(uint8_t eje) const
{
    if (eje >= NUM_EJES) return false;
    return caido[eje];
}

bool CollisionGuard::consumirAvisoSensor(uint8_t &eje)
{
    for (uint8_t i = 0; i < NUM_EJES; i++)
    {
        if (avisoCaido[i])
        {
            avisoCaido[i] = false;
            eje = i;
            return true;
        }
    }
    return false;
}

// ------------------------------------------------------------------
void CollisionGuard::setUmbral(float grados)
{
    if (grados < 1.0f)  grados = 1.0f;   // por debajo de esto es todo ruido
    if (grados > 90.0f) grados = 90.0f;
    umbralDeg = grados;
}

void CollisionGuard::setUmbralReposo(float grados)
{
    // Por debajo de 2 grados se entra en el ruido propio del encoder; por
    // encima de 20 ya no distingue nada que el umbral normal no viera.
    if (grados < 2.0f)  grados = 2.0f;
    if (grados > 20.0f) grados = 20.0f;
    umbralReposoDeg = grados;
}

void CollisionGuard::setSaltoParada(float grados, float fraccion)
{
    // La parte fija no puede bajar del ruido del promedio de reposo (~0,8
    // grados) o el chequeo dispararia solo. Arriba de 20 ya no detecta nada
    // que valga la pena avisar.
    if (grados < 1.0f)  grados = 1.0f;
    if (grados > 20.0f) grados = 20.0f;

    // La fraccion en 0 desactiva la parte proporcional: queda un umbral
    // fijo, que sirve para probar pero dispara en falso despues de muchos
    // ciclos sin parada.
    if (fraccion < 0.0f)  fraccion = 0.0f;
    if (fraccion > 0.20f) fraccion = 0.20f;

    saltoParadaDeg  = grados;
    saltoParadaFrac = fraccion;
}

void CollisionGuard::setConfirmacion(uint32_t ms)
{
    if (ms > 2000) ms = 2000; // mas que esto ya no protege nada
    confirmacionMs = ms;
}

void CollisionGuard::setObservar(bool observarSinFrenar)
{
    observar          = observarSinFrenar;
    ultimoAvisoObs_ms = 0;
}

void CollisionGuard::setMargenVelocidad(uint32_t ms)
{
    if (ms > 1000) ms = 1000;
    margenVelocidadMs = ms;
}

void CollisionGuard::setRetardo(uint32_t ms)
{
    if (ms > 500) ms = 500;
    retardoMs = ms;

    // Se resiembra el filtro en el valor actual: si no, arrastraria el
    // historial calculado con la constante vieja.
    for (uint8_t i = 0; i < NUM_EJES; i++)
    {
        cmdCompensado[i] = cmdDelta[i];
    }
}

const char *CollisionGuard::nombreEstado() const
{
    switch (est)
    {
        case DESARMADO:   return inhibidoPorConfig ? "INHIBIDO" : "DESARMADO";
        case PROMEDIANDO: return "PROMEDIANDO";
        case ARMADO:      return "ARMADO";
        default:          return "?";
    }
}
