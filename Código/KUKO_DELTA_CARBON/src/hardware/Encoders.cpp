#include "Encoders.h"

constexpr float Encoders::ALFA_FILTRO;
constexpr float Encoders::SALTO_MAX_DEG_POR_CICLO;
constexpr float Encoders::VCC_ALIMENTACION_ENCODERS;
constexpr float Encoders::VCC_REFERENCIA_ADC;
constexpr float Encoders::RAW_MAX_EFECTIVO;

const uint8_t Encoders::pinesADC[NUM_ENCODERS] = {35, 34, 39};

Encoders encoders;

// ------------------------------------------------------------------
void Encoders::begin()
{
    analogReadResolution(12); // 0-4095

    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
    {
        analogSetPinAttenuation(pinesADC[i], ADC_11db);
        canales[i] = CanalEncoder();
        offsetHoming[i] = 0.0f;
        homingCalibrado[i] = false;
        sumaAnguloContinuoHoming[i] = 0.0;
        muestrasHoming[i] = 0;
        resincronizado[i] = false;
    }

    acumulandoHoming = false;
}

// ------------------------------------------------------------------
void Encoders::update()
{
    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
    {
        uint16_t raw = leerRawPromediado(i);
        procesarLecturaValida(i, raw);

        // Acumulación para la media móvil de calibración de homing.
        // No importa si esta muestra puntual fue rechazada por el filtro
        // de plausibilidad: en ese caso anguloContinuo conserva el último
        // valor válido, y acumularlo de nuevo no introduce sesgo.
        if (acumulandoHoming && canales[i].inicializado)
        {
            sumaAnguloContinuoHoming[i] += canales[i].anguloContinuo;
            muestrasHoming[i]++;
        }
    }
}

// ------------------------------------------------------------------
uint16_t Encoders::leerRawPromediado(uint8_t indiceMotor)
{
    uint8_t pin = pinesADC[indiceMotor];
    uint32_t suma = 0;

    for (uint8_t i = 0; i < MUESTRAS_OVERSAMPLING; i++)
    {
        suma += analogRead(pin);
    }

    return (uint16_t)(suma / MUESTRAS_OVERSAMPLING);
}

// ------------------------------------------------------------------
void Encoders::procesarLecturaValida(uint8_t indiceMotor, uint16_t raw)
{
    CanalEncoder &c = canales[indiceMotor];

    float anguloNuevo = (raw * 360.0f) / RAW_MAX_EFECTIVO;
    if (anguloNuevo >= 360.0f) anguloNuevo -= 360.0f;

    if (!c.inicializado)
    {
        c.rawActual      = raw;
        c.anguloActual   = anguloNuevo;
        c.anguloFiltrado = anguloNuevo;
        c.anguloContinuo = anguloNuevo;
        c.vueltas        = 0;
        c.inicializado   = true;
        c.valido         = true;
        c.erroresSeguidos = 0;
        c.ultimaLecturaOk_ms = millis();
        return;
    }

    // --- Filtro de plausibilidad: rechaza saltos imposibles causados por ruido ---
    float diferencia = anguloNuevo - c.anguloActual;
    if (diferencia > 180.0f)  diferencia -= 360.0f;
    if (diferencia < -180.0f) diferencia += 360.0f;

    if (fabs(diferencia) > SALTO_MAX_DEG_POR_CICLO)
    {
        registrarError(indiceMotor);

        // El rechazo se compara SIEMPRE contra la última lectura aceptada.
        // Si el eje realmente se fue a más de SALTO_MAX_DEG_POR_CICLO de
        // ahí (pasa si el loop se congela unas decenas de ms con el brazo
        // a máxima velocidad), todas las lecturas siguientes también quedan
        // a esa distancia y el canal se rechaza a sí mismo para siempre.
        // Una vez que ya se dio por no válido, no hay nada que preservar:
        // conviene reengancharlo con la posición actual antes que dejarlo
        // muerto hasta el próximo reinicio.
        if (canales[indiceMotor].erroresSeguidos >= MAX_ERRORES_CONSECUTIVOS)
        {
            resincronizar(indiceMotor, raw, anguloNuevo, diferencia);
        }
        return;
    }

    // --- Desenvolver el ángulo (unwrap) para tener una posición continua ---
    c.anguloContinuo += diferencia;

    // --- Filtro exponencial simple para suavizar ruido residual ---
    c.anguloFiltrado += ALFA_FILTRO * (anguloNuevo - c.anguloFiltrado);

    // --- Velocidad angular (diagnóstico / detección de motor trabado) ---
    uint32_t ahora = millis();
    uint32_t dt_ms = ahora - c.ultimaLecturaOk_ms;
    if (dt_ms > 0 && c.ultimaLecturaOk_ms != 0)
    {
        c.velocidadDegSeg = (diferencia * 1000.0f) / (float)dt_ms;
    }

    c.rawActual          = raw;
    c.anguloActual        = anguloNuevo;
    c.valido              = true;
    c.erroresSeguidos     = 0;
    c.ultimaLecturaOk_ms  = ahora;
}

// ------------------------------------------------------------------
void Encoders::resincronizar(uint8_t indiceMotor, uint16_t raw,
                              float anguloNuevo, float diferencia)
{
    CanalEncoder &c = canales[indiceMotor];

    // El salto se toma como movimiento real que no se alcanzó a seguir, así
    // que se acumula en el ángulo continuo (si no, quedaría corrido para
    // siempre). El filtro exponencial se resiembra en el valor actual en vez
    // de arrastrarse desde uno viejo que ya no significa nada.
    c.anguloContinuo += diferencia;
    c.anguloFiltrado  = anguloNuevo;
    c.anguloActual    = anguloNuevo;
    c.rawActual       = raw;

    c.valido            = true;
    c.erroresSeguidos   = 0;
    c.velocidadDegSeg   = 0.0f;
    c.ultimaLecturaOk_ms = millis();

    // Bandera pegajosa: la lectura vuelve a servir para diagnóstico, pero
    // NO para supervisar colisiones. Puede haber sido movimiento real que se
    // perdió, o una falla del sensor; hasta el próximo homing (que rehace la
    // referencia desde cero) no hay forma de saberlo. La limpia
    // CollisionGuard al rearmarse.
    resincronizado[indiceMotor] = true;

    // Contador aparte de la bandera: la bandera la limpia CollisionGuard en
    // cada homing, y lo que la interfaz necesita mostrar es OTRA cosa --
    // cuantas veces se reengancho el canal desde que arranco el robot. Un
    // canal que se resincroniza una vez tuvo un mal momento; uno que lleva
    // veinte tiene un problema de cableado o de alimentación.
    resincronizaciones[indiceMotor]++;
}

// ------------------------------------------------------------------
bool Encoders::huboResincronizacion(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return false;
    return resincronizado[motor];
}

uint16_t Encoders::cuentaResincronizaciones(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return 0;
    return resincronizaciones[motor];
}

void Encoders::limpiarResincronizacion(uint8_t motor)
{
    if (motor >= NUM_ENCODERS) return;
    resincronizado[motor] = false;
}

// ------------------------------------------------------------------
void Encoders::registrarError(uint8_t indiceMotor)
{
    if (indiceMotor >= NUM_ENCODERS) return;

    CanalEncoder &c = canales[indiceMotor];
    if (c.erroresSeguidos < 255) c.erroresSeguidos++;

    if (c.erroresSeguidos >= MAX_ERRORES_CONSECUTIVOS)
    {
        c.valido = false;
    }
}

// ------------------------------------------------------------------
float Encoders::leerGrados(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return -1.0f;
    return canales[motor].anguloFiltrado + offsetHoming[motor];
}

float Encoders::leerGradosContinuo(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return 0.0f;
    return canales[motor].anguloContinuo + offsetHoming[motor];
}

float Encoders::leerGradosContinuoCrudo(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return 0.0f;
    return canales[motor].anguloContinuo;
}

uint16_t Encoders::leerRaw(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return 0xFFFF;
    return canales[motor].rawActual;
}

bool Encoders::esValido(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return false;
    return canales[motor].valido && canales[motor].inicializado;
}

bool Encoders::estaInicializado(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return false;
    return canales[motor].inicializado;
}

bool Encoders::estaCalibradoHoming(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return false;
    return homingCalibrado[motor];
}

uint8_t Encoders::erroresConsecutivos(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return 255;
    return canales[motor].erroresSeguidos;
}

uint32_t Encoders::tiempoDesdeUltimaLecturaOk_ms(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return UINT32_MAX;
    if (canales[motor].ultimaLecturaOk_ms == 0) return UINT32_MAX;
    return millis() - canales[motor].ultimaLecturaOk_ms;
}

float Encoders::leerVelocidad(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return 0.0f;
    return canales[motor].velocidadDegSeg;
}

// ------------------------------------------------------------------
void Encoders::iniciarAsentamientoHoming()
{
    acumulandoHoming = true;
    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
    {
        sumaAnguloContinuoHoming[i] = 0.0;
        muestrasHoming[i] = 0;
    }
}

// ------------------------------------------------------------------
void Encoders::calibrarHomingMotor(uint8_t motor, float anguloHome)
{
    if (motor >= NUM_ENCODERS) return;
    if (!canales[motor].inicializado) return;

    float referencia;

    if (muestrasHoming[motor] > 0)
    {
        // Media móvil real: promedio de TODAS las muestras acumuladas
        // desde iniciarAsentamientoHoming() -> mucho más robusto ante
        // ruido residual que una lectura puntual.
        referencia = (float)(sumaAnguloContinuoHoming[motor] / (double)muestrasHoming[motor]);
    }
    else
    {
        // Fallback: sin ventana de asentamiento activa, usa la lectura
        // instantánea (comportamiento anterior).
        referencia = canales[motor].anguloContinuo;
    }

    offsetHoming[motor] = anguloHome - referencia;
    homingCalibrado[motor] = true;
}

void Encoders::calibrarHoming(float anguloHomeM1, float anguloHomeM2, float anguloHomeM3)
{
    calibrarHomingMotor(0, anguloHomeM1 - 1.2f);
    calibrarHomingMotor(1, anguloHomeM2 - 0.7f);
    calibrarHomingMotor(2, anguloHomeM3 - 1.1f);

    acumulandoHoming = false;
}

// ------------------------------------------------------------------
void Encoders::resetearCanal(uint8_t motor)
{
    if (motor >= NUM_ENCODERS) return;
    canales[motor] = CanalEncoder();
    offsetHoming[motor] = 0.0f;
    homingCalibrado[motor] = false;
    sumaAnguloContinuoHoming[motor] = 0.0;
    muestrasHoming[motor] = 0;
    resincronizado[motor] = false;
}