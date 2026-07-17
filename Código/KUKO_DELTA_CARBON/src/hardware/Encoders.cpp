#include "Encoders.h"

constexpr float Encoders::ALFA_FILTRO;
constexpr float Encoders::SALTO_MAX_DEG_POR_CICLO;

const uint8_t Encoders::pinesADC[NUM_ENCODERS] = {35, 34, 39};

// ------------------------------------------------------------------
void Encoders::begin()
{
    analogReadResolution(12); // 0-4095

    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
    {
        // ADC_11db habilita el rango completo hasta ~3.3V.
        analogSetPinAttenuation(pinesADC[i], ADC_11db);
        canales[i] = CanalEncoder();
    }
}

// ------------------------------------------------------------------
// Lee los 3 motores en el mismo ciclo, uno inmediatamente después del
// otro (el ADC del ESP32 es único y secuencial a nivel de hardware, así
// que "simultáneo" real no existe, pero el desfasaje entre motores queda
// en el orden de microsegundos en lugar de repartirse en varias vueltas
// de loop como en un esquema round-robin).
// ------------------------------------------------------------------
void Encoders::update()
{
    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
    {
        uint16_t raw = leerRawPromediado(i);
        procesarLecturaValida(i, raw);
    }
}

// ------------------------------------------------------------------
// Oversampling reducido a propósito: prioriza velocidad/latencia baja.
// El filtrado principal de ruido ya lo hace el capacitor en hardware,
// más el filtro exponencial (ALFA_FILTRO) a continuación.
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
    float anguloNuevo = (raw * 360.0f) / 4096.0f;

    if (!c.inicializado)
    {
        // Primera lectura: la aceptamos directamente como referencia
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
    if (diferencia > 180.0f)  diferencia -= 360.0f; // camino corto considerando el wrap 0/360
    if (diferencia < -180.0f) diferencia += 360.0f;

    if (fabs(diferencia) > SALTO_MAX_DEG_POR_CICLO)
    {
        // Salto demasiado grande para un solo ciclo -> probablemente ruido.
        // Se descarta la muestra pero se conserva el último ángulo válido.
        registrarError(indiceMotor);
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
void Encoders::registrarError(uint8_t indiceMotor)
{
    if (indiceMotor >= NUM_ENCODERS) return;

    CanalEncoder &c = canales[indiceMotor];
    if (c.erroresSeguidos < 255) c.erroresSeguidos++;

    if (c.erroresSeguidos >= MAX_ERRORES_CONSECUTIVOS)
    {
        c.valido = false; // el llamador decide qué hacer (ej: detener el eje)
    }
}

// ------------------------------------------------------------------
float Encoders::leerGrados(uint8_t motor) const
{
    if (motor >= NUM_ENCODERS) return -1.0f;
    return canales[motor].anguloFiltrado;
}

float Encoders::leerGradosContinuo(uint8_t motor) const
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

void Encoders::resetearCanal(uint8_t motor)
{
    if (motor >= NUM_ENCODERS) return;
    canales[motor] = CanalEncoder();
}