#include "Encoders.h"

#define SDA_PIN 21
#define SCL_PIN 22

constexpr float Encoders::ALFA_FILTRO;
constexpr float Encoders::SALTO_MAX_DEG_POR_CICLO;

// Motor lógico -> canal físico del TCA9548A. Cableado fijo del robot:
//   índice 0 (motor1) -> canal 4
//   índice 1 (motor2) -> canal 3
//   índice 2 (motor3) -> canal 6
const uint8_t Encoders::canalesFisicos[NUM_ENCODERS] = {4, 3, 6};

// ------------------------------------------------------------------
void Encoders::begin()
{
    Wire.begin(SDA_PIN, SCL_PIN);

    // 100 kHz en lugar de 300-400 kHz: el modo "fast" es mucho más sensible
    // al ruido inducido por los drivers/fuente de 48V. A 100 kHz los flancos
    // son más largos y el bus tolera mucho mejor el acoplamiento capacitivo.
    Wire.setClock(500);

    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
    {
        canales[i] = CanalEncoder();
    }

    estado = EstadoLectura::SELECCIONAR_CANAL;
    motorActivo = 0;
    canalFisicoEnMux = 0xFF;
}

// ------------------------------------------------------------------
// Máquina de estados no bloqueante. Cada llamada avanza un único paso.
// El asentamiento del mux y el timeout de lectura se resuelven comparando
// micros(), nunca deteniendo la CPU con delay().
// ------------------------------------------------------------------
void Encoders::update()
{
    switch (estado)
    {
        case EstadoLectura::SELECCIONAR_CANAL:
        {
            if (!seleccionarCanalMux(motorActivo))
            {
                registrarError(motorActivo);
                avanzarAlSiguienteMotor();
                return;
            }
            marcaTiempo_us = micros();
            estado = EstadoLectura::ESPERAR_ASENTAMIENTO;
            break;
        }

        case EstadoLectura::ESPERAR_ASENTAMIENTO:
        {
            if ((uint32_t)(micros() - marcaTiempo_us) < TIEMPO_ASENTAMIENTO_US)
            {
                return; // todavía no pasó el tiempo de asentamiento, no bloqueamos
            }
            estado = EstadoLectura::SOLICITAR_ANGULO;
            break;
        }

        case EstadoLectura::SOLICITAR_ANGULO:
        {
            if (!iniciarSolicitudAngulo())
            {
                registrarError(motorActivo);
                avanzarAlSiguienteMotor();
                return;
            }
            marcaTiempo_us = micros();
            estado = EstadoLectura::ESPERAR_Y_LEER;
            break;
        }

        case EstadoLectura::ESPERAR_Y_LEER:
        {
            uint16_t raw;
            bool listo = leerRespuestaAngulo(raw);

            if (!listo)
            {
                if ((uint32_t)(micros() - marcaTiempo_us) > TIMEOUT_LECTURA_US)
                {
                    // el sensor no respondió a tiempo (típico síntoma de ruido)
                    registrarError(motorActivo);
                    avanzarAlSiguienteMotor();
                }
                return; // seguimos esperando, sin bloquear
            }

            if (raw == 0xFFFF)
            {
                registrarError(motorActivo);
            }
            else
            {
                procesarLecturaValida(motorActivo, raw);
            }

            avanzarAlSiguienteMotor();
            break;
        }
    }
}

// ------------------------------------------------------------------
void Encoders::avanzarAlSiguienteMotor()
{
    motorActivo = (motorActivo + 1) % NUM_ENCODERS;
    estado = EstadoLectura::SELECCIONAR_CANAL;
}

// ------------------------------------------------------------------
bool Encoders::seleccionarCanalMux(uint8_t indiceMotor)
{
    if (indiceMotor >= NUM_ENCODERS) return false;

    uint8_t canalFisico = canalesFisicos[indiceMotor];

    // Evitamos reescribir el mux si ya está en el canal correcto: menos
    // tráfico I2C, menos oportunidades de que el ruido corrompa una transacción.
    if (canalFisicoEnMux == canalFisico) return true;

    Wire.beginTransmission(TCA_ADDR);
    Wire.write((uint8_t)(1 << canalFisico));
    if (Wire.endTransmission() != 0)
    {
        canalFisicoEnMux = 0xFF; // estado del mux desconocido, forzar reselección la próxima vez
        return false;
    }

    canalFisicoEnMux = canalFisico;
    return true;
}

// ------------------------------------------------------------------
bool Encoders::iniciarSolicitudAngulo()
{
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_ANGLE_H);

    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }

    if (Wire.requestFrom((int)AS5600_ADDR, 2) != 2)
    {
        return false;
    }

    solicitudEnCurso = true;
    return true;
}

// ------------------------------------------------------------------
bool Encoders::leerRespuestaAngulo(uint16_t &rawOut)
{
    // Wire.requestFrom ya dejó los bytes en el buffer interno; los
    // consumimos apenas estén disponibles, sin sondear con delay.
    if (Wire.available() < 2)
    {
        return false;
    }

    uint8_t highByte = Wire.read();
    uint8_t lowByte  = Wire.read();
    solicitudEnCurso = false;

    rawOut = ((highByte & 0x0F) << 8) | lowByte;
    return true;
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

