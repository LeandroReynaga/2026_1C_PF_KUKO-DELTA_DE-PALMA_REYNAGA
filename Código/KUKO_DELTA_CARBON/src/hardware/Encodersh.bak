#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>
#include <Wire.h>

#define NUM_ENCODERS 3

// ------------------------------------------------------------------
// Estados de la máquina no bloqueante que gestiona cada ciclo de lectura
// (seleccionar canal del mux -> esperar asentamiento -> pedir ángulo -> leer)
// ------------------------------------------------------------------
enum class EstadoLectura : uint8_t
{
    SELECCIONAR_CANAL,
    ESPERAR_ASENTAMIENTO,
    SOLICITAR_ANGULO,
    ESPERAR_Y_LEER
};

// Estado y datos de cada encoder individual
struct CanalEncoder
{
    uint16_t rawActual          = 0xFFFF; // última lectura cruda (12 bits)
    float    anguloActual       = 0.0f;   // grados 0-360 sin filtrar
    float    anguloFiltrado     = 0.0f;   // grados filtrados (para uso general)
    float    anguloContinuo     = 0.0f;   // grados acumulados sin salto en 0/360 (para lazo cerrado)
    int32_t  vueltas            = 0;      // vueltas completas acumuladas
    bool     inicializado       = false;  // true luego de la primera lectura válida
    bool     valido             = false;  // false si la última lectura fue rechazada/falló
    uint8_t  erroresSeguidos    = 0;      // errores consecutivos (I2C o salto implausible)
    uint32_t ultimaLecturaOk_ms = 0;      // timestamp de la última lectura aceptada
    float    velocidadDegSeg    = 0.0f;   // derivada del ángulo continuo, útil para diagnósticos
};

class Encoders
{
private:
    static const uint8_t TCA_ADDR       = 0x70;
    static const uint8_t AS5600_ADDR    = 0x36;
    static const uint8_t AS5600_ANGLE_H = 0x0E;

    // Mapeo fijo: índice lógico de motor -> canal físico del TCA9548A.
    // Cableado real del robot (no se espera que cambie):
    //   motor 0 (motor1) -> canal físico 4
    //   motor 1 (motor2) -> canal físico 3
    //   motor 2 (motor3) -> canal físico 6
    // Al no barrer los 8 canales del mux, solo se genera tráfico I2C
    // para los 3 canales realmente conectados.
    static const uint8_t canalesFisicos[NUM_ENCODERS];

    // --- Parámetros de robustez (ajustables) ---
    static const uint8_t  MAX_ERRORES_CONSECUTIVOS = 8;     // luego de esto el canal se marca "no válido" (pero conserva el último valor)
    static const uint16_t TIEMPO_ASENTAMIENTO_US   = 300;   // tiempo que tarda el mux en conmutar
    static const uint16_t TIMEOUT_LECTURA_US       = 2500;  // tiempo máximo esperando respuesta del AS5600
    static constexpr float ALFA_FILTRO             = 0.35f; // suavizado exponencial (0=sin filtrar, 1=muy suave)
    static constexpr float SALTO_MAX_DEG_POR_CICLO = 40.0f; // salto máximo plausible entre lecturas consecutivas -> rechaza ruido

    CanalEncoder canales[NUM_ENCODERS];

    // Máquina de estados: se avanza un paso por cada llamada a update(), nunca bloquea
    EstadoLectura estado           = EstadoLectura::SELECCIONAR_CANAL;
    uint8_t  motorActivo           = 0;
    uint8_t  canalFisicoEnMux      = 0xFF; // último canal realmente seleccionado en el TCA9548A (evita reselección innecesaria)
    uint32_t marcaTiempo_us        = 0;
    bool     solicitudEnCurso      = false;

    bool     seleccionarCanalMux(uint8_t indiceMotor);
    bool     iniciarSolicitudAngulo();
    bool     leerRespuestaAngulo(uint16_t &rawOut);
    void     procesarLecturaValida(uint8_t indiceMotor, uint16_t raw);
    void     registrarError(uint8_t indiceMotor);
    void     avanzarAlSiguienteMotor();

public:
    void begin();

    // Debe llamarse continuamente dentro de robot.update(). Nunca usa delay().
    // Internamente recorre los 3 motores en round-robin, un paso de la máquina
    // de estados por llamada, por lo que varias llamadas son necesarias para
    // completar la lectura de un motor.
    void update();

    // "motor" es el índice lógico: 0 = motor1, 1 = motor2, 2 = motor3
    // (mapeado internamente a los canales físicos 4, 3 y 6 del mux).
    float    leerGrados(uint8_t motor) const;         // último ángulo filtrado válido (0-360)
    float    leerGradosContinuo(uint8_t motor) const; // ángulo acumulado sin discontinuidad (para lazo cerrado / detección de pasos perdidos)
    uint16_t leerRaw(uint8_t motor) const;
    bool     esValido(uint8_t motor) const;            // false si el motor superó el máximo de errores consecutivos
    bool     estaInicializado(uint8_t motor) const;
    uint8_t  erroresConsecutivos(uint8_t motor) const;
    uint32_t tiempoDesdeUltimaLecturaOk_ms(uint8_t motor) const;
    float    leerVelocidad(uint8_t motor) const;

    void resetearCanal(uint8_t motor); // fuerza a olvidar el histórico (por ejemplo tras un homing)
};

#endif