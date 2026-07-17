#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

#define NUM_ENCODERS 3

// Estado de cada encoder individual
struct CanalEncoder
{
    uint16_t rawActual          = 0xFFFF; // última lectura cruda (12 bits, 0-4095)
    float    anguloActual       = 0.0f;   // grados 0-360 sin filtrar
    float    anguloFiltrado     = 0.0f;   // grados filtrados (para uso general)
    float    anguloContinuo     = 0.0f;   // grados acumulados sin salto en 0/360 (para lazo cerrado)
    int32_t  vueltas            = 0;      // vueltas completas acumuladas
    bool     inicializado       = false;  // true luego de la primera lectura válida
    bool     valido             = false;  // false si la última lectura fue rechazada/falló
    uint8_t  erroresSeguidos    = 0;      // errores consecutivos (salto implausible)
    uint32_t ultimaLecturaOk_ms = 0;      // timestamp de la última lectura aceptada
    float    velocidadDegSeg    = 0.0f;   // derivada del ángulo continuo, útil para diagnósticos
};

class Encoders
{
private:
    // Motor lógico -> pin ADC físico. Cableado fijo del robot:
    //   índice 0 (motor1) -> GPIO35
    //   índice 1 (motor2) -> GPIO34
    //   índice 2 (motor3) -> GPIO39
    static const uint8_t pinesADC[NUM_ENCODERS];

    // --- Parámetros de robustez (ajustables) ---
    static const uint8_t  MAX_ERRORES_CONSECUTIVOS = 8;     // luego de esto el canal se marca "no válido" (pero conserva el último valor)
    static const uint8_t  MUESTRAS_OVERSAMPLING     = 16;    // promediado por lectura; bajo a propósito para minimizar latencia (el filtro RC en hardware ya limpia gran parte del ruido)
    static constexpr float ALFA_FILTRO             = 0.15f; // suavizado exponencial (más bajo = más filtrado, más retardo)
    static constexpr float SALTO_MAX_DEG_POR_CICLO = 40.0f; // salto máximo plausible entre lecturas consecutivas -> rechaza ruido

    CanalEncoder canales[NUM_ENCODERS];

    uint16_t leerRawPromediado(uint8_t indiceMotor);
    void     procesarLecturaValida(uint8_t indiceMotor, uint16_t raw);
    void     registrarError(uint8_t indiceMotor);

public:
    void begin();

    // Lee y procesa los 3 encoders, uno después del otro, dentro de la
    // misma llamada (mismo ciclo de loop). No hay round-robin: cada
    // invocación deja actualizados los 3 canales.
    void update();

    // "motor" es el índice lógico: 0 = motor1, 1 = motor2, 2 = motor3
    // (mapeado internamente a GPIO35, GPIO34 y GPIO39).
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