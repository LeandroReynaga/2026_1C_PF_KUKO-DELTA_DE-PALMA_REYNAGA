#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

#define NUM_ENCODERS 3

// Estado de cada encoder individual
struct CanalEncoder
{
    uint16_t rawActual          = 0xFFFF; // última lectura cruda (0 al raw máximo efectivo)
    float    anguloActual       = 0.0f;   // grados 0-360 sin filtrar (marco del sensor, no del robot)
    float    anguloFiltrado     = 0.0f;   // grados filtrados (marco del sensor, sin offset de home)
    float    anguloContinuo     = 0.0f;   // grados acumulados sin salto en 0/360 (marco del sensor, sin offset de home)
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

    // --- Corrección de escala (ganancia) por VCC real ---
    // El AS5600 satura su OUT proporcional a su VCC real, que medimos en
    // 3.14V (no los 3.3V nominales que asume el ADC del ESP32 a 11dB).
    // Sin esta corrección, raw nunca alcanza 4095 en la vuelta física
    // completa y el cálculo con /4096 queda sistemáticamente "corto"
    // cerca de 360°.
    static constexpr float VCC_ALIMENTACION_ENCODERS = 3.14f; // medido en la práctica
    static constexpr float VCC_REFERENCIA_ADC        = 3.30f; // referencia asumida por el ADC a 11dB
    static constexpr float RAW_MAX_EFECTIVO =
        4095.0f * (VCC_ALIMENTACION_ENCODERS / VCC_REFERENCIA_ADC); // ≈ 3896.7

    // --- Parámetros de robustez (ajustables) ---
    static const uint8_t  MAX_ERRORES_CONSECUTIVOS = 8;     // luego de esto el canal se marca "no válido" (pero conserva el último valor)
    static const uint8_t  MUESTRAS_OVERSAMPLING     = 16;   // promediado por lectura
    static constexpr float ALFA_FILTRO             = 0.15f; // suavizado exponencial (más bajo = más filtrado, más retardo)
    static constexpr float SALTO_MAX_DEG_POR_CICLO = 40.0f; // salto máximo plausible entre lecturas consecutivas -> rechaza ruido

    CanalEncoder canales[NUM_ENCODERS];

    // Offset de calibración de home, por motor: se suma a anguloContinuo/
    // anguloFiltrado al leerlos, para expresar el ángulo en el marco de
    // referencia del robot en vez del marco crudo del sensor.
    float offsetHoming[NUM_ENCODERS]      = {0.0f, 0.0f, 0.0f};
    bool  homingCalibrado[NUM_ENCODERS]   = {false, false, false};

    uint16_t leerRawPromediado(uint8_t indiceMotor);
    void     procesarLecturaValida(uint8_t indiceMotor, uint16_t raw);
    void     registrarError(uint8_t indiceMotor);

public:
    void begin();

    // Lee y procesa los 3 encoders, uno después del otro, dentro de la
    // misma llamada (mismo ciclo de loop).
    void update();

    // "motor" es el índice lógico: 0 = motor1, 1 = motor2, 2 = motor3
    // (mapeado internamente a GPIO35, GPIO34 y GPIO39).
    // leerGrados() y leerGradosContinuo() devuelven el ángulo YA
    // desplazado por el offset de home (si fue calibrado); si todavía
    // no se calibró home, el offset es 0 y devuelven el marco crudo.
    float    leerGrados(uint8_t motor) const;         // último ángulo filtrado válido, en marco del robot
    float    leerGradosContinuo(uint8_t motor) const; // ángulo acumulado sin discontinuidad, en marco del robot
    uint16_t leerRaw(uint8_t motor) const;
    bool     esValido(uint8_t motor) const;            // false si el motor superó el máximo de errores consecutivos
    bool     estaInicializado(uint8_t motor) const;
    bool     estaCalibradoHoming(uint8_t motor) const;  // false si nunca se llamó a calibrarHomingMotor() para este canal
    uint8_t  erroresConsecutivos(uint8_t motor) const;
    uint32_t tiempoDesdeUltimaLecturaOk_ms(uint8_t motor) const;
    float    leerVelocidad(uint8_t motor) const;

    // Calibración de home: alinea el ángulo actual de un motor (o los 3)
    // con el ángulo de home conocido del mecanismo. Debe llamarse una vez
    // que el eje llegó físicamente a su posición de home (ej. al terminar
    // la rutina de homing en Robot), y luego de que encoders.update() ya
    // haya generado al menos una lectura válida para ese canal en esta
    // sesión (estaInicializado(motor) == true).
    void calibrarHomingMotor(uint8_t motor, float anguloHome);
    void calibrarHoming(float anguloHomeM1, float anguloHomeM2, float anguloHomeM3);

    // fuerza a olvidar el histórico de un canal (por ejemplo tras perder
    // la lectura). También borra su calibración de home: hay que volver
    // a llamar calibrarHomingMotor() para ese canal antes de confiar en
    // leerGrados()/leerGradosContinuo() otra vez.
    void resetearCanal(uint8_t motor);
};
// Instancia global única, definida en Encoders.cpp. Cualquier archivo
// que incluya este header (main.cpp, Robot.cpp, etc.) referencia la
// MISMA instancia — evita el error de "multiple definition" y asegura
// que todos vean el mismo estado (calibración de home incluida).
extern Encoders encoders;

#endif