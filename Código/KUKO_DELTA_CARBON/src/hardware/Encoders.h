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
    static constexpr float VCC_ALIMENTACION_ENCODERS = 3.46f; // medido en la práctica
    static constexpr float VCC_REFERENCIA_ADC        = 3.30f; // referencia asumida por el ADC a 11dB
    static constexpr float RAW_MAX_EFECTIVO =
        4095.0f * (VCC_ALIMENTACION_ENCODERS / VCC_REFERENCIA_ADC); // ≈ 3896.7

    // --- Parámetros de robustez (ajustables) ---
    static const uint8_t  MAX_ERRORES_CONSECUTIVOS = 8;     // luego de esto el canal se marca "no válido" (pero conserva el último valor)
    static const uint8_t  MUESTRAS_OVERSAMPLING     = 16;   // promediado por lectura (dentro de un mismo update())
    static constexpr float ALFA_FILTRO             = 0.15f; // suavizado exponencial (más bajo = más filtrado, más retardo)
    static constexpr float SALTO_MAX_DEG_POR_CICLO = 40.0f; // salto máximo plausible entre lecturas consecutivas -> rechaza ruido

    CanalEncoder canales[NUM_ENCODERS];

    // Offset de calibración de home, por motor.
    float offsetHoming[NUM_ENCODERS]      = {0.0f, 0.0f, 0.0f};
    bool  homingCalibrado[NUM_ENCODERS]   = {false, false, false};

    // --- Media móvil para calibración de home ---
    // Mientras está activa, cada update() suma anguloContinuo al
    // acumulador de cada canal. Al calibrar, se usa el PROMEDIO de todas
    // las muestras juntadas durante la ventana (ej. el segundo de espera
    // en el endstop) en vez de una única lectura puntual -- mucho más
    // robusto ante ruido residual o micro-vibración al frenar.
    bool     acumulandoHoming = false;
    double   sumaAnguloContinuoHoming[NUM_ENCODERS] = {0.0, 0.0, 0.0};
    uint32_t muestrasHoming[NUM_ENCODERS]           = {0, 0, 0};

    // Un canal que se pasó de MAX_ERRORES_CONSECUTIVOS queda enganchado: el
    // filtro de plausibilidad compara contra la última lectura aceptada, así
    // que si el eje se fue lejos de ahí, rechaza todo para siempre. Esta
    // bandera dice que ese canal tuvo que reengancharse a la fuerza; la
    // lectura vuelve a servir, pero no es confiable para supervisar hasta
    // que el homing rehaga la referencia.
    bool resincronizado[NUM_ENCODERS] = {false, false, false};

    uint16_t leerRawPromediado(uint8_t indiceMotor);
    void     procesarLecturaValida(uint8_t indiceMotor, uint16_t raw);
    void     registrarError(uint8_t indiceMotor);
    void     resincronizar(uint8_t indiceMotor, uint16_t raw,
                            float anguloNuevo, float diferencia);

public:
    void begin();

    // Lee y procesa los 3 encoders, uno después del otro, dentro de la
    // misma llamada (mismo ciclo de loop).
    void update();

    // "motor" es el índice lógico: 0 = motor1, 1 = motor2, 2 = motor3
    // (mapeado internamente a GPIO35, GPIO34 y GPIO39).
    float    leerGrados(uint8_t motor) const;         // último ángulo filtrado válido, en marco del robot
    float    leerGradosContinuo(uint8_t motor) const; // ángulo acumulado sin discontinuidad, en marco del robot

    // Igual que leerGradosContinuo() pero SIN el offset de calibración de
    // home, o sea en el marco del sensor. Lo usa la supervisión de
    // colisiones (CollisionGuard), que compara INCREMENTOS: el offset se
    // cancelaría solo al restar, pero cambia en medio de la ventana de
    // asentamiento del homing (justo cuando se calibra), y eso sí ensuciaría
    // la referencia. Es solo un getter: no modifica ni saltea ningún filtro.
    float    leerGradosContinuoCrudo(uint8_t motor) const;
    uint16_t leerRaw(uint8_t motor) const;
    bool     esValido(uint8_t motor) const;
    bool     estaInicializado(uint8_t motor) const;
    bool     estaCalibradoHoming(uint8_t motor) const;
    uint8_t  erroresConsecutivos(uint8_t motor) const;

    // true si ese canal tuvo que reengancharse a la fuerza (ver
    // resincronizado[]). La limpia CollisionGuard al rearmar la referencia
    // en el homing.
    bool     huboResincronizacion(uint8_t motor) const;
    void     limpiarResincronizacion(uint8_t motor);
    uint32_t tiempoDesdeUltimaLecturaOk_ms(uint8_t motor) const;
    float    leerVelocidad(uint8_t motor) const;

    // Arranca la ventana de acumulación para calibración de home: resetea
    // los acumuladores de media móvil de los 3 canales. Llamar UNA vez,
    // justo cuando los 3 ejes acaban de llegar a su endstop (arranque del
    // segundo de espera en Robot).
    void iniciarAsentamientoHoming();

    // Calibra un motor usando el PROMEDIO acumulado desde
    // iniciarAsentamientoHoming() (si hay muestras); si no se inició una
    // ventana de asentamiento, cae a usar la lectura instantánea de
    // anguloContinuo (comportamiento anterior, por compatibilidad).
    void calibrarHomingMotor(uint8_t motor, float anguloHome);

    // Calibra los 3 motores usando la media acumulada.
    void calibrarHoming(float anguloHomeM1, float anguloHomeM2, float anguloHomeM3);

    void resetearCanal(uint8_t motor);
};

extern Encoders encoders;

#endif