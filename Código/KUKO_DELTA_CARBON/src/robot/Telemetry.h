#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

/**
 * Telemetria
 * ----------
 * Las tres lineas periodicas que el ESP32 le manda a la interfaz, con su
 * cadencia y su formato. Ver pc/PROTOCOLO.md, que es el contrato: si este
 * archivo y ese documento no coinciden, el equivocado es este.
 *
 * POR QUE TRES LINEAS Y NO UNA
 * ----------------------------
 * Porque no todo cambia al mismo ritmo y el puerto tiene 11,5 kB/s:
 *
 *   [T]  10 Hz    lo que se mueve: angulos, error, umbral, finales de
 *                 carrera, bomba. Es lo que dibuja los diales en vivo.
 *   [E]   1 Hz    lo que decide: modo, cola, caja, contadores.
 *   [H]   0,5 Hz  salud: encoders, ganancia, RAM libre, vueltas de loop.
 *
 * Mandar todo a 10 Hz seria gastar 5 veces mas ancho de banda para mostrar
 * un contador de piezas que cambia cada varios segundos.
 *
 * ARRANCA APAGADA
 * ---------------
 * A proposito: el firmware tiene que poder usarse con el monitor serie de
 * PlatformIO sin que lo inunde de lineas. La interfaz manda 'V1' cuando se
 * conecta.
 *
 * QUIEN ARMA LOS DATOS
 * --------------------
 * Robot: es el unico que ve al mismo tiempo la maquina de estados, el
 * guard, los encoders y los finales. Esta clase no sabe nada del robot,
 * solo formatea structs y lleva los relojes. Esa separacion es la que
 * evita que Robot.cpp se llene de Serial.print y que agregar un campo
 * obligue a tocar la maquina de estados.
 */

static const uint8_t TELE_EJES = 3;

// ------------------------------------------------------------------
//  [T] -- lo que se mueve
// ------------------------------------------------------------------
struct TeleRapida
{
    uint32_t t_ms;
    uint8_t  estado;
    bool     bomba;
    bool     finales[TELE_EJES];   // finales de carrera, true = pisado

    float ang[TELE_EJES];  // angulo medido por el encoder, grados
    float cmd[TELE_EJES];  // angulo segun los pasos emitidos, grados
    float err[TELE_EJES];  // el error que mira el guard
    float umb[TELE_EJES];  // umbral efectivo de ese eje AHORA
    float vel[TELE_EJES];  // velocidad comandada, grados/s
};

// ------------------------------------------------------------------
//  [E] -- lo que decide
// ------------------------------------------------------------------
struct TeleProceso
{
    uint32_t    t_ms;
    uint8_t     estado;
    const char *estadoNombre;

    char     modo;           // 'C', 'F' o 'A'
    char     modoPendiente;  // igual, o '-' si no hay
    bool     esperandoTapa;
    uint32_t tapaRestante_ms;

    uint8_t  cola;
    uint32_t colaAntiguedad_ms;

    bool     homed;
    uint8_t  guard;          // 0 desarmado, 1 promediando, 2 armado
    bool     observando;
    bool     paradasActivas;

    bool     cinta;
    uint8_t  cintaPwm;       // %

    const char *layout;      // 6 colores, o NULL fuera del modo caja
    const bool *llenas;      // 6 celdas,  o NULL
    uint8_t     celdaReservada;

    char     piezaColor;     // '-' si no hay pieza en curso
    char     piezaForma;
    float    piezaY;
    uint8_t  tacho;

    // Modo teach: puntos cargados en la ruta y cual se esta ejecutando
    // (0 = ninguno). Van en [E] y no en [T] porque cambian de a un punto,
    // no en cada vuelta.
    uint8_t  teachPuntos;
    uint8_t  teachIndice;

    uint32_t detectadas;
    uint32_t depositadas;
    uint32_t descartadas;
    uint32_t fallos;

    uint32_t porColor[3];    // R, G, B
    uint32_t porForma[3];    // S, H, C
};

// ------------------------------------------------------------------
//  [H] -- salud
// ------------------------------------------------------------------
struct TeleSalud
{
    uint32_t t_ms;
    uint32_t uptime_s;
    uint32_t loopHz;
    uint32_t heapLibre;

    struct Eje
    {
        const char *encoder;   // "ok", "caido" o "rango"
        float       ganancia;
        float       atraso_ms;
        float       pico;
        float       picoReposo;
        float       fuga;
        uint16_t    rawMin;
        uint16_t    rawMax;
        uint16_t    resincronizaciones;
    } ejes[TELE_EJES];
};

class Telemetria
{
public:

    // Anuncia el firmware y la version del protocolo. La interfaz compara
    // contra la suya y no habilita los controles si no coinciden: un
    // protocolo desparejo hay que verlo ANTES de mover el brazo.
    void anunciar(uint8_t cantidadEstados, uint8_t cantidadParametros) const;

    void setActiva(bool encendida);
    bool activa() const { return encendida; }

    // 'V?': una foto de [E] y [H] sin encender el stream. Es lo que pide la
    // interfaz al conectarse, antes de decidir si enciende la telemetria.
    void pedirFoto();

    // Se llama en cada vuelta del loop. Es lo que permite informar las
    // vueltas por segundo, que es el primer sintoma de que algo empezo a
    // bloquear (un delay, un Serial.print que llena la FIFO, un encoder
    // reintentando).
    void contarVuelta();

    // Vueltas por segundo de la ultima ventana. Es el primer sintoma de
    // que algo empezo a bloquear el loop.
    uint32_t loopHz() const { return vueltasHz; }

    // Relojes. Devuelven true cuando toca emitir esa linea; Robot llena
    // entonces la struct que corresponde y llama al emitir de abajo.
    bool tocaRapida(uint32_t ahora);
    bool tocaProceso(uint32_t ahora);
    bool tocaSalud(uint32_t ahora);

    void emitirRapida(const TeleRapida &d) const;
    void emitirProceso(const TeleProceso &d) const;
    void emitirSalud(const TeleSalud &d) const;

    // Periodos en ms. Son float y publicos porque se registran en la tabla
    // de parametros, que trabaja con punteros a float.
    float periodoRapida_ms  = 100.0f;
    float periodoProceso_ms = 1000.0f;
    float periodoSalud_ms   = 2000.0f;

private:

    bool encendida = false;

    uint32_t ultimaRapida_ms  = 0;
    uint32_t ultimoProceso_ms = 0;
    uint32_t ultimaSalud_ms   = 0;

    // Fotos pendientes pedidas con 'V?'. Se sirven aunque la telemetria
    // este apagada.
    bool fotoProceso = false;
    bool fotoSalud   = false;

    uint32_t vueltas         = 0;
    uint32_t vueltasHz       = 0;
    uint32_t ventanaVueltas_ms = 0;

    void imprimirEje(uint8_t i, const TeleSalud::Eje &e) const;
};

#endif
