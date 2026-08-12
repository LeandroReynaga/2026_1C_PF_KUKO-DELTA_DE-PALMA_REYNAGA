#ifndef FAULT_LOG_H
#define FAULT_LOG_H

#include <Arduino.h>

/**
 * FaultLog
 * --------
 * Registro de fallos del robot: cuantos, cuando, de que tipo y -- lo mas
 * importante para el operador -- CON QUE PIEZA. Cada fallo se imprime por
 * Serial apenas ocurre y ademas queda guardado en un buffer circular que se
 * puede volcar entero con el comando 'D'.
 *
 * El formato de las lineas es clave=valor, pensado para que la interfaz de
 * Python lo parsee sin ambiguedad (y que igual se lea comodo a ojo en el
 * monitor serie):
 *
 *   [FALLO] n=1 t=125430 tipo=COLISION eje=3 err=-12.40 dcmd=18.30
 *           denc=5.90 estado=GO_BIN pieza=1 enmano=1 color=R forma=S
 *           py=3.50 px=-4.20 tacho=1
 *
 *   n       numero de fallo desde el encendido (contador global)
 *   t       millis() en que ocurrio
 *   eje     1-3, o 0 si no aplica al eje
 *   err     diferencia encoder - pasos, en grados (lo que disparo)
 *   dcmd    grados que recorrio el eje segun los micropasos
 *   denc    grados que recorrio segun el encoder (dcmd vs denc dice si el
 *           brazo se trabo -- denc ~ 0 -- o si el encoder mide mal)
 *   estado  en que parte del ciclo estaba el robot
 *   pieza   1 si habia una pieza en curso, 0 si no
 *   enmano  1 si ya la tenia agarrada cuando fallo
 *   py, px  donde estaba esa pieza sobre la cinta, en cm
 *   tacho   a que compartimento iba (1-3), 0 si todavia no aplicaba
 */

enum TipoFallo : uint8_t
{
    FALLO_COLISION = 0, // encoder y pasos discrepan: el brazo choco o se trabo
    FALLO_ENCODER,      // un encoder dejo de ser confiable (se sigue trabajando sin supervisar ese eje)
    FALLO_HOMING,       // el homing no termino en el tiempo maximo
    FALLO_MANUAL,       // parada de emergencia pedida por el operador
    FALLO_CANT_TIPOS
};

struct RegistroFallo
{
    uint32_t    numero   = 0;
    uint32_t    t_ms     = 0;
    uint8_t     tipo     = FALLO_COLISION;
    uint8_t     eje      = 0;     // 1-3; 0 = no aplica
    float       errorDeg = 0.0f;
    float       cmdDelta = 0.0f;
    float       encDelta = 0.0f;
    const char *estado   = "?";   // literal de Robot::nombreEstado()
    bool        conPieza = false;
    bool        enMano   = false;
    char        color    = '-';
    char        forma    = '-';
    float       piezaX   = 0.0f;
    float       piezaY   = 0.0f;
    uint8_t     tacho    = 0;     // 1-3; 0 = no aplica
};

class FaultLog
{
public:

    // Ultimos fallos que se conservan para consulta. Con 16 alcanza de
    // sobra para diagnosticar una corrida; el contador total no se pierde
    // aunque el buffer de vueltas.
    static const uint8_t CAPACIDAD = 16;

    void begin();

    // Numera el fallo, lo guarda y lo imprime.
    void registrar(RegistroFallo r);

    void imprimirResumen() const;
    void imprimirHistorial() const;

    uint32_t total() const { return contador; }
    uint32_t totalTipo(uint8_t tipo) const;

    static const char *nombreTipo(uint8_t tipo);

private:

    RegistroFallo buffer[CAPACIDAD];
    uint8_t       cabeza   = 0; // proximo lugar a escribir
    uint8_t       cantidad = 0; // cuantos hay guardados (<= CAPACIDAD)
    uint32_t      contador = 0; // fallos totales desde el encendido
    uint32_t      porTipo[FALLO_CANT_TIPOS] = {0, 0, 0, 0};

    static void imprimirLinea(const RegistroFallo &r);
};

#endif
