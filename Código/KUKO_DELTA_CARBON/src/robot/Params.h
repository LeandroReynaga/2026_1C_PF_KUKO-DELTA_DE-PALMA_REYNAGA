#ifndef PARAMS_H
#define PARAMS_H

#include <Arduino.h>

/**
 * TablaParametros
 * ---------------
 * Los valores ajustables del robot, en un solo lugar y con su rango, para
 * poder cambiarlos desde la interfaz sin recompilar y sin que nadie pueda
 * mandar un numero que rompa algo.
 *
 * COMO SE USA
 * -----------
 * Las constantes de Robot.cpp que se quieren ajustables dejan de ser
 * "static const float" y pasan a ser "static float". Despues se registran:
 *
 *     params.registrar("grab_z", &GRAB_Z, -35.0f, -30.0f, "cm", 3);
 *
 * Los puntos de uso NO cambian: siguen leyendo GRAB_Z como siempre. Eso es
 * a proposito -- la alternativa (meter todo en una struct y reescribir cada
 * uso) toca 80 lugares de un archivo que hoy anda, a cambio de nada.
 *
 * POR QUE LA TABLA ES LA FUENTE DE VERDAD
 * ---------------------------------------
 * La interfaz no lleva su propia lista de parametros: pide 'P?' al
 * conectarse y arma los controles con lo que recibe, incluidos el rango, la
 * unidad y el nivel. Agregar un parametro es entonces UNA linea aca y
 * aparece solo en la pantalla, en la pestana que le corresponde. Si la
 * interfaz tuviera su propia lista, tarde o temprano las dos dirian cosas
 * distintas y el operador estaria moviendo un slider con un rango que el
 * firmware no respeta.
 *
 * QUIEN VALIDA
 * ------------
 * El firmware, siempre. La interfaz valida tambien, pero solo para no
 * ofrecer lo imposible: un valor fuera de rango que igual llegue hasta aca
 * se RECHAZA (no se satura en silencio). Saturar dejaria a la interfaz
 * mostrando un numero que el robot no esta usando, que es peor que un
 * error visible.
 *
 * PERSISTENCIA
 * ------------
 * 'P*' guarda los valores actuales en la NVS del ESP32. Sobreviven al
 * reinicio Y al reflasheo, que es lo que importa: hoy cada vez que se
 * sube firmware se pierde toda la calibracion hecha a mano.
 */

// Niveles: deciden en que pestana de la interfaz aparece cada parametro.
// No son permisos de verdad (cualquiera puede mandar el comando por serie),
// son una separacion para que la pantalla de operacion no se llene de
// numeros que no hay que tocar todos los dias.
static const uint8_t NIVEL_OPERACION = 1;
static const uint8_t NIVEL_PROCESO   = 2;
static const uint8_t NIVEL_SERVICIO  = 3;

class TablaParametros
{
public:

    enum Resultado : uint8_t
    {
        FIJADO,      // se aplico
        DESCONOCIDO, // no existe un parametro con ese nombre
        FUERA_RANGO, // el valor esta fuera de [minimo, maximo]
        BLOQUEADO    // no se puede cambiar en este momento
    };

    // Abre la NVS. Se llama ANTES de registrar nada.
    void begin();

    // Registra un parametro. El valor que tenga la variable en este momento
    // queda guardado como el "de fabrica" (el que vuelve con 'P0').
    //
    // bloqueadoConPieza marca los que no se pueden cambiar con una pieza en
    // la mano: cambiar la altura del tacho a mitad de una maniobra manda el
    // brazo a un lugar distinto del que se planifico.
    bool registrar(const char *nombre, float *valor,
                   float minimo, float maximo,
                   const char *unidad, uint8_t nivel,
                   char tipo = 'f', bool bloqueadoConPieza = false);

    // Lee de la NVS los que hayan sido guardados y los aplica. Se llama una
    // sola vez, despues de registrar todos.
    void cargarGuardados();

    Resultado fijar(const char *nombre, float valor, bool piezaEnMano);

    // 'P?': una linea [P] por parametro y una de cierre.
    void volcar() const;

    // 'P*' y 'P0'.
    void guardar();
    void restaurarDeFabrica();

    // Sube en uno cada vez que cambia algo. Robot lo mira para saber si
    // tiene que empujar los valores nuevos a los objetos que guardan copia
    // (el CollisionGuard tiene los suyos adentro), sin tener que enterarse
    // de cual cambio.
    uint32_t generacion() const { return gen; }

    uint8_t cantidad() const { return cant; }

    // Valor actual, para los avisos. Devuelve NAN si no existe.
    float leer(const char *nombre) const;

private:

    // Tope de parametros. Con el ESP32 no hay motivo para reservar memoria
    // dinamica por 40 punteros: entra en la RAM estatica y se acabo.
    static const uint8_t CAPACIDAD = 56;

    // La NVS limita las claves a 15 caracteres. Se valida al registrar, asi
    // el problema aparece en el arranque del banco de pruebas y no la
    // primera vez que alguien intenta guardar.
    static const uint8_t NOMBRE_MAX = 12;

    struct Entrada
    {
        const char *nombre;
        float      *valor;
        float       defecto;
        float       minimo;
        float       maximo;
        const char *unidad;
        uint8_t     nivel;
        char        tipo;
        bool        bloqueadoConPieza;
    };

    Entrada  tabla[CAPACIDAD];
    uint8_t  cant = 0;
    uint32_t gen  = 0;
    bool     nvsLista = false;

    int8_t buscar(const char *nombre) const;

    void imprimirEntrada(const Entrada &e) const;

    // Redondeo segun el tipo declarado: un parametro entero que llega como
    // 79.99 tiene que quedar en 80, no en 79. Los booleanos quedan en 0 o 1.
    static float ajustarAlTipo(float valor, char tipo);
};

extern TablaParametros params;

#endif
