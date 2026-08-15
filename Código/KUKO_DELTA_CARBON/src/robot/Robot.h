#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Stepper.h"
#include "CollisionGuard.h"
#include "FaultLog.h"
#include "../hardware/Endstops.h"
#include "Pinout.h"
#include "../hardware/Motors.h"

/**
 * Robot
 * ------
 * Orquestador del ciclo de clasificacion: homing -> esperar pieza ->
 * interceptarla en movimiento sobre la cinta -> dejarla en el tacho que
 * corresponde segun el modo de clasificacion -> siguiente.
 *
 * Todas las esperas son NO BLOQUEANTES (timestamps con millis()). Nunca
 * agregar un delay() en esta clase: congela la generacion de pasos, la
 * lectura de encoders y la maquina de estados entera al mismo tiempo.
 */
class Robot
{
public:

    enum RobotState
    {
        IDLE,
        HOMING,          // busca los finales de carrera y calibra los encoders
        WAIT_PIECE,      // quieto en home, sin piezas alcanzables en la cola
        GO_HOME_IDLE,    // volviendo a home; se interrumpe si aparece una pieza
        PICK_APPROACH,   // tramo 1: accel MAX al punto de aproximacion, y espera ahi
        PICK_DESCEND,    // tramo 2: accel MIN, entra a la pieza a favor de la cinta
        PICK_LIFT,       // tramo 3: accel MAX, despega la pieza de la cinta
        GO_BIN,          // tramo 4: accel MAX hasta el tacho que corresponde
        BIN_SETTLE,      // 0,2 s quieto para que la pieza caiga vertical
        RELEASE_WAIT,    // bomba apagada, esperando que la pieza se despegue
        BOX_TRANSIT,     // alfajores: accel MAX, cruza por encima de la caja
        BOX_APPROACH,    // alfajores: accel MAX, baja hasta 3 cm sobre la celda
        BOX_DESCEND,     // alfajores: accel MIN, apoya el alfajor en la celda
        BOX_LIFT,        // alfajores: accel MAX, sale de la caja hacia arriba
        COLLISION_STOP,  // colision detectada: frenado y quieto antes de recalibrar
        ERROR
    };

    // Como se decide a donde va cada pieza. Lo elige el operador desde la
    // interfaz de Python ('C', 'F' o 'A' por Serial).
    enum SortMode : uint8_t
    {
        SORT_BY_COLOR = 0, // tacho 1 rojo, 2 verde, 3 azul
        SORT_BY_SHAPE = 1, // tacho 1 cuadrado, 2 hexagono, 3 circulo
        SORT_ALFAJORES = 2 // llena una caja de 6 con circulos; el resto pasa
    };

    Robot();

    void begin();

    void update();

    // conservarContexto = true se usa SOLO en la recalibracion posterior a
    // una colision: la cinta nunca se detuvo, asi que las piezas que estaban
    // en la cola siguen teniendo timestamps validos y hay que volver a
    // mirarlas (varias ya no van a ser alcanzables, eso lo filtra
    // planificarPieza). Con false (el arranque y el reset manual) la cola se
    // descarta entera.
    void startHoming(bool conservarContexto = false);

    bool homingFinished() const;

    RobotState getState() const;

    const char *nombreEstado(RobotState s) const;

    bool goToPositionIK(float x, float y, float z,
                        const Motors::MotionLimits &limits = Motors::DEFAULT_LIMITS);

    // Parada de emergencia manual: detiene los 3 motores donde esten y pasa
    // a ERROR. Se dispara con 'R' por el monitor serie. Desde ERROR, otra
    // 'R' rehomea y reinicia el ciclo.
    void emergencyStop();

private:

    // ------------------------------------------------------------------
    //  Pieza detectada por el sistema de vision (Python)
    // ------------------------------------------------------------------
    struct Piece
    {
        float    y;             // cm, Y del centro de la pieza al cruzar la linea
        char     color;         // 'R', 'G' o 'B'
        char     shape;         // 'S', 'H' o 'C'
        uint32_t detectedAt_ms; // millis() en que llego el mensaje
    };

    // Cola circular de piezas pendientes. La consigna es "sin limite", pero
    // en un micro la memoria es finita: con este tamano entran ~32 piezas,
    // muchisimo mas que las que caben fisicamente en el tramo util de la
    // cinta. Si igual se llenara, se descarta la MAS NUEVA y se avisa por
    // Serial (perder la mas vieja seria peor: es la unica que todavia
    // podria llegar a ser alcanzable).
    static const uint8_t QUEUE_CAPACITY = 32;

    Piece   pieceQueue[QUEUE_CAPACITY];
    uint8_t queueHead  = 0;
    uint8_t queueCount = 0;

    bool queuePush(const Piece &p);
    bool queuePop(Piece &out);

    // Motores
    Stepper motor1;
    Stepper motor2;
    Stepper motor3;

    // Finales de carrera
    Endstops endstops;

    // Estado del robot
    RobotState state;

    // Estado de cada eje
    bool axis1Homed;
    bool axis2Homed;
    bool axis3Homed;
    bool homed = false;

    // ------------------------------------------------------------------
    //  Modo de clasificacion
    // ------------------------------------------------------------------
    // MODO CON EL QUE ARRANCA EL ROBOT. Es el unico lugar donde se elige:
    // mientras la vision de Python tenga tomado el puerto serie no se le
    // pueden mandar comandos a mano ('C' / 'F' / 'A'), asi que el modo de
    // arranque se fija aca y se recompila.
    //
    // OJO: el robot NO pregunta nada al encender (no hay contra que
    // comparar todavia), asi que la tapa de la caja tiene que estar puesta
    // ANTES de darle energia si esto queda en SORT_ALFAJORES -- y sacada si
    // queda en color o forma.
    //
    // Un cambio de modo que llega a mitad de una maniobra NO se aplica en
    // el momento: se guarda como pendiente y se aplica recien cuando el
    // robot no tiene ninguna pieza en la mano, para no cambiarle el destino
    // a una pieza que ya esta en vuelo.
    SortMode sortMode        = SORT_BY_COLOR;
    SortMode pendingSortMode = SORT_BY_COLOR;
    bool     sortModePending = false;

    void aplicarModoPendiente();
    const char *nombreModo(SortMode m) const;

    static bool esAlfajores(SortMode m) { return m == SORT_ALFAJORES; }

    // El modo al que el robot va a quedar: el pendiente si hay uno, el
    // activo si no. Es contra este que se compara un pedido nuevo.
    SortMode modoObjetivo() const { return sortModePending ? pendingSortMode : sortMode; }

    // ------------------------------------------------------------------
    //  Confirmacion de la tapa
    // ------------------------------------------------------------------
    // Entrar o salir del modo alfajores implica poner o sacar la tapa con
    // forma de caja que va sobre los tachos. El firmware no tiene forma de
    // ver si esta puesta, y arrancar el modo equivocado significa tirar
    // piezas contra la tapa (o dentro de tachos tapados), asi que cualquier
    // cambio que cruce ese limite se pide dos veces: el segundo pedido es
    // la confirmacion de que la tapa ya esta como corresponde.
    //
    // Cuando exista la interfaz, este mismo mecanismo es el que se va a
    // enganchar al dialogo de confirmacion (el firmware ya no depende de
    // que alguien tipee la letra dos veces: le alcanza con recibirla).
    SortMode modoAConfirmar        = SORT_BY_COLOR;
    bool     esperandoConfirmacion = false;
    uint32_t confirmacionPedida_ms = 0;

    void pedirModo(SortMode nuevo, char letra);
    void aplicarModo(SortMode nuevo);
    void vencerConfirmacion();

    // ------------------------------------------------------------------
    //  Caja de alfajores (modo SORT_ALFAJORES)
    // ------------------------------------------------------------------
    // Grilla de 2 filas x 3 columnas. Las celdas se numeran 1 a 6 desde la
    // fila mas cercana a la cinta; en el codigo van indexadas 0 a 5.
    //
    // boxLayout dice de que color tiene que ser el alfajor de cada celda, y
    // boxFilled cuales ya estan puestas. Un circulo que llega sirve solo si
    // queda alguna celda vacia de su color; todo lo demas (cuadrados,
    // hexagonos, colores que ya no faltan) se deja pasar por la cinta.
    static const uint8_t BOX_CELLS     = 6;
    static const uint8_t CELDA_NINGUNA = 0xFF;

    char    boxLayout[BOX_CELLS];
    bool    boxFilled[BOX_CELLS];
    bool    boxLayoutValido = true;
    bool    boxComplete     = false;

    // Celda que reservo la pieza que el robot tiene en la mano. Se marca
    // como llena recien cuando la solto: si la maniobra se aborta a mitad
    // (colision, ventana de agarre perdida), la celda queda libre.
    uint8_t currentCell = CELDA_NINGUNA;

    bool    piezaSirveParaCaja(const Piece &p, uint8_t &celda) const;
    void    marcarCeldaLlena(uint8_t celda);
    void    reiniciarCaja(bool avisar);
    void    imprimirCaja() const;
    uint8_t celdasLlenas() const;

    // Una caja de 6 no se puede llenar con mas de 3 alfajores del mismo
    // color (con 4 rojos y 2 verdes, o con 6 verdes, el robot esperaria
    // para siempre una pieza que nunca va a poder ubicar).
    static bool layoutValido(const char *layout);
    void        aplicarLayout(const char *layout);

    // ------------------------------------------------------------------
    //  Pieza en curso y maniobra planificada
    // ------------------------------------------------------------------
    Piece   currentPiece;
    uint8_t currentBin = 0;    // 0, 1 o 2

    float approachX = 0.0f, approachY = 0.0f, approachZ = 0.0f;

    // Destino comandado del tramo 2: sobrepasa a la pieza y baja un pelo
    // por debajo de su cara, para que el contacto ocurra EN MOVIMIENTO y a
    // la misma velocidad que la cinta (ver ConveyorIntercept.h).
    float descendEndX = 0.0f, descendEndY = 0.0f, descendEndZ = 0.0f;

    float liftX = 0.0f, liftY = 0.0f, liftZ = 0.0f;

    // Solo para diagnostico por Serial (no se usan para mover).
    float lastGrabX = 0.0f;
    float lastContactSpeedX = 0.0f;

    // Instante absoluto en que hay que lanzar el tramo 2. Es lo que fija la
    // precision del encuentro con la pieza.
    uint32_t descendStart_ms = 0;
    uint8_t  replanCount     = 0; // reintentos de la pieza actual

    bool moveIssued = false; // el movimiento del estado actual ya se comando

    bool pumpOn     = false;

    // ------------------------------------------------------------------
    //  Rutinas de estado
    // ------------------------------------------------------------------
    void updateHoming();
    void updateWaitPiece();
    void updateGoHomeIdle();
    void updatePickApproach();
    void updatePickDescend();
    void updatePickLift();
    void updateGoBin();
    void updateBinSettle();
    void updateReleaseWait();
    void updateBoxTransit();
    void updateBoxApproach();
    void updateBoxDescend();
    void updateBoxLift();
    void updateCollisionStop();

    // ------------------------------------------------------------------
    //  Supervision por encoders (lazo cerrado de seguridad)
    // ------------------------------------------------------------------
    // Los encoders NO controlan la posicion: el control sigue siendo por
    // micropasos. Lo unico que hacen es vigilar que el brazo este donde los
    // pasos dicen que deberia estar; si no lo esta, es que choco contra
    // algo. Ver CollisionGuard.h.
    CollisionGuard guard;
    FaultLog       fallos;

    // Si las colisiones frenan el robot o solo se avisan por Serial. Se
    // puede cambiar en caliente con 'G', pero el valor inicial sale de
    // GuardConfig::FRENAR_POR_COLISION: cuando el puerto serie lo esta
    // usando la vision, no hay forma de mandar comandos a mano y la unica
    // via es el codigo.
    bool supervisionHabilitada = GuardConfig::FRENAR_POR_COLISION;

    void supervisarColision();
    void dispararColision(uint8_t eje, float errorDeg, float cmdDelta, float encDelta);

    // Arma el registro con el contexto actual (estado, pieza en curso,
    // tacho de destino) y lo manda al FaultLog.
    void registrarFallo(uint8_t tipo, uint8_t eje,
                        float errorDeg = 0.0f, float cmdDelta = 0.0f, float encDelta = 0.0f);

    void imprimirEstadoSupervision() const;

    // Traza en vivo de la supervision ('M'), para diagnosticar falsos
    // positivos mirando el movimiento entero en vez de un numero suelto.
    void imprimirTraza();

    // Resumen de una linea que sale solo cada tantos segundos, para tener
    // datos de calibracion cuando el puerto serie esta tomado por la vision.
    void imprimirDiagnosticoCorto() const;

    bool     trazaActiva          = false;
    uint32_t ultimaTraza_ms       = 0;
    uint32_t ultimoDiagnostico_ms = 0;

    static const uint32_t TRAZA_INTERVALO_MS = 50; // 20 Hz

    // Donde esta AHORA una pieza sobre la cinta, segun cuando se detecto.
    float piezaXEstimada(const Piece &p) const;

    bool hayManiobraEnCurso() const;
    bool hayPiezaEnMano() const;

    uint32_t collisionStart_ms = 0;
    uint32_t homingStart_ms    = 0;

    // Colisiones seguidas sin haber completado una pieza en el medio. Si se
    // repiten, es que hay algo trabado de verdad y rehomear no lo va a
    // arreglar: mejor parar y avisar que insistir golpeando el robot.
    uint8_t colisionesSeguidas = 0;

    // Toma la proxima pieza de la cola que sea alcanzable y arranca la
    // maniobra. Devuelve false si no quedo ninguna.
    bool iniciarSiguientePieza();

    // Planifica la maniobra completa para una pieza (los 3 puntos + el
    // instante de bajada). false si ya no se llega dentro del area.
    bool planificarPieza(const Piece &p);

    uint8_t binIndexFor(const Piece &p) const;

    // ------------------------------------------------------------------
    //  Consola serie
    // ------------------------------------------------------------------
    void procesarSerial();
    void procesarComando(char *cmd, uint8_t len);

    char    cmdBuffer[32];
    uint8_t cmdLen = 0;
    bool    cmdOverflow = false; // descartando el resto de una linea larga

    uint32_t homingSettleStart_ms = 0;
    uint32_t binSettleStart_ms    = 0;
    uint32_t releaseStart_ms      = 0;

    static constexpr long MICROPASOS = 10000;

    static constexpr float HOME_ANGLE_M1 = -45.1f;
    static constexpr float HOME_ANGLE_M2 = -44.3f;
    static constexpr float HOME_ANGLE_M3 = -44.5f;

    static long angleToSteps(float angle)
    {
        return lround(angle * MICROPASOS / 360.0f);
    }

    // Los 3 motores llegaron a su objetivo.
    bool enPosicion() const
    {
        return motor1.targetReached() &&
               motor2.targetReached() &&
               motor3.targetReached();
    }
};

#endif
