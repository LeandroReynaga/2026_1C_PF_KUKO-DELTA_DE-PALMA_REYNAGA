#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Stepper.h"
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
        ERROR
    };

    // Como se decide a que tacho va cada pieza. Lo elige el operador desde
    // la interfaz de Python ('C' o 'F' por Serial).
    enum SortMode : uint8_t
    {
        SORT_BY_COLOR = 0, // tacho 1 rojo, 2 verde, 3 azul
        SORT_BY_SHAPE = 1  // tacho 1 cuadrado, 2 hexagono, 3 circulo
    };

    Robot();

    void begin();

    void update();

    void startHoming();

    bool homingFinished() const;

    RobotState getState() const;

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
    // Al encender, el robot SIEMPRE arranca clasificando por COLOR; desde
    // ahi el operador lo cambia con 'C' / 'F' desde la interfaz de Python.
    //
    // Un cambio de modo que llega a mitad de una maniobra NO se aplica en
    // el momento: se guarda como pendiente y se aplica recien cuando el
    // robot no tiene ninguna pieza en la mano, para no cambiarle el tacho
    // de destino a una pieza que ya esta en vuelo.
    SortMode sortMode        = SORT_BY_COLOR;
    SortMode pendingSortMode = SORT_BY_COLOR;
    bool     sortModePending = false;

    void aplicarModoPendiente();
    const char *nombreModo(SortMode m) const;

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
