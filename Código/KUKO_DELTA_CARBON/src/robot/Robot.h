#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Stepper.h"
#include "CollisionGuard.h"
#include "FaultLog.h"
#include "Telemetry.h"
#include "Params.h"
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
        ERROR,

        // Modo aprendizaje: el ciclo de clasificacion esta suspendido y el
        // brazo lo maneja el operador desde la interfaz (jog manual) o la
        // reproduccion de una secuencia grabada. Va AL FINAL a proposito:
        // el indice de este enum viaja tal cual en la telemetria y meter un
        // estado en el medio corre la numeracion de todos los siguientes
        // (ver PROTOCOLO.md 3.1). Por eso ERROR ya no es el ultimo.
        TEACH
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

    // La llama loop() en cada vuelta. Alimenta el contador de vueltas por
    // segundo que sale en la linea de salud de la telemetria.
    void contarVuelta() { telemetria.contarVuelta(); }

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
    SortMode sortMode        = SORT_ALFAJORES;
    SortMode pendingSortMode = SORT_ALFAJORES;
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
    //  Modo Teach (aprendizaje)
    // ------------------------------------------------------------------
    // Dos cosas distintas conviven en el mismo estado:
    //
    //   JOG        el operador mueve el brazo a mano. La interfaz manda una
    //              DIRECCION (no una posicion) y el firmware la integra en
    //              tramos cortos: asi el brazo va siempre a un destino que
    //              ya sabe que es alcanzable y esta dentro del volumen, y
    //              nunca queda persiguiendo un objetivo que se le escapa.
    //
    //   PLAYBACK   se sube una ruta de puntos y el firmware la recorre
    //              solo. Esta del lado del ESP32 y no del PC porque encadenar
    //              los tramos por serie meteria el ida y vuelta del enlace
    //              entre punto y punto; asi el salto de uno al siguiente es
    //              una vuelta de loop.
    //
    // La grabacion, el nombre de cada secuencia y los porcentajes ya
    // verificados viven en la interfaz de Python: el firmware solo ejecuta.
    static const uint8_t TEACH_MAX_PUNTOS = 150;

    struct TeachPunto
    {
        float    x, y, z;      // punta del gripper, cm
        uint16_t espera_ms;    // cuanto se queda quieto AL LLEGAR
        bool     bomba;        // estado del vacio al llegar
    };

    TeachPunto teachRuta[TEACH_MAX_PUNTOS];
    uint8_t    teachPuntos = 0;   // cargados en el buffer
    uint8_t    teachIndice = 0;   // punto que se esta ejecutando (0-based)

    bool     teachReproduciendo = false;
    bool     teachLanzado       = false; // el movimiento del punto ya salio
    bool     teachEsperando     = false; // cumpliendo la espera del punto
    uint32_t teachEsperaHasta_ms = 0;

    // Porcentaje de VEL_MAX / ACC_RAPIDA con el que corre la reproduccion.
    // Lo fija la interfaz en cada pasada (15 -> 50 -> 100), que es la
    // verificacion progresiva de una secuencia recien grabada.
    float teachEscala = 15.0f;

    // Se pidio entrar a teach pero el robot todavia esta terminando lo que
    // tenia entre manos. No se entra en el acto a proposito: se entra desde
    // home y con el brazo detenido, y eso es lo que hace que la posicion de
    // partida sea conocida sin necesitar cinematica DIRECTA en el firmware.
    bool teachPedido = false;

    // Posicion comandada de la punta (cm). Es la referencia del jog y la
    // deja escrita todo movimiento de teach.
    float teachX = 0.0f, teachY = 0.0f, teachZ = 0.0f;

    // Direccion de jog pedida por la interfaz, cada componente en [-1, 1].
    // VENCE SOLA: si la interfaz deja de refrescarla (se cerro el navegador,
    // se corto el enlace) el brazo se para en el tramo que este haciendo,
    // en vez de seguir hasta la pared del volumen.
    float    jogVx = 0.0f, jogVy = 0.0f, jogVz = 0.0f;
    uint32_t jogVigenteHasta_ms = 0;
    uint32_t jogUltimoPaso_ms   = 0;

    static const uint32_t TEACH_JOG_TICK_MS  = 80;  // cadencia de los tramos
    static const uint32_t TEACH_JOG_VIDA_MS  = 350; // vigencia de la direccion

    // Volcado de la posicion COMANDADA a 20 Hz, que la interfaz enciende
    // mientras tiene la pestana de teach a la vista ('JG1').
    //
    // Es lo que se graba. Podria grabarse en cambio la posicion MEDIDA, que
    // ya viaja en [T], pero el AS5600 analogico tiene ~1 grado de ruido y en
    // cartesiano eso son milimetros que despues se reproducen como temblor:
    // lo que el operador enseño es a donde llevo el brazo, no como vibro el
    // sensor. Y podria calcularla la interfaz integrando el jog, pero el
    // firmware saltea ticks cuando el brazo no llega, asi que las dos cuentas
    // divergirian sin que nadie se entere.
    //
    // Solo existe con la pestana abierta: 20 Hz por 38 bytes son 760 B/s, que
    // no tiene sentido gastar el 99 % del tiempo en que nadie los mira.
    bool     teachStream          = false;
    uint32_t teachStreamUltimo_ms = 0;

    static const uint32_t TEACH_STREAM_MS = 50;

    // Tramo que se esta recorriendo ahora mismo: de donde salio, cuanto
    // mide en cm y cuanto en pasos del eje que mas recorre. Con eso se
    // calcula a que altura del tramo hay que empezar a mezclar la esquina.
    float teachOrigenX = 0.0f, teachOrigenY = 0.0f, teachOrigenZ = 0.0f;
    float teachTramoCm    = 0.0f;
    long  teachTramoPasos = 0;

    void updateTeach();
    void updateTeachPlayback();

    // Limites de movimiento del modo teach al `escalaPct` %. Un solo lugar
    // para que el jog y la reproduccion no se separen nunca.
    Motors::MotionLimits limitesTeach(float escalaPct) const;

    // Si el punto `k` se puede pasar de largo sin frenar. No se mezcla un
    // punto que tenga espera o cambio de bomba (esos hay que cumplirlos
    // donde estan), ni una esquina cerrada (ahi el tiron que se quiere
    // evitar lo produce la esquina misma, no el freno).
    bool teachMezclable(uint8_t k) const;

    // Redirige al punto `siguiente` sin frenar. false si no se pudo, y en
    // ese caso no se toco nada: se sigue por el camino de siempre.
    bool teachMezclar(uint8_t siguiente);

    bool entrarTeach();
    void salirTeach();
    void teachAbortar(const char *motivo);

    // Recorta el destino al volumen de trabajo declarado. Se aplica SIEMPRE,
    // venga de donde venga el pedido: la interfaz tambien recorta, pero el
    // que no puede equivocarse es este.
    void teachRecortar(float &x, float &y, float &z) const;

    // Recorta, resuelve la cinematica y lanza el movimiento con los limites
    // reducidos. false si el punto no tiene solucion (no se mueve nada).
    bool teachMover(float x, float y, float z, float escalaPct);
    bool teachMover(float x, float y, float z, const Motors::MotionLimits &limites);

    // ------------------------------------------------------------------
    //  Ir a una coordenada escrita ('JI')
    // ------------------------------------------------------------------
    // El operador escribe X, Y, Z y el brazo va. La diferencia con 'JM' (el
    // destino del jog) no es la velocidad sino EL CAMINO: si el brazo no
    // esta en home, primero sube a home y desde ahi baja al punto.
    //
    // POR QUE: quien escribe una coordenada no ve por donde va a pasar el
    // brazo, y la recta entre dos puntos bajos del volumen va raspando la
    // cinta todo el camino. Home es brazos horizontales, o sea lo mas
    // arriba que llega el robot: subir primero y bajar despues no puede
    // tocar nada de lo que hay abajo. Cuesta un tramo de mas y se gana no
    // tener que pensar cada vez si el camino esta libre.
    //
    // Va a maxima velocidad y aceleracion (FAST_LIMITS, los mismos del ciclo
    // normal) y no a las del modo teach: los dos tramos son rectas largas
    // entre puntos conocidos, que es exactamente lo que el ciclo de
    // clasificacion hace todo el dia.
    uint8_t teachIrEtapa = 0;   // 0 = nada, 1 = subiendo a home, 2 = al punto
    float   teachIrX = 0.0f, teachIrY = 0.0f, teachIrZ = 0.0f;

    // Tolerancia para dar por bueno que el brazo ya esta en home. Home son
    // los pasos (0,0,0) exactos, pero pedir exactitud significaria mandarlo
    // a home aunque este a un centesimo de grado.
    static const long TEACH_HOME_TOL_PASOS = 30; // ~1,1 grados

    bool teachEnHome() const;
    bool teachIr(float x, float y, float z);
    void updateTeachIr();

    // Hay un movimiento de teach en curso que no es el jog: una reproduccion
    // o un 'ir a'. Es lo que decide si un pedido nuevo se rechaza.
    bool teachOcupado() const { return teachReproduciendo || teachIrEtapa != 0; }

    bool procesarComandoTeach(const char *cmd);

    void teachInformar() const;

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
    //  Telemetria y parametros
    // ------------------------------------------------------------------
    // Robot es el unico que ve al mismo tiempo la maquina de estados, el
    // guard, los encoders y los finales de carrera, asi que es el que llena
    // las structs. El formato y los relojes viven en Telemetria.
    Telemetria telemetria;

    void emitirTelemetria(uint32_t ahora);
    void registrarParametros();

    // Empuja a los objetos que guardan copia propia de un parametro (hoy
    // solo el CollisionGuard) los valores que estan en la tabla. Se llama
    // cuando la generacion de la tabla cambia, sin averiguar cual cambio:
    // son cinco setters, sale mas barato aplicarlos todos que llevar
    // cuenta de cual fue.
    void sincronizarParametros();

    uint32_t generacionParams = 0;

    // Comandos 'P...' y 'V...'. Devuelven true si consumieron el comando.
    bool procesarComandoParametro(const char *cmd);
    bool procesarComandoTelemetria(const char *cmd);

    // Fija un parametro y contesta con la linea [P] set. Es el unico camino
    // por el que se cambia un parametro: los comandos historicos
    // ('U', 'T', 'K', 'L', 'Q') tambien pasan por aca, para que la tabla no
    // quede diciendo una cosa y el guard otra.
    void aplicarParametro(const char *nombre, float valor);

    // Copia local de los parametros que el guard guarda adentro. La tabla
    // necesita punteros a float estables y el guard los tiene privados con
    // setters, asi que la fuente de verdad de estos cinco es esta copia y
    // sincronizarParametros() se encarga de bajarlos.
    float pGuardUmbral    = GuardConfig::UMBRAL_DEG;
    float pGuardReposo    = GuardConfig::UMBRAL_REPOSO_DEG;
    float pGuardSalto     = GuardConfig::SALTO_PARADA_DEG;
    float pGuardSaltoPct  = GuardConfig::SALTO_PARADA_FRAC * 100.0f;
    float pGuardConfirma  = GuardConfig::CONFIRMACION_MS;
    float pGuardMargen    = GuardConfig::MARGEN_VELOCIDAD_MS;
    float pGuardRetardo   = GuardConfig::RETARDO_ENCODER_MS;

    // ------------------------------------------------------------------
    //  Contadores de produccion
    // ------------------------------------------------------------------
    // Viven en el firmware y no en la interfaz para que sobrevivan a que
    // se cierre la interfaz, que es lo primero que uno hace cuando algo
    // anda mal. Se reinician solo con el reinicio del ESP32.
    //
    //   detectadas   piezas que la vision informo y entraron en la cola
    //   depositadas  piezas efectivamente soltadas en su destino
    //   descartadas  piezas que se dejaron pasar por no llegar a tiempo
    //
    // Las que se dejan pasar en modo caja por no hacer falta NO cuentan
    // como descartadas: no son una perdida, son el modo funcionando.
    uint32_t piezasDetectadas  = 0;
    uint32_t piezasDepositadas = 0;
    uint32_t piezasDescartadas = 0;

    uint32_t porColorOk[3] = {0, 0, 0}; // R, G, B
    uint32_t porFormaOk[3] = {0, 0, 0}; // S, H, C

    void contarDepositada(const Piece &p);

    // Antiguedad de la pieza mas vieja de la cola, en ms (0 si no hay).
    uint32_t antiguedadCola() const;

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

    // Los angulos de homing se mudaron a Robot.cpp: ahora son ajustables
    // desde la tabla de parametros ('home_a1'..'home_a3', nivel servicio) y
    // la tabla necesita punteros a variables, no constantes de compilacion.

    static long angleToSteps(float angle)
    {
        return lround(angle * MICROPASOS / 360.0f);
    }

    // La vuelta: pasos -> grados. Es lo que informa la telemetria como
    // angulo comandado de cada eje, para poder dibujarlo contra el medido.
    static float stepsToAngle(long steps)
    {
        return (float)steps * 360.0f / (float)MICROPASOS;
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
