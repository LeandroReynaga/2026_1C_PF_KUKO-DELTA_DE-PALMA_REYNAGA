#include "Robot.h"
#include "Pinout.h"
#include "hardware/Pneumatics.h"
#include "hardware/Conveyor.h"
#include "hardware/Encoders.h"
#include "kinematics/DeltaKinematics.h"
#include "motion/ConveyorIntercept.h"

#include <string.h>
#include <ctype.h>
#include <stdlib.h>

Conveyor conveyor(CINTAPWM);
Pneumatics pneumatics;

// ============================================================
//  CINTA TRANSPORTADORA
//  v = pi * d * (N/60), con d = 2,4 cm y N = 60 rpm  ->  7,54 cm/s
//  Se toma como constante y conocida: toda la intercepcion depende de este
//  numero, asi que si se cambia la polea o las rpm hay que recalcularlo
//  aca (y recalibrar el PWM de Conveyor::begin(), que hoy arranca al 60%
//  sin relacion medida con las rpm reales).
// ============================================================
static float BELT_VELOCITY_CMS = 4.50f; //antes 7.54, 6.75, 7.2, 7.1 y 7.25

// Cuanto se le manda al driver de la cinta, en porcentaje del PWM. OJO:
// esto NO cambia solo a BELT_VELOCITY_CMS, que es la velocidad MEDIDA y es
// la que usa la planificacion. Tocar uno sin el otro hace que el robot le
// erre a todas las piezas, asi que despues de mover el PWM hay que volver a
// medir la cinta y actualizar el otro numero (la interfaz muestra los dos
// juntos y la vision mide el real, para poder compararlos).
static float CONVEYOR_PWM = 40.0f; // antes 60
static float DETECTION_LINE_X  = -23.0f; // donde la camara detecta las piezas

// Ancho util de la cinta (Y). Fuera de esto el dato de vision es erroneo.
static const float BELT_MIN_Y = -1.95f;
static const float BELT_MAX_Y = 12.05f;

// ============================================================
//  LATENCIA DE LA VISION
// ============================================================
// Entre el instante en que la pieza cruza FISICAMENTE la linea de
// deteccion y el instante en que el mensaje llega al ESP32 pasa un
// tiempo muerto: exposicion de la camara, buffer del driver, el
// fotograma que hay que esperar para ver el cruce, la deteccion en
// si y el envio por Serial. Durante todo eso la pieza SIGUE
// AVANZANDO sobre la cinta.
//
// Sin compensarlo, el robot cree que la pieza esta mas atras de
// donde realmente esta, apunta detras de ella, y la pieza le queda
// adelantada: aparece a la derecha de la ventosa.
//
// Se modela como TIEMPO y no como una distancia fija a proposito.
// El error en cm es latencia * velocidad de cinta, asi que
// expresarlo en segundos lo deja valido aunque se cambien las rpm
// de la cinta; una constante en cm habria que recalibrarla.
//
// PARA CALIBRARLO: medir cuanto le erra el gripper en X y dividir
// por la velocidad de la cinta.
//
//     latencia_s = error_cm / BELT_VELOCITY_CMS
//
//   pieza ADELANTADA (a la derecha de la ventosa) -> falta, subirlo
//   pieza ATRASADA   (a la izquierda)             -> sobra, bajarlo
//
// Valor actual: 0,580 s, ajustado contra el robot hasta que el gripper
// dejo de errarle en X (antes 0,199 calculado, un rato en 0, despues
// 0,2554 y 0,2695). Con la cinta a 7,25 cm/s compensa 4,21 cm de avance.
static float VISION_LATENCY_S = 0.580f;

// ============================================================
//  GEOMETRIA DE AGARRE (coordenadas de la PUNTA del gripper)
//  DeltaKinematics ya descuenta el offset de herramienta (0,0,-2.8).
// ============================================================
static float GRAB_Z      = -32.5f; // -32.3, -32.9 y -32.6 antes. cara superior de la pieza (1 cm de alto)
// Cuanto por DETRAS de la pieza empieza el tramo de bajada. Es la "carrera"
// que tiene el gripper para acelerar a favor de la cinta antes de tocarla.
//
// NO se elige solo: sale de ACC_AGARRE. La velocidad con la que el gripper
// toca la pieza es
//
//     vX = (|APPROACH_DX| + overshoot) * velocidad_de_fraccion
//
// y la velocidad de fraccion la fija cuanto dura el tramo, que depende de la
// aceleracion. Subir la aceleracion sin alargar este numero deja al gripper
// tocando MAS LENTO que la cinta; alargarlo sin subir la aceleracion, mas
// rapido. Los pares que igualan los 7,1 cm/s de la cinta, con
// APPROACH_DZ = 2,7, y lo que cuesta cada uno:
//
//     apr_dx    acc_agarre    tramo 2   toque vertical   ciclo
//     -4,00 cm      80.485     154 ms       4,8 cm/s     813 ms
//     -3,28 cm     110.248     126 ms       5,8 cm/s     782 ms
//
// OJO: la tabla de arriba se calculo con la cinta a 7,1 cm/s, y hoy la
// cinta va a 4,50. El valor actual (-2,25 cm con acc_agarre en 180.000)
// salio de ajustar contra el robot, no de esta cuenta, y con la cinta mas
// lenta el gripper toca a 6,67 cm/s, o sea un 48 % mas rapido que ella (la
// alcanza por atras). Rehecho para 4,50 cm/s, el par que sincroniza seria
// apr_dx = -1,5, pero eso NO se toco: lo que anda en el robot manda sobre
// el modelo, y con la ventosa comprimiendo 0,25 mm el encuentro tolera
// bastante desvio. Si alguna vez se vuelve a tocar la velocidad de cinta,
// este es el numero a revisar.
//     -3,00 cm     127.232     116 ms       6,4 cm/s     770 ms
//     -2,71 cm     150.021     104 ms       7,1 cm/s     757 ms
//     -2,00 cm     248.019      77 ms       9,6 cm/s     726 ms
//     -1,50 cm     405.241      58 ms      12,8 cm/s     704 ms
//     -1,00 cm     930.584      39 ms      19,2 cm/s     682 ms   <- ver abajo
//
// Acortar este numero SI acelera el ciclo, pero el precio esta en la
// columna del medio y no es chico. Con APPROACH_DZ fijo en 2,7 la relacion
// entre las dos componentes del toque es pura geometria:
//
//     vZ / vX = (APPROACH_DZ + PRESS_DZ) / (|APPROACH_DX| + overshoot)
//
// o sea que cuanto mas corto el tramo en X, mas VERTICAL es la entrada, y
// para llegar igual a 7,1 cm/s en X hay que ir tan rapido que la componente
// de bajada se vuelve un martillazo. A 1 cm son 19 cm/s contra la cara de
// la pieza.
//
// Y ese ultimo caso ademas no sirve por otro motivo: 930.000 deja el perfil
// FUERA del regimen triangular (el recorrido pasa a ser mayor que v^2/a), o
// sea que se activa el tramo de crucero de Stepper::computeNextInterval(),
// que es justo el que Motors.h avisa que tenia dos errores en el frenado.
//
// Si se cambia la velocidad de la cinta o APPROACH_DZ, hay que rehacer la
// cuenta entera. Que el toque sea a la velocidad de la cinta es lo que evita
// que la pieza se deslice por debajo de la ventosa mientras la toca.
static float APPROACH_DX = -2.25f; // antes -2.0 y -3.28
static float APPROACH_DZ = 2.7f;   // 0.4 antes, despues 0.7. 4 mm por arriba

// Cuanto baja el tramo 2 por DEBAJO de la cara de la pieza. Es lo que hace
// que el contacto ocurra a mitad del movimiento y no al final, o sea con el
// gripper todavia andando a la velocidad de la cinta en vez de frenado
// (ver la explicacion completa en ConveyorIntercept.h). De paso comprime la
// ventosa, que ayuda al sellado.
//
// Es EL parametro para ajustar la suavidad del encuentro:
//   0,25 mm -> toca a 7,54 cm/s = velocidad de cinta (velocity-matched)
//   menos    -> toca mas lento que la cinta (la pieza se le adelanta)
//   mas      -> toca mas rapido que la cinta (el gripper la alcanza)
static float PRESS_DZ = 0.025f;

static float LIFT_DZ = 3.0f;       // despegue de la pieza de la cinta (antes 2)

// Area donde es seguro AGARRAR, validada a mano sobre el robot real
// (inspeccion visual de las rotulas en todas las posiciones).
//
// Es asimetrica a proposito. Lo que importa no es este intervalo sino el
// recorrido que termina haciendo la punta, que no es el mismo: el punto de
// aproximacion cae 2 cm por detras del agarre (APPROACH_DX), asi que
// agarrando en el borde de entrada (-10) la punta llega hasta X = -12. Con
// el techo de agarre en +12, el barrido queda simetrico en +-12.
//
// El techo estaba en +10, y eso dejaba escapar piezas. La cinta corre hacia
// +X, asi que una pieza que llega con el brazo ocupado se pierde apenas
// pasa el techo; los 2 cm que se suman son 2 / BELT_VELOCITY_CMS = 0,44 s
// mas de margen para engancharla.
//
// Verificado contra la cinematica ANTES de subirlo, en los tres puntos que
// valida ConveyorIntercept::solve() y sobre todo el ancho util de la cinta:
//
//     agarre en X    Y agarrable      margen al limite articular
//        +10,0      toda la cinta            5,1 deg
//        +12,0      toda la cinta            2,9 deg   <- este
//        +14,0      toda la cinta            0,2 deg   <- el limite real
//
// Los 2,9 grados de +12 son exactamente los mismos que el brazo ya tiene en
// X = -12, o sea en cada agarre del borde de entrada: no se le esta
// pidiendo nada que no venga haciendo. Y si alguna vez no alcanzara, el
// modo de falla es benigno -- ConveyorIntercept valida los tres puntos con
// la cinematica y marca la pieza inalcanzable, que es dejarla pasar.
static const float WORK_AREA_MIN_X = -10.0f;
static const float WORK_AREA_MAX_X = 12.0f;

// ============================================================
//  TACHOS (posicion de la punta del gripper para soltar la pieza)
//  Modo COLOR: 1 rojo, 2 verde, 3 azul
//  Modo FORMA: 1 cuadrado, 2 hexagono, 3 circulo
// ============================================================
static float BIN_X[3] = {-11.3f, 0.0f, 11.3f}; // antes +-12.0
static float BIN_Y    = -8.5f;  // antes -9.55
static float BIN_Z    = -25.5f; //antes -29.3 y -26.5

// ============================================================
//  CAJA DEL MODO BOX (SORT_BOX)
// ============================================================
//  Sobre los tachos se apoya una tapa con la forma de una caja de 6, y el
//  robot la llena con circulos ordenados por color. Vista desde arriba:
//
//                        cinta (Y positivo)
//          +---------------------------------+
//          |    1        2        3          |   Y = -6,8   (fila de adelante)
//          |    4        5        6          |   Y = -12,8  (fila del fondo)
//          +---------------------------------+
//              X=-6     X=0     X=+6
//
//  El piso de la caja esta a la misma altura que la cinta, asi que el Z de
//  soltado es el mismo que el de agarre: la punta baja hasta BOX_Z y la
//  pieza, que cuelga 1 cm por debajo, queda apoyada en el piso.
// ============================================================
static const float BOX_X[6] = {-6.0f, 0.0f, 6.0f, -6.0f, 0.0f, 6.0f};
static const float BOX_Y[6] = {-6.8f, -6.8f, -6.8f, -12.8f, -12.8f, -12.8f};

// Corrimiento de TODA la caja. La grilla de arriba es la geometria de la
// caja (6 cm entre celdas, y eso no cambia), pero la caja apoyada sobre la
// mesa nunca queda dos veces en el mismo lugar. Estos dos numeros son para
// centrarla mirando el robot, en vez de reflashear por medio centimetro.
static float BOX_DX = 0.0f;
static float BOX_DY = 0.0f;

// EL parametro de ajuste fino del modo: si las piezas quedan colgadas o
// la ventosa aplasta la caja, se corrige aca (y en ningun otro lado).
static float BOX_Z = -32.1f; // antes -32.6

// Altura a la que se PARTE la bajada sobre la celda: de aca para abajo se
// va con aceleracion minima.
//
// El descenso total desde la altura de cruce es BOX_TRANSIT_DZ (6 cm), y
// este numero decide como se reparte:
//
//     BOX_TRANSIT_DZ - BOX_APPROACH_DZ   con ACC_RAPIDA (tramo BOX_APPROACH)
//     BOX_APPROACH_DZ                    con ACC_CAJA   (tramo BOX_DESCEND)
//
// Con 1 cm queda 5 cm rapido + 1 cm suave. Antes era 3 + 3, y ese reparto
// es lo que hacia lento el ciclo: el tramo suave es el unico que no se
// puede apurar --es el que APOYA la pieza en el piso de la caja en vez de
// dejarla caer-- asi que todo centimetro que se le saque y se le de al
// tramo rapido sale gratis en precision y se nota en el tiempo.
//
// Frena mas cerca de la caja, entonces. Si alguna vez la pieza llega a
// golpear el piso, este es el numero a subir (y es lo primero a mirar, no
// BOX_Z).
//
// El minimo del parametro 'box_apr' es 0,2 cm y no 0. Medido sobre el eje
// que mas se mueve, el tramo suave son 95 micropasos por centimetro:
//
//     1,0 cm -> 95 micropasos
//     0,3 cm -> 28
//     0,2 cm -> 19        <- el minimo
//     0,1 cm ->  9        <- ya no es una rampa, es un salto
//
// Por debajo de 0,2 cm el tramo deja de ser una desaceleracion y pasa a ser
// un pisoton de un puñado de pasos, que es justo lo que este tramo existe
// para evitar. Con 0 los dos destinos serian el mismo punto y la pieza
// terminaria apoyada al final de un movimiento a ACC_RAPIDA.
static float BOX_APPROACH_DZ = 1.0f;

// Altura de CRUCE sobre la caja: a la que se entra, a la que se sale y a la
// que se vuela de una celda a otra.
//
// No es un numero elegido por comodidad. moveSynchronized escala velocidad
// y aceleracion de cada eje en proporcion a su recorrido, asi que los tres
// motores siguen el mismo perfil normalizado: el movimiento es una RECTA EN
// EL ESPACIO DE LAS ARTICULACIONES y en cartesiano se curva hacia abajo.
// Yendo desde el extremo lejano de la cinta hasta la celda 4 con los 3 cm
// de aproximacion como unica altura, esa curva hunde la punta hasta Z=-33,0
// con la pieza colgando en -34,0: por debajo del piso de la caja (-33,6).
// El brazo araria la caja antes de llegar a la celda.
//
// Con 6 cm de cruce el peor caso deja 1,4 cm de luz entre la pieza que
// lleva y la cara superior de las que ya estan puestas, y la salida hacia
// la cinta deja 1,4 cm entre la punta y esas mismas piezas.
//
// Si se cambia la geometria de la caja (celdas mas al fondo, piso mas bajo)
// hay que rehacer esa cuenta: es lo que separa "pasa por encima" de "choca".
static float BOX_TRANSIT_DZ = 6.0f;

// Disposicion por defecto de la caja: que color va en cada celda.
//
//      1 azul     2 rojo     3 verde
//      4 rojo     5 azul     6 verde
//
// Es provisoria hasta que la interfaz deje elegir los 6 colores a mano; el
// comando 'X' ya permite cambiarla sin recompilar.
static const char BOX_LAYOUT_DEFECTO[6] = {'B', 'R', 'G',
                                           'R', 'B', 'G'};

// Tope de piezas del mismo color en una caja de 6.
static const uint8_t BOX_MAX_POR_COLOR = 3;

// Cuando el color que llega sirve para mas de una celda, se elige SIEMPRE
// la de numero mas grande (un rojo que puede ir en la 1 o en la 3 va a la
// 3; un verde que puede ir en la 4, 5 o 6 va a la 6). Ver
// piezaSirveParaCaja(), que recorre las celdas de la 6 a la 1.
//
// De paso, esto cumple solo la condicion que importa para no golpear nada:
// como las celdas 4-6 son la fila del fondo, quedan llenas antes que la
// fila de adelante (1-3). El brazo entra a la caja desde la cinta, o sea
// desde Y positivo, asi que para llegar al fondo cruza por encima de la
// fila de adelante; llenandola ultima, nunca sobrevuela una pieza puesta.
//
// Esta eleccion NO decide a que pieza se va a buscar: eso lo fija el orden
// en que las detecto la vision (la cola es FIFO, ver iniciarSiguientePieza).

// Cuanto se espera con la bomba apagada antes de sacar el brazo de la caja.
// Es un tiempo aparte del RELEASE_DETACH_MS del tacho porque el caso es
// distinto: en el tacho la pieza se suelta en el aire y se cae sola, pero en
// la caja queda APOYADA con la ventosa todavia tocandola, asi que tiene
// menos ayuda para despegarse. Ajustado contra el robot: con 0 alguna
// pieza salia pegada al gripper. Si vuelve a pasar, este es el numero a
// subir.
static float BOX_RELEASE_DETACH_MS = 40; // antes 0

// Ventana para confirmar un cambio de modo que cruza el limite de la tapa.
static float CONFIRMACION_TAPA_MS = 10000;

// ============================================================
//  ANGULOS DE HOMING
// ============================================================
// Angulo real de cada brazo cuando su final de carrera esta pisado. Es la
// referencia de TODO: con estos numeros se fija la posicion en pasos al
// terminar el homing y se calibra el cero de los encoders.
//
// Estaban en Robot.h como constantes de compilacion. Se mudaron aca porque
// ahora son ajustables desde la tabla de parametros (nivel servicio) y la
// tabla trabaja con punteros a float.
//
// Se miden a mano contra el robot: son de los pocos valores que no se
// pueden calcular, solo verificar.
static float HOME_ANGLE_M1 = -45.1f;
static float HOME_ANGLE_M2 = -44.3f;
static float HOME_ANGLE_M3 = -44.5f;

// ============================================================
//  TIEMPOS (todos no bloqueantes, con millis())
// ============================================================
static float HOMING_SETTLE_WAIT_MS = 2500; // ventana de promediado de encoders
static float BIN_SETTLE_MS         = 7;  // quieto sobre el tacho antes de soltar. Antes 200 y 5

// Cuanto se espera con la bomba apagada para que la pieza se despegue del
// gripper. Con la electrovalvula montada (misma senal que la bomba) la linea
// se ventea al apagar y la pieza cae en el acto: lo que se espera aca es que
// entre el aire, no que el vacio se vaya por fuga -- que era lo que hacia
// falta esperar cuando la linea quedaba cerrada. Si alguna pieza sale pegada
// al gripper, este es el numero a subir.
static float RELEASE_DETACH_MS = 80;

// Cuanto antes del instante de bajada se prende la bomba de vacio. Ver el
// comentario largo en updatePickApproach(): tiene que alcanzar para que el
// vacio se forme, y de mas la deja soplando al aire esperando la pieza.
//
// En 0 desde el ajuste sobre el robot: con esta bomba el vacio se forma
// bastante mas rapido que los 300 ms que se le daban, asi que el adelanto
// era todo tiempo soplando al aire. Si alguna pieza no engancha en el
// primer intento, esto es lo primero a subir.
static float PUMP_LEAD_MS = 0; // antes 300

// Margen de atraso tolerable al llegar al punto de aproximacion antes de
// dar por perdido el instante de encuentro y replanificar.
static float PICK_LATE_TOLERANCE_MS = 30;
static float MAX_REPLAN_ATTEMPTS    = 2;

// ============================================================
//  RECUPERACION DE COLISIONES
// ============================================================
// Cuanto se queda quieto el robot despues de detectar una colision, antes
// de arrancar la recalibracion. No es para que "se acomode" nada: es para
// que quien este mirando alcance a ver que paso y a sacar la mano o el
// obstaculo antes de que el brazo vuelva a moverse solo.
static float COLLISION_PAUSE_MS = 3000;

// Colisiones seguidas (sin completar ninguna pieza en el medio) despues de
// las cuales se deja de reintentar. Si el brazo choca contra lo mismo tres
// veces, rehomear una cuarta no lo va a arreglar: lo unico que se logra es
// seguir golpeando el robot.
static float MAX_COLISIONES_SEGUIDAS = 3;

// Cuanto se suspende la supervision al conmutar la bomba de vacio.
//
// La bomba es una carga fuerte que arranca justo cuando el brazo termina el
// tramo rapido y todavia no empezo el lento -- o sea, exactamente donde
// aparecian los falsos positivos. La salida del AS5600 es ratiometrica a su
// VCC y el ADC del ESP32 mide contra una referencia interna, asi que
// cualquier hundida del riel al arrancar la bomba corre las tres lecturas
// de golpe sin que se haya movido nada.
//
// Ademas, el tramo de bajada TOCA la pieza a proposito (y la comprime, ver
// PRESS_DZ): ahi hay una perturbacion mecanica real y esperada, que no es
// una colision. Esta ventana cubre las dos cosas.
static float BLANQUEO_NEUMATICA_MS = 300;

// Cada cuanto se vuelca la linea [GUARD] con los numeros de la supervision.
//
// APAGADO DE FABRICA (0). Existia de cuando la unica salida del robot era el
// monitor serie y no habia forma de pedirle nada con la vision teniendo
// tomado el puerto. Hoy la interfaz recibe los mismos numeros en [T] (error
// contra umbral, 10 Hz) y en [H] (ganancia, atraso, picos, fuga, por eje), y
// una linea sola cada 15 s no aportaba nada salvo tapar la consola.
//
// Se deja el parametro ('diag_ms') para poder encenderlo cuando se esta
// calibrando el guard a mano y se quiere el volcado en el log.
static float DIAGNOSTICO_PERIODICO_MS = 0;

// ============================================================
//  MODO TEACH (aprendizaje)
// ============================================================
// VOLUMEN DE TRABAJO DEL JOG. Es un cajon mas chico que el alcance real del
// robot, elegido para que el operador no pueda llevar el brazo contra nada
// mientras lo maneja a mano:
//
//   X   el ancho util validado sobre el robot, que es donde se sabe que
//       llega con las rotulas en buen angulo.
//   Y   desde el centro de los tachos (lo mas cerca del operador a donde
//       tiene sentido ir) hasta el borde lejano de la cinta.
//   Z   desde GRAB_Z -- la cara superior de una pieza apoyada, o sea el
//       punto mas bajo al que se baja para agarrar -- hasta TEACH_ZUP por
//       encima. El piso NO es un numero aparte: cuelga de GRAB_Z para que
//       recalibrar la altura de agarre mueva tambien el limite del jog, en
//       vez de dejar los dos diciendo cosas distintas.
//
// Los cinco son parametros ('t_xmin'..'t_zup', nivel servicio): la cinta y
// los tachos se mueven, y con ellos el volumen seguro.
static float TEACH_XMIN = -12.0f;
static float TEACH_XMAX =  12.0f;
static float TEACH_YMIN = -9.55f;   // centro de los tachos (= BIN_Y)
static float TEACH_YMAX =  11.05f;  // un centimetro antes del borde lejano de la cinta
static float TEACH_ZUP  =  6.0f;    // cuanto se puede despegar por encima de GRAB_Z

// Velocidad del jog manual, en cm/s de la punta. Es un PEDIDO, no una
// garantia: cada tramo se lanza solo cuando el anterior termino, asi que si
// el brazo no llega a hacerlo en el tick, el jog se frena solo en vez de
// acumular atraso contra un destino que se le escapa.
static float TEACH_JOG_CMS = 5.0f;

// Porcentaje de VEL_MAX / TEACH_ACEL con el que se mueve el jog. Bajo a
// proposito: con el brazo manejado a mano y a centimetros de la cinta, lo
// que importa es poder soltar la tecla a tiempo, no llegar rapido.
static float TEACH_JOG_PCT = 15.0f;

// Aceleracion del modo teach, en pasos/s2. NO se usa ACC_RAPIDA (97.000):
// una trayectoria ensenada a mano son decenas de tramos cortos, y con esa
// aceleracion cada cambio de velocidad es un tiron. Este es el numero a
// bajar si el brazo vibra al reproducir, y el primero a probar antes de
// tocar cualquier otra cosa.
static float TEACH_ACEL = 40000.0f;

// ============================================================
//  MOVIMIENTO LINEAL (movL)
// ============================================================
// Largo de los tramos en los que se parte una recta, y velocidad de la
// PUNTA mientras la recorre. Los dos numeros salen medidos (ver el header
// de src/motion/Trajectory.h) y estan atados entre si:
//
//   - el paso decide cuanto se aparta de la recta: 1 cm son 0,05 mm,
//     2 cm son 0,19 mm, 5 cm son 1,06 mm;
//   - pero un tramo mas corto que la distancia de frenado no se puede
//     encadenar, asi que la velocidad le pone un piso al paso. A 20 cm/s
//     ese piso es de ~0,6 cm; a 50 cm/s ya es de 7 cm y movL deja de
//     tener sentido.
//
// Los valores de fabrica salieron de probar en el robot: 40 cm/s se ve
// fluido, y el paso se deja en el minimo a proposito -- pedir menos de lo
// que la frenada permite no rompe nada, el firmware lo sube solo hasta ahi.
// Asi el tramo siempre queda en el mas corto POSIBLE para la velocidad y la
// aceleracion que haya, que es lo mas recto que este esquema puede dar, sin
// tener que recalcularlo a mano cada vez que se toca uno de los otros dos.
static float MOVL_PASO_CM = 0.01f;
static float MOVL_VEL_CMS = 40.0f;

// Aceleracion propia del movimiento recto, aparte de la del resto de teach.
//
// Es EL numero para que una recta salga derecha a velocidad alta, y la
// razon esta en la formula de la distancia de frenado: v^2/(2a). El tramo
// no puede ser mas corto que esa distancia, asi que a velocidad fija, MAS
// ACELERACION ES MENOS TRAMO, o sea mas puntos donde se corrige el rumbo.
//
// A 40 cm/s: con 40.000 el tramo minimo son 5,4 cm (se aparta ~1,3 mm de la
// recta); con 97.000 baja a 2,2 cm (~0,23 mm). Por eso arranca en 97.000,
// que es la misma aceleracion con la que el robot mueve cada pieza en el
// ciclo normal -- o sea, ya probada en este brazo.
static float MOVL_ACEL = 97000.0f;

// Radio de mezcla de las esquinas, en cm. 0 = apagado (se frena en cada
// punto, que es como andaba antes).
//
// Al llegar a esta distancia de un punto intermedio, el brazo redirige al
// punto siguiente SIN frenar (ver Stepper::redirigir). O sea que pasa CERCA
// del punto y no exactamente por el, y a cambio no se detiene: es la unica
// forma de que una sucesion de tramos se sienta como un movimiento y no como
// veinte movimientos pegados.
//
// Cuanto mas grande, mas fluido y mas se aparta de lo ensenado. Es lo que
// hace que valga la pena la verificacion por etapas: a 15 % se ve el camino
// real, y recien despues se sube.
static float TEACH_MEZCLA_CM = 0.0f; // antes 0,5: sin mezcla de esquinas

// Hasta que angulo de esquina se mezcla. Una esquina cerrada obliga a los
// ejes a cambiar de velocidad de golpe -- que es exactamente el tiron que se
// quiere evitar -- asi que a partir de aca se frena y se arranca de nuevo,
// que ademas es lo unico que puede hacer un eje que tiene que invertir el
// sentido.
static float TEACH_ESQUINA_DEG = 30.0f;

// Tiempo maximo que puede tardar el homing en encontrar los 3 finales de
// carrera. Si se pasa, es que un eje no llega (trabado contra algo, o el
// obstaculo que causo la colision sigue ahi): se corta todo, se para la
// cinta y se espera intervencion manual. Sin esto, un brazo trabado deja
// al robot empujando contra el obstaculo para siempre.
static float HOMING_TIMEOUT_MS = 20000;

namespace {

// Recorta espacios/tabs al principio y al final, in situ. Los mensajes los
// puede tipear una persona a mano en el monitor serie mientras todavia no
// esta el programa de Python, asi que "3.5, B, S" tiene que valer igual que
// "3.5,B,S".
char *recortar(char *s)
{
    while (*s == ' ' || *s == '\t') s++;

    char *fin = s + strlen(s);
    while (fin > s && (fin[-1] == ' ' || fin[-1] == '\t')) fin--;
    *fin = '\0';

    return s;
}

// Parte "a,b,c" en floats. Devuelve cuantos leyo, o 0 si algo no cierra.
//
// Se exige que cada numero se consuma ENTERO, igual que en el resto del
// parser y por el mismo motivo: con atof() un "hola" pasa como 0.0, y un 0
// es una coordenada perfectamente valida dentro del volumen de trabajo.
uint8_t leerFloats(const char *s, float *destino, uint8_t maximo)
{
    uint8_t n = 0;

    while (*s != '\0' && n < maximo)
    {
        char       *fin = NULL;
        const float v   = strtof(s, &fin);

        if (fin == s)
        {
            return 0;
        }

        destino[n++] = v;
        s = fin;

        if (*s == ',')
        {
            s++;
            continue;
        }

        break;
    }

    return (*s == '\0') ? n : 0;
}

} // namespace

Robot::Robot() :

motor1(PUL1, DIR1, ENA, 0),
motor2(PUL2, DIR2, ENA, 1),
motor3(PUL3, DIR3, ENA, 2)

{
    state = IDLE;

    axis1Homed = false;
    axis2Homed = false;
    axis3Homed = false;

    memcpy(boxLayout, BOX_LAYOUT_DEFECTO, BOX_CELLS);

    for (uint8_t i = 0; i < BOX_CELLS; i++)
    {
        boxFilled[i] = false;
    }
}

void Robot::begin()
{
    // El movimiento lineal necesita los tres motores una sola vez; despues
    // se maneja solo desde el loop.
    lineal.begin(motor1, motor2, motor3);

    pneumatics.begin();

    motor1.begin();
    motor2.begin();
    motor3.begin();

    motor1.setSpeed(2000);
    motor2.setSpeed(2000);
    motor3.setSpeed(2000);

    endstops.begin();

    guard.begin();
    guard.setObservar(!supervisionHabilitada);
    fallos.begin();

    // Parametros: primero se registran con los valores compilados (que
    // quedan como los "de fabrica") y recien despues se carga lo guardado
    // en la NVS. El orden importa: al reves, lo guardado se tomaria por
    // valor de fabrica y 'P0' no volveria a ningun lado conocido.
    params.begin();
    registrarParametros();
    params.cargarGuardados();
    sincronizarParametros();
    generacionParams = params.generacion();

    // Una disposicion imposible de completar (mas de 3 piezas de un mismo
    // color) se detecta al arrancar y bloquea la entrada al modo: mejor eso
    // que un robot esperando para siempre una pieza que no va a poder ubicar.
    boxLayoutValido = layoutValido(boxLayout);

    Serial.println();
    Serial.println("=== KUKO DELTA CARBON ===");
    Serial.println("Comandos por Serial (uno por linea):");
    Serial.println("  Y,color,forma   pieza detectada. Ej: 3.5,B,S");
    Serial.println("                  color = R/G/B, forma = S/H/C");
    Serial.println("  C               clasificar por COLOR");
    Serial.println("  F               clasificar por FORMA");
    Serial.println("  A               modo BOX (llenar la caja de 6)");
    Serial.println("  N               caja nueva (marca las 6 celdas como vacias)");
    Serial.println("  X<6 colores>    disposicion de la caja, celda 1 a 6. Ej: XBRGRBG");
    Serial.println("  R               parada de emergencia / reinicio");
    Serial.println("  D               historial de fallos");
    Serial.println("  S               estado de la supervision por encoders");
    Serial.println("  G               prender/apagar la supervision");
    Serial.println("  M               traza en vivo del encoder vs los pasos");
    Serial.println("  U<grados>       umbral fijo de colision. Ej: U12.5");
    Serial.println("  T<ms>           tiempo de confirmacion. Ej: T80");
    Serial.println("  K<ms>           margen por velocidad. Ej: K80");
    Serial.println("  L<ms>           atraso del encoder a compensar. Ej: L70");
    Serial.println("  Q<grados>       umbral con el robot quieto en home. Ej: Q5");
    Serial.println("  J1 / J0 / J?    modo TEACH: entrar / salir / estado");
    Serial.println("  V1 / V0 / V?    telemetria: encender / apagar / una foto");
    Serial.println("  P?              listar parametros ajustables");
    Serial.println("  P<nombre>=<val> fijar un parametro. Ej: Pvis_lat=0.18");
    Serial.println("  P* / P0         guardar en la NVS / volver a fabrica");

    telemetria.anunciar((uint8_t)(TEACH + 1), params.cantidad());

    if (!boxLayoutValido)
    {
        Serial.print("[CAJA] disposicion invalida (maximo ");
        Serial.print(BOX_MAX_POR_COLOR);
        Serial.println(" piezas por color): el modo BOX queda bloqueado");
    }
}

void Robot::update()
{
    procesarSerial();

    // Supervision de colisiones: corre ANTES de la maquina de estados, en
    // cada vuelta del loop, sin importar en que estado este el robot. Si
    // detecta una colision cambia el estado a COLLISION_STOP y el switch de
    // abajo ya entra por ahi.
    supervisarColision();

    vencerConfirmacion();

    if (trazaActiva && (uint32_t)(millis() - ultimaTraza_ms) >= TRAZA_INTERVALO_MS)
    {
        ultimaTraza_ms = millis();
        imprimirTraza();
    }

    if (DIAGNOSTICO_PERIODICO_MS > 0 &&
        state != IDLE && state != ERROR &&
        (uint32_t)(millis() - ultimoDiagnostico_ms) >= (uint32_t)DIAGNOSTICO_PERIODICO_MS)
    {
        ultimoDiagnostico_ms = millis();
        imprimirDiagnosticoCorto();
    }

    // Un parametro que cambio (por 'P', por 'U'/'T'/'K'/'L'/'Q', o al
    // cargar la NVS) puede tener que bajarse a un objeto que guarda copia.
    if (params.generacion() != generacionParams)
    {
        generacionParams = params.generacion();
        sincronizarParametros();
    }

    emitirTelemetria(millis());

    switch (state)
    {
        case HOMING:         updateHoming();        break;
        case WAIT_PIECE:     updateWaitPiece();     break;
        case GO_HOME_IDLE:   updateGoHomeIdle();    break;
        case PICK_APPROACH:  updatePickApproach();  break;
        case PICK_DESCEND:   updatePickDescend();   break;
        case PICK_LIFT:      updatePickLift();      break;
        case GO_BIN:         updateGoBin();         break;
        case BIN_SETTLE:     updateBinSettle();     break;
        case RELEASE_WAIT:   updateReleaseWait();   break;
        case BOX_TRANSIT:    updateBoxTransit();    break;
        case BOX_APPROACH:   updateBoxApproach();   break;
        case BOX_DESCEND:    updateBoxDescend();    break;
        case BOX_LIFT:       updateBoxLift();       break;
        case COLLISION_STOP: updateCollisionStop(); break;
        case TEACH:          updateTeach();         break;

        default:                                    break;
    }

    motor1.update();
    motor2.update();
    motor3.update();
}

// ============================================================
//  CONSOLA SERIE
// ============================================================

void Robot::procesarSerial()
{
    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r')
        {
            if (cmdOverflow)
            {
                Serial.println("[SERIAL] linea demasiado larga, descartada");
                cmdOverflow = false;
            }
            else if (cmdLen > 0)
            {
                cmdBuffer[cmdLen] = '\0';
                procesarComando(cmdBuffer, cmdLen);
            }
            cmdLen = 0;
        }
        else if (cmdOverflow)
        {
            // Se descarta hasta el fin de linea: si se reiniciara el buffer
            // aca, la COLA de una linea larga terminaria ejecutandose como
            // si fuera un comando nuevo y valido.
        }
        else if (cmdLen < sizeof(cmdBuffer) - 1)
        {
            cmdBuffer[cmdLen++] = c;
        }
        else
        {
            cmdOverflow = true;
            cmdLen = 0;
        }
    }
}

void Robot::procesarComando(char *cmd, uint8_t len)
{
    (void)len; // se recalcula despues de recortar espacios

    cmd = recortar(cmd);
    const size_t n = strlen(cmd);

    if (n == 0)
    {
        return; // linea vacia (o solo espacios): se ignora sin ruido
    }

    // --- Comandos de un solo caracter: modo de clasificacion y emergencia ---
    // Un mensaje de pieza SIEMPRE tiene 2 comas, asi que no hay ambiguedad
    // con 'C' (modo color) ni con 'R' (reset), aunque esas mismas letras se
    // usen como forma/color adentro de un mensaje de pieza.
    if (n == 1)
    {
        const char c = toupper(cmd[0]);

        if (c == 'R')
        {
            if (state == ERROR)
            {
                Serial.println("[RESET] Rehomeando...");
                startHoming();
            }
            else
            {
                emergencyStop();
            }
            return;
        }

        if (c == 'D')
        {
            fallos.imprimirHistorial();
            return;
        }

        if (c == 'S')
        {
            imprimirEstadoSupervision();
            return;
        }

        if (c == 'M')
        {
            trazaActiva = !trazaActiva;

            if (trazaActiva)
            {
                Serial.println("[TRAZA] on. columnas por eje: raw (0-4095), enc y cmd en grados");
                Serial.println("[TRAZA] desde el homing, err = enc - cmd (el que mira el guard)");
                ultimaTraza_ms = 0;
            }
            else
            {
                Serial.println("[TRAZA] off");
            }
            return;
        }

        if (c == 'G')
        {
            supervisionHabilitada = !supervisionHabilitada;

            // El guard NO se desarma: sigue midiendo, marcando picos y
            // avisando por Serial lo que habria hecho. Lo unico que cambia
            // es si eso frena o no el robot. Asi se puede seguir
            // trabajando y calibrando al mismo tiempo, sin quedarse sin
            // datos justo cuando hacen falta.
            guard.setObservar(!supervisionHabilitada);

            if (!supervisionHabilitada)
            {
                Serial.println("[GUARD] paradas APAGADAS: sigue midiendo y avisando, pero no frena");
            }
            else
            {
                Serial.println("[GUARD] paradas ACTIVAS");
            }
            return;
        }

        if (c == 'C' || c == 'F' || c == 'A')
        {
            const SortMode nuevo = (c == 'C') ? SORT_BY_COLOR :
                                   (c == 'F') ? SORT_BY_SHAPE :
                                                SORT_BOX;
            pedirModo(nuevo, c);
            return;
        }

        if (c == 'N')
        {
            // Caja nueva: se pide cuando la caja llena ya se retiro y se
            // puso otra vacia. Es lo unico que borra el mapa de celdas.
            reiniciarCaja(true);
            return;
        }

        Serial.print("[SERIAL] comando invalido: '");
        Serial.print(cmd);
        Serial.println("'. Validos: 'C', 'F', 'A', 'N', 'R', 'D', 'S', 'G', 'CAL0'/'CAL1' o 'Y,color,forma'");
        return;
    }

    // --- Telemetria ('V1', 'V0', 'V?') y parametros ('P?', 'P*', 'P0',
    //     'Pnombre=valor') ---
    // Van antes que todo lo demas de varios caracteres porque son los
    // unicos que puede mandar la interfaz sin intervencion de una persona:
    // conviene que ni siquiera pasen por el resto del parser.
    if (procesarComandoTelemetria(cmd) || procesarComandoParametro(cmd))
    {
        return;
    }

    // --- Modo teach ('J...') ---
    // Tiene que ir ANTES del parser de piezas: 'JM-3.20,4.50,-30.10' lleva
    // dos comas y ese parser toma cualquier cosa con dos comas por un
    // mensaje de la vision. Ninguna pieza empieza con letra, asi que
    // consumir todo lo que arranca con 'J' no le saca nada a nadie.
    if (procesarComandoTeach(cmd))
    {
        return;
    }

    // --- Modo calibracion de la vision ('CAL1', 'CAL0', 'CAL?') ---
    if (procesarComandoCalibracion(cmd))
    {
        return;
    }

    // --- Disposicion de la caja: 'X' + los 6 colores, de la celda 1 a la 6 ---
    // Es el reemplazo provisorio de la interfaz que todavia no existe: deja
    // elegir que color va en cada celda sin recompilar. Ej: XBRGRBG.
    if (toupper(cmd[0]) == 'X' && strchr(cmd, ',') == NULL)
    {
        if (n != BOX_CELLS + 1)
        {
            Serial.print("[CAJA] se esperan ");
            Serial.print(BOX_CELLS);
            Serial.println(" colores (celda 1 a 6). Ej: XBRGRBG");
            return;
        }

        char nuevo[BOX_CELLS];

        for (uint8_t i = 0; i < BOX_CELLS; i++)
        {
            nuevo[i] = toupper(cmd[i + 1]);
        }

        if (!layoutValido(nuevo))
        {
            Serial.print("[CAJA] disposicion invalida: colores R/G/B y como maximo ");
            Serial.print(BOX_MAX_POR_COLOR);
            Serial.println(" de cada uno");
            return;
        }

        // Cambiar la disposicion con una pieza en vuelo la mandaria a una
        // celda que ya no significa lo mismo.
        if (currentCell != CELDA_NINGUNA)
        {
            Serial.println("[CAJA] hay una pieza en camino a una celda: probar de nuevo en un momento");
            return;
        }

        aplicarLayout(nuevo);
        return;
    }

    // --- Ajuste en caliente de la supervision: 'U<grados>', 'T<ms>',
    //     'K<ms>' (margen por velocidad), 'L<ms>' (atraso a compensar) y
    //     'Q<grados>' (umbral con el robot quieto en home) ---
    // No llevan coma, asi que no se confunden nunca con un mensaje de pieza
    // (que siempre tiene dos). Se conservan porque son cortos de tipear con
    // el robot delante, pero NO tocan el guard directamente: escriben en la
    // tabla de parametros igual que 'P'. Si escribieran el guard por su
    // cuenta, la tabla quedaria diciendo el valor viejo y el proximo
    // sincronizarGuard() pisaria el cambio hecho a mano.
    {
        const char letra = toupper(cmd[0]);

        if ((letra == 'U' || letra == 'T' || letra == 'K' || letra == 'L' ||
             letra == 'Q') &&
            strchr(cmd, ',') == NULL)
        {
            char *finValor = NULL;
            const float valor = strtof(cmd + 1, &finValor);

            if (finValor == cmd + 1 || *finValor != '\0' || valor < 0.0f)
            {
                Serial.print("[SERIAL] valor invalido: '");
                Serial.print(cmd);
                Serial.println("'. Se espera 'U12.5' (grados) o 'T80'/'K80'/'L70' (ms)");
                return;
            }

            switch (letra)
            {
                case 'U':
                    aplicarParametro("g_umbral", valor);
                    break;

                case 'T':
                    aplicarParametro("g_conf", valor);
                    break;

                case 'K':
                    aplicarParametro("g_margen", valor);
                    break;

                case 'Q':
                    aplicarParametro("g_reposo", valor);
                    break;

                default: // 'L'
                    aplicarParametro("g_retardo", valor);
                    break;
            }
            return;
        }
    }

    // --- Mensaje de pieza: "Y,color,forma" (exactamente 3 campos) ---
    char *coma1 = strchr(cmd, ',');
    if (coma1 == NULL)
    {
        Serial.print("[SERIAL] comando invalido: '");
        Serial.print(cmd);
        Serial.println("'. Validos: 'C', 'F', 'A', 'N', 'R', 'D', 'S', 'G', 'CAL0'/'CAL1' o 'Y,color,forma'");
        return;
    }

    char *coma2 = strchr(coma1 + 1, ',');
    if (coma2 == NULL)
    {
        Serial.println("[SERIAL] faltan campos, se espera 'Y,color,forma' (ej: 3.5,B,S)");
        return;
    }

    if (strchr(coma2 + 1, ',') != NULL)
    {
        Serial.println("[SERIAL] sobran campos, se espera 'Y,color,forma' (ej: 3.5,B,S)");
        return;
    }

    *coma1 = '\0';
    *coma2 = '\0';

    char *campoY     = recortar(cmd);
    char *campoColor = recortar(coma1 + 1);
    char *campoForma = recortar(coma2 + 1);

    // El campo Y tiene que ser un numero COMPLETO. Con atof() no alcanzaba:
    // devuelve 0.0 ante cualquier texto que no sea numerico, y 0.0 cae
    // dentro de la cinta, asi que un "hola,B,S" se habria aceptado en
    // silencio como una pieza en Y=0.
    char *fin = NULL;
    const float y = strtof(campoY, &fin);

    if (*campoY == '\0' || fin == campoY || *fin != '\0')
    {
        Serial.print("[SERIAL] Y invalida: '");
        Serial.print(campoY);
        Serial.println("'. Tiene que ser un numero en cm (ej: 3.5)");
        return;
    }

    if (strlen(campoColor) != 1)
    {
        Serial.print("[SERIAL] color invalido: '");
        Serial.print(campoColor);
        Serial.println("'. Tiene que ser un solo caracter: R, G o B");
        return;
    }

    if (strlen(campoForma) != 1)
    {
        Serial.print("[SERIAL] forma invalida: '");
        Serial.print(campoForma);
        Serial.println("'. Tiene que ser un solo caracter: S, H o C");
        return;
    }

    const char color = toupper(campoColor[0]);
    const char shape = toupper(campoForma[0]);

    if (color != 'R' && color != 'G' && color != 'B')
    {
        Serial.print("[SERIAL] color invalido: '");
        Serial.print(campoColor);
        Serial.println("'. Validos: R (rojo), G (verde), B (azul)");
        return;
    }

    if (shape != 'S' && shape != 'H' && shape != 'C')
    {
        Serial.print("[SERIAL] forma invalida: '");
        Serial.print(campoForma);
        Serial.println("'. Validas: S (cuadrado), H (hexagono), C (circulo)");
        return;
    }

    if (y < BELT_MIN_Y || y > BELT_MAX_Y)
    {
        Serial.print("[SERIAL] Y=");
        Serial.print(y);
        Serial.print(" fuera de la cinta (");
        Serial.print(BELT_MIN_Y);
        Serial.print(" a ");
        Serial.print(BELT_MAX_Y);
        Serial.println(" cm)");
        return;
    }

    // En modo teach el brazo lo maneja el operador, y en calibracion no lo
    // maneja nadie: encolar piezas que nadie va a ir a buscar solo llenaria
    // la cola de posiciones vencidas. Se ignoran en silencio -- no son un
    // fallo del robot ni una pieza perdida por no llegar a tiempo, asi que
    // tampoco se cuentan.
    //
    // Con `calibrando` esto es ademas la segunda defensa, detras del
    // enclavamiento de `iniciarSiguientePieza()`: la interfaz ya deberia
    // haber dejado de mandar piezas al entrar a calibracion, pero el
    // firmware no puede confiar en eso. Una PC vieja, un navegador que se
    // quedo abierto o un monitor serie a mano alcanzan para que llegue una,
    // y del otro lado hay alguien con las manos sobre la cinta.
    if (state == TEACH || teachPedido || calibrando)
    {
        return;
    }

    Piece p;
    p.y = y;
    p.color = color;
    p.shape = shape;
    // Instante en que LLEGO el mensaje, no en que la pieza cruzo la
    // linea: el desfasaje entre los dos se corrige en
    // planificarPieza() con VISION_LATENCY_S.
    p.detectedAt_ms = millis();

    if (!queuePush(p))
    {
        Serial.println("[COLA] llena, se descarta la pieza mas nueva");
        piezasDescartadas++;
        return;
    }

    // Se cuenta al ENTRAR a la cola, no al detectarla la vision: son las
    // piezas que el robot efectivamente tuvo la oportunidad de agarrar, que
    // es contra lo que tiene sentido medir cuantas termino depositando.
    piezasDetectadas++;

    Serial.print("[PIEZA] Y=");
    Serial.print(p.y);
    Serial.print(" color=");
    Serial.print(p.color);
    Serial.print(" forma=");
    Serial.print(p.shape);
    Serial.print("  en cola: ");
    Serial.println(queueCount);
}

// ============================================================
//  COMANDOS DE TELEMETRIA Y PARAMETROS
// ============================================================
//  Los dos devuelven true si consumieron el comando (aunque haya sido para
//  contestar un error): un comando que empieza con 'V' o con 'P' no puede
//  ser ninguna otra cosa, asi que dejarlo seguir al resto del parser solo
//  lograria un segundo mensaje de error mas confuso que el primero.

bool Robot::procesarComandoTelemetria(const char *cmd)
{
    if (toupper(cmd[0]) != 'V' || strchr(cmd, ',') != NULL)
    {
        return false;
    }

    if (strlen(cmd) == 2)
    {
        switch (cmd[1])
        {
            case '1':
                telemetria.setActiva(true);
                return true;

            case '0':
                telemetria.setActiva(false);
                return true;

            case '?':
                // Una foto de [E] y [H] sin encender el stream: es lo que
                // pide la interfaz al conectarse, para pintar la pantalla
                // antes de decidir nada.
                telemetria.pedirFoto();
                return true;

            default:
                break;
        }
    }

    Serial.print("[SERIAL] comando de telemetria invalido: '");
    Serial.print(cmd);
    Serial.println("'. Validos: 'V1' (on), 'V0' (off), 'V?' (una foto)");

    return true;
}

// ------------------------------------------------------------------
void Robot::aplicarParametro(const char *nombre, float valor)
{
    const TablaParametros::Resultado r = params.fijar(nombre, valor, hayPiezaEnMano());

    // La respuesta sale SIEMPRE, salga bien o mal, y con el mismo formato:
    // es lo que mantiene sincronizada a la interfaz sin que tenga que
    // volver a pedir la tabla entera despues de cada cambio.
    Serial.print("[P] set n=");
    Serial.print(nombre);

    if (r == TablaParametros::FIJADO)
    {
        Serial.print(" v=");
        Serial.print(params.leer(nombre), 4);
        Serial.println(" ok");
        return;
    }

    Serial.print(" v=");
    Serial.print(valor, 4);

    switch (r)
    {
        case TablaParametros::DESCONOCIDO:
            Serial.println(" err=desconocido");
            break;

        case TablaParametros::FUERA_RANGO:
            Serial.println(" err=rango");
            break;

        default:
            // Cambiar la altura de un tacho con la pieza ya en el aire la
            // mandaria a un lugar distinto del que se planifico.
            Serial.println(" err=bloqueado");
            break;
    }
}

// ------------------------------------------------------------------
// Modo calibracion ('CAL1' / 'CAL0' / 'CAL?').
//
// Existe para poder ajustar la vision: se paran la cinta Y el robot, se
// apoyan piezas a mano bajo la camara y se mueven los umbrales de color
// mirando como cambia la deteccion.
//
// Parar la cinta sola NO alcanzaba, y esa era la version anterior de esto:
// una pieza apoyada a mano cruza la linea de deteccion igual --la cruza el
// operador al apoyarla, no la cinta--, la vision la informa y el brazo sale
// a buscarla con alguien inclinado sobre la cinta. Paso de verdad, y por
// poco. Asi que frenar la cinta y frenar el robot son la MISMA orden: no se
// pueden pedir por separado, porque la unica razon para parar la cinta a
// mano es meter las manos.
//
// Que hace cada cosa al entrar:
//
//   la cinta se para       para que las piezas se queden quietas;
//   la cola se vacia       las piezas que ya venian viajando ya no estan
//                          donde dice la cola, y salir a buscarlas seria
//                          ir a una posicion vieja;
//   se ignoran las nuevas  ver el parser de piezas;
//   NO se frena el brazo   si esta a mitad de una maniobra la termina. Un
//                          brazo frenado en el aire con una pieza colgando
//                          de la ventosa es peor que uno que deja la pieza
//                          y vuelve a home. Lo que se corta es el arranque
//                          del ciclo SIGUIENTE.
//
// De ahi que la interfaz tenga que mirar `rep=` (en reposo) y no solo
// `cal=`: entre el pedido y el brazo quieto puede pasar una maniobra entera.
bool Robot::procesarComandoCalibracion(const char *cmd)
{
    if (strncasecmp(cmd, "CAL", 3) != 0 || strchr(cmd, ',') != NULL)
    {
        return false;
    }

    const char sub = cmd[3];

    // 'CAL' pelado, o algo mas largo que 'CALx': no es este comando. Se
    // devuelve false y sigue el parser, que va a decir que es invalido.
    if (sub == '\0' || cmd[4] != '\0')
    {
        return false;
    }

    if (sub == '?')
    {
        informarCalibracion();
        return true;
    }

    if (sub == '1')
    {
        calibrando = true;

        conveyor.stop();

        // La cola se vacia ENTERA. Las piezas que estaban en ella venian
        // viajando sobre la cinta y su posicion se calcula a partir del
        // instante en que cruzaron la linea; con la cinta parada, esa cuenta
        // deja de valer y lo que queda guardado son destinos inventados.
        queueHead  = 0;
        queueCount = 0;

        informarCalibracion();

        return true;
    }

    if (sub != '0')
    {
        return false;
    }

    calibrando = false;

    // La cinta vuelve sola, con la misma condicion que el resto del
    // firmware: sin homing o en ERROR no se arranca nada. SALIR de
    // calibracion siempre se puede -- lo que no se puede es que salir
    // ponga en marcha una celda que no esta lista.
    if (state != ERROR && homed)
    {
        conveyor.setSpeedPercent(CONVEYOR_PWM);
    }

    informarCalibracion();

    return true;
}

// El brazo esta quieto, en home y con las manos vacias. Es la condicion que
// la interfaz espera antes de dejar meter las manos sobre la cinta, y por
// eso mira el estado y no el movimiento: 'enPosicion()' es cierto tambien a
// mitad de una maniobra, entre dos tramos.
bool Robot::enReposo() const
{
    return (state == WAIT_PIECE || state == IDLE || state == ERROR);
}

void Robot::informarCalibracion()
{
    Serial.print("[CAL] ");
    Serial.print(calibrando ? "on" : "off");
    Serial.print(" rep=");
    Serial.print(enReposo() ? 1 : 0);
    Serial.print(" cinta=");
    Serial.print(conveyor.getSpeed() > 0 ? 1 : 0);
    Serial.print(" est=");
    Serial.println(nombreEstado(state));
}

bool Robot::procesarComandoParametro(const char *cmd)
{
    if (toupper(cmd[0]) != 'P')
    {
        return false;
    }

    const char *igual = strchr(cmd, '=');

    if (igual == NULL)
    {
        if (strlen(cmd) == 2 && cmd[1] == '?')
        {
            params.volcar();
            return true;
        }

        if (strlen(cmd) == 2 && cmd[1] == '*')
        {
            params.guardar();
            return true;
        }

        if (strlen(cmd) == 2 && cmd[1] == '0')
        {
            params.restaurarDeFabrica();
            return true;
        }

        Serial.print("[SERIAL] comando de parametros invalido: '");
        Serial.print(cmd);
        Serial.println("'. Validos: 'P?', 'P*', 'P0' o 'Pnombre=valor'");
        return true;
    }

    // El nombre se copia en vez de partir la linea en el lugar: el buffer
    // de comandos se reutiliza y prefiero no dejarlo modificado a mitad de
    // camino por si mas adelante alguien agrega algo despues de esto.
    char nombre[16];

    const size_t largo = (size_t)(igual - (cmd + 1));

    if (largo == 0 || largo >= sizeof(nombre))
    {
        Serial.println("[SERIAL] nombre de parametro vacio o demasiado largo");
        return true;
    }

    memcpy(nombre, cmd + 1, largo);
    nombre[largo] = '\0';

    char       *fin   = NULL;
    const float valor = strtof(igual + 1, &fin);

    // Se exige que el numero se consuma ENTERO. Con atof() un "0.18cm"
    // pasaria como 0.18 y un "hola" como 0.0, que en un parametro de
    // altura significa clavar la ventosa contra la cinta.
    if (fin == igual + 1 || *fin != '\0')
    {
        Serial.print("[SERIAL] valor invalido para '");
        Serial.print(nombre);
        Serial.print("': '");
        Serial.print(igual + 1);
        Serial.println("'. Se espera un numero. Ej: Pvis_lat=0.18");
        return true;
    }

    aplicarParametro(nombre, valor);

    return true;
}

const char *Robot::nombreModo(SortMode m) const
{
    switch (m)
    {
        case SORT_BY_COLOR:  return "por COLOR";
        case SORT_BY_SHAPE:  return "por FORMA";
        case SORT_BOX: return "BOX";
        default:             return "?";
    }
}

void Robot::aplicarModoPendiente()
{
    if (!sortModePending)
    {
        return;
    }

    sortMode = pendingSortMode;
    sortModePending = false;

    Serial.print("[MODO] aplicado -> ");
    Serial.println(nombreModo(sortMode));

    if (esBox(sortMode))
    {
        // La caja arranca SIEMPRE vacia al entrar al modo. El mapa de celdas
        // que hubiera quedado de una tanda anterior no dice nada de la caja
        // que hay puesta ahora: entre una y otra alguien la retiro y puso
        // otra, y creer que ya hay piezas puestas hace que el robot
        // saltee celdas que estan vacias.
        reiniciarCaja(false);
        imprimirCaja();
    }
}

// ============================================================
//  CAMBIO DE MODO Y CONFIRMACION DE LA TAPA
// ============================================================

void Robot::pedirModo(SortMode nuevo, char letra)
{
    if (esBox(nuevo) && !boxLayoutValido)
    {
        Serial.println("[MODO] la disposicion de la caja es invalida: no se entra a BOX");
        return;
    }

    if (nuevo == modoObjetivo())
    {
        Serial.print("[MODO] ya esta ");
        Serial.println(nombreModo(nuevo));

        // Pedir el modo en el que ya se esta sirve de consulta: es la unica
        // forma de ver como viene la caja sin borrarla con 'N'.
        if (esBox(nuevo))
        {
            imprimirCaja();
        }
        return;
    }

    // Un cambio entre COLOR y FORMA no toca nada fisico y se aplica derecho.
    // Entrar o salir de BOX, en cambio, significa poner o sacar la
    // tapa: hasta que alguien confirme que ya esta asi, no se cambia nada.
    if (esBox(nuevo) == esBox(modoObjetivo()))
    {
        aplicarModo(nuevo);
        return;
    }

    const bool confirmando = esperandoConfirmacion &&
                             modoAConfirmar == nuevo &&
                             (uint32_t)(millis() - confirmacionPedida_ms) < (uint32_t)CONFIRMACION_TAPA_MS;

    if (!confirmando)
    {
        modoAConfirmar        = nuevo;
        esperandoConfirmacion = true;
        confirmacionPedida_ms = millis();

        if (esBox(nuevo))
        {
            Serial.println("[TAPA] para entrar al modo BOX hay que PONER la tapa sobre los tachos.");
        }
        else
        {
            Serial.println("[TAPA] para salir del modo BOX hay que SACAR la tapa de los tachos.");
        }

        Serial.print("[TAPA] si ya esta asi, confirmar con '");
        Serial.print(letra);
        Serial.print("' otra vez dentro de ");
        Serial.print((uint32_t)(CONFIRMACION_TAPA_MS / 1000));
        Serial.print(" s. Queda: ");
        Serial.println(nombreModo(nuevo));
        return;
    }

    esperandoConfirmacion = false;
    aplicarModo(nuevo);
}

void Robot::aplicarModo(SortMode nuevo)
{
    // Si el robot tiene una pieza en la mano, el cambio queda pendiente
    // hasta que la suelte: cambiarle el destino a una pieza en vuelo la
    // mandaria al lugar equivocado.
    if (hayPiezaEnMano())
    {
        pendingSortMode = nuevo;
        sortModePending = true;

        Serial.print("[MODO] pendiente -> ");
        Serial.println(nombreModo(nuevo));
        return;
    }

    sortMode = nuevo;
    sortModePending = false;

    Serial.print("[MODO] ");
    Serial.println(nombreModo(sortMode));

    if (esBox(sortMode))
    {
        // La caja arranca SIEMPRE vacia al entrar al modo. El mapa de celdas
        // que hubiera quedado de una tanda anterior no dice nada de la caja
        // que hay puesta ahora: entre una y otra alguien la retiro y puso
        // otra, y creer que ya hay piezas puestas hace que el robot
        // saltee celdas que estan vacias.
        reiniciarCaja(false);
        imprimirCaja();
    }
}

void Robot::vencerConfirmacion()
{
    if (!esperandoConfirmacion ||
        (uint32_t)(millis() - confirmacionPedida_ms) < (uint32_t)CONFIRMACION_TAPA_MS)
    {
        return;
    }

    esperandoConfirmacion = false;

    Serial.print("[TAPA] sin confirmacion: el modo sigue ");
    Serial.println(nombreModo(modoObjetivo()));
}

// ============================================================
//  CAJA DEL MODO BOX
// ============================================================

bool Robot::layoutValido(const char *layout)
{
    uint8_t porColor[3] = {0, 0, 0}; // R, G, B

    for (uint8_t i = 0; i < BOX_CELLS; i++)
    {
        switch (layout[i])
        {
            case 'R': porColor[0]++; break;
            case 'G': porColor[1]++; break;
            case 'B': porColor[2]++; break;
            default:  return false;
        }
    }

    // Con mas de 3 de un color la caja no se puede terminar nunca (4 rojos y
    // 2 verdes, o 6 verdes, son los ejemplos tipicos): el robot se quedaria
    // esperando indefinidamente una pieza que no tiene donde ir.
    for (uint8_t i = 0; i < 3; i++)
    {
        if (porColor[i] > BOX_MAX_POR_COLOR)
        {
            return false;
        }
    }

    return true;
}

void Robot::aplicarLayout(const char *layout)
{
    const uint8_t habia = celdasLlenas();

    memcpy(boxLayout, layout, BOX_CELLS);
    boxLayoutValido = true;

    // La disposicion nueva no dice nada de las piezas que ya estaban: el
    // mapa arranca de cero y la caja tiene que estar vacia de verdad.
    reiniciarCaja(false);

    Serial.println("[CAJA] disposicion nueva");
    imprimirCaja();

    if (habia > 0)
    {
        Serial.print("[CAJA] ojo: habia ");
        Serial.print(habia);
        Serial.println(" piezas puestas. Sacarlas antes de seguir.");
    }
}

uint8_t Robot::celdasLlenas() const
{
    uint8_t n = 0;

    for (uint8_t i = 0; i < BOX_CELLS; i++)
    {
        if (boxFilled[i])
        {
            n++;
        }
    }

    return n;
}

bool Robot::piezaSirveParaCaja(const Piece &p, uint8_t &celda) const
{
    celda = CELDA_NINGUNA;

    if (boxComplete || p.shape != 'C')
    {
        return false;
    }

    // De la celda 6 hacia la 1: entre varias del color que llega, gana
    // siempre la de numero mas grande.
    for (int8_t c = BOX_CELLS - 1; c >= 0; c--)
    {
        if (!boxFilled[c] && boxLayout[c] == p.color)
        {
            celda = (uint8_t)c;
            return true;
        }
    }

    return false;
}

void Robot::marcarCeldaLlena(uint8_t celda)
{
    boxFilled[celda] = true;

    Serial.print("[CAJA] celda ");
    Serial.print(celda + 1);
    Serial.print(" (");
    Serial.print(boxLayout[celda]);
    Serial.print(") lista: ");
    Serial.print(celdasLlenas());
    Serial.print(" de ");
    Serial.println(BOX_CELLS);

    if (celdasLlenas() < BOX_CELLS)
    {
        return;
    }

    boxComplete = true;

    imprimirCaja();
    Serial.println("[CAJA] COMPLETA. Desde aca se deja pasar todo.");
    Serial.println("[CAJA] Sacar la caja, poner una vacia y mandar 'N'.");
}

void Robot::reiniciarCaja(bool avisar)
{
    for (uint8_t i = 0; i < BOX_CELLS; i++)
    {
        boxFilled[i] = false;
    }

    boxComplete = false;

    if (avisar)
    {
        Serial.println("[CAJA] caja nueva: las 6 celdas quedan libres");
        imprimirCaja();
    }
}

void Robot::imprimirCaja() const
{
    // Dos lineas con la forma fisica de la caja: el color que pide cada
    // celda y si ya esta puesta. La primera fila es la mas cercana a la
    // cinta.
    for (uint8_t fila = 0; fila < 2; fila++)
    {
        Serial.print("[CAJA] ");

        for (uint8_t col = 0; col < 3; col++)
        {
            const uint8_t i = fila * 3 + col;

            Serial.print(i + 1);
            Serial.print(':');
            Serial.print(boxLayout[i]);
            Serial.print(boxFilled[i] ? "[x] " : "[ ] ");
        }

        Serial.println(fila == 0 ? "  (Y=-6.8, fila de adelante)"
                                 : "  (Y=-12.8, fila del fondo)");
    }

    Serial.print("[CAJA] ");
    Serial.print(celdasLlenas());
    Serial.print(" de ");
    Serial.print(BOX_CELLS);
    Serial.println(boxComplete ? " -- COMPLETA, mandar 'N' para empezar otra" : "");
}

// ============================================================
//  COLA DE PIEZAS
// ============================================================

bool Robot::queuePush(const Piece &p)
{
    if (queueCount >= QUEUE_CAPACITY)
    {
        return false;
    }

    pieceQueue[(queueHead + queueCount) % QUEUE_CAPACITY] = p;
    queueCount++;
    return true;
}

bool Robot::queuePop(Piece &out)
{
    if (queueCount == 0)
    {
        return false;
    }

    out = pieceQueue[queueHead];
    queueHead = (queueHead + 1) % QUEUE_CAPACITY;
    queueCount--;
    return true;
}

// ============================================================
//  HOMING
// ============================================================

void Robot::startHoming(bool conservarContexto)
{
    axis1Homed = false;
    axis2Homed = false;
    axis3Homed = false;
    homed = false;

    state = HOMING;

    // Mientras dura el homing no se supervisa nada: los ejes van contra los
    // finales de carrera a proposito, y ademas setPosition() les redefine la
    // cuenta de pasos de golpe, asi que cualquier comparacion contra el
    // encoder en el medio no significa nada. Se vuelve a armar al final,
    // cuando se calibra.
    guard.desarmar();

    homingStart_ms = millis();

    motor1.setPosition(999999);  // fuerza que nunca este en home al principio
    motor2.setPosition(999999);
    motor3.setPosition(999999);

    motor1.moveContinuous(false);
    motor2.moveContinuous(false);
    motor3.moveContinuous(false);

    motor1.setSpeed(1000);
    motor2.setSpeed(1000);
    motor3.setSpeed(1000);

    // Las piezas encoladas traen timestamps de antes del corte: sus
    // posiciones ya no son confiables (la cinta estuvo parada mientras
    // tanto), asi que se descartan todas.
    //
    // La excepcion es la recalibracion despues de una colision: ahi la cinta
    // NUNCA se detuvo, por lo que los timestamps siguen siendo validos y las
    // piezas siguieron avanzando de forma perfectamente conocida. Se conserva
    // la cola entera y al terminar el homing se vuelve a mirar cual sigue
    // siendo alcanzable: las que se pasaron de largo durante los 3 segundos
    // de pausa mas el homing las descarta planificarPieza() sola, una por
    // una, avisando por Serial.
    if (!conservarContexto)
    {
        queueHead = 0;
        queueCount = 0;

        // Un cambio de modo que habia quedado pendiente pertenecia al ciclo
        // que se corto: no se arrastra al ciclo nuevo. El modo ACTIVO si se
        // mantiene (el operador ya lo eligio y no lo dio de baja).
        sortModePending = false;

        colisionesSeguidas = 0;
    }

    // Se suelta lo que hubiera quedado agarrado antes del corte, para
    // arrancar el ciclo con el gripper vacio y en estado conocido.
    pneumatics.release();
    pumpOn = false;

    // La celda que tenia reservada la pieza perdida vuelve a quedar libre.
    // El MAPA de la caja, en cambio, no se toca ni con el reset manual: las
    // piezas que ya estan puestas siguen fisicamente ahi. Solo 'N' lo
    // borra, que es cuando alguien saco la caja y puso otra.
    currentCell = CELDA_NINGUNA;

    // Un cambio de modo a medio confirmar pertenecia al ciclo que se corto.
    esperandoConfirmacion = false;

    // Teach tampoco sobrevive: se rehomea por una colision o por un paro, y
    // en los dos casos la posicion de partida del jog dejo de ser conocida.
    // Hay que volver a pedirlo desde la interfaz.
    teachPedido        = false;
    teachReproduciendo = false;
    teachLanzado       = false;
    teachEsperando     = false;
    teachIrEtapa       = 0;
    teachStream        = false;
    lineal.cancelar();
    jogVx = jogVy = jogVz = 0.0f;

    moveIssued = false;
    replanCount = 0;
    homingSettleStart_ms = 0;
}

void Robot::updateHoming()
{
    // Un eje que no llega nunca a su final de carrera es un eje trabado
    // (tipicamente: el obstaculo que provoco la colision sigue ahi). Seguir
    // empujando contra el no arregla nada y castiga la mecanica.
    if (!(axis1Homed && axis2Homed && axis3Homed) &&
        (uint32_t)(millis() - homingStart_ms) > (uint32_t)HOMING_TIMEOUT_MS)
    {
        motor1.stop();
        motor2.stop();
        motor3.stop();

        // Aca si se para la cinta: el robot no puede recuperarse solo, y
        // dejarla andando solo acumula piezas sin clasificar.
        conveyor.stop();

        registrarFallo(FALLO_HOMING, 0);

        Serial.print("[HOMING] no encontro los finales de carrera en ");
        Serial.print((uint32_t)(HOMING_TIMEOUT_MS / 1000));
        Serial.println(" s: hay un eje trabado. Revisar y mandar 'R'.");

        state = ERROR;
        return;
    }

    // Motor 1

    if (!axis1Homed)
    {
        if (endstops.readMotor1())
        {
            motor1.stop();
            motor1.setPosition(angleToSteps(HOME_ANGLE_M1));
            axis1Homed = true;
        }
    }

    // Motor 2

    if (!axis2Homed)
    {
        if (endstops.readMotor2())
        {
            motor2.stop();
            motor2.setPosition(angleToSteps(HOME_ANGLE_M2));
            axis2Homed = true;
        }
    }

    // Motor 3

    if (!axis3Homed)
    {
        if (endstops.readMotor3())
        {
            motor3.stop();
            motor3.setPosition(angleToSteps(HOME_ANGLE_M3));
            axis3Homed = true;
        }
    }

    // ¿Todos llegaron?

    if (axis1Homed && axis2Homed && axis3Homed)
    {
        if (homingSettleStart_ms == 0)
        {
            // Arranca la ventana de acumulación de media móvil por canal.
            encoders.iniciarAsentamientoHoming();

            // La misma ventana le sirve a la supervision para promediar SU
            // referencia: son 2,5 s con los 3 ejes frenados contra el
            // endstop, la unica oportunidad de tomar un cero de encoder sin
            // el ruido de +-1 grado encima. Si la referencia saliera de una
            // lectura puntual, ese error se sumaria a todas las
            // comparaciones posteriores.
            // Se arma siempre, incluso con las paradas apagadas: en ese caso
            // el guard queda observando (mide y avisa, pero no frena).
            guard.iniciarReferencia(motor1.getPosition(),
                                    motor2.getPosition(),
                                    motor3.getPosition());

            homingSettleStart_ms = millis();
            return;
        }

        if (millis() - homingSettleStart_ms < (uint32_t)HOMING_SETTLE_WAIT_MS)
        {
            return; // seguimos acumulando muestras, sin bloquear el resto del sistema
        }

        homingSettleStart_ms = 0;

        // Calibra usando el PROMEDIO de todas las muestras de la ventana de
        // espera (no una lectura puntual).
        encoders.calibrarHoming(HOME_ANGLE_M1, HOME_ANGLE_M2, HOME_ANGLE_M3);

        // Recien aca queda armada la supervision: con la referencia ya
        // promediada y los pasos de los 3 ejes en su valor de home.
        guard.fijarReferencia();

        homed = true;

        // La cinta arranca recien con el robot ya calibrado: antes de eso
        // no tendria sentido aceptar piezas. Salvo que se este calibrando
        // la vision: ahi hay alguien con las manos sobre la cinta, y un
        // homing --que se puede pedir en cualquier momento con 'R'-- no
        // puede ser la forma de que se le mueva sola.
        conveyor.begin();

        if (!calibrando)
        {
            conveyor.setSpeedPercent(CONVEYOR_PWM);
        }
        else
        {
            conveyor.stop();
        }

        Serial.println("[HOMING] OK. Robot listo.");
        Serial.print("[MODO] ");
        Serial.println(nombreModo(sortMode));

        if (esBox(sortMode))
        {
            imprimirCaja();
        }

        moveIssued = false;
        state = GO_HOME_IDLE;
    }
}

bool Robot::homingFinished() const
{
    return homed;
}

Robot::RobotState Robot::getState() const
{
    return state;
}

// ============================================================
//  MOVIMIENTO
// ============================================================

bool Robot::goToPositionIK(float x, float y, float z, const Motors::MotionLimits &limits)
{
    DeltaKinematics::DeltaAngles pose = DeltaKinematics::solveIK(x, y, z);
    if (!pose.success)
    {
        return false;
    }
    Motors::moveSynchronized(motor1, motor2, motor3, pose.steps1, pose.steps2, pose.steps3, limits);
    return true;
}

// ============================================================
//  ESPERA / VUELTA A HOME
// ============================================================

void Robot::updateGoHomeIdle()
{
    // Si aparece una pieza mientras vuelve a home, se sale a buscarla --
    // pero SOLO con el brazo ya frenado (el enPosicion()).
    //
    // POR QUE: Stepper::moveTo() fija el pin de direccion y reinicia la
    // rampa sin mirar si el eje ya venia andando (ver el comentario en
    // Stepper.cpp). Si el destino nuevo queda del otro lado, el driver
    // invierte el sentido con el rotor girando, cosa que ningun paso a paso
    // puede seguir. Medido en el banco de pruebas: cada interrupcion del
    // regreso invertia un eje a hasta 36.000 pasos/s.
    //
    // HONESTIDAD SOBRE LO QUE ESTO ARREGLA: se sospechaba que era la causa
    // de la descalibracion que aparecia sola cada tantas piezas, y NO LO
    // ES. Probado en el robot, con 9 de cada 10 ciclos interrumpidos a
    // proposito (disparando la pieza justo cuando arranca el regreso):
    //
    //     sin esta espera   deriva max por eje  19,1 / 21,0 / 19,1
    //     con esta espera   deriva max por eje  22,1 / 19,7 / 16,5
    //
    // O sea lo mismo. Se deja igual porque la inversion en movimiento es
    // real y no cuesta nada evitarla, pero la descalibracion hay que
    // buscarla en otro lado.
    //
    // Esperar cuesta muy poco: el regreso a home dura menos de medio
    // segundo, y la pieza recien detectada necesita ~1,7 s para llegar al
    // area de agarre, asi que el brazo la va a esperar igual parado en el
    // punto de aproximacion. Medido: 18 de 18 piezas agarradas, ninguna
    // perdida por esperar.
    //
    // De paso el plan sale mejor: ConveyorIntercept estima el tiempo de
    // viaje suponiendo que el brazo ARRANCA DETENIDO, cosa que solo es
    // cierta si efectivamente lo esta.
    //
    // Ojo: en la primera vuelta de este estado el brazo todavia esta parado
    // sobre el tacho (el movimiento a home ni se lanzo), asi que ahi la
    // pieza se toma en el acto y no se pierde nada de productividad.
    // Con teach pedido no se sale a buscar mas piezas: hay que llegar a
    // WAIT_PIECE, que es el unico punto en el que el brazo esta quieto, en
    // home y sin nada en la mano -- las tres condiciones que hacen que la
    // posicion de partida del jog sea conocida.
    if (!teachPedido && queueCount > 0 && enPosicion() && iniciarSiguientePieza())
    {
        return;
    }

    if (!moveIssued)
    {
        // Home = brazos horizontales (0 grados en los 3 ejes).
        Motors::moveSynchronized(motor1, motor2, motor3, 0, 0, 0, Motors::FAST_LIMITS);
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        state = WAIT_PIECE;
    }
}

void Robot::updateWaitPiece()
{
    // Aca el brazo esta quieto en home y con las manos vacias: es el momento
    // en que se puede entrar a teach sin sorprender a nadie.
    if (teachPedido)
    {
        entrarTeach();
        return;
    }

    if (queueCount > 0)
    {
        iniciarSiguientePieza();
    }
}

// ============================================================
//  PLANIFICACION DE LA MANIOBRA
// ============================================================

bool Robot::planificarPieza(const Piece &p)
{
    ConveyorIntercept::BeltConfig belt;
    belt.velocityX = BELT_VELOCITY_CMS;
    belt.detectionLineX = DETECTION_LINE_X;

    ConveyorIntercept::PickGeometry geom;
    geom.grabZ = GRAB_Z;
    geom.approachDX = APPROACH_DX;
    geom.approachDZ = APPROACH_DZ;
    geom.pressDZ = PRESS_DZ;

    // En modo Box el brazo despega mas alto: desde la cinta tiene que
    // cruzar por encima de la pared de la caja y de las piezas ya
    // colocadas, y el camino entre dos puntos se curva hacia abajo (ver
    // BOX_TRANSIT_DZ). Con los 3 cm de siempre, ese hundimiento mete la
    // pieza por debajo del piso de la caja antes de llegar a la celda.
    //
    // ConveyorIntercept valida el punto de despegue con la cinematica, asi
    // que si algun agarre no admitiera la subida mas alta, esa pieza queda
    // marcada como no alcanzable y se deja pasar en vez de forzarla.
    geom.liftDZ = esBox(sortMode) ? BOX_TRANSIT_DZ : LIFT_DZ;

    geom.workAreaMinX = WORK_AREA_MIN_X;
    geom.workAreaMaxX = WORK_AREA_MAX_X;

    // Al tiempo que paso desde que llego el mensaje se le suma la
    // latencia de la vision: cuando ese mensaje llego, la pieza ya
    // no estaba sobre la linea de deteccion, ya habia avanzado.
    const float tSinceDetection = VISION_LATENCY_S +
                                  (millis() - p.detectedAt_ms) / 1000.0f;

    ConveyorIntercept::InterceptResult r = ConveyorIntercept::solve(
        p.y, tSinceDetection, belt, geom,
        motor1.getPosition(), motor2.getPosition(), motor3.getPosition(),
        Motors::VEL_MAX, Motors::ACC_RAPIDA, Motors::ACC_AGARRE);

    if (!r.reachable)
    {
        return false;
    }

    approachX    = r.approachX;    approachY    = r.approachY;    approachZ    = r.approachZ;
    descendEndX  = r.descendEndX;  descendEndY  = r.descendEndY;  descendEndZ  = r.descendEndZ;
    liftX        = r.liftX;        liftY        = r.liftY;        liftZ        = r.liftZ;

    // Punto de contacto, solo para el log de diagnostico.
    lastGrabX = r.grabX;
    lastContactSpeedX = r.contactSpeedX;

    descendStart_ms = millis() + (uint32_t)(r.descendStartDelay * 1000.0f);

    return true;
}

bool Robot::iniciarSiguientePieza()
{
    // EL ENCLAVAMIENTO de la calibracion, y esta aca y no en los cuatro
    // lugares que llaman a esta funcion justamente para que no se pueda
    // olvidar en uno: este es el unico embudo por el que empieza una
    // maniobra. Devolver false hace que cada uno de esos cuatro lugares
    // haga lo que ya sabe hacer cuando no hay pieza -- irse a home y
    // esperar --, que es exactamente lo que se quiere.
    if (calibrando)
    {
        return false;
    }

    // Aca el robot no tiene ninguna pieza en la mano: es el momento seguro
    // para aplicar un cambio de modo pendiente, ANTES de decidir el tacho
    // de la proxima pieza.
    aplicarModoPendiente();

    Piece p;

    // ORDEN DE AGARRE: el de deteccion, siempre. La cola es FIFO, asi que
    // queuePop() devuelve la pieza mas vieja, que es la MAS ADELANTADA sobre
    // la cinta (todas avanzan a la misma velocidad). Esa es la que menos
    // tiempo queda antes de pasarse de largo, asi que es la que hay que
    // atender primero.
    //
    // Lo unico que puede saltear una pieza es que ya no se llegue a agarrar
    // (planificarPieza) o que no sirva para la caja (piezaSirveParaCaja). Ni
    // el color, ni la forma, ni la celda de destino adelantan a una pieza
    // sobre otra: el modo Box decide DONDE va cada pieza, nunca en que
    // orden se van a buscar.
    while (queuePop(p))
    {
        // En modo Box el destino se decide ANTES de planificar el
        // agarre: la mayoria de las piezas no se agarran (cuadrados,
        // hexagonos, y los circulos de un color que ya no falta) y no tiene
        // sentido calcularles la intercepcion.
        uint8_t celda = CELDA_NINGUNA;

        if (esBox(sortMode) && !piezaSirveParaCaja(p, celda))
        {
            // Con la caja ya completa no se avisa pieza por pieza: pasarian
            // todas y el aviso de caja llena ya explico lo que va a pasar.
            if (!boxComplete)
            {
                Serial.print("[CAJA] no hace falta (color=");
                Serial.print(p.color);
                Serial.print(" forma=");
                Serial.print(p.shape);
                Serial.println("), se deja pasar");
            }
            continue;
        }

        if (!planificarPieza(p))
        {
            Serial.print("[PIEZA] no alcanzable (Y=");
            Serial.print(p.y);
            Serial.println("), se deja pasar");

            // Esta SI es una perdida: la pieza estaba para agarrar y no se
            // llego. Las que se dejan pasar en modo caja por no hacer falta
            // no se cuentan aca, que esas son el modo funcionando bien.
            piezasDescartadas++;

            continue; // se prueba con la siguiente de la cola
        }

        currentPiece = p;
        currentCell = celda;
        currentBin = binIndexFor(p);
        replanCount = 0;
        moveIssued = false;
        state = PICK_APPROACH;

        Serial.print("[PIEZA] contacto en X=");
        Serial.print(lastGrabX);
        Serial.print(" Y=");
        Serial.print(p.y);
        Serial.print(" a ");
        Serial.print(lastContactSpeedX);
        Serial.print(" cm/s (cinta ");
        Serial.print(BELT_VELOCITY_CMS);

        if (currentCell != CELDA_NINGUNA)
        {
            Serial.print(") -> celda ");
            Serial.println(currentCell + 1);
        }
        else
        {
            Serial.print(") -> tacho ");
            Serial.println(currentBin + 1);
        }

        return true;
    }

    return false;
}

uint8_t Robot::binIndexFor(const Piece &p) const
{
    if (sortMode == SORT_BY_COLOR)
    {
        switch (p.color)
        {
            case 'R': return 0;
            case 'G': return 1;
            case 'B': return 2;
            default:  return 0;
        }
    }

    switch (p.shape)
    {
        case 'S': return 0;
        case 'H': return 1;
        case 'C': return 2;
        default:  return 0;
    }
}

// ============================================================
//  TRAMO 1: aproximacion (aceleracion maxima) + espera del instante justo
// ============================================================

void Robot::updatePickApproach()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(approachX, approachY, approachZ, Motors::FAST_LIMITS))
        {
            // No deberia pasar: ConveyorIntercept ya valido este punto.
            Serial.println("[PIEZA] punto de aproximacion invalido, se descarta");
            currentCell = CELDA_NINGUNA; // la celda vuelve a quedar libre
            moveIssued = false;
            state = GO_HOME_IDLE;
            return;
        }
        moveIssued = true;
    }

    if (!enPosicion())
    {
        return;
    }

    // Se espera al instante calculado para lanzar la bajada. La resta con
    // signo aguanta el desborde de millis().
    const int32_t atraso_ms = (int32_t)(millis() - descendStart_ms);

    // La bomba se prende PUMP_LEAD_MS antes de la bajada, no al llegar aca.
    //
    // Antes se prendia al llegar al punto de aproximacion. Con la pieza
    // viniendo atras del brazo eso es casi lo mismo (se baja enseguida),
    // pero con una sola pieza en la cola el brazo llega a la aproximacion y
    // se queda esperandola: ahi la bomba quedaba soplando todo el viaje de
    // la pieza por la cinta, que puede ser mas de un segundo por pieza.
    //
    // El adelanto existe porque el vacio SI necesita un rato para formarse:
    // prenderla justo al arrancar la bajada la haria tocar la pieza con la
    // ventosa a medio hacer. Con el adelanto en 300 ms -- la misma ventana
    // en que se silencia el guard por el hundimiento del riel de los
    // encoders, asi que la conmutacion sigue quedando tapada -- el
    // comportamiento no cambia para las piezas que se agarran enseguida, y
    // solo cambia para las que hay que esperar.
    if (!pumpOn && atraso_ms >= -(int32_t)PUMP_LEAD_MS)
    {
        pneumatics.grab();
        pumpOn = true;

        // El arranque de la bomba le pega a la alimentacion de los encoders
        // (ver BLANQUEO_NEUMATICA_MS): se suspende la deteccion mientras
        // dura, y el guard informa cuanto se corrio cada eje.
        guard.silenciar((uint32_t)BLANQUEO_NEUMATICA_MS);
    }

    if (atraso_ms < 0)
    {
        return; // todavia no es momento: la pieza no llego
    }

    if (atraso_ms > (int32_t)PICK_LATE_TOLERANCE_MS)
    {
        // Se perdio la ventana (el tramo 1 tardo mas de lo estimado). Se
        // replanifica con la pieza donde este ahora, si todavia da.
        if (replanCount < (uint8_t)MAX_REPLAN_ATTEMPTS && planificarPieza(currentPiece))
        {
            replanCount++;
            moveIssued = false;
            Serial.println("[PIEZA] replanificando agarre");
            return;
        }

        Serial.println("[PIEZA] se perdio la ventana de agarre, se deja pasar");
        pneumatics.release();
        pumpOn = false;
        currentCell = CELDA_NINGUNA; // la celda vuelve a quedar libre
        moveIssued = false;
        state = GO_HOME_IDLE;
        return;
    }

    moveIssued = false;
    state = PICK_DESCEND;
}

// ============================================================
//  TRAMO 2: bajada a la pieza (aceleracion minima, a favor de la cinta)
//  El destino SOBREPASA a la pieza, asi el contacto ocurre a mitad del
//  movimiento y a la misma velocidad que la cinta (ver ConveyorIntercept.h).
// ============================================================

void Robot::updatePickDescend()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(descendEndX, descendEndY, descendEndZ, Motors::AGARRE_LIMITS))
        {
            Serial.println("[PIEZA] punto de agarre invalido, se descarta");
            pneumatics.release();
            pumpOn = false;
            currentCell = CELDA_NINGUNA; // la celda vuelve a quedar libre
            moveIssued = false;
            state = GO_HOME_IDLE;
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        state = PICK_LIFT;
    }
}

// ============================================================
//  TRAMO 3: despegue de la pieza de la cinta (aceleracion maxima)
// ============================================================

void Robot::updatePickLift()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(liftX, liftY, liftZ, Motors::FAST_LIMITS))
        {
            Serial.println("[PIEZA] punto de despegue invalido");
            currentCell = CELDA_NINGUNA; // la celda vuelve a quedar libre
            moveIssued = false;
            state = GO_HOME_IDLE;
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;

        // La celda reservada es lo que decide el destino, no el modo: si el
        // modo cambiara justo ahora, esta pieza ya salio con una celda
        // asignada y tiene que terminar ahi.
        state = (currentCell != CELDA_NINGUNA) ? BOX_TRANSIT : GO_BIN;
    }
}

// ============================================================
//  TRAMO 4: traslado al tacho (aceleracion maxima)
// ============================================================

void Robot::updateGoBin()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(BIN_X[currentBin], BIN_Y, BIN_Z, Motors::FAST_LIMITS))
        {
            Serial.println("[TACHO] posicion invalida");
            emergencyStop();
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        binSettleStart_ms = millis();
        state = BIN_SETTLE;
    }
}

// ============================================================
//  ASENTAMIENTO Y SOLTADO
// ============================================================

void Robot::updateBinSettle()
{
    // Quieto sobre el tacho: si se soltara apenas frena, la pieza saldria
    // disparada con la inercia del brazo en vez de caer vertical.
    if (millis() - binSettleStart_ms < BIN_SETTLE_MS)
    {
        return;
    }

    pneumatics.release();
    pumpOn = false;
    guard.silenciar((uint32_t)BLANQUEO_NEUMATICA_MS); // apagarla tambien mueve el riel

    releaseStart_ms = millis();
    state = RELEASE_WAIT;
}

void Robot::updateReleaseWait()
{
    // Espera a que la pieza se despegue sola del gripper: con la
    // electrovalvula montada, lo que tarda en entrar el aire a la linea
    // (ver RELEASE_DETACH_MS). La caja lleva su propio tiempo porque la
    // pieza queda APOYADA y la ventosa sigue tocandola, o sea que tiene
    // menos ayuda para despegarse que una pieza que se suelta en el aire
    // (ver BOX_RELEASE_DETACH_MS).
    const uint32_t espera = (currentCell != CELDA_NINGUNA) ? BOX_RELEASE_DETACH_MS
                                                           : RELEASE_DETACH_MS;

    if (millis() - releaseStart_ms < espera)
    {
        return;
    }

    // Pieza entregada: el ciclo cerro bien, asi que la racha de colisiones
    // seguidas se corta aca (lo que hubiera chocado antes, ya se resolvio).
    colisionesSeguidas = 0;

    // Y recien aca se cuenta como depositada: soltada de verdad, en su
    // destino. Contarla al agarrarla habria inflado el numero con las que
    // se pierden a mitad de camino, que son justamente las que interesa
    // que aparezcan en la diferencia.
    contarDepositada(currentPiece);

    // En la caja el brazo esta METIDO ENTRE LAS PAREDES: antes de decidir
    // nada tiene que salir derecho para arriba. Ir desde adentro de la caja
    // a cualquier otro lado hunde el camino contra las piezas puestas.
    if (currentCell != CELDA_NINGUNA)
    {
        // La celda pasa a llena, pero currentCell se conserva hasta haber
        // salido: es de donde el tramo de salida tiene que subir.
        marcarCeldaLlena(currentCell);

        moveIssued = false;
        state = BOX_LIFT;
        return;
    }

    // Recien ahora, con la pieza ya soltada, se puede cambiar de modo.
    aplicarModoPendiente();

    // Si hay otra pieza, sale a buscarla directo desde arriba del tacho.
    if (queueCount > 0 && iniciarSiguientePieza())
    {
        return;
    }

    moveIssued = false;
    state = GO_HOME_IDLE;
}

// ============================================================
//  CAJA DEL MODO BOX: los cuatro tramos hasta dejar la pieza
// ============================================================
//
//  1. TRANSIT   accel MAX, hasta 6 cm sobre la celda. Es el tramo que cruza
//               por encima de la pared de la caja y de las piezas ya
//               puestas (ver BOX_TRANSIT_DZ: con menos altura, la curva del
//               movimiento los baaria).
//  2. APPROACH  accel MAX, baja en vertical hasta BOX_APPROACH_DZ (1 cm)
//               sobre la celda. O sea 5 de los 6 cm del descenso.
//  3. DESCEND   accel MIN, el ultimo centimetro: apoya la pieza. Se suelta
//               abajo, no antes.
//  4. LIFT      accel MAX, sale en vertical a la altura de cruce.
//
//  Los tramos 2 y 4 son verticales sobre la misma celda: ahi el camino no
//  se desvia (menos de 1,3 mm de deriva lateral medida en toda la bajada).
// ============================================================

void Robot::updateBoxTransit()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(BOX_X[currentCell] + BOX_DX, BOX_Y[currentCell] + BOX_DY,
                            BOX_Z + BOX_TRANSIT_DZ, Motors::FAST_LIMITS))
        {
            Serial.println("[CAJA] altura de cruce invalida");
            emergencyStop();
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        state = BOX_APPROACH;
    }
}

void Robot::updateBoxApproach()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(BOX_X[currentCell] + BOX_DX, BOX_Y[currentCell] + BOX_DY,
                            BOX_Z + BOX_APPROACH_DZ, Motors::FAST_LIMITS))
        {
            Serial.println("[CAJA] punto de aproximacion invalido");
            emergencyStop();
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        state = BOX_DESCEND;
    }
}

void Robot::updateBoxDescend()
{
    if (!moveIssued)
    {
        // Aceleracion minima, igual que al entrar a la pieza sobre la cinta:
        // la pieza se APOYA en el piso de la caja, no se deja caer.
        //
        // A diferencia del agarre, aca NO se silencia la supervision: no hay
        // conmutacion de la bomba ni un contacto buscado a proposito (la
        // punta baja justo hasta donde la pieza queda apoyada), y en cambio
        // la caja es un obstaculo rigido. Si esta corrida de lugar o la tapa
        // no esta donde deberia, esto es exactamente lo que hay que detectar.
        if (!goToPositionIK(BOX_X[currentCell] + BOX_DX, BOX_Y[currentCell] + BOX_DY,
                            BOX_Z, Motors::CAJA_LIMITS))
        {
            Serial.println("[CAJA] punto de soltado invalido");
            emergencyStop();
            return;
        }
        moveIssued = true;
    }

    if (enPosicion())
    {
        moveIssued = false;
        binSettleStart_ms = millis();
        state = BIN_SETTLE; // suelta la pieza, igual que en el tacho
    }
}

void Robot::updateBoxLift()
{
    if (!moveIssued)
    {
        if (!goToPositionIK(BOX_X[currentCell] + BOX_DX, BOX_Y[currentCell] + BOX_DY,
                            BOX_Z + BOX_TRANSIT_DZ, Motors::FAST_LIMITS))
        {
            Serial.println("[CAJA] salida invalida");
            emergencyStop();
            return;
        }
        moveIssued = true;
    }

    if (!enPosicion())
    {
        return;
    }

    moveIssued = false;

    // Recien aca la pieza queda del todo entregada y el brazo libre.
    currentCell = CELDA_NINGUNA;

    // Ya fuera de la caja: recien ahora se puede cambiar de modo o salir a
    // buscar la pieza siguiente.
    aplicarModoPendiente();

    if (queueCount > 0 && iniciarSiguientePieza())
    {
        return;
    }

    state = GO_HOME_IDLE;
}

// ============================================================
//  SUPERVISION POR ENCODERS (deteccion de colisiones)
// ============================================================
//
//  Los encoders NO cierran el lazo de posicion: el posicionamiento sigue
//  siendo a lazo abierto por micropasos, que da muy buen resultado. Lo que
//  cierran es un lazo de SEGURIDAD: miran si el brazo esta donde los pasos
//  dicen que deberia estar.
//
//  Mientras el robot anda bien, los pasos emitidos y el angulo medido se
//  mueven juntos y la diferencia queda chica (ruido del encoder, retardo de
//  sus filtros, atraso mecanico de la rampa). Cuando el brazo choca contra
//  algo, los pasos siguen saliendo pero el eje no gira: la diferencia se
//  abre rapido y no vuelve. Eso es lo que se detecta.
//
//  Dos condiciones, no una: la diferencia tiene que pasar el umbral Y
//  mantenerse. Una lectura suelta fuera de rango no frena nada.
//
//  Ver CollisionGuard.h para el detalle del calculo y de los parametros.
//
// ============================================================

void Robot::supervisarColision()
{
    // Quieto en home esperando piezas: la unica situacion en la que el guard
    // puede exigir un umbral chico (ver el chequeo en reposo en
    // CollisionGuard.h). WAIT_PIECE es exactamente eso -- se entra recien
    // cuando el movimiento a home termino -- asi que no hace falta mirar
    // nada mas.
    guard.setEnHome(state == WAIT_PIECE);

    // Se llama SIEMPRE, aunque las paradas esten apagadas: con 'G' el guard
    // pasa a modo observador (mide y avisa, no frena), asi la traza, los
    // picos y la ganancia medida siguen sirviendo mientras se calibra.

    // Un encoder que dejo de ser confiable no frena el robot (seria peor el
    // remedio: se pararia por no poder ver, no por haber chocado), pero si
    // queda registrado: mientras dure, ese eje esta sin vigilancia.
    uint8_t ejeCaido = 0;
    if (guard.consumirAvisoSensor(ejeCaido))
    {
        registrarFallo(FALLO_ENCODER, (uint8_t)(ejeCaido + 1));
        Serial.print("[GUARD] encoder del eje ");
        Serial.print(ejeCaido + 1);
        Serial.println(" no confiable: ese eje queda sin supervisar");
    }

    if (!guard.actualizar(motor1.getPosition(),
                          motor2.getPosition(),
                          motor3.getPosition()))
    {
        return;
    }

    dispararColision(guard.ejeDelFallo(),
                     guard.errorDelFallo(),
                     guard.cmdDeltaDelFallo(),
                     guard.encDeltaDelFallo());
}

void Robot::dispararColision(uint8_t eje, float errorDeg, float cmdDelta, float encDelta)
{
    // Lo primero, antes de cualquier print: frenar. Los tres ejes, no solo
    // el que disparo -- el brazo delta es un mecanismo cerrado, si uno se
    // trabo los otros dos estan forzando contra el.
    motor1.stop();
    motor2.stop();
    motor3.stop();

    // La pieza que estuviera agarrada se suelta. Despues del rehoming no hay
    // forma de saber si sigue pegada a la ventosa ni donde quedo, asi que
    // arrastrarla seria peor: el ciclo tiene que arrancar con el gripper
    // vacio y en estado conocido. La pieza perdida queda en el registro.
    pneumatics.release();
    pumpOn = false;

    // La cinta tambien para: si siguiera andando durante la pausa y el
    // rehoming, las piezas se acumularian y pasarian de largo sin
    // clasificar. Arranca sola de nuevo al final del homing
    // (conveyor.begin() en updateHoming), o sea recien con el robot ya
    // recalibrado y listo para trabajar.
    conveyor.stop();

    // Los dos motivos terminan igual (frenar y recalibrar), pero no son lo
    // mismo para quien despues lee el registro: uno es un golpe, el otro es
    // el robot perdiendo la calibracion solo, sin que nadie lo toque.
    const bool descalibracion =
        (guard.motivoDelFallo() == CollisionGuard::MOTIVO_DESCALIBRACION);

    registrarFallo(descalibracion ? FALLO_DESCALIBRACION : FALLO_COLISION,
                   (uint8_t)(eje + 1), errorDeg, cmdDelta, encDelta);

    if (descalibracion)
    {
        Serial.print("[DESCALIBRACION] eje ");
        Serial.print(eje + 1);
        Serial.print(": quieto en home el encoder marca ");
        Serial.print(errorDeg, 1);
        Serial.println(" grados de diferencia con los pasos.");
        Serial.println("[DESCALIBRACION] No es un choque: se perdieron pasos, o alguien");
        Serial.println("        movio el brazo, o hay algo flojo. Se recalibra.");
    }
    else
    {
        Serial.print("[COLISION] eje ");
        Serial.print(eje + 1);
        Serial.print(": el encoder marca ");
        Serial.print(errorDeg, 1);
        Serial.print(" grados de diferencia con los pasos (pasos ");
        Serial.print(cmdDelta, 1);
        Serial.print(" / encoder ");
        Serial.print(encDelta, 1);
        Serial.println(")");
    }

    colisionesSeguidas++;

    if (colisionesSeguidas >= (uint8_t)MAX_COLISIONES_SEGUIDAS)
    {
        // Chocar, rehomear y volver a chocar contra lo mismo no lo va a
        // resolver: hay algo fisico que sacar antes de seguir.
        conveyor.stop();

        Serial.print("[COLISION] ");
        Serial.print(colisionesSeguidas);
        Serial.println(" colisiones seguidas sin completar una pieza: hay algo trabado.");
        Serial.println("[COLISION] Robot detenido. Revisar y mandar 'R'.");

        moveIssued = false;
        state = ERROR;
        return;
    }

    Serial.print("[COLISION] parado ");
    Serial.print(COLLISION_PAUSE_MS / 1000.0f, 1);
    Serial.println(" s y despues recalibra");

    moveIssued = false;
    collisionStart_ms = millis();
    state = COLLISION_STOP;
}

void Robot::updateCollisionStop()
{
    // Pausa no bloqueante: el loop sigue girando entero (Serial, encoders,
    // cola de piezas). Los motores estan frenados, no hay nada que mover.
    if ((uint32_t)(millis() - collisionStart_ms) < COLLISION_PAUSE_MS)
    {
        return;
    }

    Serial.println("[COLISION] recalibrando...");

    // La cola se descarta: con la cinta detenida durante la pausa y el
    // homing, las piezas ya no estan donde su timestamp dice que estarian.
    // Las que queden sobre la cinta las vuelve a detectar la vision cuando
    // arranque de nuevo.
    startHoming(false);
}

// ============================================================
//  REGISTRO DE FALLOS
// ============================================================

void Robot::registrarFallo(uint8_t tipo, uint8_t eje,
                            float errorDeg, float cmdDelta, float encDelta)
{
    RegistroFallo r;

    r.tipo     = tipo;
    r.eje      = eje;
    r.errorDeg = errorDeg;
    r.cmdDelta = cmdDelta;
    r.encDelta = encDelta;
    r.estado   = nombreEstado(state);

    // Con que pieza fallo: color, forma, donde estaba sobre la cinta y a que
    // tacho iba. Es lo que despues necesita la interfaz para mostrar el
    // fallo con contexto y no como un numero suelto.
    if (hayManiobraEnCurso())
    {
        r.conPieza = true;
        r.enMano   = hayPiezaEnMano();
        r.color    = currentPiece.color;
        r.forma    = currentPiece.shape;
        r.piezaY   = currentPiece.y;
        r.piezaX   = piezaXEstimada(currentPiece);

        // En modo Box el destino es una celda de la caja (1-6); en los
        // otros dos, uno de los tres tachos.
        r.tacho    = (currentCell != CELDA_NINGUNA) ? (uint8_t)(currentCell + 1)
                                                    : (uint8_t)(currentBin + 1);
    }

    fallos.registrar(r);
}

float Robot::piezaXEstimada(const Piece &p) const
{
    // Misma cuenta que usa planificarPieza(): desde que se detecto, la pieza
    // avanzo velocidad * tiempo, corregido por la latencia de la vision.
    const float t = VISION_LATENCY_S + (millis() - p.detectedAt_ms) / 1000.0f;
    return DETECTION_LINE_X + BELT_VELOCITY_CMS * t;
}

bool Robot::hayManiobraEnCurso() const
{
    return state == PICK_APPROACH ||
           state == PICK_DESCEND  ||
           state == PICK_LIFT     ||
           state == GO_BIN        ||
           state == BIN_SETTLE    ||
           state == RELEASE_WAIT  ||
           state == BOX_TRANSIT   ||
           state == BOX_APPROACH  ||
           state == BOX_DESCEND   ||
           state == BOX_LIFT;
}

bool Robot::hayPiezaEnMano() const
{
    // Desde que empieza la bajada de agarre ya se cuenta como "en la mano":
    // el contacto ocurre a mitad del tramo 2, no al final. BOX_LIFT queda
    // afuera a proposito: ahi la pieza ya esta apoyada en la caja.
    return state == PICK_DESCEND ||
           state == PICK_LIFT    ||
           state == GO_BIN       ||
           state == BIN_SETTLE   ||
           state == BOX_TRANSIT  ||
           state == BOX_APPROACH ||
           state == BOX_DESCEND;
}

const char *Robot::nombreEstado(RobotState s) const
{
    switch (s)
    {
        case IDLE:           return "IDLE";
        case HOMING:         return "HOMING";
        case WAIT_PIECE:     return "WAIT_PIECE";
        case GO_HOME_IDLE:   return "GO_HOME_IDLE";
        case PICK_APPROACH:  return "PICK_APPROACH";
        case PICK_DESCEND:   return "PICK_DESCEND";
        case PICK_LIFT:      return "PICK_LIFT";
        case GO_BIN:         return "GO_BIN";
        case BIN_SETTLE:     return "BIN_SETTLE";
        case RELEASE_WAIT:   return "RELEASE_WAIT";
        case BOX_TRANSIT:    return "BOX_TRANSIT";
        case BOX_APPROACH:   return "BOX_APPROACH";
        case BOX_DESCEND:    return "BOX_DESCEND";
        case BOX_LIFT:       return "BOX_LIFT";
        case COLLISION_STOP: return "COLLISION_STOP";
        case ERROR:          return "ERROR";
        case TEACH:          return "TEACH";
        default:             return "?";
    }
}

void Robot::imprimirEstadoSupervision() const
{
    Serial.print("[GUARD] ");
    Serial.print(guard.nombreEstado());
    Serial.print(guard.armado() ? "" : (supervisionHabilitada ? " (habilitada)" : " (apagada por el operador)"));
    Serial.print("  umbral=");
    Serial.print(guard.umbral(), 1);
    Serial.print("+");
    Serial.print(guard.margenVelocidad());
    Serial.print("ms*vel  confirmacion=");
    Serial.print(guard.confirmacion());
    Serial.print(" ms  compensacion=");
    Serial.print(guard.retardo());
    Serial.println(" ms");

    Serial.print("[GUARD] en reposo (home): umbral=");
    Serial.print(guard.umbralReposo(), 1);
    Serial.print(" grados  confirmacion=");
    Serial.print(GuardConfig::CONFIRMACION_REPOSO_MS);
    Serial.println(" ms");

    for (uint8_t i = 0; i < 3; i++)
    {
        // err     lo que ve el guard ahora mismo
        // pico    el peor error desde el homing: con esto se elige el umbral
        // gan     encoder/pasos con el eje frenado. TIENE que dar ~1.00; si
        //         da menos, se estan perdiendo cuentas (y ningun margen lo
        //         arregla, hay que ir al sensor)
        // atraso  error/velocidad en marcha. Si es estable y parecido en los
        //         3 ejes, es atraso de medicion y se compensa con 'L'
        // raw     extremos de lectura cruda: dicen cuanto margen queda hasta
        //         la zona donde el ADC ya no mide (60 a 3950)
        Serial.print("[GUARD] eje ");
        Serial.print(i + 1);
        Serial.print("  err=");
        Serial.print(guard.errorActual(i), 2);
        Serial.print("  pico=");
        Serial.print(guard.picoDesdeHoming(i), 2);
        Serial.print("  pico_total=");
        Serial.print(guard.picoHistorico(i), 2);
        // pico_reposo  el peor error visto QUIETO EN HOME: es el piso de
        //              ruido real de esa pose, y con el se elige
        //              UMBRAL_REPOSO_DEG (que tiene que quedarle 2 o 3
        //              veces por encima)
        Serial.print("  pico_reposo=");
        Serial.print(guard.picoEnReposo(i), 2);
        Serial.print("  umbral_ef=");
        Serial.print(guard.umbralEfectivo(i), 1);
        Serial.print("  gan=");
        Serial.print(guard.gananciaMedida(i), 3);
        Serial.print("  atraso=");
        Serial.print(guard.atrasoMedido_ms(i), 1);
        Serial.print("ms  fuga=");
        Serial.print(guard.derivaAbsorbida(i), 2);
        Serial.print("  raw=");
        Serial.print(guard.rawMinimo(i));
        Serial.print("..");
        Serial.print(guard.rawMaximo(i));
        Serial.print("  encoder=");
        Serial.println(guard.fueraDeRango(i) ? "FUERA DE RANGO" :
                       (guard.sensorCaido(i) ? "CAIDO" : "ok"));
    }

    fallos.imprimirResumen();
}

// ============================================================
//  TRAZA EN VIVO ('M')
// ============================================================
//  Vuelca a 20 Hz lo que ve la supervision, para poder mirar un movimiento
//  entero y entender que pasa cuando salta un falso positivo. Con esto se
//  distinguen los tres casos que se parecen entre si:
//
//    ATRASO      enc va detras de cmd durante todo el movimiento y lo
//                alcanza al frenar: al final err vuelve a ~0 solo
//    PERDIDA DE  enc no alcanza a cmd nunca: al frenar queda un escalon
//    CUENTAS     que no se va, y se suma al del movimiento siguiente
//    ZONA CIEGA  raw se queda clavado en un extremo mientras cmd avanza
//
//  Formato pensado para copiar y pegar en una planilla o en Python.
// ============================================================

// Una sola linea, pensada para leerla en el log de la interfaz de Python
// cuando no se puede pedir 'S' a mano:
//
//   pico  peor error desde el homing -> con esto se elige el umbral fijo
//   prep  peor error QUIETO EN HOME -> con esto se elige UMBRAL_REPOSO_DEG
//   gan   encoder/pasos con el eje frenado; tiene que dar ~1.00
//   atr   atraso de la medicion en ms (error/velocidad en marcha)
//   fuga  grados de deriva que la fuga en reposo lleva absorbidos: si crece
//         sin parar y siempre para el mismo lado, se pierden pasos de verdad
void Robot::imprimirDiagnosticoCorto() const
{
    Serial.print("[GUARD]");

    for (uint8_t i = 0; i < 3; i++)
    {
        Serial.print(" e");
        Serial.print(i + 1);
        Serial.print(" err=");
        Serial.print(guard.errorActual(i), 1);
        Serial.print(" pico=");
        Serial.print(guard.picoDesdeHoming(i), 1);
        Serial.print(" prep=");
        Serial.print(guard.picoEnReposo(i), 1);
        Serial.print(" gan=");
        Serial.print(guard.gananciaMedida(i), 3);
        Serial.print(" atr=");
        Serial.print(guard.atrasoMedido_ms(i), 0);
        Serial.print(" fuga=");
        Serial.print(guard.derivaAbsorbida(i), 1);
    }

    Serial.print(" fallos=");
    Serial.println(fallos.total());
}

void Robot::imprimirTraza()
{
    Serial.print("[TRAZA] t=");
    Serial.print(millis());

    for (uint8_t i = 0; i < 3; i++)
    {
        Serial.print(" e");
        Serial.print(i + 1);
        Serial.print(" raw=");
        Serial.print(encoders.leerRaw(i));
        Serial.print(" enc=");
        Serial.print(guard.encDeltaActual(i), 2);
        Serial.print(" cmd=");
        Serial.print(guard.cmdDeltaActual(i), 2);
        Serial.print(" err=");
        Serial.print(guard.errorActual(i), 2);
        Serial.print(" vel=");
        Serial.print(guard.velocidadCmd(i), 0);
    }

    Serial.println();
}

// ============================================================
//  TABLA DE PARAMETROS
// ============================================================
//  Lo que la interfaz puede ajustar sin recompilar. El nivel decide en que
//  pestana aparece; el rango es lo que el firmware hace cumplir.
//
//  QUE NO ESTA ACA, Y POR QUE
//  --------------------------
//  La geometria del delta (DeltaKinematics.h), los micropasos, el ancho
//  util de la cinta y el area de trabajo NO son parametros: son limites de
//  seguridad y calibraciones de las que depende que el robot no se clave
//  contra la mesa. Un slider para eso no es flexibilidad, es una trampa.
//  Se cambian en el codigo, recompilando, que es lo que corresponde a algo
//  que hay que volver a validar despues de tocarlo.
// ============================================================

void Robot::registrarParametros()
{
    // --- Nivel 1: operacion ---
    // La latencia de vision es EL ajuste fino de cada arranque: cuanto
    // avanza la pieza entre que cruza la linea y que llega el mensaje.
    // Admite negativos a proposito (ver PROTOCOLO.md): si alguna vez la
    // correccion tuviera que ir para el otro lado, el rango ya lo permite.
    params.registrar("vis_lat", &VISION_LATENCY_S, -0.20f, 3.00f, "s", NIVEL_OPERACION);

    // --- Nivel 2: proceso (afinado del agarre y de la supervision) ---
    params.registrar("press_dz", &PRESS_DZ,    0.0f,   0.30f, "cm", NIVEL_PROCESO);
    params.registrar("apr_dx",   &APPROACH_DX, -6.0f,  0.0f,  "cm", NIVEL_PROCESO);
    params.registrar("apr_dz",   &APPROACH_DZ, 0.5f,   6.0f,  "cm", NIVEL_PROCESO);
    params.registrar("lift_dz",  &LIFT_DZ,     1.0f,   8.0f,  "cm", NIVEL_PROCESO);
    params.registrar("rel_ms",   &RELEASE_DETACH_MS, 0.0f, 2000.0f, "ms", NIVEL_PROCESO, 'i');
    params.registrar("bin_ms",   &BIN_SETTLE_MS,     0.0f, 2000.0f, "ms", NIVEL_PROCESO, 'i');

    params.registrar("g_umbral",  &pGuardUmbral,   1.0f,  45.0f,  "deg", NIVEL_PROCESO);
    params.registrar("g_reposo",  &pGuardReposo,   2.0f,  20.0f,  "deg", NIVEL_PROCESO);
    params.registrar("g_conf",    &pGuardConfirma, 10.0f, 1000.0f, "ms", NIVEL_PROCESO, 'i');
    params.registrar("g_margen",  &pGuardMargen,   0.0f,  500.0f,  "ms", NIVEL_PROCESO, 'i');
    params.registrar("g_retardo", &pGuardRetardo,  0.0f,  500.0f,  "ms", NIVEL_PROCESO, 'i');
    params.registrar("g_salto",   &pGuardSalto,    1.0f,  20.0f,   "deg", NIVEL_PROCESO);
    params.registrar("g_salto_k", &pGuardSaltoPct, 0.0f,  20.0f,   "%", NIVEL_PROCESO);

    params.registrar("col_pausa", &COLLISION_PAUSE_MS, 0.0f, 15000.0f, "ms", NIVEL_PROCESO, 'i');
    params.registrar("col_max",   &MAX_COLISIONES_SEGUIDAS, 1.0f, 10.0f, "", NIVEL_PROCESO, 'i');
    params.registrar("g_neum",    &BLANQUEO_NEUMATICA_MS,   0.0f, 2000.0f, "ms", NIVEL_PROCESO, 'i');

    // Movimiento. La velocidad esta a proposito por encima de lo alcanzable
    // (ver Motors.h): el numero que de verdad cambia el ciclo y la vibracion
    // es la aceleracion, y por eso las dos van en proceso y la velocidad en
    // servicio.
    params.registrar("acc_rap",   &Motors::ACC_RAPIDA, 20000.0f, 150000.0f, "pas/s2", NIVEL_PROCESO, 'i');
    // El techo de la aceleracion de agarre es 300.000, muy por encima de los
    // 150.000 de 'acc_rap'. No es un numero de comodidad: es donde el tramo
    // de agarre DEJA DE SER TRIANGULAR con la aproximacion mas larga del
    // rango util (apr_dx = -4 cm, 477 pasos):
    //
    //     a_maxima = VEL_MAX^2 / recorrido = 144.000.000 / 477 = 302.000
    //
    // Por encima de eso el movimiento alcanza VEL_MAX y se activa el tramo
    // de crucero de Stepper::computeNextInterval(), que es el que Motors.h
    // avisa que tenia dos errores en el frenado y que en este robot casi no
    // se ejecuta. Mientras se quede abajo, la bajada usa la misma rampa de
    // siempre, solo que mas empinada.
    //
    // OJO que este techo es de SOFTWARE. Que el firmware acepte 300.000 no
    // quiere decir que los motores lo sigan: pasado cierto punto el eje se
    // queda atras y pierde pasos, en lazo abierto y sin aviso. Lo unico que
    // lo va a delatar es el CollisionGuard, que compara el angulo medido
    // contra el comandado. Si al subir esto empiezan a saltar colisiones en
    // la bajada, ese es el limite real de la mecanica y hay que volver.
    params.registrar("acc_agarre", &Motors::ACC_AGARRE, 5000.0f, 300000.0f, "pas/s2", NIVEL_PROCESO, 'i');
    params.registrar("acc_caja",   &Motors::ACC_CAJA,   5000.0f, 150000.0f, "pas/s2", NIVEL_PROCESO, 'i');

    // Cinta: el PWM es lo que se manda, cinta_cms es lo que se mide. Se
    // tocan de a dos (ver el comentario de CONVEYOR_PWM).
    params.registrar("cinta_pwm", &CONVEYOR_PWM, 0.0f, 100.0f, "%", NIVEL_PROCESO);

    // Como se siente el jog en la mano. Van en proceso porque son gusto del
    // operador, no seguridad: el volumen es lo que protege al robot.
    params.registrar("t_jog",    &TEACH_JOG_CMS, 0.5f, 15.0f,  "cm/s", NIVEL_PROCESO);
    params.registrar("t_jogpct", &TEACH_JOG_PCT, 5.0f, 60.0f,  "%",    NIVEL_PROCESO, 'i');

    // Los tres que deciden si la reproduccion se siente fluida o a los
    // tirones. Van en proceso porque se ajustan mirando el robot moverse.
    params.registrar("t_acel",    &TEACH_ACEL,        5000.0f, 100000.0f, "pas/s2", NIVEL_PROCESO, 'i');
    params.registrar("t_mezcla",  &TEACH_MEZCLA_CM,   0.0f,    3.0f,      "cm",     NIVEL_PROCESO);
    params.registrar("t_esquina", &TEACH_ESQUINA_DEG, 0.0f,    90.0f,     "deg",    NIVEL_PROCESO, 'i');

    params.registrar("movl_paso", &MOVL_PASO_CM, 0.01f, 5.0f, "cm",   NIVEL_PROCESO);
    params.registrar("movl_acel", &MOVL_ACEL, 5000.0f, 150000.0f, "pas/s2",
                     NIVEL_PROCESO, 'i');
    params.registrar("movl_vel",  &MOVL_VEL_CMS, 1.0f, 100.0f, "cm/s", NIVEL_PROCESO);

    params.registrar("pick_tol",  &PICK_LATE_TOLERANCE_MS, 0.0f, 500.0f, "ms", NIVEL_PROCESO, 'i');
    params.registrar("pump_lead", &PUMP_LEAD_MS,           0.0f, 2000.0f, "ms", NIVEL_PROCESO, 'i');
    params.registrar("replan",    &MAX_REPLAN_ATTEMPTS,    0.0f, 5.0f,   "",   NIVEL_PROCESO, 'i');
    params.registrar("diag_ms",   &DIAGNOSTICO_PERIODICO_MS, 0.0f, 120000.0f, "ms", NIVEL_PROCESO, 'i');

    params.registrar("tele_ms", &telemetria.periodoRapida_ms,  20.0f, 2000.0f,  "ms", NIVEL_PROCESO, 'i');
    params.registrar("est_ms",  &telemetria.periodoProceso_ms, 100.0f, 10000.0f, "ms", NIVEL_PROCESO, 'i');
    params.registrar("sal_ms",  &telemetria.periodoSalud_ms,   500.0f, 30000.0f, "ms", NIVEL_PROCESO, 'i');

    // --- Nivel 3: servicio ---
    // Los Z son los que aplastan cosas si se equivoca el signo o la coma,
    // y ademas no se pueden cambiar con una pieza en la mano: le cambiarian
    // el destino a una maniobra ya planificada.
    params.registrar("grab_z", &GRAB_Z, -36.0f, -28.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("box_z",  &BOX_Z,  -36.0f, -28.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("box_apr", &BOX_APPROACH_DZ, 0.2f, 8.0f,  "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("box_tr",  &BOX_TRANSIT_DZ,  3.0f, 12.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("box_rel", &BOX_RELEASE_DETACH_MS, 0.0f, 2000.0f, "ms", NIVEL_SERVICIO, 'i');

    params.registrar("bin_x1", &BIN_X[0], -20.0f, 20.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("bin_x2", &BIN_X[1], -20.0f, 20.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("bin_x3", &BIN_X[2], -20.0f, 20.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("bin_y",  &BIN_Y,    -20.0f, 0.0f,  "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("bin_z",  &BIN_Z,    -34.0f, -20.0f, "cm", NIVEL_SERVICIO, 'f', true);

    params.registrar("home_a1", &HOME_ANGLE_M1, -60.0f, -30.0f, "deg", NIVEL_SERVICIO);
    params.registrar("home_a2", &HOME_ANGLE_M2, -60.0f, -30.0f, "deg", NIVEL_SERVICIO);
    params.registrar("home_a3", &HOME_ANGLE_M3, -60.0f, -30.0f, "deg", NIVEL_SERVICIO);

    // Estos dos existen TAMBIEN del lado de Python (config.py). Si se
    // cambian de un solo lado, el robot le empieza a errar a las piezas sin
    // que nada avise: el nucleo compara los dos al conectarse.
    params.registrar("cinta_cms", &BELT_VELOCITY_CMS, 1.0f, 30.0f, "cm/s", NIVEL_SERVICIO);
    params.registrar("linea_x",   &DETECTION_LINE_X, -40.0f, 0.0f, "cm", NIVEL_SERVICIO);

    // La caja entera se corre con estos dos. La grilla de 6 cm entre celdas
    // es la geometria de la caja y no se toca; lo que se mueve es donde
    // quedo apoyada sobre la mesa.
    params.registrar("box_dx", &BOX_DX, -4.0f, 4.0f, "cm", NIVEL_SERVICIO, 'f', true);
    params.registrar("box_dy", &BOX_DY, -4.0f, 4.0f, "cm", NIVEL_SERVICIO, 'f', true);

    // El piso baja a 5000 porque el rango util quedo ABAJO del que tenia:
    // el pico real del movimiento mas largo son ~16.400 pasos/s, asi que
    // cualquier tope por encima de eso no limita nada. Los valores que
    // sirven para recortar los tramos largos van de 6.000 a 14.000.
    params.registrar("vel_max", &Motors::VEL_MAX, 5000.0f, 120000.0f, "pas/s", NIVEL_SERVICIO, 'i');

    // Volumen de trabajo del jog manual. Es lo unico que separa al operador
    // de meter el brazo contra la cinta o contra los tachos, asi que va en
    // servicio: se toca cuando se mueve algo de la mesa, no todos los dias.
    // El piso en Z no esta aca porque cuelga de 'grab_z' (ver TEACH_ZUP).
    params.registrar("t_xmin", &TEACH_XMIN, -20.0f, 0.0f,  "cm", NIVEL_SERVICIO);
    params.registrar("t_xmax", &TEACH_XMAX,   0.0f, 20.0f, "cm", NIVEL_SERVICIO);
    params.registrar("t_ymin", &TEACH_YMIN, -20.0f, 0.0f,  "cm", NIVEL_SERVICIO);
    params.registrar("t_ymax", &TEACH_YMAX,   0.0f, 20.0f, "cm", NIVEL_SERVICIO);
    params.registrar("t_zup",  &TEACH_ZUP,    1.0f, 12.0f, "cm", NIVEL_SERVICIO);

    params.registrar("home_set", &HOMING_SETTLE_WAIT_MS, 500.0f,  10000.0f, "ms", NIVEL_SERVICIO, 'i');
    params.registrar("home_to",  &HOMING_TIMEOUT_MS,     5000.0f, 60000.0f, "ms", NIVEL_SERVICIO, 'i');
    params.registrar("tapa_ms",  &CONFIRMACION_TAPA_MS,  2000.0f, 60000.0f, "ms", NIVEL_SERVICIO, 'i');
}

// ------------------------------------------------------------------
// Baja los parametros a los objetos que guardan una COPIA adentro en vez de
// leer la variable en cada uso. Son los unicos que no se enteran solos de un
// cambio; todo lo demas lee su float directamente en el punto de uso.
void Robot::sincronizarParametros()
{
    guard.setUmbral(pGuardUmbral);
    guard.setUmbralReposo(pGuardReposo);
    guard.setSaltoParada(pGuardSalto, pGuardSaltoPct / 100.0f);
    guard.setConfirmacion((uint32_t)pGuardConfirma);
    guard.setMargenVelocidad((uint32_t)pGuardMargen);
    guard.setRetardo((uint32_t)pGuardRetardo);

    Motors::aplicarLimites();

    // El PWM se reaplica solo si la cinta ESTA andando: mandarselo con la
    // cinta parada la arrancaria sola al tocar cualquier parametro, con el
    // robot posiblemente en ERROR y nadie mirando.
    if (conveyor.getSpeed() > 0)
    {
        conveyor.setSpeedPercent(CONVEYOR_PWM);
    }
}

// ============================================================
//  TELEMETRIA
// ============================================================

// Modo -> la letra con la que viaja por serie. Es la MISMA letra que se usa
// para pedirlo ('C', 'F', 'A'), asi que la interfaz no necesita una segunda
// tabla de traduccion para leer lo que informa el firmware.
//
// La 'A' del modo Box es historica: el modo se llamaba ALFAJORES. NO se
// cambio a 'B' a proposito, porque en este mismo protocolo 'B' ya significa
// AZUL --en la disposicion de la caja ('XBRGRBG') y en los contadores por
// color--, y darle un segundo significado seria peor que la letra heredada.
static char letraModo(Robot::SortMode m)
{
    switch (m)
    {
        case Robot::SORT_BY_COLOR:  return 'C';
        case Robot::SORT_BY_SHAPE:  return 'F';
        case Robot::SORT_BOX:       return 'A';
        default:                    return '?';
    }
}

void Robot::emitirTelemetria(uint32_t ahora)
{
    if (telemetria.tocaRapida(ahora))
    {
        TeleRapida d;

        d.t_ms   = ahora;
        d.estado = (uint8_t)state;
        d.bomba  = pneumatics.isActive();

        // Los finales se leen aca y no se guardan de la ultima vez que los
        // miro el homing: la idea es justamente poder verificar uno
        // empujandolo con el dedo y ver si se pinta en la pantalla.
        d.finales[0] = endstops.readMotor1();
        d.finales[1] = endstops.readMotor2();
        d.finales[2] = endstops.readMotor3();

        const long pasos[3] = {motor1.getPosition(),
                               motor2.getPosition(),
                               motor3.getPosition()};

        for (uint8_t i = 0; i < 3; i++)
        {
            d.ang[i] = encoders.leerGrados(i);
            d.cmd[i] = stepsToAngle(pasos[i]);
            d.err[i] = guard.errorActual(i);
            d.umb[i] = guard.umbralEfectivo(i);
            d.vel[i] = guard.velocidadCmd(i);
        }

        telemetria.emitirRapida(d);
    }

    if (telemetria.tocaProceso(ahora))
    {
        TeleProceso d;

        d.t_ms         = ahora;
        d.estado       = (uint8_t)state;
        d.estadoNombre = nombreEstado(state);

        d.modo          = letraModo(sortMode);
        d.modoPendiente = sortModePending ? letraModo(pendingSortMode) : '-';

        d.esperandoTapa = esperandoConfirmacion;
        d.tapaRestante_ms = 0;

        if (esperandoConfirmacion)
        {
            const uint32_t pasado = ahora - confirmacionPedida_ms;

            d.tapaRestante_ms = (pasado < (uint32_t)CONFIRMACION_TAPA_MS)
                                    ? (uint32_t)(CONFIRMACION_TAPA_MS - pasado)
                                    : 0;
        }

        d.cola              = queueCount;
        d.colaAntiguedad_ms = antiguedadCola();

        d.homed          = homed;
        d.guard          = (uint8_t)guard.estado();
        d.observando     = guard.observando();
        d.paradasActivas = supervisionHabilitada;

        const int pwm = conveyor.getSpeed();

        d.cinta    = (pwm > 0);
        d.cintaPwm = (uint8_t)((pwm * 100) / 255);

        d.calibrando = calibrando;
        d.reposo     = enReposo();

        // La caja solo tiene sentido en el modo que la usa: fuera de ahi se
        // manda '-' para que la interfaz no dibuje una grilla que no
        // corresponde a nada.
        const bool enCaja = esBox(sortMode);

        d.layout         = enCaja ? boxLayout : NULL;
        d.llenas         = enCaja ? boxFilled : NULL;
        d.celdaReservada = (currentCell == CELDA_NINGUNA) ? 0 : (uint8_t)(currentCell + 1);

        if (hayManiobraEnCurso())
        {
            d.piezaColor = currentPiece.color;
            d.piezaForma = currentPiece.shape;
            d.piezaY     = currentPiece.y;
            d.tacho      = (uint8_t)(currentBin + 1);
        }
        else
        {
            d.piezaColor = '-';
            d.piezaForma = '-';
            d.piezaY     = 0.0f;
            d.tacho      = 0;
        }

        // Teach. 'ti' vale 0 quieto y 1..tw reproduciendo, asi la interfaz
        // puede dibujar el avance sin depender de haber visto pasar el
        // [TEACH] run -- una linea suelta se puede perder en un reinicio,
        // una linea periodica no.
        d.teachPuntos = teachPuntos;
        d.teachIndice = teachReproduciendo ? (uint8_t)(teachIndice + 1) : 0;

        d.detectadas  = piezasDetectadas;
        d.depositadas = piezasDepositadas;
        d.descartadas = piezasDescartadas;
        d.fallos      = fallos.total();

        for (uint8_t i = 0; i < 3; i++)
        {
            d.porColor[i] = porColorOk[i];
            d.porForma[i] = porFormaOk[i];
        }

        telemetria.emitirProceso(d);
    }

    if (telemetria.tocaSalud(ahora))
    {
        TeleSalud d;

        d.t_ms      = ahora;
        d.uptime_s  = ahora / 1000UL;
        d.loopHz    = telemetria.loopHz();
        d.heapLibre = ESP.getFreeHeap();

        for (uint8_t i = 0; i < 3; i++)
        {
            TeleSalud::Eje &e = d.ejes[i];

            // El orden importa: un encoder fuera de rango tambien aparece
            // como caido, y lo primero es mas especifico (dice DONDE esta
            // el problema, no solo que lo hay).
            e.encoder = guard.fueraDeRango(i) ? "rango"
                        : (guard.sensorCaido(i) ? "caido" : "ok");

            e.ganancia   = guard.gananciaMedida(i);
            e.atraso_ms  = guard.atrasoMedido_ms(i);
            e.pico       = guard.picoDesdeHoming(i);
            e.picoReposo = guard.picoEnReposo(i);
            e.fuga       = guard.derivaAbsorbida(i);
            e.rawMin     = guard.rawMinimo(i);
            e.rawMax     = guard.rawMaximo(i);
            e.resincronizaciones = encoders.cuentaResincronizaciones(i);
        }

        telemetria.emitirSalud(d);
    }
}

// ------------------------------------------------------------------
uint32_t Robot::antiguedadCola() const
{
    if (queueCount == 0)
    {
        return 0;
    }

    // queueHead es la mas VIEJA (la cola es FIFO), que es tambien la mas
    // adelantada sobre la cinta: la que menos tiempo tiene antes de
    // pasarse de largo.
    return (uint32_t)(millis() - pieceQueue[queueHead].detectedAt_ms);
}

// ------------------------------------------------------------------
void Robot::contarDepositada(const Piece &p)
{
    piezasDepositadas++;

    switch (p.color)
    {
        case 'R': porColorOk[0]++; break;
        case 'G': porColorOk[1]++; break;
        case 'B': porColorOk[2]++; break;
        default:                   break;
    }

    switch (p.shape)
    {
        case 'S': porFormaOk[0]++; break;
        case 'H': porFormaOk[1]++; break;
        case 'C': porFormaOk[2]++; break;
        default:                   break;
    }
}


// ============================================================
//  MODO TEACH
// ============================================================
//
//  Lo que hace el firmware y lo que hace la interfaz esta repartido asi:
//
//    ESP32    recorta al volumen de trabajo, resuelve la cinematica, mueve
//             y encadena los puntos de una ruta ya cargada.
//    Python   dibuja, graba, guarda las secuencias con nombre y lleva la
//             cuenta de a que porcentaje se verifico cada una.
//
//  El corte esta ahi porque lo unico que no se puede delegar es lo que
//  protege al robot: el recorte del volumen y el chequeo de alcance tienen
//  que estar de este lado aunque la interfaz ya los haga.
//
//  Encadenar los puntos tambien es de este lado, y no del PC: si cada punto
//  esperara la confirmacion de llegada por serie, entre punto y punto se
//  meteria el ida y vuelta del enlace (100 ms de telemetria en el peor caso)
//  y la secuencia se reproduciria a los tirones. Asi el salto de uno al
//  siguiente es una vuelta de loop.
// ============================================================

Motors::MotionLimits Robot::limitesTeach(float escalaPct) const
{
    if (escalaPct < 1.0f)   escalaPct = 1.0f;
    if (escalaPct > 100.0f) escalaPct = 100.0f;

    Motors::MotionLimits limites;

    limites.maxSpeed        = Motors::VEL_MAX * (escalaPct / 100.0f);
    limites.maxAcceleration = TEACH_ACEL      * (escalaPct / 100.0f);

    return limites;
}

void Robot::teachRecortar(float &x, float &y, float &z) const
{
    // El piso cuelga de GRAB_Z: es la altura a la que se agarra una pieza
    // apoyada, o sea lo mas bajo que tiene sentido bajar sin tocar la cinta.
    const float zMin = GRAB_Z;
    const float zMax = GRAB_Z + TEACH_ZUP;

    if (x < TEACH_XMIN) x = TEACH_XMIN;
    if (x > TEACH_XMAX) x = TEACH_XMAX;
    if (y < TEACH_YMIN) y = TEACH_YMIN;
    if (y > TEACH_YMAX) y = TEACH_YMAX;
    if (z < zMin)       z = zMin;
    if (z > zMax)       z = zMax;
}

bool Robot::teachMover(float x, float y, float z, float escalaPct)
{
    return teachMover(x, y, z, limitesTeach(escalaPct));
}

bool Robot::teachMover(float x, float y, float z,
                       const Motors::MotionLimits &limites)
{
    teachRecortar(x, y, z);

    if (!goToPositionIK(x, y, z, limites))
    {
        return false; // sin solucion: no se movio nada
    }

    teachX = x;
    teachY = y;
    teachZ = z;

    return true;
}

// ------------------------------------------------------------------
bool Robot::teachEnHome() const
{
    // Home son los pasos (0,0,0): brazos horizontales. No hace falta
    // cinematica directa para saberlo, que es justamente por que el camino
    // seguro se define contra home y no contra una coordenada cartesiana.
    return labs(motor1.getPosition()) <= TEACH_HOME_TOL_PASOS &&
           labs(motor2.getPosition()) <= TEACH_HOME_TOL_PASOS &&
           labs(motor3.getPosition()) <= TEACH_HOME_TOL_PASOS;
}

bool Robot::teachIr(float x, float y, float z)
{
    teachRecortar(x, y, z);

    // Se resuelve la cinematica ANTES de mover nada: si el punto no tiene
    // solucion, no tiene sentido subir a home para despues no poder bajar.
    if (!DeltaKinematics::solveIK(x, y, z).success)
    {
        return false;
    }

    teachIrX = x;
    teachIrY = y;
    teachIrZ = z;

    jogVx = jogVy = jogVz = 0.0f;
    jogVigenteHasta_ms = 0;

    if (teachEnHome())
    {
        // Ya esta arriba del todo: la recta a cualquier punto del volumen
        // baja, y bajar no puede raspar nada.
        if (!teachMover(teachIrX, teachIrY, teachIrZ, Motors::FAST_LIMITS))
        {
            return false;
        }

        teachIrEtapa = 2;
    }
    else
    {
        Motors::moveSynchronized(motor1, motor2, motor3, 0, 0, 0,
                                 Motors::FAST_LIMITS);
        teachIrEtapa = 1;
    }

    Serial.print("[TEACH] ir x=");
    Serial.print(teachIrX, 2);
    Serial.print(" y=");
    Serial.print(teachIrY, 2);
    Serial.print(" z=");
    Serial.print(teachIrZ, 2);
    Serial.print(" home=");
    Serial.println(teachIrEtapa == 1 ? 1 : 0);

    return true;
}

void Robot::updateTeachIr()
{
    // Un tramo por vez, igual que el jog: recien cuando el brazo llego se
    // emite el siguiente. Encadenarlos sin frenar (redirigirSincronizado)
    // seria redondear la esquina de home, que es lo unico que este camino
    // no puede hacer -- el rodeo por arriba es todo el punto.
    if (!enPosicion())
    {
        return;
    }

    if (teachIrEtapa == 1)
    {
        // En home. Durante este tramo teachX/Y/Z quedaron en el punto de
        // donde salio (el firmware no tiene cinematica DIRECTA para saber
        // en cartesiano donde esta home), y se corrigen aca, al lanzar el
        // tramo que si termina en un punto conocido.
        if (!teachMover(teachIrX, teachIrY, teachIrZ, Motors::FAST_LIMITS))
        {
            teachIrEtapa = 0;
            Serial.println("[TEACH] err=ik");
            return;
        }

        teachIrEtapa = 2;
        return;
    }

    teachIrEtapa = 0;
    Serial.println("[TEACH] irfin");
}

// ------------------------------------------------------------------
bool Robot::atenderLineal()
{
    const MovimientoLineal::Estado est = lineal.actualizar();

    if (est == MovimientoLineal::Estado::EN_CURSO)
    {
        return false;
    }

    // Termine como termine, la posicion comandada de teach es el ultimo
    // punto que se alcanzo a emitir: si se corto a mitad de camino, el jog
    // tiene que seguir desde ahi y no desde el destino que no se cumplio.
    const MovimientoLineal::Punto p = lineal.comandado();

    teachX = p.x;
    teachY = p.y;
    teachZ = p.z;

    if (est == MovimientoLineal::Estado::SIN_SOLUCION)
    {
        Serial.println("[TEACH] err=ik");
        return true;
    }

    if (est == MovimientoLineal::Estado::TERMINADO)
    {
        // Las frenadas son el numero que explica un movimiento que se vio a
        // los tirones: son los tramos en los que un eje invirtio el sentido
        // y no se pudo encadenar.
        Serial.print("[TEACH] lfin tramos=");
        Serial.print(lineal.tramos());
        Serial.print(" frenadas=");
        Serial.print(lineal.frenadas());
        Serial.print(" paso=");
        Serial.println(lineal.pasoEfectivoCm(), 2);
    }

    return true;
}

// ------------------------------------------------------------------
bool Robot::entrarTeach()
{
    teachPedido = false;

    if (!homed)
    {
        Serial.println("[TEACH] err=sinhoming");
        return false;
    }

    // La cinta se para: en teach nadie va a levantar lo que traiga, y con el
    // brazo manejado a mano por encima de ella es una cosa menos moviendose.
    conveyor.stop();

    queueHead  = 0;
    queueCount = 0;

    pneumatics.release();
    pumpOn = false;

    // Punto de partida: centrado y en el techo del volumen. Se entra siempre
    // desde WAIT_PIECE (brazo quieto, en home y con las manos vacias), asi
    // que este es el unico movimiento del que hace falta saber de donde sale
    // -- y por eso el firmware no necesita cinematica DIRECTA.
    const float x = 0.0f;
    const float y = 0.0f;
    const float z = GRAB_Z + TEACH_ZUP;

    if (!teachMover(x, y, z, TEACH_JOG_PCT))
    {
        Serial.println("[TEACH] err=ik");

        // Se deshace la parada de arriba, pero sin arrancar la cinta si
        // hay una calibracion en curso.
        if (!calibrando)
        {
            conveyor.setSpeedPercent(CONVEYOR_PWM);
        }

        return false;
    }

    jogVx = jogVy = jogVz = 0.0f;
    jogVigenteHasta_ms = 0;
    jogUltimoPaso_ms   = millis();

    teachReproduciendo = false;
    teachLanzado       = false;
    teachEsperando     = false;
    teachIrEtapa       = 0;
    teachIndice        = 0;
    lineal.cancelar();

    teachOrigenX = teachX;
    teachOrigenY = teachY;
    teachOrigenZ = teachZ;

    moveIssued = false;
    state      = TEACH;

    Serial.println("[TEACH] on");
    teachInformar();

    return true;
}

void Robot::salirTeach()
{
    lineal.cancelar();

    teachReproduciendo = false;
    teachLanzado       = false;
    teachEsperando     = false;
    teachIrEtapa       = 0;
    jogVx = jogVy = jogVz = 0.0f;
    teachStream = false;

    pneumatics.release();
    pumpOn = false;
    guard.silenciar((uint32_t)BLANQUEO_NEUMATICA_MS);

    // Igual que al terminar el homing: salir de teach no puede poner en
    // marcha la cinta si lo que se esta haciendo es calibrar la vision.
    if (!calibrando)
    {
        conveyor.setSpeedPercent(CONVEYOR_PWM);
    }

    // Se vuelve por GO_HOME_IDLE y no directo a WAIT_PIECE: el brazo quedo
    // donde lo dejo el operador, y ese estado es justamente el que lo lleva
    // a home antes de volver a aceptar piezas.
    moveIssued = false;
    state      = GO_HOME_IDLE;

    Serial.println("[TEACH] off");
}

void Robot::teachAbortar(const char *motivo)
{
    // Corta lo que haya en curso: una reproduccion o un 'ir a'. Los dos son
    // movimientos que el operador no esta manejando en vivo, y el boton de
    // parar de la interfaz tiene que valer para los dos.
    if (!teachOcupado())
    {
        return;
    }

    lineal.cancelar();

    teachReproduciendo = false;
    teachLanzado       = false;
    teachEsperando     = false;
    teachIrEtapa       = 0;

    motor1.stop();
    motor2.stop();
    motor3.stop();

    Serial.print("[TEACH] abort motivo=");
    Serial.println(motivo);
}

// ------------------------------------------------------------------
void Robot::updateTeach()
{
    if (teachStream &&
        (uint32_t)(millis() - teachStreamUltimo_ms) >= TEACH_STREAM_MS)
    {
        teachStreamUltimo_ms = millis();

        Serial.print("[TEACH] p x=");
        Serial.print(teachX, 2);
        Serial.print(" y=");
        Serial.print(teachY, 2);
        Serial.print(" z=");
        Serial.print(teachZ, 2);
        Serial.print(" b=");
        Serial.println(pumpOn ? 1 : 0);
    }

    if (teachReproduciendo)
    {
        updateTeachPlayback();
        return;
    }

    if (lineal.enCurso())
    {
        atenderLineal();
        return;
    }

    if (teachIrEtapa != 0)
    {
        updateTeachIr();
        return;
    }

    const uint32_t ahora = millis();

    // La direccion vence sola. Es el hombre-muerto del jog: si la interfaz
    // deja de refrescarla -- navegador cerrado, enlace caido, la pestana que
    // pasa a segundo plano -- el brazo termina el tramo que tenia y se para,
    // en vez de seguir viaje hasta la pared del volumen.
    if (jogVigenteHasta_ms != 0 && (int32_t)(ahora - jogVigenteHasta_ms) >= 0)
    {
        jogVx = jogVy = jogVz = 0.0f;
        jogVigenteHasta_ms = 0;
    }

    if (fabsf(jogVx) + fabsf(jogVy) + fabsf(jogVz) < 0.02f)
    {
        return;
    }

    // Un tramo nuevo solo cuando el anterior termino. Es lo que mantiene al
    // brazo siempre yendo a un destino conocido: Stepper::moveTo() reinicia
    // la rampa desde cero, asi que reemitir un destino con el eje todavia
    // andando lo dejaria persiguiendo un objetivo que se le escapa, y al
    // soltar la tecla tendria por delante todo el atraso acumulado.
    if (!enPosicion())
    {
        return;
    }

    if ((uint32_t)(ahora - jogUltimoPaso_ms) < TEACH_JOG_TICK_MS)
    {
        return;
    }

    jogUltimoPaso_ms = ahora;

    // La velocidad pedida sale de recorrer un tramo por tick. Si el brazo no
    // llega a hacerlo en ese tiempo, el enPosicion() de arriba saltea ticks
    // y el jog se frena solo: el pedido nunca se convierte en atraso.
    const float paso = TEACH_JOG_CMS * (TEACH_JOG_TICK_MS / 1000.0f);

    const float x = teachX + jogVx * paso;
    const float y = teachY + jogVy * paso;
    const float z = teachZ + jogVz * paso;

    // Un destino sin solucion (una esquina del cajon a la que el brazo no
    // llega) simplemente no se toma: el jog se queda donde estaba en vez de
    // trabarse, y se nota porque el brazo deja de avanzar para ese lado.
    teachMover(x, y, z, TEACH_JOG_PCT);
}

// ------------------------------------------------------------------
void Robot::updateTeachPlayback()
{
    const uint32_t ahora = millis();

    if (teachEsperando)
    {
        if ((int32_t)(ahora - teachEsperaHasta_ms) < 0)
        {
            return;
        }

        teachEsperando = false;
        teachIndice++;
        teachLanzado = false;
    }

    if (teachIndice >= teachPuntos)
    {
        teachReproduciendo = false;
        teachLanzado       = false;

        Serial.println("[TEACH] fin");
        return;
    }

    const TeachPunto &p = teachRuta[teachIndice];

    if (!teachLanzado)
    {
        teachOrigenX = teachX;
        teachOrigenY = teachY;
        teachOrigenZ = teachZ;

        if (!teachMover(p.x, p.y, p.z, teachEscala))
        {
            // La ruta se cargo ya recortada al volumen, asi que esto
            // significa que la geometria cambio desde que se grabo (otra
            // altura de agarre, por ejemplo). Se corta: saltear puntos
            // convertiria la secuencia en otra distinta, que es justo lo que
            // la verificacion por etapas trata de evitar.
            teachAbortar("ik");
            return;
        }

        teachTramoCm = sqrtf(sq(p.x - teachOrigenX) +
                             sq(p.y - teachOrigenY) +
                             sq(p.z - teachOrigenZ));

        teachTramoPasos = max(motor1.pasosRestantes(),
                              max(motor2.pasosRestantes(),
                                  motor3.pasosRestantes()));

        teachLanzado = true;
        return;
    }

    // --- Mezcla de la esquina ---
    // A TEACH_MEZCLA_CM del punto se redirige al siguiente sin frenar: el
    // brazo lo pasa cerca en vez de clavarse en el. Es lo que convierte una
    // sucesion de tramos en un movimiento solo.
    if (TEACH_MEZCLA_CM > 0.0f && teachTramoCm > 0.001f &&
        teachIndice + 1 < teachPuntos && teachMezclable(teachIndice))
    {
        const long restante = max(motor1.pasosRestantes(),
                                  max(motor2.pasosRestantes(),
                                      motor3.pasosRestantes()));

        // El radio en pasos sale de la regla de tres del tramo: los pasos y
        // los centimetros son proporcionales dentro de un mismo tramo. Se
        // topea en la mitad para no empezar a mezclar apenas arranca.
        long radio = (long)(teachTramoPasos * (TEACH_MEZCLA_CM / teachTramoCm));

        if (radio > teachTramoPasos / 2) radio = teachTramoPasos / 2;

        if (restante > 0 && restante <= radio && teachMezclar(teachIndice + 1))
        {
            teachIndice++;
            return;
        }
    }

    if (!enPosicion())
    {
        return;
    }

    // Llego al punto. La bomba se aplica AL LLEGAR y no antes: lo que se
    // grabo fue el estado del vacio en ese lugar.
    if (p.bomba != pumpOn)
    {
        if (p.bomba)
        {
            pneumatics.grab();
        }
        else
        {
            pneumatics.release();
        }

        pumpOn = p.bomba;

        // Misma ventana que en el ciclo normal: la bomba hunde el riel que
        // alimenta los encoders y corre las tres lecturas de golpe.
        guard.silenciar((uint32_t)BLANQUEO_NEUMATICA_MS);
    }

    // La espera NO se escala con el porcentaje. El porcentaje es velocidad y
    // aceleracion; una espera es el tiempo que tarda el vacio en formarse o
    // la pieza en despegarse, y eso no cambia porque el brazo vaya mas
    // rapido entre punto y punto.
    if (p.espera_ms > 0)
    {
        teachEsperando      = true;
        teachEsperaHasta_ms = ahora + p.espera_ms;
        return;
    }

    teachIndice++;
    teachLanzado = false;
}

// ------------------------------------------------------------------
bool Robot::teachMezclable(uint8_t k) const
{
    if (k + 1 >= teachPuntos)
    {
        return false; // el ultimo punto se cumple exacto, siempre
    }

    const TeachPunto &p = teachRuta[k];

    // Un punto con espera o con cambio de bomba es un punto que SIGNIFICA
    // algo: ahi se agarra o se suelta una pieza. Pasar cerca en vez de por
    // el lugar exacto seria soltar la pieza en otro lado.
    if (p.espera_ms > 0 || p.bomba != pumpOn)
    {
        return false;
    }

    // Angulo de la esquina, en cartesiano. Con el tramo que viene el brazo y
    // el que sigue casi alineados, mezclar cuesta poco y se gana todo. Con
    // una esquina cerrada, en cambio, el tiron lo produce la esquina en si
    // -- y si algun eje tiene que invertir el sentido, no hay forma de
    // hacerlo sin pasar por velocidad cero.
    const float ax = p.x - teachOrigenX;
    const float ay = p.y - teachOrigenY;
    const float az = p.z - teachOrigenZ;

    const TeachPunto &q = teachRuta[k + 1];

    const float bx = q.x - p.x;
    const float by = q.y - p.y;
    const float bz = q.z - p.z;

    const float na = sqrtf(ax * ax + ay * ay + az * az);
    const float nb = sqrtf(bx * bx + by * by + bz * bz);

    if (na < 0.001f || nb < 0.001f)
    {
        return false;
    }

    const float coseno = (ax * bx + ay * by + az * bz) / (na * nb);

    return coseno >= cosf(TEACH_ESQUINA_DEG * DEG_TO_RAD);
}

bool Robot::teachMezclar(uint8_t siguiente)
{
    const TeachPunto &q = teachRuta[siguiente];

    const DeltaKinematics::DeltaAngles pose =
        DeltaKinematics::solveIK(q.x, q.y, q.z);

    if (!pose.success)
    {
        return false;
    }

    const Motors::MotionLimits limites = limitesTeach(teachEscala);

    // El tramo nuevo arranca donde esta el brazo AHORA, no en el punto que
    // se acaba de saltear: es justamente el atajo de la esquina.
    const long pasos = max(labs(pose.steps1 - motor1.getPosition()),
                           max(labs(pose.steps2 - motor2.getPosition()),
                               labs(pose.steps3 - motor3.getPosition())));

    if (!Motors::redirigirSincronizado(motor1, motor2, motor3,
                                       pose.steps1, pose.steps2, pose.steps3,
                                       limites))
    {
        return false;
    }

    // El origen del tramo nuevo es el punto que se paso de largo: es contra
    // el que hay que medir la esquina siguiente, y el error de tomarlo por
    // la posicion real es como mucho el radio de mezcla.
    const TeachPunto &p = teachRuta[siguiente - 1];

    teachOrigenX = p.x;
    teachOrigenY = p.y;
    teachOrigenZ = p.z;

    teachTramoCm = sqrtf(sq(q.x - p.x) + sq(q.y - p.y) + sq(q.z - p.z));
    teachTramoPasos = pasos;

    teachX = q.x;
    teachY = q.y;
    teachZ = q.z;

    return true;
}

// ------------------------------------------------------------------
void Robot::teachInformar() const
{
    Serial.print("[TEACH] est=");
    Serial.print(state == TEACH ? "on" : (teachPedido ? "pedido" : "off"));
    Serial.print(" n=");
    Serial.print(teachPuntos);
    Serial.print(" i=");
    Serial.print(teachReproduciendo ? (uint16_t)(teachIndice + 1) : 0);
    Serial.print(" pct=");
    Serial.print(teachEscala, 0);
    Serial.print(" x=");
    Serial.print(teachX, 2);
    Serial.print(" y=");
    Serial.print(teachY, 2);
    Serial.print(" z=");
    Serial.print(teachZ, 2);
    Serial.print(" xmin=");
    Serial.print(TEACH_XMIN, 2);
    Serial.print(" xmax=");
    Serial.print(TEACH_XMAX, 2);
    Serial.print(" ymin=");
    Serial.print(TEACH_YMIN, 2);
    Serial.print(" ymax=");
    Serial.print(TEACH_YMAX, 2);
    Serial.print(" zmin=");
    Serial.print(GRAB_Z, 2);
    Serial.print(" zmax=");
    Serial.print(GRAB_Z + TEACH_ZUP, 2);
    Serial.print(" cap=");
    Serial.println(TEACH_MAX_PUNTOS);
}

// ------------------------------------------------------------------
//  Comandos 'J...'
// ------------------------------------------------------------------
//
//    J1                      pedir entrada a teach (entra al llegar a home)
//    J0                      salir
//    J?                      informar estado y volumen de trabajo
//    JM<x>,<y>,<z>           mover la punta a un destino absoluto, en cm
//    JI<x>,<y>,<z>           ir a un punto pasando por home, a maxima
//    JL<x>,<y>,<z>           ir en LINEA RECTA hasta el punto (movL)
//    JD<vx>,<vy>,<vz>        direccion de jog, cada una en [-1, 1]
//    JP1 / JP0               bomba de vacio
//    JC                      vaciar la ruta cargada
//    JA<x>,<y>,<z>,<b>,<w>   agregar un punto (b = bomba 0/1, w = espera ms)
//    JR<pct>                 reproducir la ruta al <pct> % de vel y acel
//    JX                      cortar la reproduccion
//    JG1 / JG0               volcado de la posicion comandada a 20 Hz
//
bool Robot::procesarComandoTeach(const char *cmd)
{
    if (toupper(cmd[0]) != 'J')
    {
        return false;
    }

    const char  sub    = toupper(cmd[1]);
    const char *resto  = cmd + 2;
    const bool  enModo = (state == TEACH);

    // --- Entrar / salir / informar ---
    if (sub == '1' && resto[0] == '\0')
    {
        if (enModo)
        {
            teachInformar();
            return true;
        }

        if (!homed)
        {
            Serial.println("[TEACH] err=sinhoming");
            return true;
        }

        // Desde ERROR o IDLE el robot no llega a WAIT_PIECE por su cuenta, y
        // ese es el unico punto por el que se entra a teach: el pedido
        // quedaria esperando para siempre y la interfaz mostrando "entrando".
        // Ademas, despues de un paro manual la posicion real dejo de
        // corresponderse con los pasos -- hay que rehomear antes de mover
        // nada a mano.
        if (state == ERROR || state == IDLE)
        {
            Serial.println("[TEACH] err=rehomear");
            return true;
        }

        // No se entra en el acto: primero hay que terminar la pieza que este
        // en vuelo y volver a home. Puede tardar un ciclo, y la interfaz
        // muestra "entrando" hasta que llega el estado TEACH.
        teachPedido = true;
        Serial.println("[TEACH] pedido");
        return true;
    }

    if (sub == '0' && resto[0] == '\0')
    {
        if (enModo)
        {
            salirTeach();
        }
        else
        {
            teachPedido = false;
            Serial.println("[TEACH] off");
        }

        return true;
    }

    if (sub == '?' && resto[0] == '\0')
    {
        teachInformar();
        return true;
    }

    // --- Carga de la ruta ---
    // Se acepta con el robot fuera de teach a proposito: asi la interfaz
    // puede ir subiendo la secuencia mientras el brazo todavia esta
    // terminando la pieza que tenia, y reproducirla apenas entra.
    if (sub == 'C' && resto[0] == '\0')
    {
        teachAbortar("carga");
        teachPuntos = 0;
        Serial.println("[TEACH] buf n=0");
        return true;
    }

    if (sub == 'A')
    {
        float v[5];

        if (leerFloats(resto, v, 5) != 5)
        {
            Serial.println("[TEACH] err=formato");
            return true;
        }

        if (teachPuntos >= TEACH_MAX_PUNTOS)
        {
            Serial.println("[TEACH] err=lleno");
            return true;
        }

        TeachPunto &p = teachRuta[teachPuntos];

        p.x = v[0];
        p.y = v[1];
        p.z = v[2];

        // Se recorta AL CARGAR y no solo al ejecutar: asi lo que se
        // reproduce es exactamente lo mismo en todas las pasadas, y no una
        // version recortada distinta cada vez.
        teachRecortar(p.x, p.y, p.z);

        p.bomba     = (v[3] != 0.0f);
        p.espera_ms = (v[4] < 0.0f) ? 0
                    : ((v[4] > 60000.0f) ? 60000 : (uint16_t)v[4]);

        teachPuntos++;

        Serial.print("[TEACH] buf n=");
        Serial.println(teachPuntos);
        return true;
    }

    if (sub == 'R')
    {
        if (!enModo)
        {
            Serial.println("[TEACH] err=nomodo");
            return true;
        }

        if (teachPuntos == 0)
        {
            Serial.println("[TEACH] err=vacio");
            return true;
        }

        if (teachIrEtapa != 0)
        {
            Serial.println("[TEACH] err=ocupado");
            return true;
        }

        float pct = 0.0f;

        if (leerFloats(resto, &pct, 1) != 1)
        {
            Serial.println("[TEACH] err=formato");
            return true;
        }

        if (pct < 1.0f)   pct = 1.0f;
        if (pct > 100.0f) pct = 100.0f;

        teachEscala        = pct;
        teachIndice        = 0;
        teachLanzado       = false;
        teachEsperando     = false;
        teachReproduciendo = true;

        jogVx = jogVy = jogVz = 0.0f;
        jogVigenteHasta_ms = 0;

        Serial.print("[TEACH] run pct=");
        Serial.print(teachEscala, 0);
        Serial.print(" n=");
        Serial.println(teachPuntos);
        return true;
    }

    if (sub == 'X' && resto[0] == '\0')
    {
        teachAbortar("manual");
        return true;
    }

    if (sub == 'G')
    {
        if ((resto[0] != '0' && resto[0] != '1') || resto[1] != '\0')
        {
            Serial.println("[TEACH] err=formato");
            return true;
        }

        teachStream          = (resto[0] == '1');
        teachStreamUltimo_ms = millis();
        return true;
    }

    // --- Todo lo que sigue mueve el brazo: solo dentro del modo ---
    if (!enModo)
    {
        Serial.println("[TEACH] err=nomodo");
        return true;
    }

    if (sub == 'P')
    {
        if ((resto[0] != '0' && resto[0] != '1') || resto[1] != '\0')
        {
            Serial.println("[TEACH] err=formato");
            return true;
        }

        // La bomba se puede tocar durante la reproduccion sin romper nada:
        // el proximo punto que traiga un estado distinto la vuelve a poner
        // donde corresponde.
        const bool encender = (resto[0] == '1');

        if (encender)
        {
            pneumatics.grab();
        }
        else
        {
            pneumatics.release();
        }

        pumpOn = encender;
        guard.silenciar((uint32_t)BLANQUEO_NEUMATICA_MS);

        return true;
    }

    if (sub == 'D')
    {
        float v[3];

        if (leerFloats(resto, v, 3) != 3)
        {
            Serial.println("[TEACH] err=formato");
            return true;
        }

        for (uint8_t i = 0; i < 3; i++)
        {
            if (v[i] < -1.0f) v[i] = -1.0f;
            if (v[i] >  1.0f) v[i] =  1.0f;
        }

        // Un jog con una reproduccion en curso serian dos cosas peleando por
        // el mismo brazo. El vector NULO es la excepcion, y no es un caso de
        // borde: la interfaz lo manda al arrancar la reproduccion, como
        // forma de decir "solte todo". Rechazarlo devolvia un err=ocupado
        // que no significaba nada.
        if (teachOcupado())
        {
            // Con una reproduccion en curso el rechazo se avisa; con un
            // 'ir a', no. La diferencia es que el 'ir a' dura un segundo y
            // el operador puede tener la tecla apretada de antes, y avisar
            // ahi son diez lineas de err por nada.
            if (teachReproduciendo &&
                fabsf(v[0]) + fabsf(v[1]) + fabsf(v[2]) > 0.02f)
            {
                Serial.println("[TEACH] err=ocupado");
            }

            return true;
        }

        jogVx = v[0];
        jogVy = v[1];
        jogVz = v[2];

        // Solo una direccion viva refresca el hombre-muerto. El vector nulo
        // es "solte la tecla" y tiene que frenar ya, no dentro de 350 ms.
        jogVigenteHasta_ms = (fabsf(jogVx) + fabsf(jogVy) + fabsf(jogVz) > 0.02f)
                                 ? (millis() + TEACH_JOG_VIDA_MS)
                                 : 0;
        return true;
    }

    if (sub == 'L')
    {
        if (teachOcupado())
        {
            Serial.println("[TEACH] err=ocupado");
            return true;
        }

        float v[3];

        if (leerFloats(resto, v, 3) != 3)
        {
            Serial.println("[TEACH] err=formato");
            return true;
        }

        teachRecortar(v[0], v[1], v[2]);

        jogVx = jogVy = jogVz = 0.0f;
        jogVigenteHasta_ms = 0;

        MovimientoLineal::Config cfg;

        cfg.pasoCm = MOVL_PASO_CM;
        cfg.velCms = MOVL_VEL_CMS;
        cfg.acel   = MOVL_ACEL;

        // El origen es la posicion COMANDADA, no la medida: el firmware no
        // tiene cinematica directa, y de todas formas la comandada es la
        // buena -- el lazo es abierto y el encoder solo supervisa.
        if (!lineal.comenzar({ teachX, teachY, teachZ },
                             { v[0], v[1], v[2] }, cfg))
        {
            // Puede ser que la RECTA se salga del alcance aunque las dos
            // puntas esten adentro: el volumen de un delta no es convexo.
            Serial.println("[TEACH] err=ik");
            return true;
        }

        Serial.print("[TEACH] l x=");
        Serial.print(v[0], 2);
        Serial.print(" y=");
        Serial.print(v[1], 2);
        Serial.print(" z=");
        Serial.print(v[2], 2);
        Serial.print(" paso=");
        Serial.print(lineal.pasoEfectivoCm(), 2);
        Serial.print(" vel=");
        Serial.println(MOVL_VEL_CMS, 1);

        return true;
    }

    if (sub == 'I')
    {
        if (teachOcupado())
        {
            Serial.println("[TEACH] err=ocupado");
            return true;
        }

        float v[3];

        if (leerFloats(resto, v, 3) != 3)
        {
            Serial.println("[TEACH] err=formato");
            return true;
        }

        if (!teachIr(v[0], v[1], v[2]))
        {
            Serial.println("[TEACH] err=ik");
        }

        return true;
    }

    if (sub == 'M')
    {
        if (teachOcupado())
        {
            Serial.println("[TEACH] err=ocupado");
            return true;
        }

        float v[3];

        if (leerFloats(resto, v, 3) != 3)
        {
            Serial.println("[TEACH] err=formato");
            return true;
        }

        jogVx = jogVy = jogVz = 0.0f;
        jogVigenteHasta_ms = 0;

        if (!teachMover(v[0], v[1], v[2], TEACH_JOG_PCT))
        {
            Serial.println("[TEACH] err=ik");
        }

        return true;
    }

    Serial.print("[SERIAL] comando de teach invalido: '");
    Serial.print(cmd);
    Serial.println("'. Validos: J1 J0 J? JM JI JL JD JP JC JA JR JX JG");

    return true;
}

// ============================================================
//  EMERGENCIA
// ============================================================

void Robot::emergencyStop()
{
    motor1.stop();
    motor2.stop();
    motor3.stop();

    conveyor.stop();

    // Con el robot frenado a mitad de camino, la posicion real deja de
    // corresponderse con los pasos comandados: no tiene sentido seguir
    // comparando hasta que se rehomee.
    guard.desarmar();

    registrarFallo(FALLO_MANUAL, 0);

    teachPedido        = false;
    teachReproduciendo = false;
    teachLanzado       = false;
    teachEsperando     = false;
    teachIrEtapa       = 0;
    teachStream        = false;
    lineal.cancelar();
    jogVx = jogVy = jogVz = 0.0f;

    Serial.println("[EMERGENCIA] Parada manual. Presiona 'R' para rehomear.");

    moveIssued = false;
    state = ERROR;
}
