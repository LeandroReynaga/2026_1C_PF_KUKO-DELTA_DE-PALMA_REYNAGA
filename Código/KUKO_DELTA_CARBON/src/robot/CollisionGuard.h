#ifndef COLLISION_GUARD_H
#define COLLISION_GUARD_H

#include <Arduino.h>

// ============================================================
//  PARAMETROS DE LA SUPERVISION POR ENCODERS
//  Unico bloque a tocar para calibrar la deteccion de colisiones.
// ============================================================
namespace GuardConfig {

// El umbral tiene DOS partes que se suman:
//
//     umbral_efectivo = UMBRAL_DEG + MARGEN_VELOCIDAD_MS * velocidad
//
// UMBRAL_DEG es la parte fija, la que manda con el brazo quieto o lento.
// Cubre el ruido propio del encoder (~1 grado), la no linealidad de la
// salida analogica del AS5600 y el error de la referencia.
//
// La parte que se suma existe porque la medicion del encoder llega SIEMPRE
// atrasada respecto de los pasos: el AS5600 tiene su propio filtro interno
// (~2 ms), mas el RC del capacitor de la placa, mas el tiempo entre lecturas
// del loop. Ese atraso es un TIEMPO, asi que el error que provoca no es una
// cantidad fija de grados: es proporcional a la velocidad. A 400 grados/s,
// 60 ms de atraso son 24 grados de diferencia con el brazo andando
// perfectamente bien. Por eso subir el umbral fijo hasta taparlo (30, 40
// grados) no sirve: deja de detectar cualquier cosa cuando el robot esta
// quieto, que es justo cuando la deteccion es mas facil y mas confiable.
//
// Expresado como tiempo, en cambio, el margen se acomoda solo a cada
// movimiento, y se desarma apenas el brazo frena.
constexpr float UMBRAL_DEG = 12.0f;

// Atraso total que se le tolera a la medicion, en ms. Es el numero a subir
// si siguen apareciendo falsos positivos en los movimientos rapidos, y el
// que hay que bajar para detectar mas rapido. Tambien fija cuanto tarda en
// detectarse una colision a maxima velocidad: el error de un brazo trabado
// crece a la velocidad comandada, asi que pasa el margen en ~este tiempo.
//
// Arrancaba en 120 ms, elegido a ojo por ser "mayor que el atraso real".
// Medido despues sobre el robot -- traza a 20 Hz durante ciclos completos,
// mirando la magnitud que el detector realmente evalua, o sea el error YA
// sin el modo comun -- resulto muy sobredimensionado:
//
//                    |err| max    |err - comun| max    margen necesario
//     eje 1             24,9              13,9                7 ms
//     eje 2             25,6              10,3            no hizo falta
//     eje 3             30,9              37,1              181 ms
//
// La primera columna es lo que asusta, pero no es lo que se compara: los
// tres ejes se mueven juntos y ese error comun se cancela con la mediana.
// Lo que queda es chico en los ejes 1 y 2.
//
// Y lo que decide un falso positivo no es el pico sino cuanto se SOSTIENE.
// Barriendo el registro con umbrales de 12 a 6 grados y margenes de 120 a
// 40 ms, en ningun caso hubo dos muestras seguidas por encima del umbral:
// solo 3 cruces aislados, todos del eje 3, que los 60 ms de confirmacion
// descartan. O sea que habia margen de sobra para bajarlo.
//
// Se baja a la mitad y no mas, para dejar factor 1,5 sobre los 40 ms que
// todavia daban cero cruces sostenidos. Validado en el robot con 'K60': 7
// ciclos completos, cero falsos positivos.
//
// Efecto: a 280 grados/s el umbral efectivo pasa de 12+34=46 grados a
// 12+17=29. Las colisiones medianas, que antes se colaban enteras por
// debajo, ahora entran.
//
// El eje 3 es el que manda en este numero: se desvia del modo comun mucho
// mas que los otros dos (37 grados contra 14 y 10) y su "atraso medido" da
// con el signo cambiado. Vale la pena mirarle el iman y el filtro RC; si se
// le encuentra la vuelta, este margen puede bajar bastante mas.
//
// En caliente se cambia con 'K60' por Serial.
constexpr uint32_t MARGEN_VELOCIDAD_MS = 60;

// El margen no sigue a la velocidad instantanea sino a un PICO que decae.
// Al terminar un movimiento la velocidad comandada se va a cero, pero el
// error NO se va con ella: el encoder sigue llegando tarde y tarda otro
// tanto en ponerse al dia. Si el margen se desarmara junto con la
// velocidad, ese error de frenado se leeria como colision -- y eso pasa
// EN CADA TRAMO, no de vez en cuando.
//
// Por eso el pico se sostiene y despues baja despacio, con esta constante
// de tiempo. Tiene que ser mayor que el atraso real de la medicion; 250 ms
// deja margen de sobra sobre los ~50-100 ms esperables.
//
// (Antes esto era una ventana fija que arrancaba en el instante del pico.
// Con un perfil triangular el pico cae a MITAD del tramo, asi que la
// ventana se vencia apenas frenaba el brazo: el margen desaparecia justo
// en el peor momento.)
constexpr uint32_t DECAIMIENTO_MARGEN_MS = 250;

// Segunda defensa, independiente del atraso: un error que esta BAJANDO no
// es una colision. Cuando el sensor se pone al dia despues de frenar, el
// error decae solo; un brazo trabado, en cambio, sostiene el error (ni el
// brazo ni los pasos se mueven, la diferencia queda congelada).
//
// Si al cumplirse el tiempo de confirmacion el error bajo a menos de esta
// fraccion de su maximo, se cancela la deteccion y se vuelve a empezar.
// 0,85 tolera decaimientos de hasta ~370 ms de constante de tiempo sin
// dejar de distinguirlos de un error plano.
constexpr float CANCELA_SI_BAJA_A = 0.85f;

// Tercera defensa: fuga lenta de la referencia con el eje EN REPOSO.
// La comparacion es contra el homing, asi que cualquier error chico que
// quede despues de cada tramo se va sumando: despues de decenas de ciclos
// el acumulado llega al umbral sin que haya pasado nada. Estando quieto, la
// referencia se corre despacio hacia lo que mide el encoder, con esta
// constante de tiempo, y ese acumulado se borra solo.
//
// Es lo que mantiene esto como detector de COLISIONES y no como monitor de
// posicion absoluta (que es lo que se pidio: la posicion la resuelven los
// micropasos). Una colision se desarrolla en decenas de ms; a 3 s de
// constante, la fuga apenas alcanza a comerse decimas de grado en ese
// tiempo, asi que no tapa nada de lo que hay que detectar.
//
// 0 = sin fuga.
constexpr float FUGA_REPOSO_SEG = 3.0f;

// Poner en false para que la supervision mida y avise por Serial pero NO
// frene el robot (mismo efecto que el comando 'G', pero desde el codigo,
// para cuando el puerto serie lo esta usando la vision y no se le pueden
// mandar comandos a mano).
constexpr bool FRENAR_POR_COLISION = true;

// El error tiene que MANTENERSE pasado del umbral durante este tiempo para
// que se declare colision. Una lectura aislada fuera de rango (ruido, una
// muestra que el filtro de plausibilidad rechazo, una vibracion al frenar)
// no frena nada: si vuelve a entrar en el margen, la deteccion se cancela y
// el contador arranca de cero la proxima vez.
//
// En caliente se cambia con 'T80' por Serial.
constexpr uint32_t CONFIRMACION_MS = 60;

// ============================================================
//  CHEQUEO EN REPOSO (el robot quieto en home)
// ============================================================
// El umbral de arriba tiene que aguantar el peor caso: brazo a maxima
// velocidad, con el encoder llegando tarde y el ruido de la rampa encima.
// Por eso es grande, y por eso una colision que no llega a ser brutal
// puede pasar desapercibida.
//
// Pero cuando el robot esta QUIETO EN HOME esperando piezas, ninguna de
// esas fuentes de error existe: no hay atraso de medicion (no se mueve
// nada), no hay rampa, no hay vibracion. Lo unico que queda es el ruido
// propio del encoder (~1 grado) y lo que le erre la linealidad del AS5600
// entre esta pose y la del homing, que se puede leer en la ganancia (con
// gan=0,98 sobre los 45 grados que separan home del endstop, es 1 grado).
//
// O sea que ahi se puede exigir MUCHO mas: si con el brazo frenado en la
// misma pose de siempre el encoder marca 5 grados de mas, no es ruido ni
// atraso -- se perdieron pasos, o alguien movio el brazo, o se aflojo una
// polea. Nada de eso lo ve el umbral de 12 grados, y todo eso arruina la
// precision del ciclo siguiente sin avisar.
//
// Como no hay ninguna urgencia (el robot esta parado), la confirmacion se
// puede hacer LARGA: 1,5 segundos de error sostenido no lo produce ningun
// ruido, y elimina de raiz los falsos positivos.
//
// LO QUE NO SE PUEDE HACER es comparar el error instantaneo contra un
// umbral chico. Medido sobre este robot, con el brazo frenado en home
// (traza a 20 Hz, 42 s):
//
//                 sigma    pico instantaneo    pico promediando 1 s
//     eje 1       1,22          4,00                  0,81
//     eje 2       1,30          5,62                  0,77
//     eje 3       1,16          5,08                  0,77
//
// O sea que el encoder tira picos sueltos de hasta 5 grados estando todo
// perfecto -- por eso el umbral en marcha es de 12. Pero ese ruido es
// RAPIDO y de media cero: promediado un segundo se cae a 0,8 grados. Una
// descalibracion, en cambio, es un corrimiento que no se va, y atraviesa el
// promedio sin perder nada.
//
// Por eso el chequeo compara el desvio PROMEDIADO (constante de tiempo
// TAU_REPOSO_S) y no el instantaneo.
//
// Aun asi queda un resto que no es ruido: promediado, el desvio en home da
// 1,5 a 2,5 grados segun el eje, siempre el mismo y siempre para el mismo
// lado. Es ERROR DE GANANCIA del encoder, no descalibracion: con gan=0,94
// sobre los 45 grados que separan home del endstop salen justo 2,7. Es una
// propiedad fija de la cadena de medicion, no algo que haya que detectar.
//
// Por eso el desvio se mide contra una FIRMA: al quedarse quieto se anota
// cuanto marca, y a partir de ahi se compara contra eso. Lo sistematico se
// cancela solo y el umbral queda midiendo unicamente lo que CAMBIA.
//
// Y LA FIRMA SE REHACE EN CADA PARADA, no una vez por homing. Esto es lo
// mas importante de todo el chequeo y se descubrio midiendo: contra la
// referencia del homing el desvio en reposo CRECE COMO UN GRADO POR CICLO,
// y despues de 6 u 8 piezas se pasa de cualquier umbral razonable. Medido
// con 8 agarres seguidos: la fuga tuvo que absorber 6,4 / 7,2 / 4,0 grados,
// y el chequeo disparo dos veces (-7,2 y -9,0 grados) con el robot
// funcionando perfecto.
//
// Eso NO es perdida de pasos: 9 grados de eje son 2,8 cm en la punta del
// gripper, y con semejante corrimiento el robot no habria agarrado una sola
// pieza -- agarro las 8, sin perder una ventana. Lo que se corre es la
// MEDICION: el angulo continuo del encoder no cierra exactamente cada
// excursion de 45 grados (se ve en la ganancia, 0,92 a 0,98), y esa
// diferencia se acumula vuelta a vuelta. Para eso existe la fuga.
//
// Rehaciendo la firma en cada parada, el chequeo mide solo lo que pasa
// MIENTRAS el robot esta quieto -- un empujon, alguien que se apoya, el
// brazo que se cae por su peso si algo se aflojo -- y es inmune a esa
// acumulacion. Lo que se descalibre durante un movimiento es problema del
// umbral en marcha, no de este.
//
// Con eso el piso queda en el ruido del promedio. Medido sobre el robot,
// "pico_reposo" durante produccion normal:
//
//     tipico          0,3 a 1,1 grados
//     picos           hasta 4,0 (aparecen despues de un ciclo, no en el
//                     reposo largo, donde se queda clavado en 0,3-0,5)
//
// Por eso el umbral va en 6 y no en 4: con 4 el margen contra el peor caso
// medido era del 0% y lo unico que evitaba el disparo era la confirmacion
// de 1,5 s -- o sea que se estaba dependiendo de la red de atras en vez del
// umbral. Con 6 quedan 1,5x sobre ese peor caso y sigue siendo menos de la
// mitad del umbral en marcha. En la punta del gripper son ~1,8 cm.
//
// Si en tu banco "pico_reposo" se queda siempre por debajo de 1, se puede
// bajar a 3 o 4 con 'Q' y ganar sensibilidad; el criterio es dejar al menos
// el doble del peor pico que veas en una corrida larga.
//
// COMO REVISARLO: 'S' informa "pico_reposo" por eje, que es el peor desvio
// (ya promediado y ya sin la firma) desde el homing. Tiene que quedar bien
// por debajo del umbral; si se le acerca, no hay que subir el umbral sin
// entender por que -- eso ya es una descalibracion de verdad.
constexpr float    UMBRAL_REPOSO_DEG      = 4.0f; // antes 6 sin falsos positivos.
constexpr float    TAU_REPOSO_S           = 1.0f;

// La confirmacion es lo que separa una descalibracion de una perturbacion
// de la medicion: si el brazo se movio de verdad el desvio es permanente
// (se queda hasta el proximo homing), mientras que una perturbacion
// electrica pasa. Un segundo y medio ya deja afuera las conocidas de este
// robot -- la conmutacion de la bomba dura ~300 ms -- y como el robot esta
// parado esperando piezas, esperar no cuesta nada.
//
// Verificado en el robot: empujando un brazo con la mano en home, lo
// detecto ~5 s despues (el promedio tarda en cargar y despues confirma), en
// el eje correcto, con denc=48,5 contra dcmd=45,1.
constexpr uint32_t CONFIRMACION_REPOSO_MS = 1500;

// Cuanto tiene que llevar quieto en home antes de anotar la firma: lo
// suficiente para que el promedio se asiente (3 constantes de tiempo).
//
// De paso es lo que hace que el chequeo NO moleste durante la produccion:
// entre pieza y pieza el robot no se queda 3 segundos quieto, asi que el
// chequeo ni se arma. Corre en los ratos muertos, que es justo para lo que
// se pidio.
//
// Medido con estos valores: firma tomada a los 3 s de llegar a home, y
// despues 100 segundos con "pico_reposo" entre 0,5 y 0,9 grados, sin
// moverse de ahi y sin un solo disparo en falso.
constexpr uint32_t FIRMA_REPOSO_MS = 3000;

// Aviso (NO frena) cuando la fuga en reposo lleva absorbidos mas de estos
// grados desde el homing. La fuga existe para borrar lo que sobra de cada
// tramo, pero si lo que borra crece sin parar y siempre para el mismo lado,
// no esta limpiando ruido: esta tapando perdida de pasos. Es la red lenta,
// complementaria del chequeo de arriba.
constexpr float DERIVA_AVISO_DEG = 15.0f;

// ============================================================
//  ZONA CIEGA DEL ADC (limites de lectura confiable)
// ============================================================
// La salida analogica del AS5600 barre de 0 V a VCC (3,46 V medidos) para
// dar 0-360 grados, pero el ADC del ESP32 a 11 dB se satura bastante antes
// de VCC y tiene una zona muerta cerca de 0 V. Con los numeros de
// Encoders.h el angulo se calcula como raw * 360 / 4293, o sea que el ADC
// llega a 4095 (su tope) recien en ~343 grados: de ahi hasta 360 la lectura
// queda CLAVADA, el eje sigue girando y el encoder no se entera. Abajo pasa
// lo mismo en los primeros ~10 grados.
//
// Resultado: hay una banda de unos 30-50 grados alrededor del cero del
// encoder donde la medicion no sirve. Si el recorrido de un eje la cruza,
// el error contra los pasos crece igual que en una colision de verdad.
//
// En este robot los 3 imanes se montaron A PROPOSITO con el cero fuera del
// alcance fisico, para dejar el tramo lineal del encoder centrado en el
// recorrido util. O sea que esto NO deberia dispararse nunca: queda como
// red de seguridad (si un iman se corre, o si alguien cambia el rango de
// trabajo, el sintoma seria identico a una colision y conviene que se
// distinga). El comando 'S' informa el minimo y el maximo de lectura cruda
// vistos, que es la forma de comprobar cuanto margen queda a cada lado.
//
// Un eje que igual entrara en esa banda deja de supervisarse hasta el
// proximo homing -- no se puede vigilar con un sensor ciego -- y queda
// registrado, en vez de frenar el robot en falso.
constexpr uint16_t RAW_MIN_FIABLE = 60;   // ~5 grados
constexpr uint16_t RAW_MAX_FIABLE = 3950; // ~331 grados

// ============================================================
//  COMPENSACION DEL ATRASO (opcional, 0 = apagada)
// ============================================================
// El margen por velocidad de arriba TOLERA el atraso; esto lo CANCELA. Si
// se le pone el atraso real de la cadena de medicion, la posicion comandada
// se pasa por el mismo pasabajos que sufre el encoder antes de comparar, y
// entonces el error se va a ~0 aunque el robot vaya a maxima velocidad. Eso
// permite bajar mucho el umbral y el margen, y detectar colisiones mucho
// mas rapido.
//
// COMO SACAR EL NUMERO: dejar unos ciclos normales y pedir 'S'. Informa
// "atraso medido" por eje, que es justamente error/velocidad en ms. Si los
// tres dan parecido y estable (por ejemplo 70-80 ms), ese es el atraso real
// de la cadena AS5600 + filtro RC + muestreo, y va aca. Si da erratico o
// crece con el tiempo, NO es atraso: es que se estan perdiendo cuentas, y
// taparlo con esto seria enmascarar el problema.
//
// En caliente se cambia con 'L70' por Serial.
constexpr uint32_t RETARDO_ENCODER_MS = 0;

// Sentido de cuenta de cada encoder respecto del eje que mide.
//   +1 -> el encoder sube cuando el brazo va hacia angulos positivos
//         (mismo sentido que los micropasos comandados)
//   -1 -> esta montado al reves (o el iman quedo espejado)
//
// COMO SABER SI HAY QUE CAMBIARLO: no hace falta medir nada a mano. Si un
// eje esta invertido, el guard lo detecta solo en el primer movimiento
// despues del homing y avisa por Serial; en ese caso NO frena el robot, se
// inhibe a si mismo para no estar parandolo en falso y pide que se corrija
// este arreglo. Un brazo REALMENTE trabado se distingue porque el encoder
// se queda quieto (denc ~ 0), no porque cuente al reves (denc ~ -dcmd).
constexpr float ENCODER_SIGN[3] = {1.0f, 1.0f, 1.0f};

} // namespace GuardConfig

/**
 * CollisionGuard
 * --------------
 * Capa de supervision del movimiento: compara, en cada vuelta del loop, el
 * angulo que miden los encoders contra el que dicen los micropasos ya
 * emitidos por los timers. Si la diferencia se pasa del margen Y SE
 * MANTIENE, declara colision.
 *
 * NO es un lazo de control de posicion: no corrige nada ni le manda nada a
 * los motores. El control de posicion sigue siendo a lazo abierto por
 * micropasos, que es lo que anda bien; esto solo mira y avisa. Que el
 * encoder sea impreciso (+-1 grado) no importa: no se le pide precision,
 * se le pide detectar un brazo que no llego a donde le pidieron.
 *
 * Todo el analisis es DIFERENCIAL. Se arma una referencia (cuanto marcaba
 * cada encoder y cuantos pasos habia comandados en ese instante) y a partir
 * de ahi se comparan INCREMENTOS:
 *
 *     error = (enc(t) - enc(ref)) - (cmd(t) - cmd(ref))
 *
 * Asi la deteccion no depende de la calibracion absoluta del encoder (ni
 * del offset de home, ni de la linealidad del AS5600, ni de cuanto se le
 * erro al angulo del endstop): cualquier offset fijo se cancela solo al
 * restar. Lo unico que tiene que ser cierto es que el encoder SIGA al eje.
 *
 * La referencia se promedia durante la ventana de asentamiento del homing
 * (2,5 s con el robot quieto), asi el ruido del encoder no se traslada a
 * la referencia: un error de +-1 grado ahi seria un corrimiento fijo de
 * todo el resto de la medicion.
 */
class CollisionGuard
{
public:

    enum Estado : uint8_t
    {
        DESARMADO,   // no supervisa (homing, pausa por colision, ERROR)
        PROMEDIANDO, // juntando muestras para la referencia, todavia no detecta
        ARMADO       // supervisando
    };

    // Que disparo el ultimo fallo. Los dos terminan igual (frenar y
    // recalibrar), pero no significan lo mismo para el que mira el log: uno
    // es un golpe, el otro es el robot perdiendo la calibracion solo.
    enum Motivo : uint8_t
    {
        MOTIVO_COLISION = 0,      // el brazo no llego donde decian los pasos
        MOTIVO_DESCALIBRACION = 1 // quieto en home, el encoder no marca lo de siempre
    };

    void begin();

    // Robot avisa si esta parado en home esperando piezas. Es la unica
    // situacion en la que se puede exigir un umbral chico, porque es la
    // unica en la que no hay ni atraso de medicion ni dinamica: misma pose
    // conocida, brazo frenado y sin nada en la mano.
    void setEnHome(bool enHome);

    // Arranca la ventana de promediado de la referencia. Se llama con el
    // robot QUIETO (los 3 ejes ya en su endstop), al mismo tiempo que
    // Encoders::iniciarAsentamientoHoming().
    void iniciarReferencia(long pasos1, long pasos2, long pasos3);

    // Congela la referencia promediada y empieza a supervisar.
    void fijarReferencia();

    // Deja de supervisar (y descarta la referencia).
    void desarmar();

    // Suspende la deteccion un rato, sin perder la referencia. Se usa al
    // conmutar la bomba de vacio: es una carga inductiva fuerte y la salida
    // del AS5600 es RATIOMETRICA a su VCC (0-360 grados = 0 a VCC), mientras
    // que el ADC del ESP32 mide contra su propia referencia interna. O sea
    // que si el riel de 3,46 V se hunde aunque sea un poco al arrancar la
    // bomba, las tres lecturas se corren juntas y de golpe, proporcional al
    // angulo absoluto de cada una: a media escala, un 2% de caida son ~4
    // grados que aparecen de la nada y no son movimiento.
    //
    // Al terminar el silencio se imprime cuanto se movio el error de cada
    // eje durante la conmutacion: si los 3 saltan juntos y para el mismo
    // lado, es la alimentacion, y se arregla en el hardware (desacoplar o
    // separar la alimentacion de los encoders), no en el umbral.
    void silenciar(uint32_t ms);

    // Se llama en CADA vuelta del loop con los pasos comandados de cada eje.
    // Devuelve true UNA sola vez: en el ciclo en que se confirma la colision
    // (y ahi mismo se desarma). Los datos del evento quedan en los getters
    // ejeDelFallo()/errorDelFallo()/cmdDeltaDelFallo()/encDeltaDelFallo().
    bool actualizar(long pasos1, long pasos2, long pasos3);

    Estado  estado() const   { return est; }
    bool    armado() const   { return est == ARMADO; }
    bool    inhibido() const { return inhibidoPorConfig; }

    Motivo  motivoDelFallo() const   { return motivoFallo; }
    uint8_t ejeDelFallo() const      { return ejeFallo; }      // 0-2
    float   errorDelFallo() const    { return errorFallo; }    // grados
    float   cmdDeltaDelFallo() const { return cmdDeltaFallo; } // grados recorridos segun los pasos
    float   encDeltaDelFallo() const { return encDeltaFallo; } // grados recorridos segun el encoder

    float errorActual(uint8_t eje) const;
    float picoDesdeHoming(uint8_t eje) const;

    // Peor error visto con el robot QUIETO EN HOME desde el ultimo homing.
    // Es el numero con el que se elige UMBRAL_REPOSO_DEG: mide el piso de
    // ruido real de esa pose, sin nada de dinamica encima.
    float picoEnReposo(uint8_t eje) const;
    float picoHistorico(uint8_t eje) const;
    bool  sensorCaido(uint8_t eje) const;

    // --- Datos crudos de la comparacion, para la traza y el diagnostico ---
    float encDeltaActual(uint8_t eje) const;  // grados recorridos segun el encoder
    float cmdDeltaActual(uint8_t eje) const;  // grados recorridos segun los pasos
    float velocidadCmd(uint8_t eje) const;    // grados/s comandados (filtrada)
    float umbralEfectivo(uint8_t eje) const;  // fijo + margen por velocidad

    // Ganancia encoder/pasos medida con el eje ya frenado despues de un
    // movimiento largo. Tiene que dar ~1,00. Si da menos, el encoder no
    // esta viendo todo el recorrido (tipico de la zona ciega del ADC); si
    // da ~0,5 o ~2, hay un problema de escala o de relacion mecanica.
    // Devuelve 0 si todavia no se pudo medir.
    //
    // Es EL numero que separa un atraso de una perdida de cuentas: con
    // atraso, el encoder llega tarde pero llega, y la ganancia da 1,00.
    float gananciaMedida(uint8_t eje) const;

    // Atraso de la medicion estimado en marcha (error / velocidad), en ms.
    // Si da estable y parecido en los 3 ejes, es el atraso real de la
    // cadena de medicion y se puede compensar (ver RETARDO_ENCODER_MS).
    // Con la compensacion activada pasa a ser el atraso que QUEDA.
    float atrasoMedido_ms(uint8_t eje) const;

    // Grados que la fuga en reposo lleva absorbidos desde el homing. Si
    // crece sin parar y siempre para el mismo lado, hay perdida de pasos
    // real (y conviene revisar la aceleracion, no la supervision).
    float derivaAbsorbida(uint8_t eje) const;

    // Extremos de lectura cruda vistos desde el ultimo homing: dicen si el
    // recorrido esta pegado a los limites utiles del ADC.
    uint16_t rawMinimo(uint8_t eje) const;
    uint16_t rawMaximo(uint8_t eje) const;
    bool     fueraDeRango(uint8_t eje) const;

    // Consume (una sola vez) el aviso de que un encoder dejo de ser
    // confiable. Mientras eso pasa, ese eje queda sin supervisar.
    bool consumirAvisoSensor(uint8_t &eje);

    // Modo observador: el guard mide y avisa por Serial lo que HABRIA
    // hecho, pero nunca declara colision. Sirve para dejar el robot
    // trabajando sin paradas mientras se calibran los parametros, y para
    // ver si un umbral nuevo hubiera aguantado un ciclo entero.
    void setObservar(bool observarSinFrenar);
    bool observando() const { return observar; }

    void     setUmbral(float grados);
    void     setConfirmacion(uint32_t ms);
    void     setMargenVelocidad(uint32_t ms);
    void     setRetardo(uint32_t ms);
    void     setUmbralReposo(float grados);
    float    umbral() const           { return umbralDeg; }
    uint32_t confirmacion() const     { return confirmacionMs; }
    uint32_t margenVelocidad() const  { return margenVelocidadMs; }
    uint32_t retardo() const          { return retardoMs; }
    float    umbralReposo() const     { return umbralReposoDeg; }

    const char *nombreEstado() const;

private:

    static const uint8_t NUM_EJES = 3;

    Estado est = DESARMADO;

    float    umbralDeg         = GuardConfig::UMBRAL_DEG;
    uint32_t confirmacionMs    = GuardConfig::CONFIRMACION_MS;
    uint32_t margenVelocidadMs = GuardConfig::MARGEN_VELOCIDAD_MS;
    uint32_t retardoMs         = GuardConfig::RETARDO_ENCODER_MS;

    // --- Chequeo en reposo ---
    float    umbralReposoDeg      = GuardConfig::UMBRAL_REPOSO_DEG;
    uint32_t confirmacionReposoMs = GuardConfig::CONFIRMACION_REPOSO_MS;

    // Lo pone Robot: true solo mientras espera piezas, parado en home.
    bool     enHome = false;

    bool     enFaltaRep[NUM_EJES]       = {false, false, false};
    uint32_t faltaRepDesde_ms[NUM_EJES] = {0, 0, 0};
    float    picoRep[NUM_EJES]          = {0.0f, 0.0f, 0.0f};

    // Desvio contra la calibracion del homing, promediado: es lo que se
    // compara contra el umbral. Sin promediar, los picos sueltos del
    // encoder (hasta 5 grados) harian imposible bajar el umbral.
    float    desvioRep[NUM_EJES]        = {0.0f, 0.0f, 0.0f};
    bool     desvioRepListo[NUM_EJES]   = {false, false, false};

    // Firma: cuanto marcaba ese desvio la primera vez que el robot se quedo
    // quieto en home despues del homing. Es el cero contra el que se mide.
    float    firmaRep[NUM_EJES]         = {0.0f, 0.0f, 0.0f};
    bool     firmaLista[NUM_EJES]       = {false, false, false};
    uint32_t reposoDesde_ms[NUM_EJES]   = {0, 0, 0};

    // Se avisa una sola vez por homing, no en cada vuelta del loop.
    bool     avisoDeriva[NUM_EJES]      = {false, false, false};

    Motivo   motivoFallo = MOTIVO_COLISION;

    // Referencia: de donde se empiezan a contar los incrementos.
    float refEnc[NUM_EJES] = {0.0f, 0.0f, 0.0f}; // grados de encoder (ya con el signo aplicado)
    long  refPasos[NUM_EJES] = {0, 0, 0};        // micropasos comandados
    bool  refValida[NUM_EJES] = {false, false, false};

    // Acumuladores de la ventana de promediado.
    double   sumaEnc[NUM_EJES]  = {0.0, 0.0, 0.0};
    uint32_t muestras[NUM_EJES] = {0, 0, 0};

    // Estado de la deteccion, por eje.
    float    errorDeg[NUM_EJES]   = {0.0f, 0.0f, 0.0f};
    float    encDelta[NUM_EJES]   = {0.0f, 0.0f, 0.0f};
    float    cmdDelta[NUM_EJES]   = {0.0f, 0.0f, 0.0f};
    bool     enFalta[NUM_EJES]    = {false, false, false};
    uint32_t faltaDesde_ms[NUM_EJES] = {0, 0, 0};
    float    faltaMax[NUM_EJES]   = {0.0f, 0.0f, 0.0f}; // peor error de esta sospecha
    float    pico[NUM_EJES]       = {0.0f, 0.0f, 0.0f}; // desde el ultimo homing
    float    picoHist[NUM_EJES]   = {0.0f, 0.0f, 0.0f}; // desde el encendido

    // --- Velocidad comandada y su pico decreciente (margen dinamico) ---
    float    cmdDegPrev[NUM_EJES]     = {0.0f, 0.0f, 0.0f};
    float    velCmd[NUM_EJES]         = {0.0f, 0.0f, 0.0f}; // grados/s, suavizada
    float    picoVel[NUM_EJES]        = {0.0f, 0.0f, 0.0f};
    uint32_t ultimoUs                 = 0;

    // --- Diagnostico: ganancia encoder/pasos medida en reposo ---
    float    ganancia[NUM_EJES]      = {0.0f, 0.0f, 0.0f};
    uint32_t quietoDesde_ms[NUM_EJES] = {0, 0, 0};

    // --- Diagnostico: atraso de la medicion estimado en marcha (segundos) ---
    float atrasoSeg[NUM_EJES] = {0.0f, 0.0f, 0.0f};

    // Total corrido por la fuga desde el homing. Es la deriva real que se
    // esta absorbiendo: si crece siempre para el mismo lado, se estan
    // perdiendo pasos de verdad.
    float fugaAcumulada[NUM_EJES] = {0.0f, 0.0f, 0.0f};

    // Posicion comandada pasada por el mismo pasabajos que se le supone al
    // encoder (solo si retardoMs > 0). Es contra ESTO que se compara.
    float cmdCompensado[NUM_EJES] = {0.0f, 0.0f, 0.0f};

    // Velocidad minima para que error/velocidad signifique algo: por debajo
    // de esto el cociente lo domina el ruido del encoder.
    static constexpr float VEL_MIN_PARA_ATRASO_DEG_S = 80.0f;

    // --- Zona ciega del ADC ---
    uint16_t rawMin[NUM_EJES] = {0xFFFF, 0xFFFF, 0xFFFF};
    uint16_t rawMax[NUM_EJES] = {0, 0, 0};

    // Pegajosa hasta el proximo homing: una vez que el eje se metio en la
    // zona donde el ADC no mide, el angulo continuo perdio los grados que
    // recorrio ahi adentro y la referencia diferencial quedo corrida para
    // siempre. No alcanza con dejar de mirar mientras dura.
    bool     ciego[NUM_EJES] = {false, false, false};

    // Velocidad por debajo de la cual se considera que el eje esta quieto,
    // a los efectos de medir la ganancia.
    static constexpr float VEL_QUIETO_DEG_S = 5.0f;

    // Recorrido minimo para que la ganancia medida signifique algo.
    static constexpr float RECORRIDO_MIN_GANANCIA_DEG = 15.0f;

    bool     caido[NUM_EJES]              = {false, false, false};
    bool     avisoCaido[NUM_EJES]         = {false, false, false};
    uint32_t ultimoAvisoCaido_ms[NUM_EJES] = {0, 0, 0};

    // Un canal en el limite puede entrar y salir de "valido" varias veces
    // por segundo; sin esto, cada rebote seria un fallo nuevo y el registro
    // se llenaria de avisos del mismo problema, tapando lo importante.
    static const uint32_t REPETIR_AVISO_SENSOR_MS = 5000;

    // Datos del evento que se acaba de confirmar.
    uint8_t ejeFallo      = 0;
    float   errorFallo    = 0.0f;
    float   cmdDeltaFallo = 0.0f;
    float   encDeltaFallo = 0.0f;

    // Se prende si se detecta que un encoder cuenta al reves: la
    // supervision se apaga sola (fail-open) en vez de frenar el robot en
    // falso una y otra vez.
    bool inhibidoPorConfig = false;

    // Mide y avisa, pero no frena.
    bool     observar             = false;
    uint32_t ultimoAvisoObs_ms    = 0;
    static const uint32_t REPETIR_AVISO_OBS_MS = 2000;

    // Silencio temporal (conmutacion de la bomba).
    uint32_t silencioHasta_ms         = 0;
    bool     silencioPendiente        = false;
    float    errorAlSilenciar[NUM_EJES] = {0.0f, 0.0f, 0.0f};

    // Grados de encoder de un eje, con el signo de montaje ya aplicado.
    float leerEncoder(uint8_t eje) const;

    // Grados de eje que representan esos micropasos.
    static float pasosAGrados(uint8_t eje, long pasos);

    void reportarSignoInvertido(uint8_t eje, float cmdDelta, float encDelta);
};

#endif
