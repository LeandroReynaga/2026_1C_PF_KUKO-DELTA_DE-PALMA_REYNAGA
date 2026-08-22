"""Como se PRESENTA cada parametro: en que grupo va, como se llama en
castellano y que hace.

Lo que el firmware manda por 'P?' es el contrato tecnico (nombre corto,
rango, unidad, nivel, tipo) y con eso solo ya se podria dibujar la pantalla.
Pero "apr_dz = 2.7 cm" no le dice nada a nadie seis meses despues, y la
diferencia entre una lista de ajustes util y una inservible es justamente
saber que pasa si se sube ese numero.

POR QUE ESTA TABLA NO ES "LA LISTA DE PARAMETROS"
-------------------------------------------------
Sigue mandando el firmware. Aca no hay rangos, ni valores, ni niveles: solo
texto. Un parametro que el firmware agregue y esta tabla no conozca APARECE
IGUAL en la pantalla, con su nombre corto y sin descripcion (ver
`describir`). Al reves no puede pasar: una entrada de mas aca no inventa
ningun control, porque los controles se arman recorriendo lo que llego por
serie.

Esa es la unica forma de que las dos mitades no se contradigan. Si esta
tabla decidiera que parametros existen, el dia que alguien agregue uno en
C++ la pantalla no lo mostraria y no habria ningun error que lo delate.
"""

from __future__ import annotations

from typing import NamedTuple


class Ficha(NamedTuple):
    grupo: str
    etiqueta: str
    ayuda: str


# Orden en que se muestran los grupos dentro de cada pestana. Los que no
# esten aca van al final, en el grupo "Otros".
ORDEN_GRUPOS = [
    # --- proceso ---
    "Agarre",
    "Movimiento",
    "Cinta",
    "Supervision de colisiones",
    "Modo teach",
    "Telemetria",
    # --- servicio ---
    "Cinta y agarre",
    "Tachos",
    "Caja de alfajores",
    "Homing",
    "Limites de movimiento",
    "Volumen del modo teach",
    "Otros",
]


FICHAS: dict[str, Ficha] = {
    # ==================================================================
    #  Nivel 1 -- operacion (se ajusta desde la pantalla de operacion)
    # ==================================================================
    "vis_lat": Ficha(
        "Cinta", "Latencia de la vision",
        "Cuanto avanza la pieza entre que cruza la linea de deteccion y que "
        "el mensaje llega al robot. Es el ajuste fino de cada arranque: si "
        "el gripper cae adelantado, hay que subirlo."),

    # ==================================================================
    #  Nivel 2 -- proceso
    # ==================================================================
    "press_dz": Ficha(
        "Agarre", "Presion sobre la pieza",
        "Cuanto sigue bajando el gripper despues de tocar la pieza, para que "
        "la ventosa selle. De menos no agarra; de mas la aplasta y la corre "
        "de lugar."),
    "apr_dx": Ficha(
        "Agarre", "Aproximacion: adelanto",
        "Donde espera el brazo respecto de la pieza. Negativo = mas atras, "
        "asi baja a favor de la cinta y no en contra."),
    "apr_dz": Ficha(
        "Agarre", "Aproximacion: altura",
        "A que altura sobre la pieza espera antes de bajar. Mas alto es mas "
        "seguro pero alarga el tramo lento, que es el que fija el ritmo."),
    "lift_dz": Ficha(
        "Agarre", "Despegue",
        "Cuanto sube con la pieza antes de trasladarse. Tiene que alcanzar "
        "para librar la pared del tacho."),
    "rel_ms": Ficha(
        "Agarre", "Espera al soltar",
        "Con la bomba ya apagada, cuanto se espera a que la pieza se "
        "despegue. Con la electrovalvula montada la linea de vacio se ventea "
        "sola y la pieza cae en el acto, asi que alcanza con 80 ms: es lo "
        "que tarda en entrar el aire, no en irse el vacio por fuga. Si "
        "alguna sale pegada al gripper es el numero a subir, pero de mas es "
        "tiempo muerto en cada ciclo."),
    "bin_ms": Ficha(
        "Agarre", "Quieto antes de soltar",
        "Cuanto se queda frenado sobre el tacho antes de apagar la bomba. "
        "Sirve para que la pieza no salga despedida por la inercia."),

    "acc_rap": Ficha(
        "Movimiento", "Aceleracion rapida",
        "Traslados, despegue y vuelta a home. Es EL numero del ciclo: subirlo "
        "acorta el tiempo por pieza y sube la vibracion (y con ella los "
        "falsos positivos del guard)."),
    "acc_suave": Ficha(
        "Movimiento", "Aceleracion suave",
        "Unico tramo donde el gripper toca la pieza (bajada al agarre y "
        "apoyo en la caja). Bajo a proposito: el encuentro tiene que ser "
        "suave para no correr la pieza."),
    "pick_tol": Ficha(
        "Movimiento", "Tolerancia de atraso",
        "Cuanto puede llegar tarde el brazo al punto de aproximacion antes "
        "de dar por perdido el encuentro y replanificar."),
    "pump_lead": Ficha(
        "Agarre", "Adelanto de la bomba",
        "Cuanto antes de arrancar la bajada se prende el vacio. Tiene que "
        "alcanzar para que la ventosa se forme antes de tocar la pieza; de "
        "mas, la bomba queda soplando al aire mientras el brazo espera que "
        "la pieza llegue por la cinta."),
    "replan": Ficha(
        "Movimiento", "Reintentos de plan",
        "Cuantas veces se vuelve a calcular el encuentro de una misma pieza "
        "antes de dejarla pasar."),

    "cinta_pwm": Ficha(
        "Cinta", "PWM de la cinta",
        "Lo que se le manda al driver. OJO: no cambia solo la velocidad "
        "MEDIDA (cinta_cms), que es la que usa la planificacion. Despues de "
        "moverlo hay que volver a medir la cinta y actualizar el otro numero, "
        "o el robot le va a errar a todas las piezas."),

    "g_umbral": Ficha(
        "Supervision de colisiones", "Umbral fijo",
        "Cuantos grados de diferencia entre el encoder y los pasos se "
        "toleran con el brazo quieto o lento. Cubre el ruido del encoder "
        "(~1 grado) y el error de la referencia."),
    "g_reposo": Ficha(
        "Supervision de colisiones", "Umbral en reposo",
        "Con el robot parado en home no hay atraso ni vibracion, asi que se "
        "puede exigir mucho mas. Es lo que detecta que se perdieron pasos o "
        "que alguien movio un brazo."),
    "g_conf": Ficha(
        "Supervision de colisiones", "Confirmacion",
        "Cuanto tiene que SOSTENERSE el error pasado del umbral para que se "
        "declare colision. Una lectura aislada no frena nada."),
    "g_margen": Ficha(
        "Supervision de colisiones", "Margen por velocidad",
        "Atraso que se le tolera a la medicion. El umbral efectivo es "
        "umbral fijo + este tiempo x velocidad. Subirlo saca falsos "
        "positivos en los tramos rapidos; bajarlo detecta antes."),
    "g_retardo": Ficha(
        "Supervision de colisiones", "Retardo del encoder",
        "Atraso que se cancela pasando la posicion comandada por el mismo "
        "pasabajos que sufre el sensor. Con esto bien puesto, el margen por "
        "velocidad puede ser mas chico."),
    "g_salto": Ficha(
        "Supervision de colisiones", "Salto entre paradas",
        "Cuanto puede cambiar la calibracion de un eje entre dos paradas en "
        "home antes de rehomear. Es lo que detecta los pasos perdidos en un "
        "tramo: el umbral en marcha no los ve (son chicos al lado del "
        "atraso del encoder) y el chequeo en reposo los descontaba como si "
        "fueran propios del sensor."),
    "g_salto_k": Ficha(
        "Supervision de colisiones", "Salto: tolerancia por recorrido",
        "Se suma al anterior, como porcentaje de los grados que recorrio el "
        "eje desde la parada anterior. Existe porque el encoder no cierra "
        "exacta cada excursion (~0,5 % medido) y esa deriva es legitima; los "
        "pasos perdidos, en cambio, aparecen de golpe. Subirlo tolera mas "
        "deriva, bajarlo detecta antes y arriesga falsos positivos despues "
        "de muchos ciclos sin parar."),
    "g_neum": Ficha(
        "Supervision de colisiones", "Silencio al conmutar la bomba",
        "Cuanto se suspende la deteccion al prender o apagar el vacio. La "
        "bomba hunde el riel y el AS5600 es ratiometrico: las tres lecturas "
        "se corren juntas sin que se mueva nada."),
    "col_pausa": Ficha(
        "Supervision de colisiones", "Pausa tras colision",
        "Cuanto se queda quieto despues de una colision antes de rehomear. "
        "No es para que se acomode nada: es para que quien mira alcance a "
        "sacar la mano o el obstaculo."),
    "col_max": Ficha(
        "Supervision de colisiones", "Colisiones seguidas",
        "Cuantas colisiones sin completar una pieza en el medio antes de "
        "dejar de reintentar. Si choca tres veces contra lo mismo, rehomear "
        "una cuarta solo sigue golpeando el robot."),

    "t_jog": Ficha(
        "Modo teach", "Velocidad del jog",
        "A que velocidad avanza la punta mientras se la mueve a mano. Es un "
        "pedido, no una garantia: si el brazo no llega a hacer el tramo, el "
        "jog se frena solo en vez de acumular atraso contra un destino que "
        "se le escapa."),
    "t_jogpct": Ficha(
        "Modo teach", "Vel. y acel. del jog",
        "Porcentaje del tope de velocidad y aceleracion con el que se mueve "
        "el brazo durante el jog. Bajo a proposito: con el brazo manejado a "
        "mano y a centimetros de la cinta, lo que importa es poder soltar la "
        "tecla a tiempo, no llegar rapido."),

    "t_acel": Ficha(
        "Modo teach", "Aceleracion",
        "La aceleracion del jog y de la reproduccion. NO se usa la del ciclo "
        "normal (97.000): una trayectoria ensenada a mano son decenas de "
        "tramos cortos, y con esa aceleracion cada cambio de velocidad es un "
        "tiron. Es el primer numero a bajar si el brazo vibra al reproducir."),
    "t_mezcla": Ficha(
        "Modo teach", "Redondeo de esquinas",
        "A esta distancia de un punto intermedio, el brazo redirige al punto "
        "siguiente SIN frenar: pasa cerca en vez de clavarse en cada uno. Es "
        "lo que convierte una sucesion de tramos en un movimiento solo. Mas "
        "grande es mas fluido y se aparta mas de lo ensenado; 0 lo apaga y "
        "vuelve a frenar en cada punto."),
    "movl_paso": Ficha(
        "Modo teach", "Paso del movimiento recto",
        "En tramos de este largo se parte una recta para que la punta vaya "
        "derecho de verdad (movL) y no por el camino mas rapido de los "
        "motores (movJ). El error contra la recta cae con el CUADRADO del "
        "paso: 1 cm da 0,05 mm, 2 cm da 0,19 mm y 5 cm ya da 1,06 mm. Bajarlo "
        "de mas no sirve: el firmware no acepta un tramo mas corto que lo que "
        "el eje necesita para frenar, asi que lo sube solo."),
    "movl_acel": Ficha(
        "Modo teach", "Aceleracion del movimiento recto",
        "EL numero para que una recta salga derecha yendo rapido, y no es "
        "obvio por que: el tramo no puede ser mas corto que la distancia de "
        "frenado, que vale v2/(2a). O sea que a velocidad fija, MAS "
        "ACELERACION ES MENOS TRAMO, y menos tramo es corregir el rumbo mas "
        "seguido. A 40 cm/s, con 40.000 el tramo minimo son 5,4 cm (1,3 mm "
        "de desvio) y con 97.000 baja a 2,2 cm (0,23 mm). Viene en 97.000, "
        "la misma con la que el robot mueve cada pieza."),
    "movl_vel": Ficha(
        "Modo teach", "Velocidad del movimiento recto",
        "Velocidad de la PUNTA mientras recorre una recta. No es un capricho "
        "que sea baja: cuanto mas rapido va, mas largo tiene que ser cada "
        "tramo para poder frenar, y mas se aparta de la recta. A 20 cm/s la "
        "recta sale a menos de una decima de milimetro; a 50 cm/s los tramos "
        "tendrian que medir 7 cm y movL se vuelve movJ."),

    "t_esquina": Ficha(
        "Modo teach", "Esquina maxima a redondear",
        "Hasta que angulo se redondea. Una esquina cerrada obliga a los ejes "
        "a cambiar de velocidad de golpe -- que es el tiron que se quiere "
        "evitar -- y si alguno tiene que invertir el sentido no hay forma de "
        "hacerlo sin pasar por cero. A partir de aca se frena y se arranca "
        "de nuevo."),

    "tele_ms": Ficha(
        "Telemetria", "Periodo de [T]",
        "Angulos, error del guard, finales y bomba. Es lo que mueve los "
        "diales de la pantalla de operacion."),
    "est_ms": Ficha(
        "Telemetria", "Periodo de [E]",
        "Modo, cola, caja y contadores de produccion."),
    "sal_ms": Ficha(
        "Telemetria", "Periodo de [H]",
        "Salud de encoders, vueltas de loop por segundo y RAM libre."),
    "diag_ms": Ficha(
        "Telemetria", "Volcado [GUARD]",
        "Cada cuanto sale sola la linea con los numeros de la supervision. "
        "Viene APAGADO (0): los mismos numeros ya llegan en [T] y en [H], y "
        "una linea mas cada tantos segundos solo tapa la consola. Se enciende "
        "cuando se esta calibrando el guard a mano y se lo quiere en el log."),

    # ==================================================================
    #  Nivel 3 -- servicio
    # ==================================================================
    "grab_z": Ficha(
        "Cinta y agarre", "Altura de agarre (Z)",
        "Cara superior de la pieza sobre la cinta. Si se equivoca el signo o "
        "la coma, el gripper se clava contra la cinta: por eso no se puede "
        "cambiar con una pieza en la mano."),
    "linea_x": Ficha(
        "Cinta y agarre", "Linea de deteccion (X)",
        "Donde la camara ve pasar las piezas, en coordenadas del robot. "
        "Existe TAMBIEN del lado de Python: si se cambia de un solo lado, el "
        "robot le empieza a errar sin que nada avise."),
    "cinta_cms": Ficha(
        "Cinta y agarre", "Velocidad medida de la cinta",
        "La velocidad REAL, medida con un cronometro o con la vision. Es la "
        "que usa la planificacion del encuentro. Se actualiza cada vez que "
        "se toca el PWM."),

    "bin_x1": Ficha("Tachos", "Tacho 1: X", "Centro del primer tacho."),
    "bin_x2": Ficha("Tachos", "Tacho 2: X", "Centro del tacho del medio."),
    "bin_x3": Ficha("Tachos", "Tacho 3: X", "Centro del tercer tacho."),
    "bin_y": Ficha("Tachos", "Tachos: Y",
                   "Los tres estan alineados, asi que comparten el Y."),
    "bin_z": Ficha("Tachos", "Tachos: Z",
                   "Altura a la que se suelta la pieza. Mas bajo hace menos "
                   "ruido; demasiado bajo la apoya contra las que ya estan."),

    "box_z": Ficha("Caja de alfajores", "Piso de la caja (Z)",
                   "Altura a la que se apoya el alfajor. Se mide con la caja "
                   "puesta, no calculada."),
    "box_apr": Ficha("Caja de alfajores", "Aproximacion a la celda",
                     "Cuanto por encima del piso frena antes de bajar despacio."),
    "box_tr": Ficha("Caja de alfajores", "Altura de cruce",
                    "Con cuanta altura viaja entre la cinta y la celda. Tiene "
                    "que librar la pared de la caja Y los alfajores ya "
                    "puestos, contando que el camino se hunde en el medio."),
    "box_rel": Ficha("Caja de alfajores", "Espera al soltar en la caja",
                     "Mas larga que en el tacho a proposito: aca el brazo "
                     "tiene que salir de adentro de la caja y no puede "
                     "arrastrar la pieza."),
    "box_dx": Ficha("Caja de alfajores", "Corrimiento de la caja: X",
                    "Mueve las 6 celdas juntas. La grilla de 6 cm es la "
                    "geometria de la caja y no se toca; esto es para "
                    "centrarla donde quedo apoyada."),
    "box_dy": Ficha("Caja de alfajores", "Corrimiento de la caja: Y",
                    "Idem, en el otro eje."),

    "home_a1": Ficha("Homing", "Angulo de home: eje 1",
                     "Angulo real del brazo con su final de carrera pisado. "
                     "Es la referencia de TODO: fija la posicion en pasos y "
                     "el cero de los encoders. Se mide a mano."),
    "home_a2": Ficha("Homing", "Angulo de home: eje 2", "Idem eje 2."),
    "home_a3": Ficha("Homing", "Angulo de home: eje 3", "Idem eje 3."),
    "home_set": Ficha("Homing", "Ventana de promediado",
                      "Cuanto se queda quieto sobre los finales promediando "
                      "los encoders para fijar la referencia del guard. Mas "
                      "largo es mas preciso y alarga cada rehoming."),
    "home_to": Ficha("Homing", "Tiempo maximo",
                     "Si en este tiempo no encontro los 3 finales, se corta "
                     "todo y se para la cinta. Sin esto, un brazo trabado "
                     "empuja contra el obstaculo para siempre."),
    "tapa_ms": Ficha("Homing", "Ventana para confirmar la tapa",
                     "Cuanto vale la primera pulsacion al entrar o salir del "
                     "modo caja antes de que haya que empezar de nuevo."),

    "t_xmin": Ficha("Volumen del modo teach", "Limite X minimo",
                    "Pared izquierda del cajon dentro del que se puede mover "
                    "el brazo a mano. Es lo unico que separa al operador de "
                    "meterlo contra algo, asi que se toca cuando se mueve "
                    "algo de la mesa, no para llegar mas lejos."),
    "t_xmax": Ficha("Volumen del modo teach", "Limite X maximo",
                    "Idem, del otro lado."),
    "t_ymin": Ficha("Volumen del modo teach", "Limite Y minimo",
                    "Lo mas cerca del operador a donde puede ir: por defecto, "
                    "el centro de los tachos."),
    "t_ymax": Ficha("Volumen del modo teach", "Limite Y maximo",
                    "Lo mas lejos: por defecto, el borde de la cinta que da "
                    "al fondo."),
    "t_zup": Ficha("Volumen del modo teach", "Techo sobre el agarre",
                   "Cuanto puede despegarse por encima de la altura de agarre. "
                   "El PISO no es un ajuste aparte: cuelga de 'grab_z', asi "
                   "que recalibrar el agarre mueve tambien el limite del jog "
                   "y no quedan los dos diciendo cosas distintas."),

    "vel_max": Ficha("Limites de movimiento", "Velocidad maxima",
                     "Techo global. Esta A PROPOSITO por encima de lo "
                     "alcanzable: asi ningun movimiento llega a velocidad de "
                     "crucero y el perfil queda siempre triangular, que es lo "
                     "que menos vibra. Bajarlo por debajo de ~30000 empieza a "
                     "hacer aparecer meseta."),
}


def describir(nombre: str) -> Ficha:
    """Ficha del parametro, o una generica si el firmware trajo uno nuevo.

    Devolver algo siempre (en vez de fallar o de saltearlo) es lo que hace
    que agregar un parametro en C++ alcance para verlo en la pantalla: sale
    con su nombre corto y sin descripcion, que es feo pero honesto, y avisa
    solo de que falta escribirle la ficha aca.
    """

    ficha = FICHAS.get(nombre)

    if ficha is not None:
        return ficha

    return Ficha("Otros", nombre, "Sin descripcion todavia (parametro nuevo "
                                  "del firmware).")


def agrupar(parametros, nivel: int) -> list[tuple[str, list]]:
    """Los parametros de un nivel, repartidos en grupos y en orden.

    Devuelve [(grupo, [parametro, ...]), ...]. Dentro de cada grupo se
    respeta el orden en que los mando el firmware, que es el orden en que
    estan registrados en Robot.cpp -- o sea, el orden en que los penso quien
    los escribio, que suele ser mejor que alfabetico.
    """

    grupos: dict[str, list] = {}

    for p in parametros:
        if p.nivel != nivel:
            continue

        grupos.setdefault(describir(p.nombre).grupo, []).append(p)

    def clave(nombre: str) -> int:
        return ORDEN_GRUPOS.index(nombre) if nombre in ORDEN_GRUPOS else 999

    return [(g, grupos[g]) for g in sorted(grupos, key=clave)]
