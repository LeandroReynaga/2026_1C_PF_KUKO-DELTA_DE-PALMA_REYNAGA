# ============================================================
# CONFIGURACIÓN GENERAL DEL SISTEMA DE VISIÓN
# ============================================================

# ----------------------------
# Cámara
# ----------------------------

# Cada fotograma pasa por tres etapas, en este orden:
#
#   1. CAPTURA   -> CAMERA_WIDTH x CAMERA_HEIGHT (sensor apaisado)
#   2. ROTACIÓN  -> CAMERA_ROTATION (gira 90° y por lo tanto
#                   INTERCAMBIA ancho y alto)
#   3. RECORTE   -> CROP_*_RATIO (se queda solo con la cinta)
#
# El resto del sistema (detección, seguimiento, línea, píxeles que
# se mandan al ESP32) trabaja SIEMPRE sobre el fotograma ya
# recortado: el origen (0, 0) es la esquina superior izquierda del
# recorte, no la de la cámara.

CAMERA_INDEX = 0

# Resolución que se le pide al sensor, ANTES de rotar y recortar.
# Es la resolución nativa apaisada de la cámara (720p). No hay que
# invertirla para compensar la rotación: rotar ya intercambia los
# lados, y pedir un modo vertical que la cámara no soporta hace que
# el driver caiga en silencio a cualquier otro modo.
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30

# Backend de captura de Windows. NO es un detalle menor: es lo que
# decide si llegamos a 30 fps o no.
#
# A 720p sin comprimir (YUY2) el video son ~55 MB/s, bastante más
# de lo que soporta USB 2.0, así que el driver baja los fps hasta
# que entre. La solución es que la cámara comprima en MJPG.
#
# Medido con esta cámara (probe de backends, agosto 2026):
#
#   DSHOW + 1280x720 -> se queda en YUY2, ignora el pedido de
#                       MJPG, y entrega 10 fps.
#   DSHOW +  640x480 -> YUY2, 30 fps (entra justo en el ancho
#                       de banda, pero se pierde resolución).
#   MSMF  + 1280x720 -> negocia MJPG solo, 30 fps. <-- este
#
# O sea que 720p a 30 fps sí se puede, pero solo con MSMF.
# Dejar en "DSHOW" únicamente si MSMF diera problemas; "AUTO" deja
# elegir a OpenCV. Si el backend elegido no abre, Camera prueba los
# otros y avisa por consola cuál terminó usando.
CAMERA_BACKEND = "MSMF"

# Pedido explícito de stream comprimido. MSMF ya lo negocia por su
# cuenta y DirectShow con esta cámara lo ignora, así que hoy no
# cambia nada, pero no molesta y ayuda si se cambia de cámara.
CAMERA_USE_MJPG = True

# ----------------------------
# Exposición
# ----------------------------

# El robot se muda de habitación, así que la iluminación cambia de
# un día para el otro. Con exposición automática la cámara se
# reacomoda sola y no hay que recalibrar nada al moverlo: por eso
# queda en automático.
#
# La contra es que la cámara promedia toda la escena y, como la
# cinta es clara y ocupa casi todo el cuadro, tiende a sobreexponer:
# más de la mitad de los píxeles de una pieza celeste llegan a
# V=255 (medido). Un píxel que satura pierde saturación y se corre
# hacia el blanco, y por eso los rangos HSV de las piezas claras
# tienen que ser más tolerantes en S que los del rojo.
#
# Si alguna vez conviene fijarla (banco con luz estable, piezas muy
# claras), poner False y ajustar CAMERA_EXPOSURE.
CAMERA_AUTO_EXPOSURE = True

# Solo se usa si CAMERA_AUTO_EXPOSURE = False.
#
# En Windows el valor es un exponente en base 2: el tiempo de
# exposición es 2^valor segundos. Más negativo = más oscuro.
#
# Barrido medido con las tres piezas sobre la cinta y las luces
# prendidas, contando lo que realmente detecta detection.py:
#
#   -7  -> 1/128 s | quemado 35%  | no detecta NADA
#   -8  -> 1/256 s | quemado 13%  | detecta 2 de 3, y el cuadrado
#                                   lo confunde con un círculo
#   -9  -> 1/512 s | quemado  5%  | detecta las 3 y bien  <-- este
#  -10  -> 1/1024s | quemado  2%  | detecta las 3, pero el círculo
#                                   azul baja a ~4600 px² de área,
#                                   peligrosamente cerca del mínimo
#  -11  -> 1/2048s |              | vuelve a confundir el cuadrado
#
# O sea que hay una ventana angosta y -9 está en el medio. Si se
# cambia la iluminación del banco hay que rehacer este barrido.
#
# Ojo con irse hacia el otro lado: una exposición más larga que
# 1/30 s no llega a 30 fps. Cualquier valor de -6 para abajo es
# seguro en ese sentido.
CAMERA_EXPOSURE = -9

# Rotación aplicada a cada fotograma apenas se captura. Usar si la
# cámara queda montada girada respecto a la cinta: lo que buscamos
# es que la cinta se vea horizontal en pantalla.
#
# None                    -> sin rotación
# "90_CLOCKWISE"          -> 90° en sentido horario
# "90_COUNTERCLOCKWISE"   -> 90° en sentido antihorario
# "180"                   -> 180°
CAMERA_ROTATION = "90_CLOCKWISE"

# ============================================================
# LÍNEA DE DETECCIÓN
# ============================================================

# La cinta se ve horizontal, así que la línea de disparo es
# VERTICAL y las piezas la cruzan avanzando en X.

# Posición horizontal de la línea respecto del fotograma RECORTADO.
#
# 0.00 = borde izquierdo
# 0.50 = centro
# 1.00 = borde derecho
#
# OJO: es una fracción del recorte, no de la imagen completa. Si se
# cambian los CROP_X_*_RATIO, la línea se mueve sola y hay que
# reajustar este número.
#
# Corrido 0,5 cm a la IZQUIERDA (antes 0.60): la línea dibujada caía
# medio centímetro corriente abajo de la línea real medida sobre la
# cinta, así que el cruce se avisaba con la pieza ya un poco pasada.
#
# La cuenta, con el fotograma recortado de 598 px de ancho y
# IMAGE_WIDTH_CM = 21,5:
#
#   escala   21,5 / 597 = 0,036 cm/px  ->  27,8 px/cm
#   0,5 cm   = 13,9 px
#   la linea se calcula con int(598 * ratio): 0.60 -> 358 px,
#            0.576 -> 344 px
#   corrimiento real = 14 px = 0,504 cm
#
# A 6,75 cm/s de cinta, eso adelanta el aviso unos 74 ms.
LINE_X_RATIO = 0.576

# Sentido de movimiento de la cinta en pantalla:
#
# "LEFT_TO_RIGHT" → de izquierda a derecha (X aumenta)
# "RIGHT_TO_LEFT" → de derecha a izquierda (X disminuye)
LINE_DIRECTION = "LEFT_TO_RIGHT"

# ----------------------------
# Detección de objetos
# ----------------------------

# Área mínima y máxima aceptada para un contorno.
#
# OJO: están en píxeles, así que dependen de la resolución. Venían
# calibrados con la cámara entregando 800x600 (600 px de ancho útil
# a lo largo de la cinta); a 720p el ancho útil pasa a 720 px, o sea
# 1.2x lineal y ~1.44x en área. Los valores de abajo ya están
# reescalados por ese factor, pero conviene verificarlos en vivo:
# si una pieza deja de detectarse, es lo primero a revisar.
MIN_CONTOUR_AREA = 3600
MAX_CONTOUR_AREA = 43000

# Tamaño del filtro morfológico (apertura y después cierre).
#
# El CIERRE es el que importa acá, y tiene un efecto que no es obvio:
# tapa los huecos DENTRO de una pieza, pero también el hueco ENTRE dos
# piezas que se están tocando. Cuanto más grande el kernel, más
# gruesa queda soldada la unión, más difícil le resulta al watershed
# encontrar el cuello por donde cortar, y más seguido dos piezas
# salen como una sola mancha.
#
# Bajado de 7 a 5 (agosto 2026). Medido sobre una escena con tres
# pares de piezas del mismo color tocándose ligeramente (hexágono +
# cuadrado rojos, cuadrado + hexágono azules, dos círculos verdes),
# 40 fotogramas, contando cuántas veces se encuentran MENOS piezas de
# las que hay:
#
#     kernel   fallos de separación   control piezas sueltas
#        3            16                     60/60
#        4            14                     60/60
#        5            13                     60/60   <- este
#        6            16                     60/60
#        7            20                     60/60
#        9            45                     60/60
#
# Hay un mínimo claro en 5, y no es monótono: bajar más tampoco
# ayuda, porque un kernel chico deja la máscara sucia y el watershed
# empieza a partir piezas sanas.
#
# El control de la derecha es importante: sobre piezas SUELTAS este
# número no cambia nada (60/60 con cualquier kernel, y el área varía
# un 0,1 %). O sea que se gana en el caso difícil sin pagar en el
# fácil.
MORPH_KERNEL_SIZE = 5  # antes 7

# Suaviza el ruido fino de borde (pixeles sueltos justo al filo
# del umbral HSV, típico en superficies texturadas/matte).
# Difumina la máscara y la vuelve a binarizar; no cambia la forma
# general, solo promedia ese "festoneado" pixel a pixel.
SMOOTH_MASK_EDGES = True
MASK_SMOOTHING_KERNEL_SIZE = 7  # debe ser impar

# ----------------------------
# Clasificación de formas
# ----------------------------

# Tolerancia con la que se simplifica el contorno antes de contar
# vértices (fracción del perímetro que se le pasa a approxPolyDP).
# Cuanto más chica, más vértices sobreviven.
#
# NO BAJAR ESTE VALOR. Sobre figuras sintéticas parecía que 0.025
# separaba mejor, pero medido contra las piezas reales el cuadrado
# azul se cae por completo:
#
#   eps 0.035 -> cuadrado azul reconocido en 60/60 fotogramas
#   eps 0.025 -> cuadrado azul reconocido en  1/60 fotogramas
#
# El borde real de la máscara tiene bastante más ruido que el
# modelo sintético, así que con una tolerancia chica el cuadrado se
# simplifica a 5 o más vértices y deja de pasar la prueba.
#
# Con 0.035 los vértices que dan las piezas reales son: cuadrado 4,
# círculo 7-8. El hexágono no se separa por vértices sino por la
# fracción de llenado (ver más abajo).
SHAPE_APPROX_EPSILON_RATIO = 0.035

# Relación ancho/alto aceptada para un cuadrado.
SQUARE_ASPECT_RATIO_MIN = 0.75
SQUARE_ASPECT_RATIO_MAX = 1.25

# Circularidad mínima para considerar un objeto como círculo.
#
# Es la ÚLTIMA prueba de classify_shape(), o sea el cajón donde cae
# todo lo que no clasificó antes. Por eso importa que sea exigente:
# con 0.80 no era un umbral, era un colador.
#
# EL BUG QUE ARREGLA (agosto 2026). Se veían hexágonos yendo al tacho
# de los círculos. La sospecha era el techo del hexágono (0.90) y
# resultó ser exactamente al revés. Medido con tandas de 5 piezas de
# forma CONOCIDA, dos pasadas cada una, 1993 detecciones:
#
#   Los 27 hexágonos mal clasificados fallaban TODOS por el PISO del
#   llenado, no por el techo: caían entre 0.41 y 0.727 contra el
#   mínimo de 0.73. Ninguno pasó de 0.90.
#
# Un hexágono con la máscara comida --dos piezas del mismo color muy
# juntas que el watershed parte desparejo, o una pieza tocando el
# borde-- pierde llenado, se cae de la ventana del hexágono, y acá
# lo levantaba el 0.80 y lo llamaba círculo. La forma no estaba mal
# medida: estaba mal la red de contención.
#
# El umbral sale del hueco real entre las dos poblaciones:
#
#     hexágonos reales   circularidad máxima  0.9625  (p99 0.9600)
#     círculos reales    mediana 0.9919, p5 0.8321
#
# 0.965 queda por encima del hexágono más redondo que se midió.
# Barrido completo, contando los dos errores por separado:
#
#                    al tacho equivocado   sin clasificar
#   circ 0.80 (antes)         85                 63
#   circ 0.92                 41                116
#   circ 0.965 (este)         41                117
#   circ 0.98                 41                126
#
# Hay meseta entre 0.92 y 0.965, así que el número no es delicado.
# Por forma, las que iban al tacho equivocado: hexágonos 31 -> 4,
# cuadrados 18 -> 1, círculos 36 -> 36.
#
# LO QUE SE PAGA, y por qué conviene igual: los errores no se
# eliminan, se CONVIERTEN. Una pieza que antes salía con otra forma
# ahora no clasifica y sigue de largo por la cinta. Eso es mucho
# mejor que meterla en el tacho equivocado sin que nadie se entere,
# porque la caja mal armada no se nota hasta el final. Los círculos
# bien clasificados bajan de 90.9 % a 89.3 %.
#
# NO subirlo más: a 0.98 se empiezan a perder círculos buenos (el
# 8.7 % de las detecciones de círculo tiene circularidad < 0.965, y
# arriba de eso la proporción crece rápido).
#
# BAJADO DE 0.965 A 0.92, y el motivo es el otro caso difícil. Cuando
# el watershed separa dos piezas pegadas, el corte deja un borde
# RECTO donde la pieza real es curva, así que el pedazo legítimo sale
# con la circularidad castigada. Un umbral muy alto lo rechaza y la
# pieza se pierde -- justo en la situación en la que ya cuesta
# detectarla. Medido sobre la escena de piezas pegadas (120 casos):
#
#     circularidad   fallos con piezas pegadas   piezas al tacho
#                     (kernel morfológico 5)      equivocado
#        0.80                  9                       85
#        0.85                 11                       ~
#        0.92                 13                       41   <- este
#        0.965                19                       41
#
# 0.92 y 0.965 dan EXACTAMENTE lo mismo contra los hexágonos (41 en
# los dos), pero 0.92 recupera 6 detecciones de piezas pegadas. O sea
# que no es un punto medio de compromiso: 0.965 era peor en un caso e
# igual en el otro.
#
# El margen sigue estando: el hexágono más redondo de los que se
# caían de la ventana de llenado --que son los que llegan a esta
# prueba-- medía 0.903. Los hexágonos bien formados llegan a 0.9625,
# pero esos pasan por la ventana y nunca llegan hasta acá.
CIRCLE_CIRCULARITY_MIN = 0.92  # antes 0.80, después 0.965

# Condición principal del hexágono: qué fracción del círculo que la
# envuelve llena la figura. Separa mucho mejor que la circularidad,
# porque no depende del perímetro (que es justo lo que más ensucia
# el ruido de borde de la máscara).
#
# Medido sobre las piezas reales, 60 fotogramas:
#
#   CUADRADO azul   fill 0.671 - 0.721
#   CUADRADO rojo   fill 0.676 - 0.689
#   ---- ventana del hexágono ----
#   hexágono regular teórico:  0.827
#   ------------------------------
#   CIRCULO azul    fill 0.880 - 0.948
#   CIRCULO rojo    fill 0.958 - 0.973
#
# VERIFICADO CON HEXÁGONOS REALES (agosto 2026), y la ventana quedó
# como estaba. Tandas de 5 piezas de forma conocida, dos pasadas,
# 674 detecciones de hexágono:
#
#     hexágono real   llenado  p5 0.651   mediana 0.863   máx 0.899
#     cuadrado real   llenado  p5 0.656   mediana 0.687   máx 0.720
#     círculo real    llenado  p5 0.835   mediana 0.953   máx 0.978
#
# O sea que la ventana 0.73-0.90 estaba bien elegida: el techo no
# corta ni un hexágono (el más lleno dio 0.899) y el piso lo separa
# del cuadrado.
#
# OJO con la tentación de bajar el piso. Los hexágonos con la
# máscara comida caen por debajo de 0.73, así que bajar el piso para
# recuperarlos parece lo obvio -- pero el cuadrado llega a 0.720 con
# su p5 en 0.656, o sea que están pegados, y lo que se gana de un
# lado se pierde del otro. Medido, ya con la circularidad en 0.965 y
# contando las piezas que van al tacho equivocado:
#
#     piso    total   cuadrados  hexágonos  círculos
#     0.73      41        1          4         36     <- este
#     0.66      49        6          4         39
#     0.62      55       11          4         40
#
# Los hexágonos ya no mejoran (quedan en 4 con cualquier piso) y los
# cuadrados empeoran parejo. El que resuelve ese caso es
# CIRCLE_CIRCULARITY_MIN, que no toca esta separación.
#
# Si aparecen hexágonos clasificados como círculos, mirar PRIMERO la
# circularidad y no esta ventana: fue el error de diagnóstico que se
# cometió la primera vez.
HEXAGON_FILL_RATIO_MIN = 0.73
HEXAGON_FILL_RATIO_MAX = 0.90# antes 0.86

# Circularidad MÁXIMA de un hexágono. 1.0 desactiva la prueba y deja el
# comportamiento anterior.
#
# Es la condición que le faltaba a la ventana de llenado, y resuelve el
# caso que ninguna otra podía: un CÍRCULO MORDIDO. Cuando dos piezas se
# tocan y hay que cortarlas, al círculo le queda un pedazo menos y su
# llenado cae justo adentro de la ventana del hexágono. Medido sobre una
# escena con las tres formas pegadas, etiquetando cada pieza por su
# posición (así se sabe qué es cada una de verdad):
#
#                    llenado          circularidad
#     hexágono     0.817 - 0.848      0.927 - 0.940
#     círculo      0.812 - 0.934      0.965 - 0.992
#
# El llenado se solapa POR COMPLETO: con esa prueba sola, un círculo
# mordido y un hexágono son indistinguibles, y no es cuestión de mover el
# umbral. La circularidad, en cambio, deja un hueco limpio -- morderle un
# pedazo a un círculo le baja el área pero no lo vuelve menos redondo en
# el resto del borde.
#
# El valor sale del hueco entre las dos poblaciones, teniendo en cuenta
# también los hexágonos ENTEROS de la medición anterior, que llegaban a
# 0.9625. Barrido sobre dos escenas de piezas pegadas (240 casos):
#
#     techo    aciertos
#     0.945     132/240   <- empieza a rechazar hexágonos buenos
#     0.955     194/240
#     0.962     194/240   <- este, en el medio de la meseta
#     0.964     194/240
#     0.970     156/240   <- los círculos mordidos vuelven a colarse
#
# Contra 148/240 sin esta prueba. La meseta 0.955-0.964 es angosta pero
# real, y 0.962 es el único punto que además respeta el 0.9625 de los
# hexágonos enteros.
HEXAGON_CIRCULARITY_MAX = 0.962

# ----------------------------
# Rangos HSV por color
# ----------------------------

# Cada color es una lista de rangos (mínimo, máximo) en HSV. Son
# varios y no uno solo porque un color puede ocupar más de un sector
# del espacio HSV (el caso del rojo, que está partido en las dos
# puntas de la rueda de Hue).
#
# Para agregar un color nuevo alcanza con sumar una entrada acá: el
# resto del sistema recorre este diccionario, no hay ninguna lista
# de colores hardcodeada en otro lado. Para calibrarlo está
# hsv_calibrator.py, que arma sus presets a partir de esta tabla.
COLOR_HSV_RANGES = {
    # Rojo. Confirmado con calibración en vivo y muy estable: es el
    # color que mejor se separa de la cinta.
    #
    # El primer rango cubre el sector cerca de H=0. Hoy no aporta
    # píxeles nuevos (quedó redundante con el segundo), pero se deja
    # cubriendo la otra punta de la rueda por si cambia la luz.
    "ROJO": (
        ((0, 50, 80), (15, 255, 255)),
        ((145, 35, 100), (179, 255, 255)),
    ),
    # Celeste/azul. Es el color difícil: la cinta gris tiene H≈95 de
    # mediana, o sea pegado al azul, así que el límite inferior de H
    # es lo único que separa la pieza del fondo y de las sombras.
    #
    # Rango elegido comparando candidatos sobre 60 fotogramas, y
    # midiendo CALIDAD DE FORMA, no cobertura de píxeles. La
    # diferencia importa: ensanchar el rango sube la cantidad de
    # píxeles capturados pero mete el halo difuso del borde, ablanda
    # el contorno y termina empeorando la clasificación.
    #
    #                              cuadrado OK   fill del círculo
    #   (90,90,100)-(115,200,255)     60/60          0.908   <- viejo
    #   (97,70, 80)-(110,255,255)     54/60          0.872
    #   (94,85, 90)-(112,255,255)     53/60          0.899
    #   (96,95,100)-(112,255,255)     60/60          0.914   <- este
    #
    # Las dos diferencias contra el viejo:
    #
    #   H arranca en 96 y no en 90 -> la cinta gris tiene H≈95 de
    #       mediana, así que el rango viejo la tenía adentro. Esto
    #       es lo que hacía que se confundiera con luces y sombras.
    #   S llega a 255 y no a 200   -> recupera las zonas más
    #       intensas de la pieza, que el tope viejo recortaba.
    "AZUL": (
        ((98, 95, 100), (112, 255, 255)),
    ),
    # Verde. Recalibrado para el filamento verde PURO (agosto 2026).
    # El rango anterior estaba hecho sobre la pieza verde CHILLÓN, la
    # que se sobreexponía; con el filamento nuevo ese rango no da una
    # detección peor, da CERO: 0/120 círculos, medido.
    #
    # Medido con la cinta detenida, 60 fotogramas, dos círculos por
    # fotograma, sobre el núcleo erosionado de cada pieza y no sobre
    # el borde (el borde es una franja mezclada con la cinta, y da un
    # valor que no es ni de la pieza ni del fondo):
    #
    #                    H (p5-p95)   S (p5-p95)   V (p5-p95)
    #     pieza verde       63-69        82-98      176-193
    #     cinta gris        84-99        17-50      122-161
    #
    # Contra el verde viejo cambian dos cosas, y la segunda da vuelta
    # el criterio entero:
    #
    #   H bajó de 74-77 a 63-69. Eso es "más puro": más cerca del
    #       verde primario (60).
    #   V bajó de 255 a ~180, o sea que la pieza YA NO SATURA. Y ahí
    #       se cae la regla vieja. V era EL número porque la cinta
    #       llegaba a 174 y la pieza estaba clavada en 255; hoy la
    #       cinta pica hasta V=220 en los bordes claros del recorte y
    #       la pieza está en 180. V YA NO SEPARA NADA: queda sólo
    #       como piso contra la sombra profunda.
    #
    # Separan H y S, y cada una alcanza SOLA. Medido en píxeles de
    # cinta que entran a la máscara, que es el margen que la columna
    # "falsas" no muestra: una falsa recién aparece cuando la fuga
    # junta los 3600 px² de MIN_CONTOUR_AREA, así que un rango puede
    # estar a un pelo de fallar y mostrar igual "falsas 0".
    #
    #   H≤74 sola        ->     0 px de cinta, incluso con S≥34
    #   S≥50 sola        ->     0 px de media (pico de 2), con H≤86
    #   S≥40 con H≤100   -> 25.000 px por fotograma (así se ve fugar)
    #
    # Barrido sobre los mismos 60 fotogramas --los mismos, no una
    # captura nueva por candidato: entre dos rangos parecidos, la
    # diferencia sería ruido de luz-- midiendo calidad de forma y no
    # cantidad de píxeles capturados. Es la misma lección que el
    # azul: ensanchar siempre sube los píxeles, mete el halo del
    # borde, ablanda el contorno y el círculo termina HEXÁGONO.
    #
    #                             círculo OK   llenado (mín)  falsas
    #   (55,60,200)-(85,255,255)     0/120      0.000 (0.00)    0  <- el viejo
    #   (50,65,180)-(80,255,255)    48/120      0.602 (0.53)    0
    #   (50,68,100)-(74,255,255)   120/120      0.939 (0.72)    0
    #   (50,52,100)-(74,255,255)   120/120      0.940 (0.92)    0
    #   (50,56,100)-(74,255,255)   120/120      0.939 (0.92)    0  <- este
    #   (50,60,100)-(74,255,255)   120/120      0.939 (0.91)    0
    #   (50,56,100)-(76,255,255)   120/120      0.942 (0.92)    0
    #
    # Los tres umbrales del mínimo, y por qué cada uno:
    #
    #   H=50-74 es la barrera principal. La pieza está en 63-69 y la
    #       cinta empieza en 84. El techo 74 además recorta el halo
    #       del borde, y es lo que sube el llenado; de 78 para arriba
    #       el llenado deja de mejorar y empieza a caer, y con el
    #       piso de S un poco más alto aparece el primer HEXÁGONO. El
    #       piso 50 es plano de 36 a 60 (no hay nada más en esa
    #       franja de Hue) y queda 10 por debajo del mínimo de la
    #       pieza.
    #   S=56  segunda barrera, independiente de la primera. La fuga
    #       de cinta se corta sola entre S=45 (7 px por fotograma) y
    #       S=50 (0), y la pieza no baja de S=61, así que 56 cae en
    #       el medio del hueco. No subirlo: de 68 para arriba empieza
    #       a agujerear la pieza y el llenado mínimo se desploma a
    #       0,72.
    #   V=100 ya no separa nada, es sólo un piso contra la sombra
    #       profunda. Está deliberadamente LEJOS de la pieza (176):
    #       el rango viejo murió justo por tener el piso de V pegado
    #       al valor de la pieza, y ese modo de falla es silencioso y
    #       total --no detecta peor, no detecta nada.
    #
    # Todo el vecindario del elegido da 120/120 con llenado ~0.94: es
    # una meseta y no una punta, que es justo lo que le faltaba al
    # rango anterior. Si alguna vez hay que moverlo, ese es el
    # chequeo: que los vecinos también den bien.
    #
    # (El pronóstico que estaba escrito acá salió mitad y mitad: el
    # piso de V efectivamente tuvo que bajar, pero el de S no pudo
    # subir. La saturación del verde puro resultó igual a la del
    # chillón, 82-98 contra 78-109.)
    "VERDE": (
        ((50, 56, 100), (74, 255, 255)),
    ),
}

# ----------------------------
# Seguimiento de objetos
# ----------------------------

# Distancia máxima en píxeles para considerar
# que una detección pertenece al mismo objeto.
# También reescalado 1.2x por el cambio a 720p.
MAX_TRACK_DISTANCE = 110.0

# Cantidad máxima de fotogramas en los que un objeto
# puede desaparecer antes de eliminar su ID.
MAX_MISSED_FRAMES = 10

# ----------------------------
# Separación de piezas que se tocan
# ----------------------------

# Cuando dos piezas del mismo color se tocan, la máscara las une
# en un solo blob. Para separarlas se buscan los máximos locales
# de la transformada de distancia (los puntos más alejados del
# borde): cada pieza real tiene su propio pico cerca de su
# centro, sin importar si es más chica o más grande que la otra.

# Separación mínima en píxeles entre dos picos para considerarlos
# piezas distintas. Aproximadamente el radio de la pieza más chica.
#
# Muy alto -> puede fusionar dos piezas chicas en una sola.
# Muy bajo -> puede detectar varios picos falsos dentro de una
# sola pieza (por textura o ruido) y separarla de más.
WATERSHED_MIN_PEAK_DISTANCE = 18

# Distancia mínima al borde (en píxeles) para que un pico cuente
# como centro real. Filtra ruido cerca del borde de la máscara.
WATERSHED_MIN_PEAK_HEIGHT = 5

# Limpieza de cada pedazo DESPUÉS de cortar. 0 lo desactiva.
#
# El watershed corta bien, pero deja en cada pedazo una PÚA fina
# apuntando al otro, sobre la línea del corte. Esa púa no se ve casi
# en la máscara y arruina la clasificación, porque classify_shape()
# trabaja sobre el CASCO CONVEXO: el casco se traga la púa, el
# círculo envolvente crece para cubrirla, y el llenado se desploma.
#
# Medido sobre dos círculos verdes pegados:
#
#     sin limpieza    llenado 0.744 y 0.764  ->  los dos HEXÁGONO
#     limpieza 13     llenado 0.825 y 0.853  ->  los dos CÍRCULO
#
# O sea que el síntoma no era "no separa las piezas" --las separaba
# bien-- sino "las separa y las clasifica mal". Una pieza contada
# pero con la forma equivocada va al tacho equivocado igual.
#
# Una apertura morfológica es justo la herramienta: borra lo que sea
# más fino que el kernel (la púa) y deja intacto lo que es más ancho
# (el disco). El costo es un 1 % de área.
SPLIT_CLEANUP_KERNEL_SIZE = 13

# NOTA sobre lo que YA NO está acá: la ventana de OpenCV, la bandera para
# mostrar las máscaras y la configuración del puerto serie. Vivían en este
# archivo cuando la visión era un programa suelto (`main.py`) que abría su
# propia ventana y le hablaba al ESP32 por su cuenta. Hoy el bucle de visión
# es `pc/kuko/vision.py`: no dibuja ninguna ventana --deja el fotograma
# anotado en memoria y lo sirve como MJPEG-- y no toca el puerto, que es del
# enlace (`pc/kuko/enlace.py`, donde también viven el puerto y los baudios).
#
# Para mirar las máscaras está `hsv_calibrator.py`, que además deja mover
# los rangos en vivo.

# ============================================================
# RECORTE DE LA CINTA
# ============================================================

# La cámara está bastante elevada y ve mucho más que la cinta
# (mesa, herramientas, enchufes). En vez de tapar esa zona con una
# máscara, directamente se RECORTA el fotograma: lo que queda es
# solo la cinta, a resolución nativa (no se reescala nada, no se
# pierde detalle) y encima se procesan menos píxeles por fotograma.
#
# Los valores son fracciones del fotograma YA ROTADO. Como la cinta
# se ve horizontal, el recorte que importa es el vertical: hay que
# dejar afuera lo que está por arriba y por debajo de la cinta.
#
# Para ajustarlos sin ir a prueba y error está crop_calibrator.py:
# muestra la imagen completa con el recorte encima y cuatro sliders,
# e imprime estos mismos valores listos para pegar acá.
CROP_ENABLED = True

# Límites horizontales. Por la izquierda entra la mesa de madera,
# así que se recorta; por la derecha la cinta llega hasta el borde
# de la imagen y no hay nada que sacar.
CROP_X_MIN_RATIO = 0.155
CROP_X_MAX_RATIO = 0.985

# Límites verticales: bordes superior e inferior de la cinta. Es lo
# que saca de la imagen la mesa, el enchufe y la máquina del fondo.
CROP_Y_MIN_RATIO = 0.365
CROP_Y_MAX_RATIO = 0.675

# ============================================================
# COORDENADAS DEL ROBOT (píxeles -> centímetros)
# ============================================================

# Lo que se muestra en pantalla y lo que se le manda al ESP32 son
# centímetros en el sistema de referencia del ROBOT, no píxeles.
# La conversión vive en coordinates.py y sale de las cuatro medidas
# de acá abajo, todas tomadas con regla sobre la cinta.
#
# El eje Y queda invertido respecto de OpenCV, que mide hacia abajo:
# acá Y crece hacia ARRIBA de la imagen, como en el robot.
#
# IMPORTANTE: las cuatro dependen del recorte. Si se tocan los
# CROP_*_RATIO hay que volver a medir, porque el fotograma pasa a
# abarcar otro pedazo de cinta.

# Cuánto abarca el fotograma YA RECORTADO, a lo largo de la cinta.
IMAGE_WIDTH_CM = 21.5

# Cuánto abarca a lo ancho. Hoy el recorte está justo al ancho de la
# cinta, así que es el ancho de la cinta.
IMAGE_HEIGHT_CM = 14.0

# Ancla del eje X: en qué X del robot cae la línea de detección.
# Está detrás del robot, por eso es negativa.
LINE_X_CM = -23.0

# Ancla del eje Y: en qué Y del robot cae el borde INFERIOR de la
# imagen. El borde superior queda entonces en
# IMAGE_BOTTOM_Y_CM + IMAGE_HEIGHT_CM = 12.05 cm.
# Calibrado contra el robot: con -2.8 el gripper agarraba 0,85 cm corrido,
# y habia que compensarlo moviendo el recorte 24 px (que ademas desencuadraba
# la imagen). El encuadre no era el problema: lo que estaba mal medido era
# donde cae el borde inferior de la imagen en el sistema del robot.
#
#     24 px * (14,0 cm / 396 px) = 0,85 cm
#
# OJO: este valor tiene su par en el firmware (BELT_MIN_Y en Robot.cpp) y los
# dos tienen que moverse juntos, o el firmware empieza a rechazar piezas de
# un borde de la cinta por creerlas fuera de rango.
#
# Y POR ESO NO SE TOCA MAS PARA RECENTRAR EL AGARRE: el desencuadre de cada
# vez que se mueve la camara o se vuelve a armar la cinta se corrige con el
# parametro 'vis_dy' de la ventana de Servicio, que corre la Y informada sin
# mover ninguno de estos dos numeros duplicados ni la ventana de recorte.
# Esto es la MEDICION de donde esta el borde; aquello es el ajuste fino.
IMAGE_BOTTOM_Y_CM = -1.95