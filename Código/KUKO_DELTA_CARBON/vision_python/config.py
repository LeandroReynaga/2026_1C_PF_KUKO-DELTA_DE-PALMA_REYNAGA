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

WINDOW_NAME = "KUKO - Vision artificial"

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
#   main.py usa int(598 * ratio): 0.60 -> 358 px, 0.576 -> 344 px
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

# Tamaño del filtro morfológico.
MORPH_KERNEL_SIZE = 7

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
# OJO: un hexágono regular tiene circularidad 0.91, o sea que este
# umbral NO alcanza para separarlo de un círculo. Por eso el
# hexágono se prueba antes, contando vértices.
CIRCLE_CIRCULARITY_MIN = 0.80

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
# La ventana de abajo entra justo en el hueco, con margen para los
# dos lados. Si aparecen hexágonos clasificados como círculos,
# subir el máximo; si un círculo pasa por hexágono, bajarlo.
#
# PENDIENTE DE VERIFICAR CON UN HEXÁGONO REAL: los límites salen de
# piezas cuadradas y redondas más el valor teórico, porque todavía
# no hay una pieza hexagonal impresa para medir.
HEXAGON_FILL_RATIO_MIN = 0.73
HEXAGON_FILL_RATIO_MAX = 0.90# antes 0.86

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
    # Verde. Calibrado sobre la pieza CLARA, que es la problemática:
    # es tan brillante que se sobreexpone y llega a V=255 en toda su
    # superficie. Aun así conserva color (S≈99), así que se detecta
    # bien; lo que la hacía fallar no era el color sino el BORDE.
    #
    # Medido con la cinta quieta, 30 fotogramas por zona:
    #
    #                    H (p5-p95)   S (p5-p95)   V (p5-p95)
    #     pieza verde       74-77       78-109      255-255
    #     cinta gris        82-95       31-56       138-174
    #
    # El problema estaba en el piso de V. Con V bajo entraba en la
    # máscara el halo de píxeles de CINTA que rodea la pieza, y ese
    # halo le comía el contorno: el círculo dejaba de llenar el 90%
    # de su círculo envolvente y classify_shape lo leía HEXÁGONO
    # (ver HEXAGON_FILL_RATIO_MAX). O sea que el síntoma no era
    # "no la detecta" sino "la detecta con la forma equivocada".
    #
    # Barrido sobre 60 fotogramas, midiendo lo mismo que se midió
    # para elegir el rango del azul (calidad de forma, no cantidad
    # de píxeles capturados):
    #
    #                            círculo OK   llenado (mín)   falsas
    #   (40,60,60)-(85,255,255)     50/60       0.896 (0.50)     0
    #   (40,50,80)-(85,255,255)     25/60       0.761 (0.62)     5
    #   (55,55,200)-(85,255,255)    60/60       0.974 (0.97)     0
    #   (55,60,200)-(85,255,255)    60/60       0.974 (0.97)     0  <- este
    #   (60,60,190)-(85,255,255)    60/60       0.972 (0.97)     0
    #
    # Los tres umbrales del mínimo, y por qué cada uno:
    #
    #   V=200  es EL número. La cinta llega a V=174, así que 200 la
    #       deja afuera con margen. Y hay acantilado: con V=170 se
    #       cae a 12/25 círculos. La pieza está clavada en 255, así
    #       que subir hasta 200 no le cuesta nada.
    #   S=60   segunda barrera, apenas por encima del techo de la
    #       cinta (56). No se puede subir mucho más: la pieza tiene
    #       zonas de S≈78 y se empezaría a agujerear.
    #   H=55-85 tercera barrera. La pieza está en 74-77, bien
    #       centrada, y el techo 85 queda por debajo del piso de Hue
    #       de la cinta (82-95). No subirlo.
    #
    # Cuando lleguen las piezas de verde OSCURO Y PURO hay que
    # rehacer esto: van a tener S alta y V media, o sea que el piso
    # de V va a tener que BAJAR bastante (200 las dejaría afuera) y
    # el de S va a poder subir. El procedimiento es este mismo:
    # hsv_calibrator.py para los números, y después mirar cuántos
    # círculos de 60 salen bien, que es lo único que importa.
    "VERDE": (
        ((55, 60, 200), (85, 255, 255)),
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

# ----------------------------
# Visualización
# ----------------------------

# Cada ventana extra cuesta unos milisegundos por fotograma. Para
# mirar las máscaras conviene hsv_calibrator.py, que además deja
# tocar los rangos en vivo.
SHOW_COLOR_MASKS = False

# ----------------------------
# Comunicación serial
# ----------------------------

# Con esto en True, cada pieza que cruza la línea se le manda sola
# al ESP32. Poner en False para probar únicamente la cámara, sin
# robot conectado: la visión sigue funcionando igual.
SERIAL_ENABLED = True

# "AUTO" busca el puerto del ESP32 por el chip USB-serie de la placa
# (CP210x, CH340, FTDI). Conviene dejarlo así: Windows le cambia el
# número de COM según el puerto USB donde se enchufe, y además
# descarta solo los COM de Bluetooth, que también aparecen listados.
#
# Si hubiera más de una placa conectada, poner el puerto a mano
# ("COM5", "COM7", ...).
SERIAL_PORT = "AUTO"

SERIAL_BAUDRATE = 115200

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
# IMAGE_BOTTOM_Y_CM + IMAGE_HEIGHT_CM = 11.2 cm.
IMAGE_BOTTOM_Y_CM = -2.8