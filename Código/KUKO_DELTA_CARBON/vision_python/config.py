# ============================================================
# CONFIGURACIÓN GENERAL DEL SISTEMA DE VISIÓN
# ============================================================

# ----------------------------
# Cámara
# ----------------------------

CAMERA_INDEX = 1

CAMERA_WIDTH = 480
CAMERA_HEIGHT = 640
CAMERA_FPS = 30

WINDOW_NAME = "KUKO - Vision artificial"

# Rotación aplicada a cada fotograma apenas se captura, antes de
# cualquier procesamiento. Usar si la cámara queda montada girada
# respecto a la cinta.
#
# None                    -> sin rotación
# "90_CLOCKWISE"          -> 90° en sentido horario
# "90_COUNTERCLOCKWISE"   -> 90° en sentido antihorario
# "180"                   -> 180°
CAMERA_ROTATION = "90_CLOCKWISE"

# ============================================================
# LÍNEA DE DETECCIÓN
# ============================================================

# Posición vertical de la línea respecto de la imagen completa.
#
# 0.00 = parte superior
# 0.50 = centro
# 1.00 = parte inferior
LINE_Y_RATIO = 0.50

# Sentido de movimiento de la cinta:
#
# "BOTTOM_TO_TOP" → de abajo hacia arriba
# "TOP_TO_BOTTOM" → de arriba hacia abajo
LINE_DIRECTION = "BOTTOM_TO_TOP"

# ----------------------------
# Detección de objetos
# ----------------------------

# Área mínima y máxima aceptada para un contorno.
MIN_CONTOUR_AREA = 2500
MAX_CONTOUR_AREA = 30000

# Tamaño del filtro morfológico.
MORPH_KERNEL_SIZE = 7

# Suaviza el ruido fino de borde (pixeles sueltos justo al filo
# del umbral HSV, típico en superficies texturadas/matte).
# Difumina la máscara y la vuelve a binarizar; no cambia la forma
# general, solo promedia ese "festoneado" pixel a pixel.
SMOOTH_MASK_EDGES = True
MASK_SMOOTHING_KERNEL_SIZE = 7  # debe ser impar

# Relación ancho/alto aceptada para un cuadrado.
SQUARE_ASPECT_RATIO_MIN = 0.75
SQUARE_ASPECT_RATIO_MAX = 1.25

# Circularidad mínima para considerar un objeto como círculo.
CIRCLE_CIRCULARITY_MIN = 0.80

# ----------------------------
# Rangos HSV
# ----------------------------

# El rojo ocupa dos sectores diferentes del espacio HSV.
#
# Rango 1: margen de seguridad cerca de H=0. En la calibración
# actual no aportó pixeles nuevos (quedó redundante con el rango 2,
# ver notas de calibración), pero se deja cubriendo la otra punta
# de la rueda de Hue por si la iluminación cambia.
RED_LOWER_1 = (0, 50, 80)
RED_UPPER_1 = (15, 255, 255)

# Rango 2: confirmado con calibración en vivo (piezas reales,
# iluminación de banco).
RED_LOWER_2 = (145, 35, 100)
RED_UPPER_2 = (179, 255, 255)

# Azul: confirmado con calibración en vivo, ajustado para cubrir
# tanto la pieza lisa como el cuadrado de terminación matte.
BLUE_LOWER = (90, 90, 100)
BLUE_UPPER = (115, 200, 255)

# ----------------------------
# Seguimiento de objetos
# ----------------------------

# Distancia máxima en píxeles para considerar
# que una detección pertenece al mismo objeto.
MAX_TRACK_DISTANCE = 90.0

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
WATERSHED_MIN_PEAK_DISTANCE = 15

# Distancia mínima al borde (en píxeles) para que un pico cuente
# como centro real. Filtra ruido cerca del borde de la máscara.
WATERSHED_MIN_PEAK_HEIGHT = 5

# ----------------------------
# Visualización
# ----------------------------

SHOW_COLOR_MASKS = True

# ----------------------------
# Comunicación serial
# ----------------------------

# Dejar en False mientras probamos únicamente la cámara.
SERIAL_ENABLED = False

SERIAL_PORT = "COM5"
SERIAL_BAUDRATE = 115200

# ============================================================
# REGIÓN DE INTERÉS DE LA CINTA
# ============================================================

# Límites horizontales del área que se procesará.
ROI_X_MIN_RATIO = 0.35
ROI_X_MAX_RATIO = 0.70

# Límites verticales del área que se procesará.
ROI_Y_MIN_RATIO = 0.02
ROI_Y_MAX_RATIO = 0.96