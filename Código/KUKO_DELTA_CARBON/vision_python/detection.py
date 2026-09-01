from __future__ import annotations

import time                       # TEMPORAL: solo para LOG_FORMAS
from dataclasses import dataclass
from math import pi
from pathlib import Path          # TEMPORAL: solo para LOG_FORMAS

import cv2
import numpy as np

import config

# El MODULO, no los valores: `from config import X` copia el numero UNA VEZ
# al importar, y despues de eso cambiarlo en config.py no cambia nada aca.
# Eso alcanzaba mientras la unica forma de tocar un umbral era editar el
# archivo y reiniciar el programa, pero la pestana de Vision los mueve con
# la camara andando, y ahi la copia congelada es exactamente el bug: los
# sliders se mueven, la deteccion no cambia y no hay nada en pantalla que
# lo explique. Leyendo `config.X` en el punto de uso, escribir el atributo
# del modulo se ve en el fotograma siguiente.
#
# El costo es una busqueda de atributo por uso, que al lado de un inRange
# sobre 240.000 pixeles no se mide.


@dataclass
class Detection:
    """Información obtenida para una pieza en un fotograma."""

    center: tuple[int, int]

    color: str
    shape: str

    bbox: tuple[int, int, int, int]

    contour: object

    area: float
    circularity: float


def clean_mask(mask: np.ndarray) -> np.ndarray:
    """Elimina puntos pequeños y completa huecos en la máscara."""

    kernel = np.ones(
        (config.MORPH_KERNEL_SIZE, config.MORPH_KERNEL_SIZE),
        dtype=np.uint8,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel,
    )

    if config.SMOOTH_MASK_EDGES:
        # Una superficie texturada/matte, con un rango HSV bien
        # ajustado, deja pixeles sueltos justo al filo del umbral
        # en el borde de la pieza ("festoneado"). Difuminar la
        # máscara y volver a binarizarla promedia ese ruido fino
        # sin cambiar la forma general de la pieza.
        blurred = cv2.GaussianBlur(
            mask,
            (config.MASK_SMOOTHING_KERNEL_SIZE, config.MASK_SMOOTHING_KERNEL_SIZE),
            0,
        )

        _, mask = cv2.threshold(
            blurred,
            127,
            255,
            cv2.THRESH_BINARY,
        )

    return mask


def create_color_masks(
    hsv_image: np.ndarray,
) -> dict[str, np.ndarray]:
    """Crea una máscara por cada color de COLOR_HSV_RANGES.

    No hay colores hardcodeados: se recorre la tabla de config.py,
    así que sumar un color nuevo (verde, amarillo, lo que sea) es
    agregar una entrada allá y nada más.
    """

    masks: dict[str, np.ndarray] = {}

    for color, hsv_ranges in config.COLOR_HSV_RANGES.items():
        color_mask: np.ndarray | None = None

        # Un color puede estar partido en varios sectores del
        # espacio HSV (el rojo). Se unen todos sus rangos.
        for lower, upper in hsv_ranges:
            range_mask = cv2.inRange(
                hsv_image,
                np.array(lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8),
            )

            if color_mask is None:
                color_mask = range_mask
            else:
                color_mask = cv2.bitwise_or(
                    color_mask,
                    range_mask,
                )

        if color_mask is not None:
            masks[color] = clean_mask(color_mask)

    return masks


def classify_shape(
    contour: object,
) -> tuple[str | None, float]:
    """Clasifica un contorno como cuadrado, hexágono o círculo.

    Se usa el casco convexo (convex hull) del contorno antes de
    medir circularidad y vértices. Un brillo/reflejo sobre la
    pieza genera una "mordida" cóncava en la máscara que reduce
    artificialmente la circularidad y puede alterar la cantidad
    de vértices detectados. Como las piezas reales son siempre
    convexas, esto ignora ese ruido sin cambiar la forma real.

    El orden de las comprobaciones importa: el hexágono se prueba
    ANTES que el círculo. Un hexágono regular tiene circularidad
    0.91, muy por encima del umbral de círculo, así que si se
    probara el círculo primero todo hexágono caería ahí.
    """

    hull = cv2.convexHull(contour)

    area = cv2.contourArea(hull)
    perimeter = cv2.arcLength(hull, True)

    if perimeter <= 0:
        return None, 0.0

    circularity = (
        4.0 * pi * area
    ) / (perimeter * perimeter)

    approximation = cv2.approxPolyDP(
        hull,
        config.SHAPE_APPROX_EPSILON_RATIO * perimeter,
        True,
    )

    vertices = len(approximation)

    # ----------------------------
    # Comprobación de cuadrado
    # ----------------------------

    if vertices == 4 and cv2.isContourConvex(approximation):
        _, _, width, height = cv2.boundingRect(approximation)

        if height > 0:
            aspect_ratio = width / float(height)

            if (
                config.SQUARE_ASPECT_RATIO_MIN
                <= aspect_ratio
                <= config.SQUARE_ASPECT_RATIO_MAX
            ):
                return "CUADRADO", circularity

    # ----------------------------
    # Comprobación de hexágono
    # ----------------------------

    # Se aceptan 5 y 7 vértices además de 6: con el borde de la
    # máscara sucio, una esquina del hexágono se puede perder o
    # partir en dos.
    #
    # Son TRES condiciones y cada una ataja algo distinto:
    #
    #   vértices   descarta el cuadrado y el ruido grueso.
    #   llenado    separa el hexágono del cuadrado (0.69) por abajo.
    #   circularidad  separa el hexágono del CÍRCULO MORDIDO por arriba,
    #       y es la única que puede. Un círculo al que hubo que cortarle
    #       un pedazo --porque estaba pegado a otra pieza-- pierde área y
    #       su llenado cae justo adentro de esta ventana: medido, círculos
    #       mordidos entre 0.812 y 0.934 contra hexágonos entre 0.817 y
    #       0.848, o sea solapamiento total. En circularidad, en cambio,
    #       quedan separados (0.965-0.992 contra 0.927-0.940): morderle un
    #       pedazo a un círculo le baja el área, pero no lo vuelve menos
    #       redondo en el resto del borde.
    if vertices in (5, 6, 7) and circularity <= config.HEXAGON_CIRCULARITY_MAX:
        _, enclosing_radius = cv2.minEnclosingCircle(hull)

        if enclosing_radius > 0:
            fill_ratio = area / (
                pi * enclosing_radius * enclosing_radius
            )

            if (
                config.HEXAGON_FILL_RATIO_MIN
                <= fill_ratio
                <= config.HEXAGON_FILL_RATIO_MAX
            ):
                return "HEXAGONO", circularity

    # ----------------------------
    # Comprobación de círculo
    # ----------------------------

    if circularity >= config.CIRCLE_CIRCULARITY_MIN:
        return "CIRCULO", circularity

    return None, circularity


# ======================================================================
#  DIAGNOSTICO TEMPORAL DE FORMAS  --  BORRAR JUNTO CON LOG_FORMAS
# ======================================================================

_registro = {"archivo": None, "sin_volcar": 0}


def _registrar_forma(color, contour, shape) -> None:
    """Deja en un CSV los números con los que se decidió esta forma.

    Rehace la cuenta de classify_shape() en vez de que classify_shape la
    devuelva: es código temporal y no vale la pena cambiarle la firma a una
    función que anda. Se borra junto con LOG_FORMAS, así que no hay riesgo
    de que las dos cuentas se separen con el tiempo.
    """

    hull = cv2.convexHull(contour)

    area = cv2.contourArea(hull)
    perimeter = cv2.arcLength(hull, True)

    if perimeter <= 0:
        return

    circularity = (4.0 * pi * area) / (perimeter * perimeter)

    vertices = len(cv2.approxPolyDP(
        hull, config.SHAPE_APPROX_EPSILON_RATIO * perimeter, True))

    _, enclosing_radius = cv2.minEnclosingCircle(hull)
    fill_ratio = (area / (pi * enclosing_radius * enclosing_radius)
                  if enclosing_radius > 0 else 0.0)

    if _registro["archivo"] is None:
        ruta = Path(__file__).resolve().parents[1] / config.LOG_FORMAS_ARCHIVO
        nuevo = not ruta.exists()

        _registro["archivo"] = open(ruta, "a", encoding="utf-8")

        if nuevo:
            _registro["archivo"].write(
                "hora,color,forma,vertices,llenado,circularidad,area\n")

        print(f"[FORMAS] midiendo en {ruta}")

    _registro["archivo"].write(
        f"{time.time():.3f},{color},{shape or 'NINGUNA'},{vertices},"
        f"{fill_ratio:.4f},{circularity:.4f},{area:.0f}\n")

    # Volcado cada tanto y no en cada linea: son ~90 detecciones por segundo
    # con tres piezas en el cuadro. Cada 50 se pierde medio segundo si
    # alguien mata el programa, que no importa para esto.
    _registro["sin_volcar"] += 1

    if _registro["sin_volcar"] >= 50:
        _registro["archivo"].flush()
        _registro["sin_volcar"] = 0


# ======================================================================


def split_touching_blob(
    mask: np.ndarray,
    contour: np.ndarray,
) -> list[np.ndarray]:
    """Separa un contorno que puede contener varias piezas tocándose.

    Se recorta la zona del contorno, se calcula la transformada de
    distancia (qué tan lejos está cada pixel blanco del borde) y se
    buscan los picos de esa distancia: cada pieza real tiene su
    propio pico cerca de su centro, mientras que el punto de
    contacto entre dos piezas es un "cuello" con distancia baja.
    Esos picos se usan como semillas para watershed, que corta el
    blob exactamente por el cuello.

    Si el contorno resulta ser una sola pieza (un solo pico), se
    devuelve sin modificar.
    """

    x, y, width, height = cv2.boundingRect(contour)

    margin = 4

    x0 = max(x - margin, 0)
    y0 = max(y - margin, 0)
    x1 = min(x + width + margin, mask.shape[1])
    y1 = min(y + height + margin, mask.shape[0])

    blob_mask = np.zeros(
        (y1 - y0, x1 - x0),
        dtype=np.uint8,
    )

    cv2.drawContours(
        blob_mask,
        [contour],
        -1,
        255,
        -1,
        offset=(-x0, -y0),
    )

    distance = cv2.distanceTransform(
        blob_mask,
        cv2.DIST_L2,
        5,
    )

    if distance.max() <= 0:
        return [contour]

    # En vez de comparar contra un porcentaje del máximo de todo
    # el blob (lo cual falla cuando las dos piezas tienen tamaños
    # distintos), se buscan los máximos LOCALES de la transformada
    # de distancia: cada pieza tiene su propio pico cerca de su
    # centro, sin importar si es más chica o más grande que la
    # otra pieza con la que se está tocando.
    #
    # El pico real no es un único pixel matemáticamente exacto,
    # es una "mesetita" de varios pixeles con valores casi
    # iguales. Comparar con igualdad exacta de punto flotante
    # puede partir esa mesetita en dos o más islas por simple
    # error de redondeo interno, haciendo parecer que hay dos
    # piezas cuando en realidad es una sola. Redondear antes de
    # comparar evita ese falso positivo.
    distance_rounded = np.round(distance).astype(np.int32)

    kernel_size = 2 * config.WATERSHED_MIN_PEAK_DISTANCE + 1

    dilated_distance = cv2.dilate(
        distance_rounded.astype(np.float32),
        np.ones((kernel_size, kernel_size), dtype=np.uint8),
    ).astype(np.int32)

    peaks = (
        (distance_rounded == dilated_distance)
        & (distance_rounded > config.WATERSHED_MIN_PEAK_HEIGHT)
    )

    sure_foreground = (
        peaks.astype(np.uint8) * 255
    )

    num_markers, markers = cv2.connectedComponents(
        sure_foreground,
    )

    # Un solo pico (más fondo): no hay nada que separar.
    if num_markers <= 2:
        return [contour]

    # Cada pixel de la mancha va al pico MAS CERCANO.
    #
    # Antes esto lo hacia cv2.watershed() sobre blob_mask, y ahi habia un
    # problema de fondo: watershed inunda siguiendo el RELIEVE de la imagen,
    # y blob_mask es binaria, o sea completamente plana por dentro. Sin
    # relieve no hay cresta que seguir y el resultado que da es justamente
    # este reparto por cercania -- pero pagando dos costos que no hacen
    # falta:
    #
    #   - marca los pixeles de frontera con -1 y esos quedan afuera de las
    #     dos piezas, o sea una linea de pixeles perdidos en cada corte;
    #   - deja sobre la linea del corte una PUA fina apuntando a la otra
    #     pieza, y classify_shape() mira el CASCO CONVEXO: el casco se traga
    #     la pua, el circulo envolvente crece para cubrirla y el llenado se
    #     derrumba. Dos circulos pegados salian con llenado 0.74 y se
    #     clasificaban HEXAGONO los dos.
    #
    # Medido sobre dos escenas de piezas pegadas, este reparto explicito da
    # 52/120 contra 48/120 del watershed, y con el techo de circularidad del
    # hexagono la diferencia se agranda (92/120 contra 54/120).
    #
    # OJO que las dos cosas van juntas: este reparto SIN
    # HEXAGON_CIRCULARITY_MAX da peor que el watershed en una de las dos
    # escenas (85/120 contra 100/120). Revertir una sola deja el sistema en
    # un punto peor que cualquiera de los dos completos.
    ys, xs = np.nonzero(blob_mask)

    semillas = []

    for label in range(1, num_markers):
        py, px = np.nonzero(markers == label)
        semillas.append((px.mean(), py.mean()))

    distancias = np.stack([
        np.hypot(xs - sx, ys - sy) for sx, sy in semillas
    ])

    dueno = np.argmin(distancias, axis=0)

    separated_contours: list[np.ndarray] = []

    for indice in range(len(semillas)):
        piece_mask = np.zeros(
            blob_mask.shape,
            dtype=np.uint8,
        )

        piece_mask[ys[dueno == indice], xs[dueno == indice]] = 255

        # Limpieza del corte: borra lo que quede mas fino que el kernel sin
        # tocar la pieza, que es mas ancha. Ver SPLIT_CLEANUP_KERNEL_SIZE.
        if config.SPLIT_CLEANUP_KERNEL_SIZE > 0:
            piece_mask = cv2.morphologyEx(
                piece_mask,
                cv2.MORPH_OPEN,
                np.ones(
                    (config.SPLIT_CLEANUP_KERNEL_SIZE, config.SPLIT_CLEANUP_KERNEL_SIZE),
                    dtype=np.uint8,
                ),
            )

        piece_contours, _ = cv2.findContours(
            piece_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
            offset=(x0, y0),
        )

        separated_contours.extend(piece_contours)

    return separated_contours


def find_piece_contours(
    mask: np.ndarray,
) -> list[np.ndarray]:
    """Encuentra los contornos de la máscara, separando piezas
    del mismo color que estén tocándose entre sí."""

    raw_contours, _ = cv2.findContours(
        mask.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    result: list[np.ndarray] = []

    for contour in raw_contours:
        result.extend(
            split_touching_blob(mask, contour)
        )

    return result


def detect_objects(
    frame: np.ndarray,
) -> tuple[list[Detection], dict[str, np.ndarray]]:
    """Detecta círculos y cuadrados rojos o azules.

    El fotograma que llega ya viene recortado a la cinta por
    Camera.read(), así que no hace falta enmascarar nada: todo lo
    que se ve es zona útil.
    """

    blurred_frame = cv2.GaussianBlur(
        frame,
        (5, 5),
        0,
    )

    hsv_image = cv2.cvtColor(
        blurred_frame,
        cv2.COLOR_BGR2HSV,
    )

    masks = create_color_masks(hsv_image)

    detections: list[Detection] = []

    for color, mask in masks.items():
        contours = find_piece_contours(mask)

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < config.MIN_CONTOUR_AREA:
                continue

            if area > config.MAX_CONTOUR_AREA:
                continue

            shape, circularity = classify_shape(contour)

            # TEMPORAL (ver LOG_FORMAS en config.py). Va ANTES del descarte
            # a proposito: la pieza que no clasifica es justamente la que
            # hay que medir.
            if config.LOG_FORMAS:
                _registrar_forma(color, contour, shape)

            if shape is None:
                continue

            moments = cv2.moments(contour)

            if moments["m00"] == 0:
                continue

            center_x = int(
                moments["m10"] / moments["m00"]
            )

            center_y = int(
                moments["m01"] / moments["m00"]
            )

            bounding_box = cv2.boundingRect(contour)

            detection = Detection(
                center=(center_x, center_y),
                color=color,
                shape=shape,
                bbox=bounding_box,
                contour=contour,
                area=area,
                circularity=circularity,
            )

            detections.append(detection)

    return detections, masks