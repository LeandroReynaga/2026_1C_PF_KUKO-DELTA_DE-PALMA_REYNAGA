from __future__ import annotations

from dataclasses import dataclass
from math import pi

import cv2
import numpy as np

from config import (
    BLUE_LOWER,
    BLUE_UPPER,
    CIRCLE_CIRCULARITY_MIN,
    MASK_SMOOTHING_KERNEL_SIZE,
    MAX_CONTOUR_AREA,
    MIN_CONTOUR_AREA,
    MORPH_KERNEL_SIZE,
    RED_LOWER_1,
    RED_LOWER_2,
    RED_UPPER_1,
    RED_UPPER_2,
    ROI_X_MAX_RATIO,
    ROI_X_MIN_RATIO,
    ROI_Y_MAX_RATIO,
    ROI_Y_MIN_RATIO,
    SMOOTH_MASK_EDGES,
    SQUARE_ASPECT_RATIO_MAX,
    SQUARE_ASPECT_RATIO_MIN,
    WATERSHED_MIN_PEAK_DISTANCE,
    WATERSHED_MIN_PEAK_HEIGHT,
)


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
        (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE),
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

    if SMOOTH_MASK_EDGES:
        # Una superficie texturada/matte, con un rango HSV bien
        # ajustado, deja pixeles sueltos justo al filo del umbral
        # en el borde de la pieza ("festoneado"). Difuminar la
        # máscara y volver a binarizarla promedia ese ruido fino
        # sin cambiar la forma general de la pieza.
        blurred = cv2.GaussianBlur(
            mask,
            (MASK_SMOOTHING_KERNEL_SIZE, MASK_SMOOTHING_KERNEL_SIZE),
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
    """Crea las máscaras para objetos rojos y azules."""

    red_mask_1 = cv2.inRange(
        hsv_image,
        np.array(RED_LOWER_1, dtype=np.uint8),
        np.array(RED_UPPER_1, dtype=np.uint8),
    )

    red_mask_2 = cv2.inRange(
        hsv_image,
        np.array(RED_LOWER_2, dtype=np.uint8),
        np.array(RED_UPPER_2, dtype=np.uint8),
    )

    red_mask = cv2.bitwise_or(
        red_mask_1,
        red_mask_2,
    )

    blue_mask = cv2.inRange(
        hsv_image,
        np.array(BLUE_LOWER, dtype=np.uint8),
        np.array(BLUE_UPPER, dtype=np.uint8),
    )

    return {
        "ROJO": clean_mask(red_mask),
        "AZUL": clean_mask(blue_mask),
    }


def classify_shape(
    contour: object,
) -> tuple[str | None, float]:
    """Clasifica un contorno como círculo o cuadrado.

    Se usa el casco convexo (convex hull) del contorno antes de
    medir circularidad y vértices. Un brillo/reflejo sobre la
    pieza genera una "mordida" cóncava en la máscara que reduce
    artificialmente la circularidad y puede alterar la cantidad
    de vértices detectados. Como las piezas reales son siempre
    convexas, esto ignora ese ruido sin cambiar la forma real.
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
        0.035 * perimeter,
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
                SQUARE_ASPECT_RATIO_MIN
                <= aspect_ratio
                <= SQUARE_ASPECT_RATIO_MAX
            ):
                return "CUADRADO", circularity

    # ----------------------------
    # Comprobación de círculo
    # ----------------------------

    if circularity >= CIRCLE_CIRCULARITY_MIN:
        return "CIRCULO", circularity

    return None, circularity


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

    kernel_size = 2 * WATERSHED_MIN_PEAK_DISTANCE + 1

    dilated_distance = cv2.dilate(
        distance_rounded.astype(np.float32),
        np.ones((kernel_size, kernel_size), dtype=np.uint8),
    ).astype(np.int32)

    peaks = (
        (distance_rounded == dilated_distance)
        & (distance_rounded > WATERSHED_MIN_PEAK_HEIGHT)
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

    markers = markers + 1

    unknown = cv2.subtract(blob_mask, sure_foreground)
    markers[unknown == 255] = 0

    blob_mask_bgr = cv2.cvtColor(
        blob_mask,
        cv2.COLOR_GRAY2BGR,
    )

    cv2.watershed(blob_mask_bgr, markers)

    separated_contours: list[np.ndarray] = []

    for label in range(2, num_markers + 1):
        piece_mask = np.zeros(
            blob_mask.shape,
            dtype=np.uint8,
        )

        piece_mask[markers == label] = 255

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
    """Detecta círculos y cuadrados rojos o azules."""

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

    roi_mask = create_roi_mask(frame)

    masks["ROJO"] = cv2.bitwise_and(
    masks["ROJO"],
    roi_mask,
)

    masks["AZUL"] = cv2.bitwise_and(
    masks["AZUL"],
    roi_mask,
)

    detections: list[Detection] = []

    for color, mask in masks.items():
        contours = find_piece_contours(mask)

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < MIN_CONTOUR_AREA:
                continue

            if area > MAX_CONTOUR_AREA:
                continue

            shape, circularity = classify_shape(contour)

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

def create_roi_mask(
    image: np.ndarray,
) -> np.ndarray:
    """Crea una máscara que limita la detección a la cinta."""

    height, width = image.shape[:2]

    x_min = int(width * ROI_X_MIN_RATIO)
    x_max = int(width * ROI_X_MAX_RATIO)

    y_min = int(height * ROI_Y_MIN_RATIO)
    y_max = int(height * ROI_Y_MAX_RATIO)

    roi_mask = np.zeros(
        (height, width),
        dtype=np.uint8,
    )

    cv2.rectangle(
        roi_mask,
        (x_min, y_min),
        (x_max, y_max),
        255,
        -1,
    )

    return roi_mask