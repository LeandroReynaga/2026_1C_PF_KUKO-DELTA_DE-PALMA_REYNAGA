"""Deja una imagen lista para el panel de portada de la interfaz.

Recorta al centro a la proporcion del panel (1600 x 340) y la guarda como
pc/assets/portada.png. La original no se toca.

    pc\\.venv\\Scripts\\python pc/tools/preparar_portada.py <imagen>

Sin argumento, busca en pc/assets/ cualquier archivo que empiece con
"portada_original".
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2

ANCHO, ALTO = 1600, 340

ASSETS = Path(__file__).resolve().parents[1] / "assets"
SALIDA = ASSETS / "portada.png"


def buscar_original() -> Path | None:
    for archivo in sorted(ASSETS.glob("portada_original*")):
        if archivo.suffix.lower() in (".png", ".jpg", ".jpeg", ".webp", ".bmp"):
            return archivo

    return None


def main() -> int:
    origen = Path(sys.argv[1]) if len(sys.argv) > 1 else buscar_original()

    if origen is None or not origen.exists():
        print("No encontre la imagen. Guardala en pc/assets/ como "
              "portada_original.png (o pasala como argumento).")
        return 1

    imagen = cv2.imread(str(origen), cv2.IMREAD_UNCHANGED)

    if imagen is None:
        print(f"No pude leer {origen.name}: formato no reconocido.")
        return 1

    alto, ancho = imagen.shape[:2]

    # Se recorta al centro a la proporcion final y recien despues se escala.
    # Al reves (escalar y despues recortar) se deforma la imagen.
    proporcion = ANCHO / ALTO

    if ancho / alto > proporcion:
        nuevo_ancho = int(alto * proporcion)
        x0 = (ancho - nuevo_ancho) // 2
        recorte = imagen[:, x0:x0 + nuevo_ancho]
    else:
        nuevo_alto = int(ancho / proporcion)
        y0 = (alto - nuevo_alto) // 2
        recorte = imagen[y0:y0 + nuevo_alto, :]

    # INTER_AREA es el que corresponde al achicar: promedia los pixeles de
    # origen en vez de muestrear, asi los bordes y el texto no quedan
    # dentados.
    final = cv2.resize(recorte, (ANCHO, ALTO), interpolation=cv2.INTER_AREA)

    cv2.imwrite(str(SALIDA), final)

    print(f"{origen.name} ({ancho}x{alto}) -> {SALIDA.name} ({ANCHO}x{ALTO})")

    return 0


if __name__ == "__main__":
    sys.exit(main())
