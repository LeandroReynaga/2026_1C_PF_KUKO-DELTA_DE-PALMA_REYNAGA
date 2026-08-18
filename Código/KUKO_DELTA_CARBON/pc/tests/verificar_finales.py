"""Verificacion manual de los finales de carrera y del ritmo del loop.

Mira la telemetria durante una ventana de tiempo e informa que finales de
carrera se vieron activarse. Es la unica prueba del sistema que necesita una
persona: hay que apretar los tres con el dedo.

    pc\\.venv\\Scripts\\python pc/tests/verificar_finales.py [segundos] [COM5]

No mueve el robot: solo escucha. El ESP32 igual se reinicia al abrirse el
puerto, asi que va a rehomear una vez al principio.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import serial

from kuko import protocolo as pr

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aceptacion_firmware import buscar_puerto, mandar


def main() -> int:
    segundos = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    puerto = sys.argv[2] if len(sys.argv) > 2 else buscar_puerto()

    vistos = [False, False, False]
    cambios = 0
    loops: list[int] = []
    errores: list[list[float]] = []

    with serial.Serial(puerto, 115200, timeout=0.2) as ser:
        time.sleep(2.0)
        ser.reset_input_buffer()

        mandar(ser, "V1\n")

        print(f"Escuchando {segundos:.0f} s. APRETA LOS TRES FINALES, UNO POR UNO.")
        print()

        anterior: list[bool] = []
        fin = time.monotonic() + segundos

        while time.monotonic() < fin:
            cruda = ser.readline()

            if not cruda:
                continue

            m = pr.parsear(cruda.decode("utf-8", errors="replace").strip())

            if isinstance(m, pr.Telemetria):
                if m.finales and len(m.finales) == 3:
                    if anterior and m.finales != anterior:
                        cambios += 1
                        pisados = [str(i + 1) for i, v in enumerate(m.finales) if v]

                        print(f"  fc = {''.join('1' if v else '0' for v in m.finales)}"
                              f"   pisado: {', '.join(pisados) if pisados else 'ninguno'}")

                    for i, v in enumerate(m.finales):
                        vistos[i] = vistos[i] or v

                    anterior = list(m.finales)

                if all(v is not None for v in m.error):
                    errores.append([abs(v) for v in m.error])

            elif isinstance(m, pr.Salud):
                if m.loop_hz:
                    loops.append(m.loop_hz)

        mandar(ser, "V0\n")
        time.sleep(0.3)

    print()
    print("=" * 60)

    for i, v in enumerate(vistos):
        print(f"  final de carrera {i + 1}: {'SE ACTIVO' if v else 'nunca se activo'}")

    print(f"  cambios de estado detectados: {cambios}")

    if loops:
        print(f"  vueltas de loop por segundo: min={min(loops)} max={max(loops)} "
              f"muestras={len(loops)}")

    if errores:
        # El peor error de cada eje durante toda la ventana: es el piso de
        # ruido con el robot quieto, contra el que se elige el umbral.
        peor = [max(e[i] for e in errores) for i in range(3)]
        print(f"  peor error del guard por eje: {[round(p, 2) for p in peor]} grados")

    print("=" * 60)

    return 0 if all(vistos) else 1


if __name__ == "__main__":
    sys.exit(main())
