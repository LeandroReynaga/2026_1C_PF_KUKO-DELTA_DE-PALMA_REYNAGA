"""Prueba de aceptacion del firmware contra el robot real.

Verifica, con la placa conectada, que lo que el ESP32 emite es exactamente
lo que pc/kuko/protocolo.py sabe leer. Es la prueba que no se puede hacer
sin hardware: el resto del contrato ya se verifica sin robot en
test_protocolo.py.

    pc\\.venv\\Scripts\\python pc/tests/aceptacion_firmware.py [COM5]

OJO: abrir el puerto REINICIA el ESP32 y el robot arranca el homing solo.
No correr esto con alguien con las manos adentro del area de trabajo.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import serial
import serial.tools.list_ports

from kuko import protocolo as pr


PUERTO_POR_DEFECTO = "COM5"

# Fabricantes de los chips USB-serie que montan las placas ESP32, para
# encontrar la placa sola entre todos los COM del sistema (los de Bluetooth
# no tienen VID de USB).
VID_USB_SERIE = {0x10C4, 0x1A86, 0x0403, 0x303A}


class Resultado:
    """Lleva la cuenta de lo que paso y con que detalle."""

    def __init__(self) -> None:
        self.ok = 0
        self.mal = 0

    def check(self, condicion: bool, titulo: str, detalle: str = "") -> bool:
        if condicion:
            self.ok += 1
            print(f"  [ok]    {titulo}")
        else:
            self.mal += 1
            print(f"  [FALLA] {titulo}")

        if detalle:
            print(f"          {detalle}")

        return condicion


def buscar_puerto() -> str:
    for p in serial.tools.list_ports.comports():
        if p.vid in VID_USB_SERIE:
            return p.device

    return PUERTO_POR_DEFECTO


def leer_hasta(ser: serial.Serial, segundos: float) -> list[str]:
    """Junta lineas durante una ventana de tiempo."""

    fin = time.monotonic() + segundos
    lineas: list[str] = []

    while time.monotonic() < fin:
        cruda = ser.readline()

        if not cruda:
            continue

        texto = cruda.decode("utf-8", errors="replace").strip()

        if texto:
            lineas.append(texto)

    return lineas


def mandar(ser: serial.Serial, comando: str) -> None:
    ser.write(comando.encode("ascii"))
    ser.flush()


def main() -> int:
    puerto = sys.argv[1] if len(sys.argv) > 1 else buscar_puerto()

    print(f"Abriendo {puerto} a 115200. El ESP32 se va a reiniciar y homear.")
    print()

    r = Resultado()

    with serial.Serial(puerto, 115200, timeout=0.2) as ser:
        # El reset por DTR tarda; los primeros bytes son basura previa al
        # arranque del firmware.
        time.sleep(2.0)

        # --- 1. Arranque -------------------------------------------------
        print("1. Arranque")

        arranque = leer_hasta(ser, 3.0)
        boots = [m for m in map(pr.parsear, arranque) if isinstance(m, pr.Boot)]

        if r.check(bool(boots), "sale la linea [BOOT]"):
            boot = boots[0]
            r.check(
                boot.compatible,
                f"version de protocolo = {pr.VERSION_PROTOCOLO}",
                f"proto={boot.proto} fw={boot.fw}",
            )
            r.check(
                boot.estados == len(pr.EstadoRobot),
                "la cantidad de estados coincide con EstadoRobot",
                f"firmware={boot.estados} python={len(pr.EstadoRobot)}",
            )
            r.check(
                bool(boot.params) and boot.params > 0,
                f"declara {boot.params} parametros",
            )

        # --- 2. Tabla de parametros --------------------------------------
        print()
        print("2. Tabla de parametros ('P?')")

        mandar(ser, "P?\n")
        volcado = [pr.parsear(l) for l in leer_hasta(ser, 3.0)]

        filas = [m for m in volcado if isinstance(m, pr.Parametro)]
        fines = [m for m in volcado if isinstance(m, pr.FinParametros)]

        r.check(bool(filas), f"llegan filas [P] ({len(filas)})")
        r.check(bool(fines), "llega la linea de cierre [P] fin")

        if filas and fines:
            r.check(
                len(filas) == fines[0].cantidad,
                "la cuenta del cierre coincide con las filas recibidas",
                f"filas={len(filas)} fin n={fines[0].cantidad}",
            )

        porNombre = {f.nombre: f for f in filas}

        r.check(
            all(f.minimo is not None and f.maximo is not None for f in filas),
            "todas las filas traen rango",
        )
        r.check(
            all(f.nivel in (1, 2, 3) for f in filas),
            "todas las filas traen un nivel valido",
        )
        r.check(
            all(len(f.nombre) <= 12 for f in filas),
            "ningun nombre pasa los 12 caracteres (limite de la NVS)",
        )

        # El slider de latencia de la interfaz necesita justamente este
        # rango: si el firmware no lo permite, medio slider no hace nada.
        lat = porNombre.get("vis_lat")

        if r.check(lat is not None, "existe el parametro 'vis_lat'"):
            r.check(
                lat.minimo < 0,
                "'vis_lat' acepta negativos (el slider va de -0,1 a 0,3)",
                f"min={lat.minimo} max={lat.maximo} u={lat.unidad}",
            )

        # --- 3. Telemetria ------------------------------------------------
        print()
        print("3. Telemetria ('V1')")

        mandar(ser, "V1\n")
        vivo = [pr.parsear(l) for l in leer_hasta(ser, 4.0)]

        rapidas = [m for m in vivo if isinstance(m, pr.Telemetria)]
        procesos = [m for m in vivo if isinstance(m, pr.Proceso)]
        saludes = [m for m in vivo if isinstance(m, pr.Salud)]

        r.check(len(rapidas) >= 20, f"llegan lineas [T] a ~10 Hz ({len(rapidas)} en 4 s)")
        r.check(len(procesos) >= 2, f"llegan lineas [E] a ~1 Hz ({len(procesos)} en 4 s)")
        r.check(len(saludes) >= 1, f"llegan lineas [H] cada 2 s ({len(saludes)} en 4 s)")

        if rapidas:
            t = rapidas[-1]

            r.check(
                all(v is not None for v in t.angulo),
                "la [T] trae los 3 angulos de encoder",
                f"a={t.angulo}",
            )
            r.check(
                all(v is not None for v in t.comandado),
                "la [T] trae los 3 angulos comandados",
                f"c={t.comandado}",
            )
            r.check(len(t.finales) == 3, f"la [T] trae los 3 finales de carrera: fc={t.finales}")
            r.check(t.estado is not None, f"el estado se reconoce: {t.estado.name if t.estado else '?'}")

            # El angulo medido y el comandado tienen que parecerse: si no,
            # o el robot no homeo, o hay un problema de calibracion.
            if all(v is not None for v in t.angulo + t.comandado):
                dif = [abs(a - c) for a, c in zip(t.angulo, t.comandado)]
                r.check(
                    max(dif) < 15.0,
                    "el angulo medido y el comandado concuerdan (< 15 grados)",
                    f"diferencias={[round(d, 2) for d in dif]}",
                )

        if procesos:
            e = procesos[-1]

            r.check(not e.discrepancia_estado,
                    "el nombre del estado coincide con el indice",
                    f"st={e.estado} sn={e.estado_nombre}")
            r.check(e.modo is not None, f"informa el modo: {e.modo.name if e.modo else '?'}")
            r.check(e.guard is not None, f"informa el guard: {e.guard.name if e.guard else '?'}")
            r.check(e.detectadas is not None, "trae los contadores de produccion")

        if saludes:
            h = saludes[-1]

            r.check(bool(h.loop_hz), f"informa vueltas de loop por segundo: {h.loop_hz}")
            r.check(bool(h.heap), f"informa RAM libre: {h.heap} bytes")

            for i, eje in enumerate(h.ejes):
                r.check(
                    eje.encoder in ("ok", "caido", "rango"),
                    f"encoder {i + 1}: {eje.encoder}"
                    + (f" ganancia={eje.ganancia}" if eje.ganancia else ""),
                )

        # --- 4. Ancho de banda -------------------------------------------
        print()
        print("4. Ancho de banda")

        ser.reset_input_buffer()
        inicio = time.monotonic()
        crudo = leer_hasta(ser, 3.0)
        transcurrido = time.monotonic() - inicio

        bytes_por_seg = sum(len(l) + 2 for l in crudo) / transcurrido
        capacidad = 115200 / 10.0  # 8N1: 10 bits por byte

        r.check(
            bytes_por_seg < capacidad * 0.35,
            f"la telemetria usa {bytes_por_seg:.0f} B/s "
            f"({100 * bytes_por_seg / capacidad:.0f} % del puerto)",
            "queda lugar de sobra para los mensajes de pieza de la vision",
        )

        # --- 5. Fijar un parametro ---------------------------------------
        print()
        print("5. Cambiar un parametro")

        mandar(ser, "V0\n")   # se apaga para que no ensucie las respuestas
        time.sleep(0.4)
        ser.reset_input_buffer()

        original = lat.valor if lat else 0.15

        mandar(ser, "Pvis_lat=0.18\n")
        resp = [pr.parsear(l) for l in leer_hasta(ser, 1.5)]
        sets = [m for m in resp if isinstance(m, pr.ParametroCambiado)]

        if r.check(bool(sets), "contesta [P] set"):
            r.check(sets[0].ok and abs((sets[0].valor or 0) - 0.18) < 1e-6,
                    "acepta un valor dentro de rango",
                    f"n={sets[0].nombre} v={sets[0].valor}")

        # Fuera de rango: tiene que RECHAZAR, no saturar en silencio.
        mandar(ser, "Pvis_lat=5\n")
        resp = [pr.parsear(l) for l in leer_hasta(ser, 1.5)]
        sets = [m for m in resp if isinstance(m, pr.ParametroCambiado)]

        if r.check(bool(sets), "contesta ante un valor fuera de rango"):
            r.check(not sets[0].ok and sets[0].error == "rango",
                    "rechaza fuera de rango (no lo satura)",
                    f"err={sets[0].error}")

        # Un parametro que no existe.
        mandar(ser, "Pinventado=1\n")
        resp = [pr.parsear(l) for l in leer_hasta(ser, 1.5)]
        sets = [m for m in resp if isinstance(m, pr.ParametroCambiado)]

        if r.check(bool(sets), "contesta ante un parametro inexistente"):
            r.check(sets[0].error == "desconocido", "lo marca como desconocido")

        # Los comandos historicos tienen que escribir en la MISMA tabla.
        mandar(ser, "U11\n")
        resp = [pr.parsear(l) for l in leer_hasta(ser, 1.5)]
        sets = [m for m in resp if isinstance(m, pr.ParametroCambiado)]

        if r.check(bool(sets), "el comando historico 'U' contesta [P] set"):
            r.check(sets[0].nombre == "g_umbral" and sets[0].ok,
                    "'U11' escribe en el parametro g_umbral",
                    f"n={sets[0].nombre} v={sets[0].valor}")

        # Se deja como estaba: esta prueba no tiene que dejar el robot
        # descalibrado.
        mandar(ser, f"Pvis_lat={original:.4f}\n")
        time.sleep(0.3)
        mandar(ser, f"Pg_umbral={porNombre['g_umbral'].valor:.4f}\n"
                    if "g_umbral" in porNombre else "U12\n")
        time.sleep(0.3)
        ser.reset_input_buffer()

        # --- 6. Robustez --------------------------------------------------
        print()
        print("6. Robustez del parser del firmware")

        for basura, descripcion in (
            ("Pvis_lat=hola\n", "valor no numerico"),
            ("Pvis_lat=\n", "valor vacio"),
            ("P=0.5\n", "nombre vacio"),
            ("V9\n", "subcomando de telemetria invalido"),
            ("P" + "x" * 40 + "=1\n", "linea mas larga que el buffer"),
        ):
            mandar(ser, basura)
            respuesta = leer_hasta(ser, 1.0)

            r.check(bool(respuesta),
                    f"contesta algo ante {descripcion} (no se cuelga)",
                    respuesta[0][:90] if respuesta else "")

        # Que despues de la basura el firmware siga respondiendo es lo que
        # importa de verdad.
        mandar(ser, "V?\n")
        despues = [pr.parsear(l) for l in leer_hasta(ser, 2.0)]

        r.check(
            any(isinstance(m, pr.Proceso) for m in despues),
            "sigue respondiendo despues de la basura ('V?' devuelve [E])",
        )

        mandar(ser, "V0\n")
        time.sleep(0.3)

    print()
    print("=" * 60)
    print(f"  {r.ok} verificaciones ok, {r.mal} fallidas")
    print("=" * 60)

    return 1 if r.mal else 0


if __name__ == "__main__":
    sys.exit(main())
