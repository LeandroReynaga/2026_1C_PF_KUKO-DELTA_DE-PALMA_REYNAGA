"""Estado agregado del sistema y los cinco chequeos de componentes.

Junta lo ultimo que dijo cada linea de telemetria y decide, con eso, de que
color va cada puntito de la pantalla. Ver pc/PROTOCOLO.md §5: cuatro de los
cinco chequeos son mediciones reales; el de neumatica es de estado, porque
sin vacuostato el firmware sabe si MANDO prender la bomba, no si hay vacio.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

from . import protocolo as pr

VERDE = "ok"
AMBAR = "aviso"
ROJO = "falla"
GRIS = "sin_datos"

# Sin telemetria por mas de esto, el enlace se da por caido. Es el chequeo
# mas importante de todos: sin el, la pantalla sigue mostrando el ultimo
# dato como si fuera actual.
ENLACE_TIMEOUT_S = 1.5


@dataclass
class Chequeo:
    estado: str = GRIS
    detalle: str = "sin datos"


@dataclass
class EstadoSistema:
    """Lo ultimo que se sabe del robot. Lo escribe el enlace, lo lee la UI."""

    conectado: bool = False
    puerto: str = ""
    error_enlace: str = ""

    boot: Optional[pr.Boot] = None
    t: Optional[pr.Telemetria] = None
    e: Optional[pr.Proceso] = None
    h: Optional[pr.Salud] = None

    ultimo_t: float = 0.0
    parametros: dict[str, pr.Parametro] = field(default_factory=dict)
    consola: list[str] = field(default_factory=list)
    fallos: list[pr.Fallo] = field(default_factory=list)

    # Lo mide la vision siguiendo las piezas sobre la cinta (cm/s), o None
    # si no hay ninguna a la vista para medir.
    cinta_medida: Optional[float] = None
    fps_camara: float = 0.0

    # Suavizado de los angulos de encoder para las agujas de los diales: el
    # AS5600 analogico tiene ~1 grado de ruido y a 10 Hz la aguja tiembla.
    # No se toca el dato crudo, solo lo que se dibuja.
    angulo_suave: list[Optional[float]] = field(default_factory=lambda: [None, None, None])

    def enlace_vivo(self) -> bool:
        return self.conectado and (time.monotonic() - self.ultimo_t) < ENLACE_TIMEOUT_S

    def suavizar(self, alfa: float = 0.35) -> None:
        if not self.t:
            return

        for i, valor in enumerate(self.t.angulo):
            if valor is None:
                continue

            previo = self.angulo_suave[i]
            self.angulo_suave[i] = valor if previo is None else previo + alfa * (valor - previo)

    # ------------------------------------------------------------------
    #  Los cinco chequeos
    # ------------------------------------------------------------------
    def chequeos(self) -> dict[str, Chequeo]:
        if not self.enlace_vivo():
            return {n: Chequeo(GRIS, "sin enlace")
                    for n in ("cinta", "encoders", "endstops", "motores", "neumatica")}

        return {
            "cinta": self._cinta(),
            "encoders": self._encoders(),
            "endstops": self._endstops(),
            "motores": self._motores(),
            "neumatica": self._neumatica(),
        }

    def _encoders(self) -> Chequeo:
        if not self.h:
            return Chequeo(GRIS, "sin datos")

        malos = [i + 1 for i, x in enumerate(self.h.ejes) if not x.encoder_ok]

        if malos:
            return Chequeo(ROJO, f"eje {malos[0]}: {self.h.ejes[malos[0] - 1].encoder}"
                                 + (f" (+{len(malos) - 1})" if len(malos) > 1 else ""))

        # Ganancia: 1,00 es el encoder viendo todo el recorrido. Por debajo
        # de 0,90 se pierden cuentas de verdad; entre 0,90 y 0,97 avisa sin
        # gritar, porque un valor estable ahi no es lo mismo que un canal
        # que se esta yendo.
        ganancias = [x.ganancia for x in self.h.ejes if x.ganancia]

        if ganancias:
            peor = min(ganancias)

            if peor < 0.90:
                return Chequeo(ROJO, f"ganancia {peor:.2f}: pierde pasos")

            if peor < 0.97:
                return Chequeo(AMBAR, f"ganancia {peor:.2f} (ideal 1,00)")

        estrechos = [i + 1 for i, x in enumerate(self.h.ejes)
                     if x.margen_raw is not None and x.margen_raw < 100]

        if estrechos:
            return Chequeo(AMBAR, f"eje {estrechos[0]} al limite del ADC")

        return Chequeo(VERDE, "3 canales ok")

    def _endstops(self) -> Chequeo:
        if not self.t or not self.t.finales:
            return Chequeo(GRIS, "sin datos")

        pisados = [i + 1 for i, v in enumerate(self.t.finales) if v]

        # Con el brazo lejos de home no puede haber ninguno pisado. Si lo
        # hay, esta trabado o el cable en corto.
        quieto_en_home = self.t.estado in (pr.EstadoRobot.HOMING, pr.EstadoRobot.IDLE)

        if pisados and not quieto_en_home:
            return Chequeo(ROJO, f"FC{pisados[0]} pisado sin homing")

        if pisados:
            return Chequeo(VERDE, f"pisado: {', '.join(map(str, pisados))}")

        return Chequeo(VERDE, "los 3 libres")

    def _motores(self) -> Chequeo:
        if not self.t:
            return Chequeo(GRIS, "sin datos")

        margenes = [self.t.margen(i) for i in range(3)]
        validos = [m for m in margenes if m is not None]

        if not validos:
            return Chequeo(GRIS, "sin datos")

        peor = max(validos)

        if peor >= 1.0:
            return Chequeo(ROJO, f"eje {margenes.index(peor) + 1} pasa el umbral")

        if peor > 0.7:
            return Chequeo(AMBAR, f"error al {peor * 100:.0f} % del umbral")

        return Chequeo(VERDE, f"error al {peor * 100:.0f} % del umbral")

    def _cinta(self) -> Chequeo:
        if not self.e:
            return Chequeo(GRIS, "sin datos")

        if not self.e.cinta:
            return Chequeo(GRIS, "detenida")

        esperada = self.parametros["cinta_cms"].valor if "cinta_cms" in self.parametros else None

        if self.cinta_medida is None or esperada is None:
            return Chequeo(VERDE, f"en marcha ({self.e.cinta_pwm or 0} %)")

        # Margen deliberadamente ancho: la medicion por vision tiene ruido y
        # una variacion chica no significa nada. Solo interesa el caso duro
        # -- la cinta trabada o yendo muchisimo mas lento de lo que deberia.
        if self.cinta_medida < esperada * 0.4:
            return Chequeo(ROJO, f"trabada: {self.cinta_medida:.1f} de {esperada:.1f} cm/s")

        return Chequeo(VERDE, f"{self.cinta_medida:.1f} cm/s")

    def _neumatica(self) -> Chequeo:
        # Estado comandado, no medido: no hay vacuostato en el robot y no
        # esta previsto ponerlo. Nunca se pinta en rojo a proposito -- seria
        # inventar una falla que este sistema no puede detectar.
        if not self.t:
            return Chequeo(GRIS, "sin datos")

        return Chequeo(VERDE, "bomba activa" if self.t.bomba else "en reposo")
