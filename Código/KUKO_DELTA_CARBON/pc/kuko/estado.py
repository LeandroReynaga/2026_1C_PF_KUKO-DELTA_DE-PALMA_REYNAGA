"""Estado agregado del sistema y los seis chequeos de componentes.

Junta lo ultimo que dijo cada linea de telemetria y decide, con eso, de que
color va cada puntito de la pantalla. Ver pc/PROTOCOLO.md §5: cinco de los
seis chequeos son mediciones reales; el de neumatica es de estado, porque
sin vacuostato el firmware sabe si MANDO prender la bomba, no si hay vacio.

El de la camara es el unico que NO depende del enlace serie: la camara
cuelga del USB de la PC y el robot del suyo, asi que se pueden caer por
separado y hay que poder verlo. Los otros cinco se apagan a gris sin enlace
porque sin telemetria no hay con que decidirlos.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

from . import protocolo as pr
from .rendimiento import Rendimiento

VERDE = "ok"
AMBAR = "aviso"
ROJO = "falla"
GRIS = "sin_datos"

# Sin telemetria por mas de esto, el enlace se da por caido. Es el chequeo
# mas importante de todos: sin el, la pantalla sigue mostrando el ultimo
# dato como si fuera actual.
ENLACE_TIMEOUT_S = 1.5

# Lo mismo para la camara, y por el mismo motivo, que ahi es todavia peor:
# una camara USB desenchufada no da error, simplemente deja de entregar
# fotogramas. Sin este vencimiento la pantalla muestra la ultima imagen
# congelada y unos FPS clavados en el ultimo valor bueno, y no hay nada
# en pantalla que diga que se dejo de ver la cinta.
CAMARA_TIMEOUT_S = 2.0


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

    # Historia de la corrida: cuanto tiempo estuvo el robot en cada
    # situacion, cuando paso cada fallo y a que ritmo produjo. Vive aca y no
    # en el firmware porque el ESP32 no tiene RAM para guardar historia y no
    # la necesita para moverse; la PC ya esta leyendo todas las lineas igual.
    # Lo alimenta el enlace y lo dibuja la pestana de Rendimiento.
    rendimiento: Rendimiento = field(default_factory=Rendimiento)

    # ------------------------------------------------------------------
    #  Modo teach
    # ------------------------------------------------------------------
    # `teach` es el último volcado de `J?`, que es de donde salen los límites
    # del volumen de trabajo ya resueltos (el piso en Z lo deriva el firmware
    # de `grab_z` y no existe como parámetro suelto).
    teach: Optional[pr.Teach] = None

    # Posición COMANDADA de la punta y estado del vacío, del volcado a 20 Hz
    # que se enciende mientras la pestaña de teach está a la vista. Es lo que
    # se graba: la medida por encoders trae ruido que reproducido es temblor.
    teach_pos: Optional[tuple[float, float, float]] = None
    teach_bomba: Optional[bool] = None

    # Último evento del modo (`fin`, `abort`, `err`...) y un contador que sube
    # con cada uno. La interfaz compara el contador contra el último que
    # atendió: así un evento se consume una sola vez aunque el refresco pase
    # veinte veces por segundo, y ninguno se pierde entre dos refrescos.
    teach_evento: Optional[pr.Teach] = None
    teach_evento_n: int = 0

    # Lo mide la vision siguiendo las piezas sobre la cinta (cm/s), o None
    # si no hay ninguna a la vista para medir.
    cinta_medida: Optional[float] = None

    # ------------------------------------------------------------------
    #  Camara
    # ------------------------------------------------------------------
    # Los escribe el hilo de vision. `fps_camara` es instantaneo (1/dt del
    # ultimo fotograma) y sirve para el chip de arriba; el promedio de los
    # ultimos cinco minutos sale del historial, que va guardando el CONTADOR
    # de fotogramas.
    fps_camara: float = 0.0

    #: Hay una camara prevista, o sea que se arranco sin --sin-vision. Con
    #: la vision apagada no corresponde ni punto rojo ni alarma: no falta
    #: nada, se pidio que no estuviera.
    camara_presente: bool = False

    camara_abierta: bool = False
    camara_backend: str = ""
    camara_error: str = ""

    #: `time.monotonic()` del ultimo fotograma bueno. Es lo que hace que una
    #: camara desenchufada se note: el reloj sigue corriendo y ella no.
    ultimo_fotograma: float = 0.0

    #: Fotogramas leidos desde que arranco el programa, y cuantas veces hubo
    #: que volver a abrir la camara. Varias reaperturas en un turno son un
    #: cable flojo, que no se ve en ningun otro lado.
    fotogramas: int = 0
    reconexiones_camara: int = 0

    # Suavizado de los angulos de encoder para las agujas de los diales: el
    # AS5600 analogico tiene ~1 grado de ruido y a 10 Hz la aguja tiembla.
    # No se toca el dato crudo, solo lo que se dibuja.
    angulo_suave: list[Optional[float]] = field(default_factory=lambda: [None, None, None])

    def enlace_vivo(self) -> bool:
        return self.conectado and (time.monotonic() - self.ultimo_t) < ENLACE_TIMEOUT_S

    def camara_viva(self) -> bool:
        """Hay una camara abierta Y esta entregando fotogramas ahora.

        Las dos condiciones, no una: `VideoCapture` sigue diciendo que esta
        abierto despues de que se desenchufa el USB, asi que preguntarle a
        el es preguntarle al que no se entero.
        """

        return (self.camara_abierta
                and (time.monotonic() - self.ultimo_fotograma) < CAMARA_TIMEOUT_S)

    def suavizar(self, alfa: float = 0.35) -> None:
        if not self.t:
            return

        for i, valor in enumerate(self.t.angulo):
            if valor is None:
                continue

            previo = self.angulo_suave[i]
            self.angulo_suave[i] = valor if previo is None else previo + alfa * (valor - previo)

    # ------------------------------------------------------------------
    #  Los seis chequeos
    # ------------------------------------------------------------------
    def chequeos(self) -> dict[str, Chequeo]:
        # La camara va FUERA del corte por enlace: cuelga del USB de la PC,
        # no del ESP32, y las dos cosas se caen por separado. Meterla adentro
        # la apagaria a gris cada vez que se desenchufa el robot, que es
        # justo cuando uno mira la pantalla para entender que falta.
        if not self.enlace_vivo():
            chequeos = {n: Chequeo(GRIS, "sin enlace")
                        for n in ("cinta", "encoders", "endstops", "motores",
                                  "neumatica")}
        else:
            chequeos = {
                "cinta": self._cinta(),
                "encoders": self._encoders(),
                "endstops": self._endstops(),
                "motores": self._motores(),
                "neumatica": self._neumatica(),
            }

        chequeos["camara"] = self._camara()

        return chequeos

    def _camara(self) -> Chequeo:
        """Esta entregando imagen, o dejo de hacerlo.

        Importa mas de lo que parece: sin camara el firmware no recibe una
        sola pieza y el robot se queda esperando en WAIT_PIECE, o sea
        perfectamente sano y perfectamente inutil. Es la falla que mejor
        se disfraza de "hoy no vinieron piezas".
        """

        if not self.camara_presente:
            return Chequeo(GRIS, "apagada (--sin-vision)")

        if self.camara_viva():
            detalle = f"{self.fps_camara:.0f} fps"

            if self.reconexiones_camara:
                # Anda, pero se reengancho: casi siempre es el cable.
                veces = "vez" if self.reconexiones_camara == 1 else "veces"

                return Chequeo(
                    AMBAR,
                    f"{detalle}, se reconecto {self.reconexiones_camara} {veces}")

            return Chequeo(VERDE, detalle)

        if self.camara_error:
            return Chequeo(ROJO, f"no abre: {self.camara_error[:44]}")

        if not self.ultimo_fotograma:
            return Chequeo(GRIS, "abriendo")

        sin_imagen = time.monotonic() - self.ultimo_fotograma

        return Chequeo(ROJO, f"sin imagen hace {sin_imagen:.0f} s")

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
