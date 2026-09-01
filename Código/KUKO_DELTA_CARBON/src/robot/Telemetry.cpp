#include "Telemetry.h"

// Version del contrato de pc/PROTOCOLO.md. Se sube cuando cambia el
// formato de una linea de forma que rompa a la interfaz vieja: agregar un
// campo al final NO rompe nada (el parser de Python ignora lo que no
// conoce), pero renombrar o sacar uno si.
static const uint8_t VERSION_PROTOCOLO = 4;

// Fecha de compilacion como identificacion del firmware. No es un hash de
// git, pero alcanza para lo que hace falta: darse cuenta de que la placa
// tiene una version distinta de la que uno cree.
// __DATE__ trae espacios ("Aug 17 2026", y dos espacios si el dia es de un
// digito), y un valor con espacios rompe el formato clave=valor: el parser
// cortaria en "fw=Aug" y se perderia el resto. Se compacta a un solo token.
static void imprimirFirmwareId()
{
    const char *d = __DATE__;

    for (uint8_t i = 0; d[i] != 0; i++)
    {
        Serial.print(d[i] == ' ' ? '-' : d[i]);
    }
}

// ------------------------------------------------------------------
void Telemetria::anunciar(uint8_t cantidadEstados, uint8_t cantidadParametros) const
{
    Serial.print("[BOOT] proto=");
    Serial.print(VERSION_PROTOCOLO);
    Serial.print(" fw=");
    imprimirFirmwareId();
    Serial.print(" estados=");
    Serial.print(cantidadEstados);
    Serial.print(" params=");
    Serial.println(cantidadParametros);
}

// ------------------------------------------------------------------
void Telemetria::setActiva(bool activada)
{
    encendida = activada;

    if (encendida)
    {
        // Los relojes se ponen en cero para que la primera linea salga en
        // la vuelta siguiente y no haya que esperar un periodo entero: la
        // interfaz acaba de conectarse y esta mirando una pantalla vacia.
        ultimaRapida_ms  = 0;
        ultimoProceso_ms = 0;
        ultimaSalud_ms   = 0;

        Serial.println("[TELE] on");
    }
    else
    {
        Serial.println("[TELE] off");
    }
}

// ------------------------------------------------------------------
void Telemetria::pedirFoto()
{
    fotoProceso = true;
    fotoSalud   = true;
}

// ------------------------------------------------------------------
void Telemetria::contarVuelta()
{
    vueltas++;

    const uint32_t ahora = millis();

    if (ventanaVueltas_ms == 0)
    {
        ventanaVueltas_ms = ahora;
        return;
    }

    const uint32_t transcurrido = ahora - ventanaVueltas_ms;

    if (transcurrido >= 1000)
    {
        // Se normaliza a 1 s en vez de suponer que la ventana duro
        // exactamente eso: si el loop se bloqueo, la ventana se paso de
        // largo y la cuenta cruda daria un numero mas alto que el real,
        // justo en el caso que uno quiere detectar.
        vueltasHz = (vueltas * 1000UL) / transcurrido;

        vueltas = 0;
        ventanaVueltas_ms = ahora;
    }
}

// ------------------------------------------------------------------
//  Relojes
// ------------------------------------------------------------------
//  La resta con uint32_t es correcta aunque millis() de la vuelta (a los
//  49 dias): la aritmetica sin signo hace que la diferencia siga dando el
//  tiempo transcurrido. Por eso no se compara "ahora >= ultima + periodo",
//  que ahi si se rompe.

bool Telemetria::tocaRapida(uint32_t ahora)
{
    if (!encendida || periodoRapida_ms <= 0.0f)
    {
        return false;
    }

    if ((uint32_t)(ahora - ultimaRapida_ms) < (uint32_t)periodoRapida_ms)
    {
        return false;
    }

    ultimaRapida_ms = ahora;
    return true;
}

bool Telemetria::tocaProceso(uint32_t ahora)
{
    if (fotoProceso)
    {
        fotoProceso = false;
        ultimoProceso_ms = ahora;
        return true;
    }

    if (!encendida || periodoProceso_ms <= 0.0f)
    {
        return false;
    }

    if ((uint32_t)(ahora - ultimoProceso_ms) < (uint32_t)periodoProceso_ms)
    {
        return false;
    }

    ultimoProceso_ms = ahora;
    return true;
}

bool Telemetria::tocaSalud(uint32_t ahora)
{
    if (fotoSalud)
    {
        fotoSalud = false;
        ultimaSalud_ms = ahora;
        return true;
    }

    if (!encendida || periodoSalud_ms <= 0.0f)
    {
        return false;
    }

    if ((uint32_t)(ahora - ultimaSalud_ms) < (uint32_t)periodoSalud_ms)
    {
        return false;
    }

    ultimaSalud_ms = ahora;
    return true;
}

// ------------------------------------------------------------------
//  [T]
// ------------------------------------------------------------------
void Telemetria::emitirRapida(const TeleRapida &d) const
{
    Serial.print("[T] t=");
    Serial.print(d.t_ms);
    Serial.print(" st=");
    Serial.print(d.estado);
    Serial.print(" pm=");
    Serial.print(d.bomba ? 1 : 0);

    Serial.print(" fc=");
    for (uint8_t i = 0; i < TELE_EJES; i++)
    {
        Serial.print(d.finales[i] ? 1 : 0);
    }

    // Dos decimales alcanzan: el encoder analogico tiene ~1 grado de ruido,
    // asi que el tercer decimal seria ruido con formato.
    for (uint8_t i = 0; i < TELE_EJES; i++)
    {
        Serial.print(" a");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.print(d.ang[i], 2);
    }

    for (uint8_t i = 0; i < TELE_EJES; i++)
    {
        Serial.print(" c");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.print(d.cmd[i], 2);
    }

    for (uint8_t i = 0; i < TELE_EJES; i++)
    {
        Serial.print(" e");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.print(d.err[i], 2);
    }

    for (uint8_t i = 0; i < TELE_EJES; i++)
    {
        Serial.print(" u");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.print(d.umb[i], 1);
    }

    for (uint8_t i = 0; i < TELE_EJES; i++)
    {
        Serial.print(" v");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.print(d.vel[i], 0);
    }

    Serial.println();
}

// ------------------------------------------------------------------
//  [E]
// ------------------------------------------------------------------
void Telemetria::emitirProceso(const TeleProceso &d) const
{
    Serial.print("[E] t=");     Serial.print(d.t_ms);
    Serial.print(" st=");       Serial.print(d.estado);
    Serial.print(" sn=");       Serial.print(d.estadoNombre);
    Serial.print(" md=");       Serial.print(d.modo);
    Serial.print(" mp=");       Serial.print(d.modoPendiente);
    Serial.print(" cf=");       Serial.print(d.esperandoTapa ? 1 : 0);
    Serial.print(" cr=");       Serial.print(d.tapaRestante_ms);
    Serial.print(" q=");        Serial.print(d.cola);
    Serial.print(" qa=");       Serial.print(d.colaAntiguedad_ms);
    Serial.print(" hm=");       Serial.print(d.homed ? 1 : 0);
    Serial.print(" gd=");       Serial.print(d.guard);
    Serial.print(" ob=");       Serial.print(d.observando ? 1 : 0);
    Serial.print(" sup=");      Serial.print(d.paradasActivas ? 1 : 0);
    Serial.print(" cv=");       Serial.print(d.cinta ? 1 : 0);
    Serial.print(" cvp=");      Serial.print(d.cintaPwm);
    Serial.print(" cal=");      Serial.print(d.calibrando ? 1 : 0);
    Serial.print(" rep=");      Serial.print(d.reposo ? 1 : 0);

    Serial.print(" bx=");

    if (d.layout == NULL)
    {
        Serial.print("-");
    }
    else
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            Serial.print(d.layout[i]);
        }
    }

    Serial.print(" bf=");

    if (d.llenas == NULL)
    {
        Serial.print("-");
    }
    else
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            Serial.print(d.llenas[i] ? 1 : 0);
        }
    }

    Serial.print(" bc=");       Serial.print(d.celdaReservada);
    Serial.print(" pc=");       Serial.print(d.piezaColor);
    Serial.print(" pf=");       Serial.print(d.piezaForma);
    Serial.print(" py=");       Serial.print(d.piezaY, 2);
    Serial.print(" pb=");       Serial.print(d.tacho);
    Serial.print(" tw=");       Serial.print(d.teachPuntos);
    Serial.print(" ti=");       Serial.print(d.teachIndice);
    Serial.print(" nd=");       Serial.print(d.detectadas);
    Serial.print(" nk=");       Serial.print(d.depositadas);
    Serial.print(" nx=");       Serial.print(d.descartadas);
    Serial.print(" nf=");       Serial.print(d.fallos);
    Serial.print(" kr=");       Serial.print(d.porColor[0]);
    Serial.print(" kg=");       Serial.print(d.porColor[1]);
    Serial.print(" kb=");       Serial.print(d.porColor[2]);
    Serial.print(" ks=");       Serial.print(d.porForma[0]);
    Serial.print(" kh=");       Serial.print(d.porForma[1]);
    Serial.print(" kc=");       Serial.println(d.porForma[2]);
}

// ------------------------------------------------------------------
//  [H]
// ------------------------------------------------------------------
void Telemetria::imprimirEje(uint8_t i, const TeleSalud::Eje &e) const
{
    const uint8_t n = i + 1;

    Serial.print(" enc"); Serial.print(n); Serial.print("=");  Serial.print(e.encoder);
    Serial.print(" gan"); Serial.print(n); Serial.print("=");  Serial.print(e.ganancia, 3);
    Serial.print(" atr"); Serial.print(n); Serial.print("=");  Serial.print(e.atraso_ms, 0);
    Serial.print(" pic"); Serial.print(n); Serial.print("=");  Serial.print(e.pico, 2);
    Serial.print(" rep"); Serial.print(n); Serial.print("=");  Serial.print(e.picoReposo, 2);
    Serial.print(" fug"); Serial.print(n); Serial.print("=");  Serial.print(e.fuga, 2);
    Serial.print(" rmn"); Serial.print(n); Serial.print("=");  Serial.print(e.rawMin);
    Serial.print(" rmx"); Serial.print(n); Serial.print("=");  Serial.print(e.rawMax);
    Serial.print(" rsy"); Serial.print(n); Serial.print("=");  Serial.print(e.resincronizaciones);
}

void Telemetria::emitirSalud(const TeleSalud &d) const
{
    Serial.print("[H] t=");
    Serial.print(d.t_ms);
    Serial.print(" up=");
    Serial.print(d.uptime_s);
    Serial.print(" loop=");
    Serial.print(d.loopHz);
    Serial.print(" heap=");
    Serial.print(d.heapLibre);

    for (uint8_t i = 0; i < TELE_EJES; i++)
    {
        imprimirEje(i, d.ejes[i]);
    }

    Serial.println();
}
