#include "FaultLog.h"

void FaultLog::begin()
{
    cabeza   = 0;
    cantidad = 0;
    contador = 0;

    for (uint8_t i = 0; i < FALLO_CANT_TIPOS; i++)
    {
        porTipo[i] = 0;
    }
}

// ------------------------------------------------------------------
const char *FaultLog::nombreTipo(uint8_t tipo)
{
    switch (tipo)
    {
        case FALLO_COLISION:      return "COLISION";
        case FALLO_ENCODER:       return "ENCODER";
        case FALLO_HOMING:        return "HOMING";
        case FALLO_MANUAL:        return "MANUAL";
        case FALLO_DESCALIBRACION: return "DESCALIBRACION";
        default:                  return "?";
    }
}

// ------------------------------------------------------------------
void FaultLog::registrar(RegistroFallo r)
{
    contador++;

    r.numero = contador;
    r.t_ms   = millis();

    if (r.tipo < FALLO_CANT_TIPOS)
    {
        porTipo[r.tipo]++;
    }

    buffer[cabeza] = r;
    cabeza = (cabeza + 1) % CAPACIDAD;
    if (cantidad < CAPACIDAD)
    {
        cantidad++;
    }

    imprimirLinea(r);
}

// ------------------------------------------------------------------
void FaultLog::imprimirLinea(const RegistroFallo &r)
{
    Serial.print("[FALLO] n=");   Serial.print(r.numero);
    Serial.print(" t=");          Serial.print(r.t_ms);
    Serial.print(" tipo=");       Serial.print(nombreTipo(r.tipo));
    Serial.print(" eje=");        Serial.print(r.eje);
    Serial.print(" err=");        Serial.print(r.errorDeg, 2);
    Serial.print(" dcmd=");       Serial.print(r.cmdDelta, 2);
    Serial.print(" denc=");       Serial.print(r.encDelta, 2);
    Serial.print(" estado=");     Serial.print(r.estado);
    Serial.print(" pieza=");      Serial.print(r.conPieza ? 1 : 0);
    Serial.print(" enmano=");     Serial.print(r.enMano ? 1 : 0);
    Serial.print(" color=");      Serial.print(r.color);
    Serial.print(" forma=");      Serial.print(r.forma);
    Serial.print(" py=");         Serial.print(r.piezaY, 2);
    Serial.print(" px=");         Serial.print(r.piezaX, 2);
    Serial.print(" tacho=");      Serial.println(r.tacho);
}

// ------------------------------------------------------------------
void FaultLog::imprimirResumen() const
{
    Serial.print("[FALLOS] total=");
    Serial.print(contador);

    for (uint8_t i = 0; i < FALLO_CANT_TIPOS; i++)
    {
        Serial.print(" ");
        Serial.print(nombreTipo(i));
        Serial.print("=");
        Serial.print(porTipo[i]);
    }

    Serial.print(" guardados=");
    Serial.println(cantidad);
}

// ------------------------------------------------------------------
void FaultLog::imprimirHistorial() const
{
    imprimirResumen();

    // Del mas viejo al mas nuevo de los que quedaron en el buffer.
    const uint8_t primero = (uint8_t)((cabeza + CAPACIDAD - cantidad) % CAPACIDAD);

    for (uint8_t i = 0; i < cantidad; i++)
    {
        imprimirLinea(buffer[(primero + i) % CAPACIDAD]);
    }

    Serial.println("[FALLOS] fin");
}

// ------------------------------------------------------------------
uint32_t FaultLog::totalTipo(uint8_t tipo) const
{
    if (tipo >= FALLO_CANT_TIPOS)
    {
        return 0;
    }
    return porTipo[tipo];
}
