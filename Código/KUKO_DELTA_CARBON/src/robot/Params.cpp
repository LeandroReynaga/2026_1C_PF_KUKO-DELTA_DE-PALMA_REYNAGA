#include "Params.h"

#include <Preferences.h>
#include <math.h>
#include <string.h>

TablaParametros params;

// Espacio de nombres de la NVS. Cambiarlo hace que se ignore todo lo
// guardado antes (util si alguna vez hay que invalidar calibraciones
// viejas a proposito).
static const char *NVS_NAMESPACE = "kuko";

static Preferences prefs;

// ------------------------------------------------------------------
void TablaParametros::begin()
{
    // La NVS puede no abrir si la particion no esta inicializada. No es
    // motivo para no arrancar: sin persistencia el robot funciona igual,
    // con los valores compilados. Se avisa y se sigue.
    nvsLista = prefs.begin(NVS_NAMESPACE, false);

    if (!nvsLista)
    {
        Serial.println("[PARAM] no se pudo abrir la NVS: los cambios no se van a guardar");
    }
}

// ------------------------------------------------------------------
bool TablaParametros::registrar(const char *nombre, float *valor,
                                float minimo, float maximo,
                                const char *unidad, uint8_t nivel,
                                char tipo, bool bloqueadoConPieza)
{
    if (cant >= CAPACIDAD)
    {
        Serial.print("[PARAM] tabla llena, no entra '");
        Serial.print(nombre);
        Serial.println("'. Subir CAPACIDAD en Params.h");
        return false;
    }

    if (nombre == NULL || valor == NULL || strlen(nombre) > NOMBRE_MAX)
    {
        Serial.print("[PARAM] nombre invalido o demasiado largo: '");
        Serial.print(nombre ? nombre : "(null)");
        Serial.println("'");
        return false;
    }

    if (buscar(nombre) >= 0)
    {
        Serial.print("[PARAM] nombre repetido: '");
        Serial.print(nombre);
        Serial.println("'");
        return false;
    }

    // El valor compilado ES el de fabrica. Se captura ahora, antes de que
    // cargarGuardados() lo pise con lo que haya en la NVS.
    Entrada &e = tabla[cant++];

    e.nombre  = nombre;
    e.valor   = valor;
    e.defecto = *valor;
    e.minimo  = minimo;
    e.maximo  = maximo;
    e.unidad  = unidad;
    e.nivel   = nivel;
    e.tipo    = tipo;
    e.bloqueadoConPieza = bloqueadoConPieza;

    return true;
}

// ------------------------------------------------------------------
int8_t TablaParametros::buscar(const char *nombre) const
{
    for (uint8_t i = 0; i < cant; i++)
    {
        // Comparacion sin distinguir mayusculas: el resto de la consola
        // serie acepta los comandos en cualquier caja y seria raro que
        // justo los parametros fueran la excepcion.
        if (strcasecmp(tabla[i].nombre, nombre) == 0)
        {
            return (int8_t)i;
        }
    }

    return -1;
}

// ------------------------------------------------------------------
float TablaParametros::ajustarAlTipo(float valor, char tipo)
{
    if (tipo == 'b')
    {
        return (valor != 0.0f) ? 1.0f : 0.0f;
    }

    if (tipo == 'i')
    {
        return roundf(valor);
    }

    return valor;
}

// ------------------------------------------------------------------
void TablaParametros::cargarGuardados()
{
    if (!nvsLista)
    {
        return;
    }

    uint8_t recuperados = 0;

    for (uint8_t i = 0; i < cant; i++)
    {
        Entrada &e = tabla[i];

        if (!prefs.isKey(e.nombre))
        {
            continue;
        }

        const float guardado = prefs.getFloat(e.nombre, e.defecto);

        // Un valor guardado fuera de rango significa que el firmware nuevo
        // achico los limites desde la ultima vez. Se descarta y queda el de
        // fabrica: cargar a ciegas lo que ya no es valido es exactamente lo
        // que la tabla existe para impedir.
        if (guardado < e.minimo || guardado > e.maximo)
        {
            Serial.print("[PARAM] guardado fuera de rango, se ignora: ");
            Serial.print(e.nombre);
            Serial.print("=");
            Serial.println(guardado, 4);
            continue;
        }

        *e.valor = guardado;
        recuperados++;
    }

    if (recuperados > 0)
    {
        gen++;

        Serial.print("[PARAM] recuperados de la NVS: ");
        Serial.println(recuperados);
    }
}

// ------------------------------------------------------------------
TablaParametros::Resultado TablaParametros::fijar(const char *nombre, float valor,
                                                  bool piezaEnMano)
{
    const int8_t i = buscar(nombre);

    if (i < 0)
    {
        return DESCONOCIDO;
    }

    Entrada &e = tabla[i];

    if (e.bloqueadoConPieza && piezaEnMano)
    {
        return BLOQUEADO;
    }

    valor = ajustarAlTipo(valor, e.tipo);

    // isnan se chequea aparte: NAN no es mayor ni menor que nada, asi que
    // las dos comparaciones de rango le dan false y pasaria derecho.
    if (isnan(valor) || valor < e.minimo || valor > e.maximo)
    {
        return FUERA_RANGO;
    }

    *e.valor = valor;
    gen++;

    return FIJADO;
}

// ------------------------------------------------------------------
void TablaParametros::imprimirEntrada(const Entrada &e) const
{
    Serial.print("[P] n=");    Serial.print(e.nombre);
    Serial.print(" v=");       Serial.print(*e.valor, 4);
    Serial.print(" d=");       Serial.print(e.defecto, 4);
    Serial.print(" min=");     Serial.print(e.minimo, 4);
    Serial.print(" max=");     Serial.print(e.maximo, 4);
    Serial.print(" u=");       Serial.print(e.unidad);
    Serial.print(" l=");       Serial.print(e.nivel);
    Serial.print(" t=");       Serial.println(e.tipo);
}

// ------------------------------------------------------------------
void TablaParametros::volcar() const
{
    for (uint8_t i = 0; i < cant; i++)
    {
        imprimirEntrada(tabla[i]);
    }

    // La linea de cierre le dice a la interfaz que ya estan todos: sin
    // esto tendria que adivinar con un timeout cuando termino el volcado.
    Serial.print("[P] fin n=");
    Serial.println(cant);
}

// ------------------------------------------------------------------
void TablaParametros::guardar()
{
    if (!nvsLista)
    {
        Serial.println("[PARAM] sin NVS: no se guardo nada");
        return;
    }

    uint8_t escritos = 0;

    for (uint8_t i = 0; i < cant; i++)
    {
        const Entrada &e = tabla[i];

        // Solo se escribe lo que difiere de fabrica. La NVS es flash y
        // tiene ciclos de escritura contados; no tiene sentido gastarlos
        // guardando valores que ya estan en el codigo.
        if (fabsf(*e.valor - e.defecto) < 1e-9f)
        {
            if (prefs.isKey(e.nombre))
            {
                prefs.remove(e.nombre);
            }
            continue;
        }

        prefs.putFloat(e.nombre, *e.valor);
        escritos++;
    }

    Serial.print("[PARAM] guardados en NVS: ");
    Serial.print(escritos);
    Serial.println(" (el resto quedo en valores de fabrica)");
}

// ------------------------------------------------------------------
void TablaParametros::restaurarDeFabrica()
{
    for (uint8_t i = 0; i < cant; i++)
    {
        *tabla[i].valor = tabla[i].defecto;
    }

    if (nvsLista)
    {
        prefs.clear();
    }

    gen++;

    Serial.println("[PARAM] valores de fabrica restaurados y NVS borrada");
}

// ------------------------------------------------------------------
float TablaParametros::leer(const char *nombre) const
{
    const int8_t i = buscar(nombre);

    return (i < 0) ? NAN : *tabla[i].valor;
}
