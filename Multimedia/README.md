# Multimedia — fotos y videos

Acá van las imágenes y videos que referencia el [README principal](../README.md).

## Cómo agregar una imagen

1. **Guardá el archivo en esta carpeta** con el nombre exacto de la tabla de abajo.
2. **Commiteá y pusheá** — la imagen tiene que estar *dentro* del repositorio para que
   GitHub la muestre. No sirve un enlace a Drive o a la nube.

   ```bash
   git add Multimedia/01-robot-completo.jpg
   git commit -m "Agrego foto del robot completo"
   git push
   ```

3. **Reemplazá el bloque `IMAGEN PENDIENTE`** del README principal por la línea que
   figura debajo de cada bloque.

## Reglas para los nombres de archivo

> Los nombres **sin espacios, sin acentos y en minúscula** no son un capricho: un
> archivo llamado `Robot terminado (final).jpg` obliga a escribir la ruta como
> `Robot%20terminado%20(final).jpg` en el README, y cualquier error ahí deja la imagen
> rota sin ningún aviso.

- Usar `-` en lugar de espacios
- Sin `á é í ó ú ñ`
- Minúsculas
- Prefijo numérico para que la carpeta quede ordenada

## Archivos esperados

**Referenciados hoy en el README principal** — al subirlos, la imagen aparece sola:

| # | Archivo | Qué debe mostrar | Dónde va |
| :-: | :--- | :--- | :--- |
| 1 | `01-robot-completo.jpg` | El robot completo de frente, con cinta, tachos y gabinete | Portada |
| 2 | `02-celda-en-marcha.gif` | 5–8 s en bucle del agarre sobre la cinta en movimiento | § 2 Brief |
| 9 | `09-video-demo.mp4` | Ciclo completo de clasificación, con la interfaz visible | § 2 Brief |

**Opcionales** — no hay ningún bloque esperándolos en el README, pero sirven para el informe
y para la defensa. Si se suben, hay que agregar la referencia a mano:

| # | Archivo | Qué debe mostrar |
| :-: | :--- | :--- |
| 3 | `03-vision-deteccion.png` | Ventana de visión con piezas detectadas y línea de cruce |
| 4 | `04-interfaz-operacion.png` | Pestaña Operación completa, con los 6 puntos en verde |
| 5 | `05-interfaz-teach.png` | Pestaña Teach con el brazo dibujado y una secuencia grabada |
| 6 | `06-interfaz-rendimiento.png` | Pestaña Rendimiento tras una corrida real |
| 7 | `07-modo-box.jpg` | La caja de 6 celdas llena, vista desde arriba |
| 8 | `08-electronica-gabinete.jpg` | Gabinete abierto: drivers, ESP32, fuente y cableado |

## Tamaños y formatos

| Tipo | Formato | Tamaño recomendado |
| :--- | :--- | :--- |
| Fotos del robot | `.jpg` | Ancho de 1600 px, menos de 1 MB |
| Capturas de la interfaz | `.png` | Resolución nativa, menos de 1 MB |
| Animación corta | `.gif` | Menos de 5 MB (si no, tarda en cargar el README) |
| Video | `.mp4` | Menos de 50 MB, o subirlo a YouTube y enlazarlo |

> **Límites de GitHub:** avisa a partir de 50 MB por archivo y **rechaza** los de más
> de 100 MB. Un video de la demo en buena calidad pasa esos límites fácil — para eso
> conviene YouTube o Drive, y dejar el enlace en el README.
