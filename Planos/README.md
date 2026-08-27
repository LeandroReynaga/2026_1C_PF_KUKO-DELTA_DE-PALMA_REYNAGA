# Planos y esquemáticos

Acá van los planos exportados que referencia el [README principal](../README.md).

Los archivos **editables** de Fritzing viven en [`../Electrónica/`](../Electr%C3%B3nica/).
Esta carpeta es para las **exportaciones** que se pueden ver sin instalar nada:
un `.fzz` en GitHub no se previsualiza, un `.png` sí.

## Archivos esperados

| Archivo | Qué es | Cómo se genera |
| :--- | :--- | :--- |
| `diagrama-conexion.png` | Diagrama de conexión completo | Abrir `Electrónica/Diagrama_de_Conexión_KUKO_DELTA_rev4.fzz` en Fritzing → *Archivo* → *Exportar* → *como imagen* → *PNG* |
| `diagrama-bloques.png` | Diagrama de bloques del sistema, con alimentaciones y niveles de tensión | A mano (draw.io, Visio, Inkscape) |
| `plano-mecanico.pdf` | Plano mecánico del robot delta acotado | Export desde el CAD |
| `espacio-trabajo.png` | Volumen de trabajo alcanzable, en corte | Opcional |

## Nombres de archivo

Sin espacios, sin acentos, en minúscula y separados con `-`. Un archivo con espacios
obliga a escribir la ruta codificada (`%20`) en el README, y cualquier error ahí deja
la imagen rota sin ningún aviso.

## Dato

La única fuente de verdad del **pinout** en el código es
[`include/Pinout.h`](../C%C3%B3digo/KUKO_DELTA_CARBON/include/Pinout.h): ahí se definen todos los
números de GPIO y ningún otro archivo debería hardcodearlos. Si se recablea algo, hay que tocar
ese header **y** el diagrama de Fritzing, o los dos empiezan a decir cosas distintas.
