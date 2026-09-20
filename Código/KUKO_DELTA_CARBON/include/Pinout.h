#ifndef PINOUT_H
#define PINOUT_H

// -- PINOUT KUKO DELTA CARBON --

// ENABLE
#define ENA 13

// DRIVERS NEMA 23
#define PUL1 14
#define DIR1 25

#define PUL2 27
#define DIR2 33

#define PUL3 26
#define DIR3 32

// SENSORES DE FIN DE CARRERA
#define FC1 16
#define FC2 17
#define FC3 18

// CINTA TRANSPORTADORA
#define CINTAPWM 19

// ENCODERS AS5600 (salida analogica, entrada ADC)
// Solo de entrada: no admiten pinMode(OUTPUT).
#define ENC1 35
#define ENC2 34
#define ENC3 39

// BOMBA HIDRÁULICA
#define BOMBA 23


#endif
