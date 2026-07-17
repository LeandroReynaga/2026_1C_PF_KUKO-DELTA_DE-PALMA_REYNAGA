#include <Arduino.h>

// Definición de pines para los 3 encoders
#define ENCODER_1_PIN 35
#define ENCODER_2_PIN 34
#define ENCODER_3_PIN 39

void setup() {
    Serial.begin(115200);
    analogReadResolution(12); // Configuración global de 12 bits (0-4095)
    
    // Configuración de atenuación a 11dB para cada pin individualmente
    analogSetPinAttenuation(ENCODER_1_PIN, ADC_11db);
    analogSetPinAttenuation(ENCODER_2_PIN, ADC_11db);
    analogSetPinAttenuation(ENCODER_3_PIN, ADC_11db);
}

// Función genérica para promediar lecturas y filtrar ruido eléctrico
uint16_t readAveraged(int pin, int samples = 32) {
    uint32_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);
        delayMicroseconds(100);
    }
    return sum / samples;
}

void loop() {
    // 1. Lectura del valor RAW de cada encoder
    uint16_t raw1 = readAveraged(ENCODER_1_PIN);
    uint16_t raw2 = readAveraged(ENCODER_2_PIN);
    uint16_t raw3 = readAveraged(ENCODER_3_PIN);
    
    // 2. Cálculo directo de los ángulos nativos (0 a 360°)
    float angleEnc1 = (raw1 * 360.0f) / 4095.0f;
    float angleEnc2 = (raw2 * 360.0f) / 4095.0f;
    float angleEnc3 = (raw3 * 360.0f) / 4095.0f;

    // 3. Impresión limpia de los 3 ángulos en una sola línea
    Serial.printf("Enc1: %6.2f°  |  Enc2: %6.2f°  |  Enc3: %6.2f°\n", angleEnc1, angleEnc2, angleEnc3);
    
    delay(50); 
}
