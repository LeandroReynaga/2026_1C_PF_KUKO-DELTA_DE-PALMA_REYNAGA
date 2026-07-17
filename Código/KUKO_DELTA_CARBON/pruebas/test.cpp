#include <Arduino.h>

#define ENCODER_PIN 34   // ADC1_CH6, pin válido para analogRead

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);                       // 0-4095
    analogSetPinAttenuation(ENCODER_PIN, ADC_11db);  // rango hasta ~3.3V
}

uint16_t readAveraged(int pin, int samples = 32) {
    uint32_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);
        delayMicroseconds(100);
    }
    return sum / samples;
}

void loop() {
    uint16_t raw = readAveraged(ENCODER_PIN);
    float voltage = raw * 3.3f / 4095.0f;
    float angle   = raw * 360.0f / 4095.0f;

    Serial.printf("RAW: %4u  V: %.3f  Angle: %.2f\n", raw, voltage, angle);
    delay(20);
}