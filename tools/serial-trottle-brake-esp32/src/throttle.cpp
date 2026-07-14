/**
 * @file throttle.cpp
 * @brief Obsługa przepustnicy (throttle) — odczyt ADC z filtracją.
 */

#include "throttle.h"
#include "pinout.h"
#include <Arduino.h>

// ============================================================================
// Odczyt przepustnicy — mediana z N próbek + outlier rejection
// ============================================================================

float read_throttle() {
    uint16_t samples[THR_SAMPLES];
    for (int i = 0; i < THR_SAMPLES; i++) {
        samples[i] = analogRead(PIN_THROTTLE);
        delayMicroseconds(100);
    }

    // Sortuj (insertion sort dla małego N)
    for (int i = 1; i < THR_SAMPLES; i++) {
        uint16_t key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }

    uint16_t median = samples[THR_SAMPLES / 2];

    uint32_t sum = 0;
    int count = 0;
    for (int i = 0; i < THR_SAMPLES; i++) {
        int diff = (int)samples[i] - (int)median;
        if (abs(diff) <= THR_OUTLIER_THRESH) {
            sum += samples[i];
            count++;
        }
    }

    uint16_t avg = (count > 0) ? (uint16_t)(sum / count) : median;

    if (avg <= THR_ADC_MIN) return 0.0f;
    if (avg >= THR_ADC_MAX) return 1.0f;
    return (float)(avg - THR_ADC_MIN) / (float)(THR_ADC_MAX - THR_ADC_MIN);
}
