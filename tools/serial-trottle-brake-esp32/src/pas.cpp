/**
 * @file pas.c
 * @brief Obsługa czujnika PAS (Pedal Assist Sensor).
 *
 * Działanie:
 *  - Przerwanie na każdym zboczu GPIO (CHANGE).
 *  - Mierzy czasy HIGH i LOW.
 *  - Kierunek z proporcji R = H/L (R > prog = FWD).
 *  - Kadencja = 60e6 / okres / magnesy, uśredniana z N próbek.
 */

#include "pas.h"
#include <Arduino.h>
#include "pinout.h"

// ============================================================================
// Stałe wewnętrzne
// ============================================================================

/// Minimalny poprawny półokres [µs] — odrzuca zakłócenia
#define PAS_MIN_HALF_US      5000

// ============================================================================
// Zmienne współdzielone ISR ↔ loop
// ============================================================================

static volatile uint32_t g_last_edge_us  = 0;
static volatile uint32_t g_high_us       = 0;
static volatile uint32_t g_low_us        = 0;
static volatile uint32_t g_period_us     = 0;
static volatile bool     g_new_period    = false;
static volatile uint32_t g_pulse_count   = 0;

// ============================================================================
// Zmienne przetwarzania (loop)
// ============================================================================

static pas_config_t  g_cfg;
static float         g_cad_buf[10];      // bufor do średniej
static uint8_t       g_buf_idx   = 0;
static uint8_t       g_buf_count = 0;
static int8_t        g_dir_score = 0;    // licznik potwierdzeń kierunku
static bool          g_forward   = false;
static bool          g_dir_valid = false;

// ============================================================================
// ISR — przerwanie na każdym zboczu
// ============================================================================

static void IRAM_ATTR pas_isr() {
    uint32_t now = micros();
    uint32_t dt = now - g_last_edge_us;
    g_last_edge_us = now;

    // Odrzuć zbyt krótkie zbocze (zakłócenie)
    if (dt < PAS_MIN_HALF_US) {
        return;
    }

    bool pin_high = (GPIO.in >> PIN_PAS) & 1;

    if (pin_high) {
        // RISING — koniec LOW
        g_low_us = dt;
    } else {
        // FALLING — koniec HIGH
        g_high_us = dt;
        if (g_low_us > 0) {
            g_period_us = g_high_us + g_low_us;
            g_new_period = true;
            g_pulse_count++;
        }
    }
}

// ============================================================================
// Inicjalizacja
// ============================================================================

void pas_init(const pas_config_t* config) {
    // Kopiuj konfigurację z domyślnymi wartościami
    g_cfg = *config;
    if (g_cfg.magnets == 0)          g_cfg.magnets = 12;
    if (g_cfg.dir_ratio_thresh == 0) g_cfg.dir_ratio_thresh = 1.0f;
    if (g_cfg.dir_confirm == 0)      g_cfg.dir_confirm = 3;
    if (g_cfg.cadence_avg_n == 0)    g_cfg.cadence_avg_n = 10;
    if (g_cfg.cadence_avg_n > 10)    g_cfg.cadence_avg_n = 10;

    pinMode(PIN_PAS, INPUT_PULLUP);
    g_last_edge_us = micros();

    // Reset stanu
    g_forward   = false;
    g_dir_valid = false;
    g_dir_score = 0;
    g_buf_idx   = 0;
    g_buf_count = 0;

    attachInterrupt(digitalPinToInterrupt(PIN_PAS), pas_isr, CHANGE);
}

// ============================================================================
// Odczyt danych — wywołuj w loop()
// ============================================================================

bool pas_get_data(pas_data_t* out) {
    if (!g_new_period) {
        return false;
    }
    g_new_period = false;

    // --- Snapshot zmiennych ISR ---
    uint32_t high   = g_high_us;
    uint32_t low    = g_low_us;
    uint32_t period = g_period_us;
    uint32_t count  = g_pulse_count;

    // --- Kierunek z proporcji H/L ---
    float ratio   = (low > 0) ? (float)high / (float)low : 1.0f;
    bool raw_fwd  = (ratio > g_cfg.dir_ratio_thresh);
    if (g_cfg.direction_invert) {
        raw_fwd = !raw_fwd;
    }

    if (raw_fwd) {
        if (g_dir_score < (int8_t)g_cfg.dir_confirm) g_dir_score++;
    } else {
        if (g_dir_score > -(int8_t)g_cfg.dir_confirm) g_dir_score--;
    }

    if (g_dir_score >= (int8_t)g_cfg.dir_confirm) {
        g_forward   = true;
        g_dir_valid = true;
    } else if (g_dir_score <= -(int8_t)g_cfg.dir_confirm) {
        g_forward   = false;
        g_dir_valid = true;
    } else {
        g_dir_valid = false;
    }

    // --- Surowe RPM ---
    float pulse_rpm = (period > 0) ? 60000000.0f / (float)period : 0.0f;
    float cad_raw   = (g_cfg.magnets > 0) ? pulse_rpm / (float)g_cfg.magnets : 0.0f;

    // --- Średnia ruchoma kadencji (tylko poprawny kierunek FWD) ---
    float cad_avg = 0.0f;
    if (g_dir_valid && g_forward && cad_raw > 0.0f) {
        g_cad_buf[g_buf_idx] = cad_raw;
        g_buf_idx  = (g_buf_idx + 1) % g_cfg.cadence_avg_n;
        if (g_buf_count < g_cfg.cadence_avg_n) {
            g_buf_count++;
        }

        for (uint8_t i = 0; i < g_buf_count; i++) {
            cad_avg += g_cad_buf[i];
        }
        cad_avg /= (float)g_buf_count;
    } else {
        // Reverse lub brak pedałowania — reset bufora
        g_buf_count = 0;
        g_buf_idx   = 0;
    }

    // --- Wypełnij wynik ---
    out->pedal_rpm       = cad_avg;
    out->pulse_rpm       = pulse_rpm;
    out->forward         = g_forward;
    out->direction_valid = g_dir_valid;
    out->high_us         = high;
    out->low_us          = low;
    out->period_us       = period;
    out->pulse_count     = count;

    return true;
}
