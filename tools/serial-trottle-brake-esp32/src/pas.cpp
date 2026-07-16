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
#include <queue>
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
static float         g_cad_ema = 0.0f;   // EMA kadencji
static uint32_t      g_last_ema_us = 0;  // timestamp poprzedniej aktualizacji EMA
static int8_t        g_dir_score = 0;    // licznik potwierdzeń kierunku
static bool          g_forward   = false;
static bool          g_dir_valid = false;

// ============================================================================
// kolejka z danymi z przerwań
// ============================================================================
std::deque<pas_raw_data_t> g_pas_queue;


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
            // Dodaj dane do kolejki
            pas_raw_data_t raw;
            raw.pulse_timestamp_us = now;
            raw.high_us  = g_high_us;
            raw.low_us   = g_low_us;
            raw.period   = g_period_us; // okres w µs
            g_pas_queue.push_back(raw);
            if (g_pas_queue.size() > PAS_QUEUE_SIZE) {
                g_pas_queue.pop_front(); // usuń najstarszy element, jeśli kolejka jest zbyt długa
            }
        }
    }
}

// ============================================================================
// Inicjalizacja
// ============================================================================

void pas_init(const pas_config_t* config) {
    // Kopiuj konfigurację z domyślnymi wartościami
    g_cfg = *config;
    if (g_cfg.magnets == 0)          g_cfg.magnets = PAS_DEFAULT_MAGNETS;
    if (g_cfg.dir_ratio_thresh == 0) g_cfg.dir_ratio_thresh = 1.0f;
    if (g_cfg.cadence_data_count == 0) g_cfg.cadence_data_count = PAS_MIN_SAMPLE_COUNT;
    if (g_cfg.cadence_tau_s == 0) g_cfg.cadence_tau_s = PAS_DEFAULT_CADENCE_TAU_S;

    pinMode(PIN_PAS, INPUT_PULLUP);
    g_last_edge_us = micros();

    // Reset stanu
    g_forward     = false;
    g_dir_valid   = false;
    g_dir_score   = 0;
    g_cad_ema     = 0.0f;
    g_last_ema_us = 0;

    attachInterrupt(digitalPinToInterrupt(PIN_PAS), pas_isr, CHANGE);
}

void pas_get_data(pas_data_t* out) {
    if (out == nullptr) return;

    // Domyślne wartości
    out->pedal_rpm = 0.0f;
    out->direction = g_forward;
    uint32_t total_period = 0;
    uint32_t total_forward = 0;
    uint32_t total_backward = 0;  
    uint64_t now_us = micros();

    // Usuń stare dane z kolejki (starsze niż g_cfg.cadence_tau_s sekund)
    for(auto it = g_pas_queue.begin(); it != g_pas_queue.end(); ) {
        if (it->pulse_timestamp_us + g_cfg.cadence_tau_s*1000000 < now_us) {
            it = g_pas_queue.erase(it); // usuń stare dane
        } else {
            ++it;
        }
    }
    // Jeśli kolejka jest pusta, zwróć brak danych
    if (g_pas_queue.empty()) {
        return; // brak danych
    }
    // Jeśli jest mniej próbek niż wymagane do wygładzania, zwróć brak danych
    if (g_pas_queue.size() < g_cfg.cadence_data_count) {
        // Za mało próbek do obliczeń
        return;
    }
    //przetwarzamy dane z kolejki, liczymy sumaryczny okres i kierunek    
    for (const auto& raw : g_pas_queue) {
        total_period += raw.period;
        uint8_t dir = (raw.high_us > raw.low_us) ? 1 : 0;
        if (dir == 1) {
            total_forward++;
        } else {
            total_backward++;
        }
    }

    if (total_forward > total_backward) {
        out->direction = true; // FWD
        out->pedal_rpm = (float)60000000 / (total_period / g_pas_queue.size() * g_cfg.magnets); 
    } else if (total_backward > total_forward) {
        out->direction = false; // REV
        g_forward = false;
        out->pedal_rpm = (float)60000000 / (total_period / g_pas_queue.size() * g_cfg.magnets);
    }
}

