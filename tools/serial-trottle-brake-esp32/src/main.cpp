/**
 * @file main.cpp
 * @brief SmartESC UART Bridge — ESP32
 *
 * Mostek między czujnikami (PAS, przepustnica, hamulec) + wyświetlaczem S866
 * a kontrolerem ESC SmartESC_STM32_v2 (protokół VESC przez UART).
 *
 * ## Architektura
 * - Serial  (UART0): USB → PC (debug 115200)
 * - Serial1 (UART1): GPIO4/18 → ESC (VESC protokół, 115200)
 * - Serial2 (UART2): GPIO16/17/5 → wyświetlacz S866 (9600)
 *
 * ## Przepływ danych
 * loop() co ~10 ms:
 *   1. Serwis wyświetlacza S866 (odbiór ramek, wysyłanie odpowiedzi)
 *   2. Odczyt czujników (PAS, przepustnica, hamulec)
 *   3. Obliczenie docelowego duty/prądu na podstawie trybu sterowania
 *   4. Wysłanie komendy do ESC (co ESC_CMD_INTERVAL_MS)
 *   5. Odczyt telemetrii ESC (co ESC_TELEM_INTERVAL_MS)
 *
 * ## Tryby sterowania (z wyświetlacza S866, P10):
 *   P10=0: PAS + przepustnica
 *   P10=1: tylko przepustnica
 *   P10=2: tylko PAS
 *
 * ## Mapowanie poziomów wspomagania na prędkość
 *   Poziomy 0-5 (lub 0-9) z wyświetlacza mapowane na prędkość docelową:
 *   level 0 = 0 km/h (off)
 *   level 1 =  8 km/h
 *   level 2 = 15 km/h (domyślnie)
 *   level 3 = 20 km/h
 *   level 4 = 25 km/h
 *   level 5 = 30 km/h (lub P08 limit)
 *   Poziomy 6-9 = interpolowane do max 45 km/h
 */

#include <Arduino.h>
#include "pinout.h"
#include "display_s866.h"
#include "vesc_proto.h"

// ============================================================================
// Zmienne globalne — czujniki
// ============================================================================

/// Przefiltrowana wartość przepustnicy [0.0–1.0]
static float g_throttle = 0.0f;

/// Hamulec aktywny (debounced)
static bool g_brake_active = false;

/// Kadencja PAS [RPM]
static float g_pas_cadence_rpm = 0.0f;

/// Kierunek pedałowania PAS (true = do przodu)
static bool g_pas_forward = false;

/// Czas ostatniego impulsu PAS do przodu [ms]
static unsigned long g_pas_last_fwd_ms = 0;

/// Czy PAS jest aktywny (ktoś pedałuje do przodu)
static bool g_pas_active = false;

// ============================================================================
// Zmienne globalne — sterowanie
// ============================================================================

/// Docelowe duty do wysłania do ESC [-1.0..1.0]
static float g_target_duty = 0.0f;

/// Czy ESC jest w trybie RUN (status z telemetrii)
static bool g_esc_running = false;

/// Ostatni raz wysłano komendę do ESC [ms]
static unsigned long g_last_esc_cmd_ms = 0;

/// Ostatni raz odpytywano telemetrię ESC [ms]
static unsigned long g_last_telem_poll_ms = 0;

/// Ostatni raz odebrano poprawną telemetrię [ms]
static unsigned long g_last_telem_valid_ms = 0;

/// Kontekst wyświetlacza S866
static s866_display_t g_display;

/// Telemetria ESC
static esc_telem_t g_esc_telem;

// ============================================================================
// PAS — sprzętowy timer próbkujący (2 kHz) + filtr cyfrowy
// ============================================================================

/// Timer próbkujący PAS
static hw_timer_t* g_pas_timer = nullptr;

/// Zmienne volatile dla ISR timera PAS
static volatile bool     g_pas_raw_state = false;      ///< Ostatnia odczytana wartość pinu
static volatile uint8_t  g_pas_filter_cnt = 0;         ///< Licznik zgodnych próbek
static volatile bool     g_pas_filtered = false;        ///< Przefiltrowany stan
static volatile uint32_t g_pas_rising_us = 0;           ///< Timestamp ostatniego RISING [µs]
static volatile uint32_t g_pas_falling_us = 0;          ///< Timestamp ostatniego FALLING [µs]
static volatile uint32_t g_pas_high_us = 0;             ///< Czas trwania HIGH [µs]
static volatile uint32_t g_pas_period_us = 0;           ///< Pełny okres [µs]
static volatile bool     g_pas_dir_fwd = true;          ///< Kierunek z asymetrii
static volatile uint32_t g_pas_last_edge_us = 0;        ///< Ostatnia krawędź (dowolna) [µs]
static volatile uint32_t g_pas_last_fwd_edge_us = 0;    ///< Ostatnia krawędź FORWARD [µs]

/// Głębokość filtra: PAS_DEBOUNCE_US / 500µs
#define PAS_FILTER_DEPTH   6   // 3000µs / 500µs

/// ISR timera PAS — próbkuje pin co 500 µs
static void IRAM_ATTR pas_timer_isr() {
    bool state = digitalRead(PIN_PAS);

    if (state != g_pas_filtered) {
        g_pas_filter_cnt++;
        if (g_pas_filter_cnt >= PAS_FILTER_DEPTH) {
            // Stan się zmienił
            uint32_t now = micros();
            bool old_state = g_pas_filtered;
            g_pas_filtered = state;
            g_pas_filter_cnt = 0;
            g_pas_last_edge_us = now;

            if (state && !old_state) {
                // RISING edge
                g_pas_rising_us = now;
                if (g_pas_falling_us > 0) {
                    g_pas_high_us = now - g_pas_falling_us;
                }
            } else if (!state && old_state) {
                // FALLING edge
                g_pas_falling_us = now;
                if (g_pas_rising_us > 0) {
                    uint32_t period = now - g_pas_rising_us;
                    if (period > 1000) {  // min 1ms okres (~60000 RPM kadencji)
                        g_pas_period_us = period + g_pas_high_us;  // pełny okres

                        // Detekcja kierunku z asymetrii duty cycle
                        if (g_pas_period_us > 0 && g_pas_high_us > 0) {
                            uint32_t low_us = g_pas_period_us - g_pas_high_us;
                            int asymmetry = (int)g_pas_high_us - (int)low_us;
                            int threshold = (int)(g_pas_period_us * PAS_DIR_ASYMMETRY_PCT / 100);
                            if (asymmetry > threshold) {
                                g_pas_dir_fwd = true;
                            } else if (-asymmetry > threshold) {
                                g_pas_dir_fwd = false;
                            }
                            // else: zachowaj poprzedni kierunek
                        }

                        // Jeśli kierunek do przodu — zapisz timestamp
                        if (g_pas_dir_fwd) {
                            g_pas_last_fwd_edge_us = now;
                        }
                    }
                }
            }
        }
    } else {
        g_pas_filter_cnt = 0;
    }
}

// ============================================================================
// Odczyt przepustnicy — mediana z N próbek + outlier rejection
// ============================================================================

static float read_throttle() {
    // Zbierz THR_SAMPLES próbek
    uint16_t samples[THR_SAMPLES];
    for (int i = 0; i < THR_SAMPLES; i++) {
        samples[i] = analogRead(PIN_THROTTLE);
        delayMicroseconds(100);
    }

    // Sortuj (proste insertion sort dla małego N)
    for (int i = 1; i < THR_SAMPLES; i++) {
        uint16_t key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }

    // Mediana
    uint16_t median = samples[THR_SAMPLES / 2];

    // Outlier rejection: odrzuć skrajne, weź średnią z pozostałych
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

    // Mapuj na 0.0–1.0
    if (avg <= THR_ADC_MIN) return 0.0f;
    if (avg >= THR_ADC_MAX) return 1.0f;
    return (float)(avg - THR_ADC_MIN) / (float)(THR_ADC_MAX - THR_ADC_MIN);
}

// ============================================================================
// Odczyt hamulca — debounce
// ============================================================================

static bool g_brake_raw = false;
static unsigned long g_brake_change_ms = 0;
static bool g_brake_stable = false;

static bool read_brake() {
    bool raw = !digitalRead(PIN_BRAKE);  // aktywny LOW → invert

    if (raw != g_brake_raw) {
        g_brake_raw = raw;
        g_brake_change_ms = millis();
    }

    if (millis() - g_brake_change_ms >= BRAKE_DEBOUNCE_MS) {
        g_brake_stable = g_brake_raw;
    }

    return g_brake_stable;
}

// ============================================================================
// Aktualizacja stanu PAS z danych ISR (wołane w loop)
// ============================================================================

static void update_pas() {
    unsigned long now_ms = millis();
    uint32_t now_us = micros();

    // Odczytaj dane z ISR (kopie lokalne dla bezpieczeństwa)
    uint32_t period_us = g_pas_period_us;
    bool dir_fwd = g_pas_dir_fwd;
    uint32_t last_fwd_edge = g_pas_last_fwd_edge_us;

    // Oblicz kadencję [RPM]
    if (period_us > 0 && period_us < 1000000UL) {  // max 1s okres = 1 RPM
        float cadence_rpm = 60000000.0f / (float)period_us;  // 60s / okres[s]
        if (cadence_rpm <= PAS_MAX_CADENCE_RPM) {
            g_pas_cadence_rpm = cadence_rpm;
        }
    }

    g_pas_forward = dir_fwd;

    // Sprawdź timeout PAS (brak impulsów → kadencja = 0)
    if (last_fwd_edge > 0 && (now_us - last_fwd_edge) > (PAS_TIMEOUT_MS * 1000UL)) {
        g_pas_cadence_rpm = 0.0f;
        g_pas_last_fwd_edge_us = 0;  // zapobiega wielokrotnemu timeoutowi
    }

    // Czy PAS aktywny (ktoś pedałuje do przodu)
    g_pas_active = (g_pas_cadence_rpm > 5.0f) && g_pas_forward;

    // Aktualizuj timestamp ostatniego impulsu do przodu
    if (g_pas_active) {
        g_pas_last_fwd_ms = now_ms;
    }
}

// ============================================================================
// Mapowanie poziomu wspomagania na prędkość docelową
// ============================================================================

/// Prędkości docelowe dla poziomów 0–9 [km/h]
static const float s_assist_speed_map[] = {
    0.0f,   // level 0 — off
    8.0f,   // level 1
    15.0f,  // level 2
    20.0f,  // level 3
    25.0f,  // level 4
    30.0f,  // level 5
    33.0f,  // level 6
    37.0f,  // level 7
    41.0f,  // level 8
    45.0f,  // level 9
};
#define ASSIST_LEVELS_COUNT (sizeof(s_assist_speed_map) / sizeof(s_assist_speed_map[0]))

static float get_target_speed_kmh(uint8_t assist_level, uint8_t speed_limit) {
    if (assist_level == 0) return 0.0f;

    // Ogranicz poziom do zakresu mapy
    if (assist_level >= ASSIST_LEVELS_COUNT) {
        assist_level = ASSIST_LEVELS_COUNT - 1;
    }

    float target = s_assist_speed_map[assist_level];

    // Ogranicz do P08 speed limit (jeśli ustawiony > 0)
    if (speed_limit > 0 && target > (float)speed_limit) {
        target = (float)speed_limit;
    }

    return target;
}

// ============================================================================
// Obliczanie docelowego duty na podstawie trybu sterowania
// ============================================================================

static float compute_target_duty() {
    s866_rx_params_t* rx = &g_display.rx;

    uint8_t drive_mode = rx->throttle_mode;  // P10: 0=PAS+gaz, 1=gaz, 2=PAS
    uint8_t assist_lvl = rx->assist_level;
    uint8_t speed_limit = rx->speed_max_limit;

    // --- Hamulec → zawsze 0 ---
    if (g_brake_active) {
        return 0.0f;
    }

    // --- Tryb 1: tylko przepustnica ---
    if (drive_mode == 1) {
        return g_throttle;  // 0.0–1.0
    }

    // --- Tryb 2: tylko PAS ---
    if (drive_mode == 2) {
        if (!g_pas_active) return 0.0f;

        float target_kmh = get_target_speed_kmh(assist_lvl, speed_limit);
        if (target_kmh <= 0.0f) return 0.0f;

        // Aktualna prędkość z ESC
        float current_kmh = 0.0f;
        if (g_esc_telem.valid) {
            // Przelicz ERPM → km/h
            // 15 par biegunów, koło 26" (P06), ale upraszczamy:
            // wheeltime_ms z wyświetlacza albo erpm z ESC
            // erpm / 15 = RPM mechaniczne
            float rpm = g_esc_telem.rpm;
            // Załóżmy koło 26" ≈ 2.07m obwód → 1 RPM = 0.124 km/h
            // Dokładniej: użyjemy P06 z wyświetlacza
            float wheel_diameter_inch = rx->wheel_size_inch_x10 / 10.0f;  // cale
            if (wheel_diameter_inch < 10.0f) wheel_diameter_inch = 26.0f; // fallback
            float wheel_circumference_m = wheel_diameter_inch * 0.0254f * PI;  // metry
            current_kmh = rpm * wheel_circumference_m * 60.0f / 1000.0f;
        }

        // Prosty regulator P dla prędkości
        float error = target_kmh - current_kmh;
        float kp = 0.02f;  // wzmocnienie: 1 km/h błędu → 2% duty
        float duty = error * kp;

        // Ograniczenia
        if (duty < 0.0f) duty = 0.0f;
        if (duty > 1.0f) duty = 1.0f;

        // Soft-start: nie więcej niż 5% duty na krok
        static float g_pas_duty = 0.0f;
        float max_step = 0.05f;
        if (duty > g_pas_duty + max_step) duty = g_pas_duty + max_step;
        if (duty < g_pas_duty - max_step) duty = g_pas_duty - max_step;
        g_pas_duty = duty;

        return duty;
    }

    // --- Tryb 0: PAS + przepustnica (domyślny) ---
    // Przepustnica ma priorytet — jeśli używana, działa jak tryb 1
    if (g_throttle > 0.05f) {
        return g_throttle;
    }

    // W przeciwnym razie działamy jak tryb 2 (PAS)
    if (!g_pas_active) return 0.0f;

    float target_kmh = get_target_speed_kmh(assist_lvl, speed_limit);
    if (target_kmh <= 0.0f) return 0.0f;

    // Regulator P
    float current_kmh = 0.0f;
    if (g_esc_telem.valid) {
        float wheel_diameter_inch = rx->wheel_size_inch_x10 / 10.0f;
        if (wheel_diameter_inch < 10.0f) wheel_diameter_inch = 26.0f;
        float wheel_circumference_m = wheel_diameter_inch * 0.0254f * PI;
        current_kmh = g_esc_telem.rpm * wheel_circumference_m * 60.0f / 1000.0f;
    }

    float error = target_kmh - current_kmh;
    float duty = error * 0.02f;

    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    static float g_pas_duty2 = 0.0f;
    float max_step = 0.05f;
    if (duty > g_pas_duty2 + max_step) duty = g_pas_duty2 + max_step;
    if (duty < g_pas_duty2 - max_step) duty = g_pas_duty2 - max_step;
    g_pas_duty2 = duty;

    return duty;
}

// ============================================================================
// Wysyłanie komendy do ESC
// ============================================================================

static void send_esc_command() {
    // Aktualny limit prądu z wyświetlacza (P14), 0 = brak limitu
    uint8_t current_limit = g_display.rx.current_limit_a;

    // Użyj COMM_SET_CURRENT zamiast COMM_SET_DUTY jeśli mamy limit prądu
    // W przeciwnym razie użyj duty
    if (current_limit > 0 && g_target_duty > 0.01f) {
        // Mapuj duty na prąd: duty 1.0 → current_limit amperów
        float target_current = g_target_duty * (float)current_limit;
        vesc_set_current(target_current);
    } else {
        vesc_set_duty(g_target_duty);
    }
}

// ============================================================================
// Aktualizacja danych do wyświetlacza
// ============================================================================

static void update_display_tx() {
    g_display.tx.error = g_esc_telem.valid ? g_esc_telem.fault : 0;
    g_display.tx.brake_active = g_brake_active ? 1 : 0;

    // Prąd ×10 [0.1A]
    float current = g_esc_telem.valid ? g_esc_telem.current_motor : 0.0f;
    g_display.tx.current_x10 = (uint16_t)(current * 10.0f);

    // Czas obrotu koła [ms]
    if (g_esc_telem.valid && g_esc_telem.rpm > 10.0f) {
        float wheel_rpm = g_esc_telem.rpm;  // mechaniczne RPM koła
        if (wheel_rpm > 0.5f) {
            g_display.tx.wheeltime_ms = (uint16_t)(60000.0f / wheel_rpm);
        } else {
            g_display.tx.wheeltime_ms = 0;
        }
    } else {
        g_display.tx.wheeltime_ms = 0;
    }
}

// ============================================================================
// Debug — drukowanie stanu
// ============================================================================

static unsigned long g_last_debug_ms = 0;

static void print_debug() {
    unsigned long now = millis();
    if (now - g_last_debug_ms < 1000) return;
    g_last_debug_ms = now;

    Serial.printf(
        "[DBG] thr=%.2f brake=%d PAS: cad=%.0f fwd=%d act=%d | "
        "S866: lvl=%d conn=%d P10=%d | "
        "ESC: duty=%.2f V=%.1f I=%.1fA rpm=%.0f run=%d\n",
        g_throttle, g_brake_active,
        g_pas_cadence_rpm, g_pas_forward, g_pas_active,
        g_display.rx.assist_level, g_display.connected, g_display.rx.throttle_mode,
        g_target_duty,
        g_esc_telem.valid ? g_esc_telem.v_in : 0.0f,
        g_esc_telem.valid ? g_esc_telem.current_motor : 0.0f,
        g_esc_telem.valid ? g_esc_telem.rpm : 0.0f,
        g_esc_running
    );
}

// ============================================================================
// setup() i loop()
// ============================================================================

void setup() {
    // --- Serial debug ---
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== SmartESC UART Bridge v1.0 ===");
    Serial.printf("Platform: %s | CPU: %d MHz\n", ESP.getChipModel(), ESP.getCpuFreqMHz());

    // --- GPIO ---
    pinMode(PIN_PAS, INPUT_PULLUP);
    pinMode(PIN_BRAKE, INPUT_PULLUP);
    pinMode(PIN_THROTTLE, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // --- Wyświetlacz S866 ---
    s866_init(&g_display);

    // --- ESC UART ---
    vesc_init(PIN_ESC_RX, PIN_ESC_TX, ESC_BAUD_RATE);

    // --- Timer PAS (2 kHz) ---
    g_pas_timer = timerBegin(0, 80, true);  // prescaler 80 → 1 MHz (1 tick = 1 µs)
    timerAttachInterrupt(g_pas_timer, &pas_timer_isr, true);
    timerAlarmWrite(g_pas_timer, 500, true);  // 500 µs = 2 kHz
    timerAlarmEnable(g_pas_timer);

    Serial.println("[SETUP] done.");
}

void loop() {
    unsigned long now = millis();

    // --- 1. Serwis wyświetlacza S866 ---
    s866_service(&g_display);

    // --- 2. Odczyt czujników ---
    g_throttle = read_throttle();
    g_brake_active = read_brake();
    update_pas();

    // --- 3. Serwis VESC (odbiór telemetrii) ---
    if (vesc_service(&g_esc_telem)) {
        g_last_telem_valid_ms = now;
        g_esc_running = (g_esc_telem.status == 1);
    }

    // --- 4. Okresowe poll telemetrii ---
    if (now - g_last_telem_poll_ms >= ESC_TELEM_INTERVAL_MS) {
        g_last_telem_poll_ms = now;
        vesc_send(COMM_GET_VALUES, nullptr, 0);  // wyślij zapytanie
    }

    // Sprawdź timeout telemetrii
    if (g_esc_telem.valid && (now - g_last_telem_valid_ms > ESC_TELEM_TIMEOUT_MS)) {
        g_esc_telem.valid = false;
        g_esc_running = false;
        Serial.println("[ESC] telemetry timeout");
    }

    // --- 5. Oblicz docelowe duty ---
    g_target_duty = compute_target_duty();

    // --- 6. Wyślij komendę do ESC ---
    if (now - g_last_esc_cmd_ms >= ESC_CMD_INTERVAL_MS) {
        g_last_esc_cmd_ms = now;
        send_esc_command();
    }

    // --- 7. Aktualizuj dane dla wyświetlacza ---
    update_display_tx();

    // --- 8. Debug ---
    print_debug();

    // --- Loop timing: ~10 ms ---
    delay(5);
}
