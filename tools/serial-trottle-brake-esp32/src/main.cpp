// /**
//  * @file main.cpp
//  * @brief SmartESC UART Bridge — ESP32
//  *
//  * Mostek między czujnikami (PAS, przepustnica, hamulec) + wyświetlaczem S866
//  * a kontrolerem ESC SmartESC_STM32_v2 (protokół VESC przez UART).
//  *
//  * ## Architektura
//  * - Serial  (UART0): USB → PC (debug 115200)
//  * - Serial1 (UART1): GPIO4/18 → ESC (VESC protokół, 115200)
//  * - Serial2 (UART2): GPIO16/17/5 → wyświetlacz S866 (9600)
//  *
//  * ## Przepływ danych
//  * loop() co ~10 ms:
//  *   1. Serwis wyświetlacza S866 (odbiór ramek, wysyłanie odpowiedzi)
//  *   2. Odczyt czujników (PAS z przerwania, przepustnica, hamulec)
//  *   3. Obliczenie docelowego duty/prądu na podstawie trybu sterowania
//  *   4. Wysłanie komendy do ESC (co ESC_CMD_INTERVAL_MS)
//  *   5. Odczyt telemetrii ESC (co ESC_TELEM_INTERVAL_MS)
//  */

#include <Arduino.h>
#include "pinout.h"
#include "display_s866.h"
#include "vesc_proto.h"
#include "pas.h"
#include "throttle.h"

#define MAX_ASSIST_LEVEL 15
#define MIN_THROTTLE 0.05f

// // ============================================================================
// // PAS — konfiguracja (nowy moduł z przerwaniem CHANGE)
// // ============================================================================

static pas_config_t g_pas_cfg = {
    .magnets          = PAS_DEFAULT_MAGNETS,
    .dir_ratio_thresh = 1.0f,
    .cadence_data_count = PAS_MIN_SAMPLE_COUNT,
    .cadence_tau_s    = PAS_DEFAULT_CADENCE_TAU_S,
    .direction_invert = false,
};

// ============================================================================
// Zmienne globalne — czujniki
// ============================================================================

static pas_data_t g_pas_data;  ///< Dane PAS (kadencja, kierunek)

/// Przefiltrowana wartość przepustnicy [0.0–1.0]
static float g_throttle = 0.0f;

/// Hamulec aktywny (debounced)
static bool g_brake_active = false;

// ============================================================================
// Zmienne globalne — sterowanie
// ============================================================================

/// Docelowe duty do wysłania do ESC [-1.0..1.0]
static float g_target_current = 0.0f;

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
// Odczyt hamulca — debounce
// ============================================================================

static bool g_brake_raw = false;
static unsigned long g_brake_change_ms = 0;
static bool g_brake_stable = false;


// @brief Odczyt stanu hamulca (GPIO z pull-up, aktywny LOW) z debouncingiem
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
// Mapowanie poziomu wspomagania na docelowy prąd [A]
// ============================================================================

float get_target_current(uint8_t assist_level, pas_data_t* pas, uint8_t current_limit) {
    if (assist_level == 0 || current_limit == 0) return 0.0f;

    
    float target = (float)current_limit / MAX_ASSIST_LEVEL * assist_level;

    return target;
}



// ============================================================================
// Obliczanie docelowego prądu na podstawie trybu sterowania i danych z czujników
// ============================================================================

static float compute_target_current() {
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
        return g_throttle * (float)rx->current_limit_a ; // przeskaluj do limitu prądu
    }

    // --- Tryb 2: tylko PAS ---
    if (drive_mode == 0) {
        if (g_pas_data.pedal_rpm == 0) return 0.0f;

        if (g_pas_data.pedal_rpm > PAS_MAX_RPM) {
            return rx->current_limit_a; // maksymalny prąd
        }
        else if (g_pas_data.pedal_rpm < PAS_MIN_RPM) {
            return 0.0f; // brak wspomagania
        }

        // Docelowy prąd = limit prądu * (kadencja / PAS_MAX_RPM) * (assist_level / MAX_ASSIST_LEVEL)
        return (float)rx->current_limit_a * (g_pas_data.pedal_rpm / PAS_MAX_RPM) * (float)rx->assist_level / MAX_ASSIST_LEVEL; 

    }

    // --- Tryb 2: PAS + przepustnica (domyślny) ---
    float throtle_current = g_throttle * (float)rx->current_limit_a;
    float pas_current = (float)rx->current_limit_a * (g_pas_data.pedal_rpm / PAS_MAX_RPM) * (float)rx->assist_level / MAX_ASSIST_LEVEL; 
    if (throtle_current > pas_current) {
        return throtle_current;
    }else {
        return pas_current;
    }

}

// ============================================================================
// Wysyłanie komendy do ESC
// ============================================================================

static void send_esc_command() {
    uint8_t current_limit = g_display.rx.current_limit_a;

    if (current_limit > 0 ) {
        vesc_set_current(g_target_current);
    } else {
        vesc_set_current(0.0f);
    }
}

// // ============================================================================
// // Debug
// // ============================================================================

static unsigned long g_last_debug_ms = 0;

static void print_debug() {
    unsigned long now = millis();
    if (now - g_last_debug_ms < 1000) return;
    g_last_debug_ms = now;

    if (!g_esc_telem.valid) {
        unsigned long since_last = g_last_telem_valid_ms > 0 ? (now - g_last_telem_valid_ms) : 0;
        Serial.printf("[DBG] thr=%.2f brake=%d | "
                      "PAS: cad=%.0f fwd=%d | "
                      "S866: lvl=%d conn=%d P10=%d  P13=%d P14=%d | "
                      "ESC: --- brak telemetrii (od %lums) ---\n",
                      g_throttle, g_brake_active,
                      g_pas_data.pedal_rpm, g_pas_data.direction,
                      g_display.rx.assist_level, g_display.connected, g_display.rx.throttle_mode, g_display.rx.speed_magnets, g_display.rx.current_limit_a, 
                      since_last);
        return;
    }

    Serial.printf(
        "[DBG] thr=%.2f brake=%d | "
        "PAS: cad=%.0f fwd=%d | "
        "S866: lvl=%d conn=%d P10=%d  P13=%d P14=%d| "
        "ESC: V=%.1fV Ibat=%.1fA Imot=%.1fA rpm=%.0f "
        "current=%.1fA temp=%.0f/%.0f\u00B0C Wh=%.1f run=%d fault=%d\n",
        g_throttle, g_brake_active,
        g_pas_data.pedal_rpm, g_pas_data.direction, 
        g_display.rx.assist_level, g_display.connected, g_display.rx.throttle_mode,g_display.rx.speed_magnets, g_display.rx.current_limit_a,
        g_esc_telem.v_in,
        g_esc_telem.current_in,
        g_esc_telem.current_motor,
        g_esc_telem.rpm,
        g_target_current ,
        g_esc_telem.temp_mos1,
        g_esc_telem.temp_mos2,
        g_esc_telem.wh,
        g_esc_running,
        g_esc_telem.fault
    );
}

// // ============================================================================
// // setup() i loop()
// // ============================================================================

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== SmartESC UART Bridge v2.0 ===");
    Serial.printf("Platform: %s | CPU: %d MHz\n", ESP.getChipModel(), ESP.getCpuFreqMHz());

    // --- GPIO ---
    pinMode(PIN_BRAKE, INPUT_PULLUP);
    pinMode(PIN_THROTTLE, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // --- PAS (przerwanie CHANGE) ---
    pas_init(&g_pas_cfg);
    Serial.printf("PAS: GPIO%d | magnets: %u | dir_invert: %d\n",
                  PIN_PAS, g_pas_cfg.magnets, g_pas_cfg.direction_invert);

    // --- Wyświetlacz S866 ---
    s866_init(&g_display);

    // // --- Zasilanie płytki M365 (TPS_ENA przez GPIO19) ---
    // pinMode(PIN_M365_POWER, OUTPUT);
    // digitalWrite(PIN_M365_POWER, HIGH);
    // delay(M365_POWER_ON_DELAY_MS);

    // --- ESC UART ---
    vesc_init(PIN_ESC_RX, PIN_ESC_TX, ESC_BAUD_RATE);

    // --- Sprawdzenie czy ESC odpowiada ---
    Serial.print("[ESC] telemetry poll... ");
    esc_telem_t startup_telem;
    if (vesc_poll_telem(&startup_telem, 200)) {
        g_esc_telem = startup_telem;
        g_last_telem_valid_ms = millis();
        g_esc_running = (startup_telem.status == 1);
        Serial.printf("OK (%.1fV)\n", startup_telem.v_in);
    } else {
        Serial.println("BRAK ODPOWIEDZI");
        Serial.println("!!! Sprawdź: zasilanie ESC, UART piny, baud rate !!!");
    }

    Serial.println("[SETUP] done.");
}

void loop() {
    unsigned long now = millis();

    // --- 1. Serwis wyświetlacza S866 ---
    s866_service(&g_display);

    // --- 2. Odczyt czujników ---
    pas_get_data(&g_pas_data);
    g_throttle = read_throttle();
    g_brake_active = read_brake();

    // --- 3. Serwis VESC (odbiór telemetrii) ---
    if (vesc_service(&g_esc_telem)) {
        g_last_telem_valid_ms = now;
        g_esc_running = (g_esc_telem.status == 1);
    }

    // --- 4. Okresowe poll telemetrii ---
    if (now - g_last_telem_poll_ms >= ESC_TELEM_INTERVAL_MS) {
        g_last_telem_poll_ms = now;
        vesc_send(COMM_GET_VALUES, nullptr, 0);
    }

    if (g_esc_telem.valid && (now - g_last_telem_valid_ms > ESC_TELEM_TIMEOUT_MS)) {
        g_esc_telem.valid = false;
        g_esc_running = false;
        Serial.println("[ESC] telemetry timeout");
    }

    // --- 5. Oblicz docelowe duty ---
    g_target_current = compute_target_current();

    // --- 6. Wyślij komendę do ESC ---
    if (now - g_last_esc_cmd_ms >= ESC_CMD_INTERVAL_MS) {
        g_last_esc_cmd_ms = now;
        send_esc_command();
    }

    // --- 7. Aktualizuj dane dla wyświetlacza ---
    s866_update_tx(&g_display, &g_esc_telem, g_brake_active);

    // --- 8. Debug ---
    print_debug();

    delay(2);
}
