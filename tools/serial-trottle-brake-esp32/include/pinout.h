/**
 * @file pinout.h
 * @brief Definicje pinów ESP32 dla mostka UART SmartESC
 *
 * PCB z projektu bldc_driver_esp32 — wykorzystane tylko:
 *  - ESP32 + wejścia czujników + UART do wyświetlacza S866
 *  - Driver MOSFET i mostki NIE są wlutowane
 *
 * UARTy:
 *  - Serial  (UART0): USB → PC (debug/monitor)
 *  - Serial1 (UART1): GPIO4(RX)/GPIO18(TX) → ESC (VESC protokół, 115200 baud)
 *  - Serial2 (UART2): GPIO16(RX)/GPIO17(TX)/GPIO5(EN) → wyświetlacz S866 (9600 baud)
 */

#ifndef PINOUT_H
#define PINOUT_H

#include <Arduino.h>

// ============================================================================
// UART — wyświetlacz S866 (Serial2)
// ============================================================================

/** @brief UART RX — dane z wyświetlacza S866 */
#define PIN_DISPLAY_RX          16  // GPIO16

/** @brief UART TX — dane do wyświetlacza S866 */
#define PIN_DISPLAY_TX          17  // GPIO17

/** @brief UART Enable — sterowanie TXB0102 level shifter (5 V) */
#define PIN_DISPLAY_EN          5   // GPIO5

/** @brief Baud rate wyświetlacza S866 */
#define DISPLAY_BAUD_RATE       9600

// ============================================================================
// UART — ESC SmartESC_STM32_v2 (Serial1, VESC protokół binarny)
// ============================================================================

/** @brief UART RX — dane z ESC */
#define PIN_ESC_RX              4   // GPIO4 (wolny — był Hall A)

/** @brief UART TX — dane do ESC */
#define PIN_ESC_TX              18  // GPIO18 (wolny — był Hall B)

/** @brief Baud rate ESC */
#define ESC_BAUD_RATE           115200

// ============================================================================
// Wejścia czujników
// ============================================================================

/** @brief Wejście przepustnicy / gazu (ADC1_CH5, 0–3.3 V) */
#define PIN_THROTTLE            33  // GPIO33

/** @brief Wejście czujnika PAS (Pedal Assist Sensor) — GPIO z pull-up */
#define PIN_PAS                 22  // GPIO22

/** @brief Wejście hamulca — GPIO z pull-up, aktywny LOW */
#define PIN_BRAKE               23  // GPIO23

// ============================================================================
// ADC — konfiguracja przepustnicy
// ============================================================================

/// Dolny próg przepustnicy [ADC RAW] — poniżej = 0%
#define THR_ADC_MIN             200

/// Górny próg przepustnicy [ADC RAW] — powyżej = 100%
#define THR_ADC_MAX             2600

/// Ilość próbek do mediany throttle
#define THR_SAMPLES             8

/// Maksymalne odchylenie od mediany do odrzucenia outliera [ADC RAW]
#define THR_OUTLIER_THRESH      150

// ============================================================================
// PAS — konfiguracja
// ============================================================================

/// Domyślny czas debounce PAS [µs] (minimalny półokres między krawędziami)
#define PAS_DEBOUNCE_US         3000

/// Próg asymetrii do detekcji kierunku [%] (różnica HIGH vs LOW > X% → direction)
#define PAS_DIR_ASYMMETRY_PCT   5

/// Maksymalna kadencja PAS [RPM] (do ograniczenia zakresu)
#define PAS_MAX_CADENCE_RPM     120

/// Timeout PAS: brak impulsów przez X ms → kadencja = 0
#define PAS_TIMEOUT_MS          500

/// Czas potwierdzenia stałego pedałowania do przodu przed startem [ms]
#define PAS_START_DELAY_MS      300

// ============================================================================
// Hamulec — konfiguracja
// ============================================================================

/// Czas debounce hamulca [ms]
#define BRAKE_DEBOUNCE_MS       30

// ============================================================================
// Sterowanie ESC — stałe czasowe
// ============================================================================

/// Interwał wysyłania komendy duty/current do ESC [ms]
#define ESC_CMD_INTERVAL_MS     50

/// Interwał odpytywania telemetrii ESC [ms]
#define ESC_TELEM_INTERVAL_MS   200

/// Timeout telemetrii ESC [ms] — po tym czasie uznajemy ESC za rozłączony
#define ESC_TELEM_TIMEOUT_MS    1000

#endif // PINOUT_H
