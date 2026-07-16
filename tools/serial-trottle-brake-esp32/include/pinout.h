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
// Zasilanie płytki M365 (sterowanie TPS_ENA przez ESP32)
// ============================================================================

/**
 * @brief GPIO sterujące tranzystorem podtrzymującym zasilanie płytki M365.
 *
 * Podłączone do linii TPS_ENA (PC15 na STM32 po usunięciu oryginalnego MCU).
 * HIGH (3.3V) = płytka WŁĄCZONA (podtrzymanie zasilania)
 * LOW  (0V)  = płytka WYŁĄCZONA (odcięcie zasilania)
 */
#define PIN_M365_POWER           19  // GPIO19 → TPS_ENA

/// Opóźnienie po włączeniu zasilania M365 przed inicjalizacją UART [ms]
#define M365_POWER_ON_DELAY_MS   500

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
#define THR_ADC_MIN             400

/// Górny próg przepustnicy [ADC RAW] — powyżej = 100%
#define THR_ADC_MAX             2600

/// Ilość próbek do mediany throttle
#define THR_SAMPLES             8

/// Maksymalne odchylenie od mediany do odrzucenia outliera [ADC RAW]
#define THR_OUTLIER_THRESH      150

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
