/**
 * @file display_s866.h
 * @brief Obsługa wyświetlacza S866 — protokół nr 2
 *
 * Protokół komunikacyjny z wyświetlaczem e-bike S866 (tzw. "protocol 2").
 * Wyświetlacz jest masterem — wysyła ramkę 20 bajtów co ~100 ms.
 * Kontroler odpowiada ramką 14 bajtów z danymi telemetrycznymi.
 * Checksum: XOR wszystkich bajtów oprócz ostatniego.
 *
 * ## UART (Serial2)
 * - RX: GPIO16 — dane z wyświetlacza
 * - TX: GPIO17 — dane do wyświetlacza
 * - EN: GPIO5  — sterowanie TXB0102 level shifter
 * - Baud: 9600
 *
 * ## Ramka RX (wyświetlacz → kontroler, 20 bajtów)
 * | Bajt | Opis                                    |
 * |------|-----------------------------------------|
 * | 0-2  | Nagłówek                                |
 * | 3    | Throttle mode (P10)                      |
 * | 4    | Assist level (0-9, bity 0-3)            |
 * | 5    | Flagi: bit6=ZeroStart, bit5=Headlight, bit1=PushAssist |
 * | 6    | P07: liczba magnesów czujnika prędkości  |
 * | 7-8  | P06: rozmiar koła ×10 cali               |
 * | 9    | P11: czułość PAS                         |
 * | 10   | P12: siła startu PAS                     |
 * | 12   | P08: limit prędkości max                 |
 * | 13   | P14: limit prądu [A]                     |
 * | 14-15| P15: napięcie min ×10 [V]                |
 * | 18   | bits0-3=P13 magnesy PAS, bit6=Cruise     |
 * | 19   | Checksum (XOR bajtów 0-18)              |
 *
 * ## Ramka TX (kontroler → wyświetlacz, 14 bajtów)
 * | Bajt | Opis                                     |
 * |------|------------------------------------------|
 * | 0    | 0x02 (start)                             |
 * | 1    | 0x0E (długość = 14)                      |
 * | 2    | 0x01 (typ)                               |
 * | 3    | Error code                               |
 * | 4    | Brake active (bit 5)                     |
 * | 5    | 0x00                                     |
 * | 6-7  | Prąd ×10 [0.1A] (HiByte, LoByte)        |
 * | 8-9  | Czas obrotu koła [ms] (HiByte, LoByte)  |
 * | 10   | 0x00                                     |
 * | 11   | 0x00                                     |
 * | 12   | 0xFF                                     |
 * | 13   | Checksum (XOR bajtów 0-12)               |
 */

#ifndef DISPLAY_S866_H
#define DISPLAY_S866_H

#include <Arduino.h>
#include "vesc_proto.h"

// ============================================================================
// Stałe protokołu
// ============================================================================

#define S866_RX_FRAME_LEN       20      ///< Ramka od wyświetlacza
#define S866_TX_FRAME_LEN       14      ///< Ramka do wyświetlacza
#define S866_BAUD_RATE          9600    ///< Prędkość UART wyświetlacza
#define S866_TIMEOUT_MS         2000    ///< Timeout rozłączenia [ms]
#define S866_CONNECT_CONFIRM_MS 1000    ///< Ciągła komunikacja wymagana do potwierdzenia [ms]
#define S866_INTERBYTE_TIMEOUT_MS 50   ///< Timeout między bajtami w ramce [ms]

// ============================================================================
// Struktury danych
// ============================================================================

/**
 * @brief Parametry odebrane od wyświetlacza (ramka RX).
 */
typedef struct {
    uint8_t  assist_level;          ///< Poziom wspomagania 0-9 (raw z bajtu 4, bity 0-3)
    uint8_t  headlight;             ///< Światło przednie 0/1
    uint8_t  push_assist;           ///< Asystent pchania 0/1
    uint8_t  zero_start;            ///< Start od zera 0/1
    uint8_t  cruise_control;        ///< Tempomat 0/1
    uint8_t  throttle_mode;         ///< Tryb przepustnicy P10 (bajt 3)
    uint8_t  speed_magnets;         ///< P07: magnesy czujnika prędkości (bajt 6)
    uint8_t  start_delay_pas;       ///< P11: czułość PAS (bajt 9)
    uint8_t  boost_power;           ///< P12: siła startu PAS (bajt 10)
    uint8_t  speed_max_limit;       ///< P08: limit prędkości max (bajt 12)
    uint8_t  current_limit_a;       ///< P14: limit prądu [A] (bajt 13)
    uint16_t voltage_min_x10;       ///< P15: napięcie min ×10 [V] (bajty 14-15)
    uint16_t wheel_size_inch_x10;   ///< P06: rozmiar koła ×10 cali (bajty 7-8)
    uint8_t  num_pas_magnets;       ///< P13: magnesy PAS (bajt 18, bity 0-3)
} s866_rx_params_t;

/**
 * @brief Parametry wysyłane do wyświetlacza (ramka TX).
 */
typedef struct {
    uint8_t  error;                 ///< Kod błędu (0 = OK)
    uint8_t  brake_active;          ///< Hamulec aktywny 0/1
    uint16_t current_x10;           ///< Prąd ×10 [0.1 A]
    uint16_t wheeltime_ms;          ///< Czas obrotu koła [ms] (0 = stoi)
} s866_tx_params_t;

/**
 * @brief Kontekst wyświetlacza S866 — cały stan komunikacji.
 */
typedef struct {
    uint8_t          rx_buf[S866_RX_FRAME_LEN];  ///< Bufor odbioru ramki
    uint8_t          rx_count;                    ///< Liczba odebranych bajtów
    s866_rx_params_t rx;                          ///< Sparsowane parametry RX
    s866_tx_params_t tx;                          ///< Parametry do wysłania TX
    bool             connected;                   ///< Wyświetlacz podłączony?
    unsigned long    last_valid_ms;               ///< Czas ostatniej poprawnej ramki
    unsigned long    first_valid_ms;              ///< Czas pierwszej ramki w serii (0=brak)
    unsigned long    last_byte_ms;                ///< Czas ostatniego odebranego bajtu
} s866_display_t;

// ============================================================================
// Funkcje publiczne
// ============================================================================

/**
 * @brief Włącza tryb wyświetlacza: EN=HIGH, Serial2.begin(9600).
 */
void s866_init(s866_display_t* ctx);

/**
 * @brief Wyłącza tryb wyświetlacza: EN=LOW, Serial2.end().
 */
void s866_deinit();

/**
 * @brief Obsługa komunikacji z wyświetlaczem — wywoływać w loop().
 *
 * Czyta bajty z Serial2, parsuje ramki, wysyła odpowiedzi.
 * Ustawia ctx->connected na true/false.
 */
void s866_service(s866_display_t* ctx);

/**
 * @brief Aktualizuje dane TX wyświetlacza na podstawie telemetrii ESC i stanu hamulca.
 *
 * Wypełnia ctx->tx (error, brake, current, wheeltime) przed wysłaniem ramki.
 *
 * @param ctx          Kontekst wyświetlacza
 * @param telem        Aktualna telemetria ESC (może być NULL)
 * @param brake_active Czy hamulec jest aktywny
 */
void s866_update_tx(s866_display_t* ctx, const esc_telem_t* telem, bool brake_active);

#endif // DISPLAY_S866_H
