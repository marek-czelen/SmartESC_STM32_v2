/**
 * @file vesc_proto.h
 * @brief Protokół VESC — binarna komunikacja z ESC SmartESC_STM32_v2
 *
 * Format ramki:
 *   [0x02] [len] [cmd_id + payload...] [CRC16 hi] [CRC16 lo] [0x03]
 *
 * Komendy używane przez mostek:
 *   COMM_GET_VALUES    = 0x04 — pobranie telemetrii
 *   COMM_SET_DUTY      = 0x05 — ustawienie duty cycle
 *   COMM_SET_CURRENT   = 0x06 — ustawienie prądu
 *   COMM_TERMINAL_CMD  = 0x14 — komenda tekstowa CLI (diagnostyka)
 */

#ifndef VESC_PROTO_H
#define VESC_PROTO_H

#include <Arduino.h>

// ============================================================================
// ID komend VESC
// ============================================================================

#define COMM_GET_VALUES         0x04
#define COMM_SET_DUTY           0x05
#define COMM_SET_CURRENT        0x06
#define COMM_TERMINAL_CMD       0x14

// ============================================================================
// Stałe protokołu
// ============================================================================

#define VESC_FRAME_START        0x02
#define VESC_FRAME_STOP         0x03
#define VESC_MAX_PAYLOAD        64      ///< Maksymalna długość payloadu

// ============================================================================
// Struktura telemetrii ESC
// ============================================================================

typedef struct {
    float    temp_mos1;         ///< Temperatura MOS 1 [°C]
    float    temp_mos2;         ///< Temperatura MOS 2 [°C]
    float    current_motor;     ///< Prąd fazowy silnika [A]
    float    current_in;        ///< Prąd wejściowy (z baterii) [A]
    float    id;                ///< FOC Id [A]
    float    iq;                ///< FOC Iq [A]
    float    duty;              ///< Duty cycle [0.0–1.0]
    int32_t  erpm;              ///< ERPM silnika
    float    v_in;              ///< Napięcie wejściowe [V]
    float    ah;                ///< Zużyte Ah
    float    ah_charged;        ///< Naładowane Ah (regen)
    float    wh;                ///< Zużyte Wh
    float    wh_charged;        ///< Naładowane Wh (regen)
    int32_t  tacho;             ///< Licznik obrotów
    int32_t  tacho_abs;         ///< Licznik obrotów (absolutny)
    uint8_t  fault;             ///< Kod błędu
    float    pid_pos;           ///< PID position
    uint8_t  ctrl_id;           ///< ID kontrolera
    float    vd;                ///< FOC Vd [V]
    float    vq;                ///< FOC Vq [V]
    uint8_t  status;            ///< Status (0=off, 1=run)
    // Pochodne
    float    rpm;               ///< RPM mechaniczne (erpm / 15 par biegunów M365)
    float    power_w;           ///< Moc [W] (v_in * current_in)
    bool     valid;             ///< Czy dane są aktualne
} esc_telem_t;

// ============================================================================
// API protokołu
// ============================================================================

/**
 * @brief Inicjalizacja UART dla ESC.
 * @param rxPin  Pin RX (GPIO)
 * @param txPin  Pin TX (GPIO)
 * @param baud   Baud rate (domyślnie 115200)
 */
void vesc_init(uint8_t rxPin, uint8_t txPin, uint32_t baud = ESC_BAUD_RATE);

/**
 * @brief Oblicza CRC16 XMODEM dla danych.
 */
uint16_t vesc_crc16(const uint8_t* data, size_t len);

/**
 * @brief Wysyła ramkę VESC do ESC.
 * @param cmdId  ID komendy
 * @param payload  Dane payload (bez cmdId)
 * @param len  Długość payloadu
 */
void vesc_send(uint8_t cmdId, const uint8_t* payload, size_t len);

/**
 * @brief Wysyła COMM_SET_DUTY do ESC.
 * @param duty  Duty cycle [-1.0..1.0], ujemny = hamowanie/regen
 */
void vesc_set_duty(float duty);

/**
 * @brief Wysyła COMM_SET_CURRENT do ESC.
 * @param current  Prąd [A], ujemny = hamowanie/regen
 */
void vesc_set_current(float current);

/**
 * @brief Przetwarza odebrane dane z ESC — wywoływać w loop().
 *
 * Odczytuje bajty z Serial1, parsuje ramki, aktualizuje telemetrię.
 *
 * @param telem  Wskaźnik na strukturę telemetrii do aktualizacji
 * @return true jeśli odebrano nową, poprawną ramkę telemetrii
 */
bool vesc_service(esc_telem_t* telem);

/**
 * @brief Wysyła COMM_GET_VALUES i czeka na odpowiedź (blokujące).
 * @param telem  Wskaźnik na strukturę telemetrii do wypełnienia
 * @param timeout_ms  Timeout oczekiwania [ms]
 * @return true jeśli otrzymano poprawną odpowiedź
 */
bool vesc_poll_telem(esc_telem_t* telem, uint32_t timeout_ms = 100);

#endif // VESC_PROTO_H
