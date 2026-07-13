/**
 * @file vesc_proto.cpp
 * @brief Implementacja protokołu VESC dla komunikacji z ESC SmartESC_STM32_v2
 *
 * Format ramki: [0x02] [len] [cmd+payload] [CRC16 hi] [CRC16 lo] [0x03]
 * CRC16: XMODEM (polynomial 0x1021)
 */

#include "vesc_proto.h"
#include "pinout.h"

// UART dla ESC
static HardwareSerial* g_esc_serial = nullptr;

// Bufor odbioru
static uint8_t  g_rx_buf[VESC_MAX_PAYLOAD + 6];  // start + len + payload + crc2 + stop
static size_t   g_rx_count = 0;
static bool     g_rx_in_frame = false;
static uint8_t  g_rx_payload_len = 0;

// ============================================================================
// CRC16 XMODEM
// ============================================================================

uint16_t vesc_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

// ============================================================================
// Funkcje publiczne
// ============================================================================

void vesc_init(uint8_t rxPin, uint8_t txPin, uint32_t baud) {
    g_esc_serial = &Serial1;
    g_esc_serial->end();
    delay(10);
    g_esc_serial->begin(baud, SERIAL_8N1, rxPin, txPin);
    Serial.printf("[VESC] init OK, baud=%lu, RX=GPIO%d, TX=GPIO%d\n",
                  baud, rxPin, txPin);
}

void vesc_send(uint8_t cmdId, const uint8_t* payload, size_t len) {
    if (!g_esc_serial) return;

    // Buduj payload: [cmdId] + [dane]
    uint8_t pl[VESC_MAX_PAYLOAD + 1];
    pl[0] = cmdId;
    if (payload && len > 0) {
        memcpy(pl + 1, payload, len);
    }
    size_t plen = 1 + len;

    uint16_t crc = vesc_crc16(pl, plen);

    // Ramka: [0x02] [len] [payload...] [CRC hi] [CRC lo] [0x03]
    g_esc_serial->write(VESC_FRAME_START);
    g_esc_serial->write((uint8_t)plen);
    g_esc_serial->write(pl, plen);
    g_esc_serial->write((uint8_t)(crc >> 8));
    g_esc_serial->write((uint8_t)(crc & 0xFF));
    g_esc_serial->write(VESC_FRAME_STOP);
    g_esc_serial->flush();
}

void vesc_set_duty(float duty) {
    // COMM_SET_DUTY: int32_t duty × 100000
    int32_t duty_scaled = (int32_t)(duty * 100000.0f);
    uint8_t payload[4];
    payload[0] = (duty_scaled >> 24) & 0xFF;
    payload[1] = (duty_scaled >> 16) & 0xFF;
    payload[2] = (duty_scaled >> 8) & 0xFF;
    payload[3] = duty_scaled & 0xFF;
    vesc_send(COMM_SET_DUTY, payload, 4);
}

void vesc_set_current(float current) {
    // COMM_SET_CURRENT: int32_t current × 1000
    int32_t cur_scaled = (int32_t)(current * 1000.0f);
    uint8_t payload[4];
    payload[0] = (cur_scaled >> 24) & 0xFF;
    payload[1] = (cur_scaled >> 16) & 0xFF;
    payload[2] = (cur_scaled >> 8) & 0xFF;
    payload[3] = cur_scaled & 0xFF;
    vesc_send(COMM_SET_CURRENT, payload, 4);
}

// ============================================================================
// Parsowanie telemetrii
// ============================================================================

static int16_t buf_get_int16(const uint8_t* data, size_t* idx) {
    int16_t v = ((int16_t)data[*idx] << 8) | data[*idx + 1];
    *idx += 2;
    return v;
}

static int32_t buf_get_int32(const uint8_t* data, size_t* idx) {
    int32_t v = ((int32_t)data[*idx] << 24) | ((int32_t)data[*idx + 1] << 16) |
                ((int32_t)data[*idx + 2] << 8) | data[*idx + 3];
    *idx += 4;
    return v;
}

static bool parse_get_values(const uint8_t* payload, size_t len, esc_telem_t* t) {
    // Payload: [cmdId=0x04] [dane...]
    if (len < 2 || payload[0] != COMM_GET_VALUES) return false;

    size_t idx = 1;  // pomiń cmdId
    memset(t, 0, sizeof(*t));

    t->temp_mos1     = buf_get_int16(payload, &idx) / 10.0f;
    t->temp_mos2     = buf_get_int16(payload, &idx) / 10.0f;
    t->current_motor = buf_get_int32(payload, &idx) / 100.0f;
    t->current_in    = buf_get_int32(payload, &idx) / 100.0f;
    t->id            = buf_get_int32(payload, &idx) / 100.0f;
    t->iq            = buf_get_int32(payload, &idx) / 100.0f;
    t->duty          = buf_get_int16(payload, &idx) / 1000.0f;
    t->erpm          = buf_get_int32(payload, &idx);
    t->v_in          = buf_get_int16(payload, &idx) / 10.0f;
    t->ah            = buf_get_int32(payload, &idx) / 10000.0f;
    t->ah_charged    = buf_get_int32(payload, &idx) / 10000.0f;
    t->wh            = buf_get_int32(payload, &idx) / 10000.0f;
    t->wh_charged    = buf_get_int32(payload, &idx) / 10000.0f;
    t->tacho         = buf_get_int32(payload, &idx);
    t->tacho_abs     = buf_get_int32(payload, &idx);
    t->fault         = payload[idx++];
    t->pid_pos       = buf_get_int32(payload, &idx) / 1000000.0f;
    t->ctrl_id       = payload[idx++];

    // 3× temp MOS (nieużywane)
    idx += 6;  // skip 3 × int16

    t->vd   = buf_get_int32(payload, &idx) / 1000.0f;
    t->vq   = buf_get_int32(payload, &idx) / 1000.0f;
    if (idx < len) {
        t->status = payload[idx++];
    }

    // Obliczenia pochodne
    t->rpm     = t->erpm / 15.0f;          // 15 par biegunów M365
    t->power_w = t->v_in * t->current_in;
    t->valid   = true;

    return true;
}

// ============================================================================
// Serwis odbioru
// ============================================================================

bool vesc_service(esc_telem_t* telem) {
    if (!g_esc_serial) return false;

    bool got_telem = false;

    while (g_esc_serial->available()) {
        uint8_t b = g_esc_serial->read();

        if (!g_rx_in_frame) {
            // Szukaj startu ramki
            if (b == VESC_FRAME_START) {
                g_rx_in_frame = true;
                g_rx_count = 0;
                g_rx_buf[g_rx_count++] = b;
            }
            continue;
        }

        // W trakcie ramki
        g_rx_buf[g_rx_count++] = b;

        // Minimalna ramka: [0x02] [len] [cmd...] [CRC2] [0x03] = min 6 bajtów
        if (g_rx_count >= 3 && g_rx_payload_len == 0) {
            g_rx_payload_len = g_rx_buf[1];  // bajt 1 = długość payloadu
        }

        if (g_rx_payload_len > 0) {
            size_t expected = 2 + g_rx_payload_len + 2 + 1;  // start + len + payload + crc2 + stop
            if (g_rx_count >= expected) {
                // Sprawdź stop
                if (g_rx_buf[expected - 1] == VESC_FRAME_STOP) {
                    // Sprawdź CRC
                    const uint8_t* pl = g_rx_buf + 2;  // za start+len
                    uint16_t crc_calc = vesc_crc16(pl, g_rx_payload_len);
                    uint16_t crc_recv = ((uint16_t)g_rx_buf[expected - 3] << 8) |
                                        g_rx_buf[expected - 2];

                    if (crc_calc == crc_recv) {
                        // Poprawna ramka — parsuj jeśli to telemetria
                        if (pl[0] == COMM_GET_VALUES) {
                            got_telem = parse_get_values(pl, g_rx_payload_len, telem);
                        }
                    }
                }
                // Reset
                g_rx_in_frame = false;
                g_rx_payload_len = 0;
                g_rx_count = 0;
            }
        }

        // Zabezpieczenie przed overflow
        if (g_rx_count >= sizeof(g_rx_buf)) {
            g_rx_in_frame = false;
            g_rx_payload_len = 0;
            g_rx_count = 0;
        }
    }

    return got_telem;
}

bool vesc_poll_telem(esc_telem_t* telem, uint32_t timeout_ms) {
    // Wyślij COMM_GET_VALUES (bez payloadu)
    vesc_send(COMM_GET_VALUES, nullptr, 0);

    // Czekaj na odpowiedź
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        if (vesc_service(telem)) {
            return true;
        }
        delay(5);
    }
    return false;
}
