/**
 * @file display_s866.cpp
 * @brief Implementacja protokołu wyświetlacza S866 ("protocol 2")
 *
 * Bazuje na implementacji z bldc_driver_esp32 (EBiCS_Firmware — GPL v3).
 * Zaadaptowane na potrzeby mostka UART SmartESC.
 *
 * ## Synchronizacja ramek
 * 1. Zbieramy bajty do bufora 20-bajtowego
 * 2. Po zapełnieniu sprawdzamy checksum (XOR bajtów 0-18 == bajt 19)
 * 3. Jeśli poprawny → parsujemy i odpowiadamy
 * 4. Jeśli nie → sliding window (przesuwamy o 1 bajt)
 * 5. Timeout międzybajtowy (50 ms) resetuje bufor
 */

#include "display_s866.h"
#include "pinout.h"

// ============================================================================
// Funkcje wewnętrzne
// ============================================================================

/**
 * @brief Oblicza checksum XOR dla ramki.
 */
static uint8_t s866_calc_checksum(const uint8_t* buf, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len - 1; i++) {
        cs ^= buf[i];
    }
    return cs;
}

/**
 * @brief Parsuje odebraną ramkę 20-bajtową z wyświetlacza.
 */
static void s866_parse_rx_frame(s866_display_t* ctx) {
    const uint8_t* f = ctx->rx_buf;

    ctx->rx.assist_level        = f[4] & 0x0F;                 // 0-15 (zwykle 0-5 lub 0-9)
    ctx->rx.throttle_mode       = f[3];                         // P10
    ctx->rx.headlight           = (f[5] >> 5) & 0x01;
    ctx->rx.push_assist         = (f[5] >> 1) & 0x01;
    ctx->rx.zero_start          = (f[5] >> 6) & 0x01;
    ctx->rx.speed_magnets       = f[6];                         // P07
    ctx->rx.wheel_size_inch_x10 = ((uint16_t)f[7] << 8) | f[8]; // P06
    ctx->rx.start_delay_pas     = f[9];                         // P11
    ctx->rx.boost_power         = f[10];                        // P12
    ctx->rx.speed_max_limit     = f[12];                        // P08
    ctx->rx.current_limit_a     = f[13];                        // P14
    ctx->rx.voltage_min_x10     = ((uint16_t)f[14] << 8) | f[15]; // P15
    ctx->rx.num_pas_magnets     = f[18] & 0x0F;                 // P13
    ctx->rx.cruise_control      = (f[18] >> 6) & 0x01;
}

/**
 * @brief Wysyła ramkę odpowiedzi (14 bajtów) do wyświetlacza.
 */
static void s866_send_response(s866_display_t* ctx) {
    uint8_t tx[S866_TX_FRAME_LEN] = {
        0x02,                                        // [0]  Start
        0x0E,                                        // [1]  Length = 14
        0x01,                                        // [2]  Type
        ctx->tx.error,                               // [3]  Error code
        (uint8_t)(ctx->tx.brake_active << 5),        // [4]  Brake flag (bit 5)
        0x00,                                        // [5]
        (uint8_t)(ctx->tx.current_x10 >> 8),         // [6]  Current hi
        (uint8_t)(ctx->tx.current_x10 & 0xFF),       // [7]  Current lo
        (uint8_t)(ctx->tx.wheeltime_ms >> 8),        // [8]  Wheeltime hi
        (uint8_t)(ctx->tx.wheeltime_ms & 0xFF),      // [9]  Wheeltime lo
        0x00,                                        // [10]
        0x00,                                        // [11]
        0xFF,                                        // [12]
        0x00                                         // [13] Checksum (poniżej)
    };
    tx[S866_TX_FRAME_LEN - 1] = s866_calc_checksum(tx, S866_TX_FRAME_LEN);

    Serial2.write(tx, S866_TX_FRAME_LEN);
}

// ============================================================================
// Funkcje publiczne
// ============================================================================

void s866_init(s866_display_t* ctx) {
    memset(ctx, 0, sizeof(*ctx));

    // Włącz konwerter poziomów TXB0102DCU
    pinMode(PIN_DISPLAY_EN, OUTPUT);
    digitalWrite(PIN_DISPLAY_EN, HIGH);
    delay(10);

    // Serial2: RX=GPIO16, TX=GPIO17
    Serial2.end();
    delay(10);
    Serial2.begin(S866_BAUD_RATE, SERIAL_8N1, PIN_DISPLAY_RX, PIN_DISPLAY_TX);

    Serial.println("[S866] init OK");
}

void s866_deinit() {
    digitalWrite(PIN_DISPLAY_EN, LOW);
    Serial2.end();
    delay(10);
    Serial.println("[S866] deinit");
}

void s866_service(s866_display_t* ctx) {
    unsigned long now = millis();

    // --- Odbiór bajtów i synchronizacja ramki ---
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        ctx->last_byte_ms = now;

        if (ctx->rx_count < S866_RX_FRAME_LEN) {
            ctx->rx_buf[ctx->rx_count++] = b;
        }

        // Pełna ramka (20 bajtów) — sprawdź checksum
        if (ctx->rx_count >= S866_RX_FRAME_LEN) {
            uint8_t expected = s866_calc_checksum(ctx->rx_buf, S866_RX_FRAME_LEN);

            if (ctx->rx_buf[S866_RX_FRAME_LEN - 1] == expected) {
                // Poprawna ramka
                s866_parse_rx_frame(ctx);
                ctx->last_valid_ms = now;
                if (ctx->first_valid_ms == 0) {
                    ctx->first_valid_ms = now;
                }
                if (!ctx->connected && (now - ctx->first_valid_ms >= S866_CONNECT_CONFIRM_MS)) {
                    ctx->connected = true;
                    Serial.println("[S866] CONNECTED");
                }

                s866_send_response(ctx);
                ctx->rx_count = 0;
            } else {
                // Zły checksum — sliding window resync
                memmove(ctx->rx_buf, ctx->rx_buf + 1, S866_RX_FRAME_LEN - 1);
                ctx->rx_count = S866_RX_FRAME_LEN - 1;
            }
        }
    }

    // --- Timeout międzybajtowy ---
    if (ctx->rx_count > 0 && (now - ctx->last_byte_ms > S866_INTERBYTE_TIMEOUT_MS)) {
        ctx->rx_count = 0;
    }

    // --- Timeout połączenia ---
    if (ctx->connected && (now - ctx->last_valid_ms > S866_TIMEOUT_MS)) {
        ctx->connected = false;
        ctx->first_valid_ms = 0;
        Serial.println("[S866] DISCONNECTED");
    }
    if (!ctx->connected && ctx->first_valid_ms > 0 &&
        (now - ctx->last_valid_ms > S866_TIMEOUT_MS)) {
        ctx->first_valid_ms = 0;
    }
}
