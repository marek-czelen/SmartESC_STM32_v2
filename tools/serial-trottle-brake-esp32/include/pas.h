#ifndef PAS_H
#define PAS_H

#define PAS_DEFAULT_MAGNETS 12
#define PAS_DEFAULT_CADENCE_TAU_S 2
#define PAS_QUEUE_SIZE 50
#define PAS_MIN_SAMPLE_COUNT 5
#define PAS_MAX_RPM 200.0f
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Konfiguracja PAS — wypełnij przed pas_init()
// ============================================================================

typedef struct {
    uint8_t  magnets;              ///< Liczba magnesów na tarczy (domyślnie 12)
    float    dir_ratio_thresh;     ///< Próg R=H/L do kierunku (1.0 = H>L = FWD)
    uint8_t  cadence_data_count;   ///< Ile próbek do wygładzenia kadencji (EMA)
    uint8_t  cadence_tau_s;          ///< Stała czasowa EMA kadencji [s] (domyślnie 2.0)
    bool     direction_invert;     ///< Odwróć FWD/REV
} pas_config_t;


// ============================================================================
// Surowe dane z przerwania (ISR)
// ============================================================================
typedef struct {
    uint64_t pulse_timestamp_us;  ///< Timestamp impulsu [µs]
    uint32_t high_us;              ///< Ostatni czas HIGH [µs]
    uint32_t low_us;               ///< Ostatni czas LOW [µs]
    uint32_t period;                ///< Surowe RPM impulsów magnetycznych (obliczane w ISR)
} pas_raw_data_t;

// ============================================================================
// Wynik pas_get_data()
// ============================================================================

typedef struct {
    float    pedal_rpm;            ///< Wygładzona kadencja [RPM] (0 = brak)
    bool     direction;           ///< true = FWD, false = REV
} pas_data_t;

// ============================================================================
// API
// ============================================================================

/**
 * @brief Inicjalizacja PAS.
 * Konfiguruje pin, ustawia parametry i podłącza przerwanie CHANGE.
 * @param config  Wskaźnik do struktury z parametrami (kopiowana wewnętrznie).
 */
void pas_init(const pas_config_t* config);

/**
 * @brief Wywoływane w loop(). Zwraca true gdy jest nowy okres.
 * Wypełnia `out` aktualnymi danymi (kadencja, kierunek, czasy).
 * @param out  Wskaźnik do struktury wynikowej.
 * @return true gdy są nowe dane (out jest świeży).
 */
void pas_get_data(pas_data_t* out);

#endif // PAS_H
