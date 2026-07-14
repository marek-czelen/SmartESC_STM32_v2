#ifndef PAS_H
#define PAS_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Konfiguracja PAS — wypełnij przed pas_init()
// ============================================================================

typedef struct {
    uint8_t  magnets;              ///< Liczba magnesów na tarczy (domyślnie 12)
    float    dir_ratio_thresh;     ///< Próg R=H/L do kierunku (1.0 = H>L = FWD)
    uint8_t  dir_confirm;          ///< Ile zgodnych impulsów do zmiany kierunku
    uint8_t  cadence_avg_n;        ///< Próbek do średniej ruchomej kadencji
    bool     direction_invert;     ///< Odwróć FWD/REV
} pas_config_t;

// ============================================================================
// Wynik pas_get_data()
// ============================================================================

typedef struct {
    float    pedal_rpm;            ///< Wygładzona kadencja [RPM] (0 = brak)
    float    pulse_rpm;            ///< Surowe RPM impulsów magnetycznych
    bool     forward;              ///< true = FWD, false = REV
    bool     direction_valid;      ///< Czy kierunek został potwierdzony
    uint32_t high_us;              ///< Ostatni czas HIGH [µs]
    uint32_t low_us;               ///< Ostatni czas LOW [µs]
    uint32_t period_us;            ///< Ostatni okres HIGH+LOW [µs]
    uint32_t pulse_count;          ///< Licznik wszystkich odebranych impulsów
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
bool pas_get_data(pas_data_t* out);

#endif // PAS_H
