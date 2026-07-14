#ifndef THROTTLE_H
#define THROTTLE_H

#include <stdint.h>

/**
 * @brief Odczyt przepustnicy z filtracją (mediana + outlier rejection).
 *
 * Próbkuje THR_SAMPLES razy, sortuje, bierze medianę,
 * odrzuca skrajne odchyły, zwraca średnią z pozostałych.
 *
 * @return Wartość 0.0–1.0 (0 = puszczony gaz, 1 = pełny gaz).
 */
float read_throttle();

#endif // THROTTLE_H
