# SmartESC UART Bridge — ESP32

**Mostek UART między czujnikami roweru elektrycznego a kontrolerem ESC.**

## Przeznaczenie

To oprogramowanie działa na płytce **ESP32** (PCB z projektu `bldc_driver_esp32`, bez wlutowanych driverów MOSFET). Jego zadaniem jest:

1. Odbieranie sygnałów z czujników rowerowych:
   - **PAS** (Pedal Assist Sensor) — kadencja i kierunek pedałowania
   - **Przepustnica** (throttle) — analogowa 0–3.3 V
   - **Hamulec** (brake) — sygnał cyfrowy
2. Odbieranie parametrów z **wyświetlacza S866** (protokół UART nr 2, 9600 baud)
3. Wysyłanie komend sterujących do **ESC SmartESC_STM32_v2** (protokół VESC binarny, 115200 baud)

Aplikacja **NIE steruje bezpośrednio silnikiem** — nie ma PWM na MOSFETy. Zamiast tego tłumaczy intencje rowerzysty na komendy VESC wysyłane przez UART do zewnętrznego kontrolera STM32.

## Pinout

| Funkcja | GPIO | Kierunek | Opis |
|---------|------|----------|------|
| `PIN_DISPLAY_EN` | 5 | OUT | Enable TXB0102 level shifter (5 V → wyświetlacz) |
| `PIN_DISPLAY_TX` | 17 | OUT | TX do wyświetlacza S866 (UART2, 9600 bd) |
| `PIN_DISPLAY_RX` | 16 | IN | RX z wyświetlacza S866 (UART2) |
| `PIN_THROTTLE` | 33 | ADC | Przepustnica 0–3.3 V (ADC1_CH5) |
| `PIN_PAS` | 22 | IN+PULLUP | Czujnik PAS (cyfrowy, impulsy) |
| `PIN_BRAKE` | 23 | IN+PULLUP | Hamulec (aktywny LOW) |
| `PIN_ESC_RX` | 4 | IN | RX z ESC (UART1, 115200 bd) |
| `PIN_ESC_TX` | 18 | OUT | TX do ESC (UART1) |

## Architektura UART

```
ESP32
├── Serial  (UART0) ── USB ── PC (debug/monitor, 115200)
├── Serial1 (UART1) ── GPIO4/18 ── ESC SmartESC_STM32_v2 (VESC protokół, 115200)
└── Serial2 (UART2) ── GPIO16/17 ── wyświetlacz S866 (protokół 2, 9600)
```

## Protokoły

### VESC (do ESC)
- Ramka: `[0x02] [len] [cmd+payload] [CRC16 hi] [CRC16 lo] [0x03]`
- CRC16: XMODEM (wielomian 0x1021)
- Używane komendy:
  - `0x04` COMM_GET_VALUES — pobranie telemetrii (V_in, I_motor, ERPM, duty, temp)
  - `0x05` COMM_SET_DUTY — ustawienie duty cycle (-1.0..1.0, int32 ×100000)
  - `0x06` COMM_SET_CURRENT — ustawienie prądu (A, int32 ×1000)

### S866 Display (z/do wyświetlacza)
- Ramka RX (wyświetlacz → ESP): 20 bajtów, checksum XOR, ~100ms interwał
- Ramka TX (ESP → wyświetlacz): 14 bajtów, checksum XOR
- Kluczowe parametry odbierane:
  - `P06` — rozmiar koła ×10 cali (bajty 7-8)
  - `P08` — limit prędkości (bajt 12)
  - `P10` — tryb sterowania: 0=PAS+throttle, 1=throttle, 2=PAS (bajt 3)
  - `P14` — limit prądu [A] (bajt 13)
  - `assist_level` — poziom wspomagania 0-9 (bajt 4, bity 0-3)

## Logika sterowania

1. **Hamulec** → natychmiast `duty=0`, wysłane do ESC
2. **P10=0** (PAS+throttle): przepustnica ma priorytet; przy braku przepustnicy → PAS
3. **P10=1** (tylko throttle): ADC mapowane liniowo na duty 0..1
4. **P10=2** (tylko PAS): kadencja pedałowania × poziom wspomagania → prędkość docelowa → regulator P → duty

Poziomy wspomagania mapowane są na prędkość docelową:
- Level 0 = off
- Level 1 = 8 km/h
- Level 2 = 15 km/h
- Level 3 = 20 km/h
- Level 4 = 25 km/h
- Level 5 = 30 km/h
- Level 6-9 = 33-45 km/h

Limit z P08 nadpisuje mapę jeśli jest niższy.

## Filtracja wejść

| Wejście | Metoda |
|---------|--------|
| Przepustnica (ADC) | Mediana z 8 próbek + odrzucanie outlierów (>150 ADC od mediany) |
| PAS | Timer sprzętowy 2 kHz + filtr cyfrowy (6 próbek = ~3ms debounce) |
| Hamulec | Debounce 30 ms (stan musi się utrzymać) |

## Pliki źródłowe

| Plik | Zawartość |
|------|-----------|
| `include/pinout.h` | Wszystkie definicje pinów GPIO, ADC, stałe czasowe |
| `include/display_s866.h` | Struktury i API protokołu wyświetlacza S866 |
| `include/vesc_proto.h` | Struktury i API protokołu VESC (ESC) |
| `src/main.cpp` | Główna pętla: setup(), loop(), filtracja, logika sterowania |
| `src/display_s866.cpp` | Implementacja protokołu S866 (odbiór ramek RX, wysyłanie TX) |
| `src/vesc_proto.cpp` | Implementacja VESC (CRC16, ramkowanie, parsowanie telemetrii) |

## Zależności

- **PlatformIO**, framework **Arduino** dla **ESP32** (espressif32)
- Brak zewnętrznych bibliotek — tylko standardowe Arduino + ESP32 SDK

## Budowanie

```bash
cd tools/serial-trottle-brake-esp32
pio run -t upload
pio device monitor
```

## Uwagi projektowe

- PCB z `bldc_driver_esp32` — NIE wlutowuj driverów IR2103 ani MOSFETów; wykorzystane jest tylko ESP32 + wejścia czujników + goldpiny UART.
- GPIO5 (DISPLAY_EN) jest pinem strap ESP32 — musi być HIGH przy boot. Kod ustawia go HIGH dopiero w `setup()`, co jest bezpieczne.
- GPIO4 i GPIO18 (ESC UART) były oryginalnie na czujniki Halla A/B — w tej aplikacji są wolne i użyte do komunikacji z ESC.
- Do programowania przez USB potrzebny jest mostek USB-UART na płytce (standardowo CP2102/CH340).
