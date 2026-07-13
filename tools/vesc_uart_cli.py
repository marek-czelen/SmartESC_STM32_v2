#!/usr/bin/env python3
"""
SmartESC UART CLI — sterowanie M365/G30 przez VESC protokół binarny
Użycie: python vesc_uart_cli.py COM3
"""

import sys
import struct
import time
import threading
import serial


# ─── CRC16 XMODEM ───────────────────────────────────────────────
def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
    return crc & 0xFFFF


# ─── VESC packet functions ──────────────────────────────────────
def vesc_send(ser, cmd_id: int, payload: bytes = b''):
    """Wyślij ramkę VESC: [2] [len] [cmd_id + payload] [CRC16] [3]"""
    pl = bytes([cmd_id]) + payload
    plen = len(pl)
    crc = crc16(pl)
    packet = b'\x02' + bytes([plen]) + pl + bytes([crc >> 8, crc & 0xFF]) + b'\x03'
    ser.write(packet)
    ser.flush()


def vesc_recv(ser, timeout=0.3) -> bytes:
    """Odbierz i zdekoduj ramkę VESC, zwróć payload (bez cmd_id)"""
    start = time.time()
    buf = b''
    while time.time() - start < timeout:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
        # Szukaj znacznika startu
        if len(buf) >= 5:
            # Znajdź 0x02
            idx = buf.find(b'\x02')
            if idx >= 0 and len(buf) >= idx + 5:
                plen = buf[idx + 1]
                end = idx + 2 + plen + 2 + 1  # start + len + payload + crc2 + stop
                if len(buf) >= end and buf[end - 1] == 0x03:
                    payload = buf[idx + 2 : idx + 2 + plen]
                    buf = buf[end:]  # usuń przetworzone
                    if len(payload) > 1:
                        return payload  # cmd_id + data
        time.sleep(0.01)
    return b''


# ─── Telemetria: parsowanie COMM_GET_VALUES (0x04) ──────────────
def _buf_get_int16(data: bytes, idx: int) -> tuple:
    return struct.unpack('>h', data[idx:idx+2])[0], idx + 2

def _buf_get_int32(data: bytes, idx: int) -> tuple:
    return struct.unpack('>i', data[idx:idx+4])[0], idx + 4

def parse_get_values(payload: bytes) -> dict:
    """Dekoduje odpowiedź COMM_GET_VALUES z pełną maską (0xFFFFFFFF).
       Payload musi zaczynać się OD cmd_id (0x04)."""
    idx = 1  # pomiń cmd_id
    r = {}

    r['temp_mos1'],   idx = _buf_get_int16(payload, idx); r['temp_mos1'] /= 10.0
    r['temp_mos2'],   idx = _buf_get_int16(payload, idx); r['temp_mos2'] /= 10.0
    r['current_motor'], idx = _buf_get_int32(payload, idx); r['current_motor'] /= 100.0
    r['current_in'],  idx = _buf_get_int32(payload, idx); r['current_in'] /= 100.0
    r['id'],          idx = _buf_get_int32(payload, idx); r['id'] /= 100.0
    r['iq'],          idx = _buf_get_int32(payload, idx); r['iq'] /= 100.0
    r['duty'],        idx = _buf_get_int16(payload, idx); r['duty'] /= 1000.0
    r['erpm'],        idx = _buf_get_int32(payload, idx)
    r['v_in'],        idx = _buf_get_int16(payload, idx); r['v_in'] /= 10.0
    r['ah'],          idx = _buf_get_int32(payload, idx); r['ah'] /= 10000.0
    r['ah_charged'],  idx = _buf_get_int32(payload, idx); r['ah_charged'] /= 10000.0
    r['wh'],          idx = _buf_get_int32(payload, idx); r['wh'] /= 10000.0
    r['wh_charged'],  idx = _buf_get_int32(payload, idx); r['wh_charged'] /= 10000.0
    r['tacho'],       idx = _buf_get_int32(payload, idx)
    r['tacho_abs'],   idx = _buf_get_int32(payload, idx)
    r['fault'],       idx = payload[idx], idx + 1
    r['pid_pos'],     idx = _buf_get_int32(payload, idx); r['pid_pos'] /= 1000000.0
    r['ctrl_id']      = payload[idx]; idx += 1
    # 3x temp MOS (nieużywane = 0)
    _, idx = _buf_get_int16(payload, idx)
    _, idx = _buf_get_int16(payload, idx)
    _, idx = _buf_get_int16(payload, idx)
    r['vd'],          idx = _buf_get_int32(payload, idx); r['vd'] /= 1000.0
    r['vq'],          idx = _buf_get_int32(payload, idx); r['vq'] /= 1000.0
    r['status']       = payload[idx] if idx < len(payload) else 0

    # Obliczenia pochodne
    rpm = r['erpm'] / 15.0 if r.get('erpm') else 0  # 15 par biegunów M365
    r['rpm'] = rpm
    r['power'] = r['v_in'] * r['current_in']  # waty
    return r


def get_values(ser):
    """Pobierz i zdekoduj dane telemetryczne z kontrolera"""
    vesc_send(ser, 0x04)
    time.sleep(0.08)
    resp = vesc_recv(ser)
    if resp and resp[0] == 0x04:
        return parse_get_values(resp)
    return None


def monitor_loop(ser, stop_event: threading.Event):
    """Wyświetla na żywo dane telemetryczne (osobny wątek)"""
    print("\n  ╔══════════════════════════════════════════════════════════╗")
    print("  ║               TELEMETRIA — odświeżanie 5Hz               ║")
    print("  ╠══════╤═══════╤══════╤══════╤═══════╤══════╤══════╤══════╣")
    print("  ║  V   │  Iin  │ Imot │ duty │  ERPM │  RPM │ temp │ P(W) ║")
    print("  ╠══════╪═══════╪══════╪══════╪═══════╪══════╪══════╪══════╣")
    last_line = [""]

    while not stop_event.is_set():
        d = get_values(ser)
        if d:
            line = (f"  ║ {d['v_in']:4.1f}V │ {d['current_in']:5.1f}A │ "
                    f"{d['current_motor']:4.1f}A │ {d['duty']:4.0%} │ "
                    f"{d['erpm']:5d} │ {d['rpm']:4.0f} │ {d['temp_mos1']:3.0f}°C │ "
                    f"{d['power']:4.0f}W ║")
            if line != last_line[0]:
                print(line)
                last_line[0] = line
        time.sleep(0.2)

    print("  ╚══════╧═══════╧══════╧══════╧═══════╧══════╧══════╧══════╝")


# ─── Terminal text commands (przez COMM_TERMINAL_CMD = 0x14) ────
def term_cmd(ser, text: str):
    """Wyślij komendę tekstową do CLI"""
    vesc_send(ser, 0x14, text.encode())
    time.sleep(0.15)
    resp = vesc_recv(ser)
    if resp:
        # Payload: [cmd_id=0x14] [tekst...]
        print(resp[1:].decode(errors='replace').strip())


# ─── Motor control commands ────────────────────────────────────
def set_current(ser, amps: float):
    """Ustaw prąd (moment) w amperach, dodatni=przód"""
    val = int(amps * 1000)  # mA
    payload = struct.pack('>i', val)
    vesc_send(ser, 0x06, payload)

def set_brake(ser, amps: float):
    """Hamulec, prąd w amperach (dodatni)"""
    val = int(amps * 1000)
    payload = struct.pack('>i', val)
    vesc_send(ser, 0x07, payload)

def set_rpm(ser, erpm: int):
    """Ustaw prędkość w ERPM"""
    payload = struct.pack('>i', erpm)
    vesc_send(ser, 0x08, payload)

def set_duty(ser, duty: float):
    """Ustaw wypełnienie PWM (0.0 - 1.0)"""
    val = int(duty * 100000)
    payload = struct.pack('>i', val)
    vesc_send(ser, 0x05, payload)

def set_neutral(ser):
    """Zerowy prąd = silnik wybiega"""
    vesc_send(ser, 0x06, b'\x00\x00\x00\x00')


# ─── Keep-alive: ciągłe wysyłanie komend (timeout ~1s!) ─────────
class VescKeepalive:
    """Wysyła ostatnią komendę cyklicznie co 40ms, omija timeout"""
    def __init__(self, ser):
        self.ser = ser
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        # typ komendy: 'current', 'brake', 'rpm', 'duty', None=stop
        self._cmd_type = None
        self._value = 0.0

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        with self._lock:
            self._cmd_type = None
            self._value = 0.0
        self._running = False
        set_neutral(self.ser)
        if self._thread:
            self._thread.join(timeout=1)

    def set_current(self, amps: float):
        with self._lock:
            self._cmd_type = 'current'
            self._value = amps
        self.start()

    def set_brake(self, amps: float):
        with self._lock:
            self._cmd_type = 'brake'
            self._value = amps
        self.start()

    def set_rpm(self, erpm: int):
        with self._lock:
            self._cmd_type = 'rpm'
            self._value = float(erpm)
        self.start()

    def set_duty(self, duty: float):
        with self._lock:
            self._cmd_type = 'duty'
            self._value = duty
        self.start()

    def _loop(self):
        while self._running:
            with self._lock:
                ct = self._cmd_type
                val = self._value
            if ct == 'current':
                set_current(self.ser, val)
            elif ct == 'brake':
                set_brake(self.ser, val)
            elif ct == 'rpm':
                set_rpm(self.ser, int(val))
            elif ct == 'duty':
                set_duty(self.ser, val)
            else:
                pass  # stop — nic nie wysyłamy
            time.sleep(0.04)  # 40ms = 25Hz


# ─── Interactive CLI ────────────────────────────────────────────
def interactive(port: str):
    ser = serial.Serial(port, 115200, timeout=0.05)
    print(f"Połączono z {port} @ 115200")
    print("┌───────────────────────────────────────────────┐")
    print("│ Komendy (trzymają silnik — auto-keepalive):    │")
    print("│  c <A>     — prąd (np. c 2.5)                 │")
    print("│  b <A>     — hamulec (np. b 1.0)              │")
    print("│  r <ERPM>  — prędkość (np. r 5000)            │")
    print("│  d <0-1>   — duty PWM (np. d 0.15)            │")
    print("│  s         — STOP silnika                     │")
    print("│  ping      — test komunikacji                 │")
    print("│  top       — info o taskach/pamięci           │")
    print("│  openloop  — test FOC openloop                │")
    print("│  monitor   — telemetria NA ŻYWO (Esc=koniec)  │")
    print("│  data      — jednorazowy odczyt telemetrii    │")
    print("│  keyboard  — sterowanie klawiszami W/S/spacja  │")
    print("│  q         — wyjście                          │")
    print("└───────────────────────────────────────────────┘")
    print("UWAGA: c/d/r/b wysyłane są cyklicznie (co 40ms).")
    print("       Silnik NIE zatrzyma się sam — wpisz 's'!")
    print("       'monitor' działa RÓWNOLEGLE z jazdą!")

    ka = VescKeepalive(ser)

    while True:
        try:
            cmd = input("\n> ").strip()
            if not cmd:
                continue

            parts = cmd.split()
            head = parts[0].lower()

            if head == 'q':
                ka.stop()
                break
            elif head == 's':
                ka.stop()
                print("  → STOP — silnik zatrzymany")
            elif head == 'c' and len(parts) >= 2:
                amps = float(parts[1])
                ka.set_current(amps)
                print(f"  → prąd: {amps:.1f}A (wysyłane cyklicznie, 's'=stop)")
            elif head == 'b' and len(parts) >= 2:
                amps = float(parts[1])
                ka.set_brake(amps)
                print(f"  → hamulec: {amps:.1f}A (wysyłane cyklicznie, 's'=stop)")
            elif head == 'r' and len(parts) >= 2:
                erpm = int(parts[1])
                ka.set_rpm(erpm)
                print(f"  → prędkość: {erpm} ERPM (wysyłane cyklicznie, 's'=stop)")
            elif head == 'd' and len(parts) >= 2:
                duty = float(parts[1])
                ka.set_duty(duty)
                print(f"  → duty: {duty*100:.0f}% (wysyłane cyklicznie, 's'=stop)")
            elif head == 'ping':
                ka.stop()
                time.sleep(0.1)
                term_cmd(ser, "ping")
            elif head == 'top':
                ka.stop()
                time.sleep(0.1)
                term_cmd(ser, "top")
            elif head == 'openloop':
                ka.stop()
                time.sleep(0.1)
                curr = parts[1] if len(parts) >= 2 else "500"
                erpm = parts[2] if len(parts) >= 3 else "3000"
                term_cmd(ser, f"foc_openloop {curr} {erpm}")
                print(f"  → foc_openloop {curr}mA / {erpm} ERPM")
            elif head == 'mode_speed' and len(parts) >= 3:
                term_cmd(ser, f"mode_speed {parts[1]} {parts[2]}")
            elif head == 'limits' and len(parts) >= 2:
                term_cmd(ser, f"limits {parts[1]}")
            elif head == 'data':
                # Jednorazowy odczyt telemetrii
                d = get_values(ser)
                if d:
                    print(f"  Napięcie:   {d['v_in']:.1f} V")
                    print(f"  Prąd we:    {d['current_in']:.2f} A")
                    print(f"  Prąd siln:  {d['current_motor']:.2f} A")
                    print(f"  Iq/Id:      {d['iq']:.2f} / {d['id']:.2f} A")
                    print(f"  Vd/Vq:      {d['vd']:.2f} / {d['vq']:.2f} V")
                    print(f"  ERPM:       {d['erpm']}")
                    print(f"  RPM:        {d['rpm']:.0f}")
                    print(f"  Duty:       {d['duty']*100:.1f} %")
                    print(f"  Temp MOS:   {d['temp_mos1']:.1f} °C")
                    print(f"  Moc:        {d['power']:.0f} W")
                    print(f"  Ah/Wh:      {d['ah']:.4f} Ah / {d['wh']:.1f} Wh")
                    print(f"  Fault:      {d['fault']}")
                else:
                    print("  Brak odpowiedzi — kontroler podłączony?")
            elif head == 'monitor':
                print("  Telemetria NA ŻYWO (naciśnij Esc aby zakończyć)...")
                stop_ev = threading.Event()
                mon_thread = threading.Thread(target=monitor_loop, args=(ser, stop_ev), daemon=True)
                mon_thread.start()
                try:
                    import msvcrt
                    while mon_thread.is_alive():
                        if msvcrt.kbhit():
                            k = msvcrt.getch()
                            if k == b'\x1b':  # Esc
                                stop_ev.set()
                                break
                        time.sleep(0.1)
                except ImportError:
                    input("  Naciśnij Enter aby zakończyć monitor...\n")
                    stop_ev.set()
                mon_thread.join(timeout=1)
                print("  Monitor zatrzymany.")
            elif head == 'keyboard':
                ka.stop()
                time.sleep(0.1)
                print("  Sterowanie: W=gaz  S=hamulec  Spacja=neutral  Q=koniec")
                print("  (kliknij w to okno i używaj klawiszy)")
                try:
                    import msvcrt
                    while True:
                        if msvcrt.kbhit():
                            k = msvcrt.getch().lower()
                            if k == b'w':
                                ka.set_current(3.0)
                                print("  GAZ ↑")
                            elif k == b's':
                                ka.set_current(-2.0)
                                print("  HAMULEC ↓")
                            elif k == b' ':
                                ka.stop()
                                print("  NEUTRAL - STOP")
                            elif k == b'q':
                                ka.stop()
                                break
                        time.sleep(0.04)
                except ImportError:
                    print("  msvcrt tylko na Windows.")
                ka.stop()
                print("  Klawiatura wyłączona.")
            else:
                print("  Nieznana komenda. Dostępne: c, b, r, d, s, ping, top, openloop, keyboard, q")

        except KeyboardInterrupt:
            ka.stop()
            print("\nZatrzymano.")
            break
        except Exception as e:
            print(f"  Błąd: {e}")

    ser.close()
    print("Rozłączono.")


# ─── Main ───────────────────────────────────────────────────────
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Użycie: python vesc_uart_cli.py <COMx>")
        print("Przykład: python vesc_uart_cli.py COM3")
        sys.exit(1)

    interactive(sys.argv[1])
