import serial
import struct
import threading
import queue
import time

# ── Serial configuration ──────────────────────────────────────────────────────
PORT     = 'COM3'    # Windows: 'COM3', 'COM4', etc.  Linux/WSL: '/dev/ttyACM0'
BAUDRATE = 115200
TIMEOUT  = 1

# ── Packet format (11 bytes, little-endian) ───────────────────────────────────
# Byte  0    : sync byte  0xAA
# Byte  1    : node ID    0x00–0x07
# Bytes 2–5  : timestamp  uint32 (microseconds)
# Bytes 6–9  : phase      float32 (radians)
# Byte  10   : checksum   XOR of bytes 0–9
SYNC_BYTE   = 0xAA
PACKET_SIZE = 11
HEADER_FMT  = '<BBIf'   # sync, node_id, timestamp_us, phase_rad  (little-endian)

# Shared queue: serial thread pushes packets, consumer pulls them
packet_queue = queue.Queue()

# ── Packet parser ─────────────────────────────────────────────────────────────

def validate_checksum(raw: bytes) -> bool:
    """XOR of bytes 0–9 must equal byte 10."""
    expected = 0
    for b in raw[:10]:
        expected ^= b
    return expected == raw[10]


def parse_packet(raw: bytes) -> dict | None:
    """
    Parse a validated 11-byte raw packet.
    Returns a dict with node_id, timestamp_us, phase_rad, or None on error.
    """
    if len(raw) != PACKET_SIZE:
        return None
    if raw[0] != SYNC_BYTE:
        return None
    if not validate_checksum(raw):
        return None

    _, node_id, timestamp_us, phase_rad = struct.unpack(HEADER_FMT, raw[:10])
    return {
        'node_id':      node_id,
        'timestamp_us': timestamp_us,
        'phase_rad':    phase_rad,
    }


# ── Serial reader thread ──────────────────────────────────────────────────────

def serial_reader(ser: serial.Serial, out_queue: queue.Queue, stop_event: threading.Event):
    """
    Background thread: continuously reads bytes from the serial port,
    re-syncs on 0xAA, reads a full packet, validates it, and pushes
    parsed dicts onto out_queue.
    """
    print(f"[serial] Reader started on {ser.port} @ {ser.baudrate} baud")
    stats = {'received': 0, 'bad_checksum': 0, 'bad_sync': 0}

    while not stop_event.is_set():
        try:
            # ── Step 1: find sync byte ────────────────────────────────────────
            byte = ser.read(1)
            if not byte:
                continue                     # timeout — loop again
            if byte[0] != SYNC_BYTE:
                stats['bad_sync'] += 1
                continue

            # ── Step 2: read the remaining 10 bytes ──────────────────────────
            rest = ser.read(PACKET_SIZE - 1)
            if len(rest) < PACKET_SIZE - 1:
                continue                     # incomplete read — discard

            raw = byte + rest

            # ── Step 3: validate and parse ────────────────────────────────────
            if not validate_checksum(raw):
                stats['bad_checksum'] += 1
                continue

            packet = parse_packet(raw)
            if packet:
                out_queue.put(packet)
                stats['received'] += 1

        except serial.SerialException as e:
            print(f"[serial] SerialException: {e}")
            break
        except Exception as e:
            print(f"[serial] Unexpected error: {e}")
            break

    print(f"[serial] Reader stopped. Stats: {stats}")


# ── Public API ────────────────────────────────────────────────────────────────

class NucleoSerial:
    """
    Opens the serial port and starts the background reader thread.

    Usage:
        ns = NucleoSerial()
        ns.start()
        pkt = ns.get_packet(timeout=0.1)   # returns dict or None
        ns.stop()
    """

    def __init__(self, port: str = PORT, baudrate: int = BAUDRATE):
        self.ser        = serial.Serial(port=port, baudrate=baudrate, timeout=TIMEOUT)
        self._queue     = packet_queue
        self._stop      = threading.Event()
        self._thread    = threading.Thread(target=serial_reader,
                                           args=(self.ser, self._queue, self._stop),
                                           daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)
        if self.ser.is_open:
            self.ser.close()
        print("[serial] Port closed.")

    def get_packet(self, timeout: float = 0.1) -> dict | None:
        """
        Pull one parsed packet from the queue.
        Returns None if nothing arrives within timeout seconds.
        """
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def get_all_packets(self) -> list[dict]:
        """Drain everything currently in the queue without blocking."""
        packets = []
        while not self._queue.empty():
            try:
                packets.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return packets


# ── Standalone test ───────────────────────────────────────────────────────────

if __name__ == '__main__':
    ns = NucleoSerial()
    ns.start()

    print("Listening for packets (Ctrl+C to stop)...\n")
    try:
        while True:
            pkt = ns.get_packet(timeout=0.5)
            if pkt:
                print(f"  Node {pkt['node_id']:1d} | "
                      f"t={pkt['timestamp_us']:10d} us | "
                      f"phase={pkt['phase_rad']:+.4f} rad")
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ns.stop()
