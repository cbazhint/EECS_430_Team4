"""
NucleoSerial — serial reader for the Phase-Coherent DoA system.

The Nucleo-H753ZI sends 41-byte binary snapshots over its ST-Link VCP
(USART3, 115200 baud) whenever streaming is enabled.

Host packet format (41 bytes, little-endian):
    [0]      SYNC_A     0xBB
    [1]      SYNC_B     0xCC
    [2..3]   seq        uint16  (wraps at 65535)
    [4..7]   timestamp  uint32  (Nucleo millis)
    [8..39]  phases[8]  float32 each  (calibration-corrected, radians)
    [40]     checksum   XOR of bytes [0..39]

Each snapshot contains one corrected phase per node, already adjusted by the
calibration vector computed at boot.  Pass phases directly into the MUSIC
snapshot assembler as:
    snapshot_vector = np.exp(1j * phases)

Usage:
    ns = NucleoSerial(port='/dev/ttyACM0')
    ns.start()
    ns.send_command('run')            # tell Nucleo to start streaming

    while True:
        snap = ns.get_snapshot(timeout=0.1)
        if snap:
            phases = np.array(snap['phases'])   # shape (8,), radians
            snapshot_vector = np.exp(1j * phases)
            # feed into MUSIC processor ...

    ns.send_command('stop')
    ns.stop()
"""

import serial
import struct
import threading
import queue
import time

# ── Serial configuration ──────────────────────────────────────────────────────
PORT     = '/dev/ttyACM0'   # Linux/WSL ST-Link VCP. Windows: 'COM3', etc.
BAUDRATE = 115200
TIMEOUT  = 1                # serial read timeout (seconds)

# ── Host packet constants ─────────────────────────────────────────────────────
HOST_SYNC_A   = 0xBB
HOST_SYNC_B   = 0xCC
HOST_PKT_SIZE = 41          # 2 sync + 2 seq + 4 ts + 32 phases + 1 chk
_PHASES_FMT   = '<8f'       # 8 × float32 LE starting at byte 8
_SEQ_FMT      = '<H'        # uint16 LE at byte 2
_TS_FMT       = '<I'        # uint32 LE at byte 4


def _validate_host_checksum(raw: bytes) -> bool:
    """XOR of bytes [0..39] must equal byte [40]."""
    chk = 0
    for b in raw[:HOST_PKT_SIZE - 1]:
        chk ^= b
    return chk == raw[HOST_PKT_SIZE - 1]


def _parse_host_packet(raw: bytes) -> dict | None:
    """
    Parse a validated 41-byte host packet.

    Returns:
        {'seq': int, 'timestamp_ms': int, 'phases': list[float]}
        or None on bad length / bad checksum.
    """
    if len(raw) != HOST_PKT_SIZE:
        return None
    if raw[0] != HOST_SYNC_A or raw[1] != HOST_SYNC_B:
        return None
    if not _validate_host_checksum(raw):
        return None

    seq      = struct.unpack_from(_SEQ_FMT,   raw, 2)[0]
    ts_ms    = struct.unpack_from(_TS_FMT,    raw, 4)[0]
    phases   = list(struct.unpack_from(_PHASES_FMT, raw, 8))

    return {
        'seq':          seq,
        'timestamp_ms': ts_ms,
        'phases':       phases,   # list of 8 floats, radians, already corrected
    }


# ── Serial reader thread ──────────────────────────────────────────────────────

def _serial_reader(ser: serial.Serial,
                   out_queue: queue.Queue,
                   stop_event: threading.Event):
    """
    Background thread.

    Hunts byte-by-byte for the 0xBB 0xCC two-byte sync header, then reads
    the remaining 39 bytes, validates the XOR checksum, and pushes parsed
    snapshot dicts onto out_queue.

    Text debug output from the Nucleo is quietly ignored during the sync hunt.
    """
    print(f"[serial] Reader started on {ser.port} @ {ser.baudrate} baud")
    stats = {'received': 0, 'bad_checksum': 0}

    state = 'HUNT_A'    # state machine: HUNT_A → HUNT_B → (read + parse)

    while not stop_event.is_set():
        try:
            raw_byte = ser.read(1)
            if not raw_byte:
                continue            # timeout — loop again

            b = raw_byte[0]

            if state == 'HUNT_A':
                if b == HOST_SYNC_A:
                    state = 'HUNT_B'

            elif state == 'HUNT_B':
                if b == HOST_SYNC_B:
                    # Found both sync bytes — read remaining 39 bytes
                    rest = ser.read(HOST_PKT_SIZE - 2)
                    if len(rest) == HOST_PKT_SIZE - 2:
                        raw = bytes([HOST_SYNC_A, HOST_SYNC_B]) + rest
                        pkt = _parse_host_packet(raw)
                        if pkt is not None:
                            out_queue.put(pkt)
                            stats['received'] += 1
                        else:
                            stats['bad_checksum'] += 1
                    # Either way, reset and hunt for next packet
                    state = 'HUNT_A'

                elif b == HOST_SYNC_A:
                    pass            # could be 0xBB 0xBB 0xCC — stay in HUNT_B

                else:
                    state = 'HUNT_A'

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
    Opens the ST-Link VCP and starts the background reader thread.

    Snapshots arrive as dicts:
        {
            'seq':          int,          # rolling counter (0–65535)
            'timestamp_ms': int,          # Nucleo millis() at send time
            'phases':       list[float],  # 8 corrected phases, radians
        }

    Example
    -------
        ns = NucleoSerial(port='/dev/ttyACM0')
        ns.start()
        ns.send_command('run')

        while True:
            snap = ns.get_snapshot(timeout=0.1)
            if snap:
                phases = np.array(snap['phases'])
                sv = np.exp(1j * phases)   # steering-vector snapshot

        ns.send_command('stop')
        ns.stop()
    """

    def __init__(self, port: str = PORT, baudrate: int = BAUDRATE):
        self.ser     = serial.Serial(port=port, baudrate=baudrate, timeout=TIMEOUT)
        self.ser.dtr = False   # prevent DTR toggling from resetting the Nucleo on open
        self.ser.rts = False
        self._queue  = queue.Queue()
        self._stop   = threading.Event()
        self._thread = threading.Thread(
            target=_serial_reader,
            args=(self.ser, self._queue, self._stop),
            daemon=True,
        )

    def start(self):
        """Start the background reader thread."""
        self._thread.start()

    def stop(self):
        """Stop the reader thread and close the port."""
        self._stop.set()
        self._thread.join(timeout=2)
        if self.ser.is_open:
            self.ser.close()
        print("[serial] Port closed.")

    def send_command(self, cmd: str):
        """
        Send a text command to the Nucleo (terminated with \\n).

        Useful commands: 'run', 'stop', 'cal', 'status', 'nrf'
        """
        self.ser.write((cmd + '\n').encode())

    def get_snapshot(self, timeout: float = 0.1) -> dict | None:
        """
        Pull one snapshot from the queue.
        Returns None if nothing arrives within `timeout` seconds.
        """
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def get_all_snapshots(self) -> list[dict]:
        """Drain all snapshots currently in the queue without blocking."""
        snaps = []
        while not self._queue.empty():
            try:
                snaps.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return snaps


# ── Standalone test ───────────────────────────────────────────────────────────

if __name__ == '__main__':
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    print(f"Connecting to {port} @ {BAUDRATE} baud...")

    ns = NucleoSerial(port=port)
    ns.start()

    print("Waiting 2 s for Nucleo to finish booting...")
    time.sleep(2)
    print("Sending 'run' command...")
    ns.send_command('run')
    print("Listening for snapshots (Ctrl+C to stop)...\n")

    try:
        prev_seq = None
        while True:
            snap = ns.get_snapshot(timeout=0.5)
            if snap is None:
                continue

            # Detect dropped snapshots
            if prev_seq is not None:
                gap = (snap['seq'] - prev_seq) & 0xFFFF
                if gap > 1:
                    print(f"  [WARN] Dropped {gap - 1} snapshots (seq {prev_seq} → {snap['seq']})")
            prev_seq = snap['seq']

            phases_str = '  '.join(f'{p:+.3f}' for p in snap['phases'])
            print(f"  seq={snap['seq']:5d}  t={snap['timestamp_ms']:8d} ms  "
                  f"phases: [{phases_str}]")

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ns.send_command('stop')
        ns.stop()
