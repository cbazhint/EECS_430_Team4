#!/usr/bin/env python3
"""
================================================================================
MUSIC.py — Live 2D MUSIC Direction-of-Arrival Processor
EECS 430 Team 4 — Phase-Coherent DoA System
================================================================================

WHAT THIS DOES
--------------
Reads phase snapshots from the Nucleo-H753ZI over USB serial, applies the
2D MUSIC algorithm to estimate the direction of arrival of a WiFi transmitter,
and displays a live heatmap of the spatial spectrum in direction-cosine (u-v)
space.

ARRAY GEOMETRY
--------------
2×4 planar array: Nx=2 azimuth columns, Ny=4 elevation rows, 8 nodes total.
Nodes are numbered in Kronecker / column-major order (index = col*Ny + row):

    ← azimuth (x-axis) →
    col 0    col 1
     [0]      [4]      ↑
     [1]      [5]      | elevation
     [2]      [6]      | (y-axis)
     [3]      [7]      ↓

    d = half-wavelength spacing between adjacent elements in both dimensions.

DIRECTION COSINES
-----------------
The heatmap axes are:
    u = sin(θ)·cos(φ)    (azimuth component)
    v = sin(θ)·sin(φ)    (elevation component)

Points inside the unit circle (u²+v² ≤ 1) are physically reachable.
The peak in the heatmap gives the estimated direction:
    θ = arcsin(√(u²+v²))    elevation angle from broadside
    φ = arctan2(v, u)        azimuth angle

QUICK START
-----------
1. Flash the ESP32 nodes with firmware/ESP/src/main.cpp.
   Set NODE_ID 0..7 matching the physical layout above.

2. Flash the Nucleo with firmware/NUCLEO/src/main.cpp.
   At boot it runs calibration automatically.

3. Run this script:

   # On Linux / WSL2:
   python MUSIC.py --port /dev/ttyACM0

   # On Windows:
   python MUSIC.py --port COM5

   # Without hardware (simulate a source at θ=30°, φ=45°):
   python MUSIC.py --simulate --theta 30 --phi 45

4. The Nucleo starts streaming when this script connects.
   Wait ~1–2 seconds for the buffer to fill before the heatmap appears.

COMMAND-LINE OPTIONS
--------------------
  --port PORT       Serial port (default: /dev/ttyACM0)
  --baud BAUD       Baud rate   (default: 115200)
  --sources K       Number of signal sources for MUSIC (default: 1)
  --snapshots N     Covariance window size (default: 64)
  --update N        Recompute MUSIC every N new snapshots (default: 16)
  --scan N          u-v grid resolution, N×N points (default: 100)
  --vmin DB         Heatmap lower colour limit in dB (default: -30)
  --simulate        Generate synthetic data instead of using serial
  --theta DEG       Simulated source elevation angle (default: 30)
  --phi   DEG       Simulated source azimuth angle   (default: 45)
  --snr   DB        Simulated SNR in dB              (default: 20)
  --save            Save each frame as music_frame_NNNN.png instead of showing

NOTES
-----
- WSL2 + WSLg: the live window should work. If it doesn't, add --save and open
  the PNG files with: explorer.exe music_frame_0001.png
- The Nucleo applies the calibration correction before sending phases.
  If calibration failed (e.g., nRF24 not connected), all corrections are 0
  and the array will not be phase-coherent — results will be unreliable.
- MUSIC requires the number of snapshots > number of array elements (8).
  More snapshots give a better covariance estimate. 64 is a good default.
- Increasing --scan improves angular resolution but linearly increases
  computation time. 100×100 runs in <50 ms on a typical laptop.
================================================================================
"""

import sys
import time
import argparse
import threading
from collections import deque

import numpy as np
import matplotlib
import matplotlib.patches as mpatches
# matplotlib.pyplot is imported lazily inside launch_display() after
# _setup_backend() selects an interactive backend. Importing pyplot at
# module level would lock in the default (Agg) backend before we can change it.

# ── Array parameters ──────────────────────────────────────────────────────────
Nx = 2      # azimuth columns
Ny = 4      # elevation rows
Nr = Nx * Ny  # 8 total elements
d  = 0.5    # half-wavelength element spacing


# ══════════════════════════════════════════════════════════════════════════════
#  Precomputed steering matrix
# ══════════════════════════════════════════════════════════════════════════════

def build_steering_matrix(Nx, Ny, d, n_scan):
    """
    Precompute steering vectors for every (u, v) grid point inside the unit
    circle.  Returns:
        A_steer   (N_valid, Nr)  complex64  — rows are steering vectors
        valid_mask (n_scan, n_scan)  bool   — True where u²+v² ≤ 1
        u_scan, v_scan  (n_scan,)   float  — grid axes

    Kronecker ordering: A_steer[n, i*Ny + j] = exp(2πj·d·u·i) · exp(2πj·d·v·j)
    matches the node layout:  col i, row j  →  node index i*Ny + j
    """
    u_scan = np.linspace(-1, 1, n_scan)
    v_scan = np.linspace(-1, 1, n_scan)
    UU, VV = np.meshgrid(u_scan, v_scan, indexing='ij')  # (n_scan, n_scan)
    valid_mask = (UU ** 2 + VV ** 2) <= 1.0

    valid_u = UU[valid_mask]   # (N_valid,)
    valid_v = VV[valid_mask]

    # Phase progression along each axis for all valid points simultaneously
    # A_x[n, i] = exp(2πj·d·u_n·i),  shape (N_valid, Nx)
    A_x = np.exp(2j * np.pi * d * np.outer(valid_u, np.arange(Nx))).astype(np.complex64)
    # A_y[n, j] = exp(2πj·d·v_n·j),  shape (N_valid, Ny)
    A_y = np.exp(2j * np.pi * d * np.outer(valid_v, np.arange(Ny))).astype(np.complex64)

    # Kronecker product per row: A_steer[n, i*Ny+j] = A_x[n,i] * A_y[n,j]
    # Broadcast: (N_valid, Nx, 1) * (N_valid, 1, Ny) → (N_valid, Nx, Ny)
    A_steer = (A_x[:, :, np.newaxis] * A_y[:, np.newaxis, :]).reshape(-1, Nr)

    return A_steer, valid_mask, valid_u, valid_v, u_scan, v_scan


# ══════════════════════════════════════════════════════════════════════════════
#  MUSIC algorithm
# ══════════════════════════════════════════════════════════════════════════════

def run_music(snapshot_matrix, A_steer, valid_mask, n_scan, num_sources):
    """
    Run the 2D MUSIC algorithm on a batch of snapshots.

    Parameters
    ----------
    snapshot_matrix : (Nr, N) complex ndarray
        Each column is one snapshot vector exp(1j * corrected_phases).
    A_steer : (N_valid, Nr) complex64
        Precomputed steering matrix from build_steering_matrix().
    valid_mask : (n_scan, n_scan) bool
        Mask for valid (u, v) grid points.
    n_scan : int
        Grid resolution.
    num_sources : int
        Number of signal sources (determines noise subspace size).

    Returns
    -------
    spectrum_db : (n_scan, n_scan) float  — MUSIC pseudo-spectrum in dB,
                  normalized to 0 dB peak. NaN outside the unit circle.
    peak_idx : int  — flat index into valid_mask for the peak point.
    """
    N = snapshot_matrix.shape[1]

    # Sample covariance matrix  R = (1/N) X X^H
    R = (snapshot_matrix @ snapshot_matrix.conj().T) / N

    # Eigendecomposition — eigh guarantees real eigenvalues for Hermitian R
    # and returns them in ascending order, so noise subspace = first Nr-K cols
    _, V = np.linalg.eigh(R.astype(np.complex128))
    V_noise = V[:, :Nr - num_sources].astype(np.complex64)  # (Nr, Nr-K)

    # Vectorized MUSIC metric for all valid (u, v) at once:
    #   P_MUSIC(a) = 1 / ||V_noise^H · a||²
    # proj[n] = V_noise^H · a_n  →  shape (N_valid, Nr-K)
    proj = A_steer @ V_noise.conj()                          # (N_valid, Nr-K)
    noise_power = np.sum(np.abs(proj) ** 2, axis=1)          # (N_valid,)
    music_vals  = 1.0 / np.maximum(noise_power, 1e-12)

    # Place results back into the (n_scan, n_scan) grid
    spectrum = np.full((n_scan, n_scan), np.nan, dtype=np.float32)
    spectrum[valid_mask] = music_vals

    # Normalize to dB relative to peak
    peak = np.nanmax(spectrum)
    if peak > 0:
        spectrum_db = 10.0 * np.log10(spectrum / peak)
    else:
        spectrum_db = spectrum

    peak_idx = int(np.argmax(music_vals))
    return spectrum_db, peak_idx


# ══════════════════════════════════════════════════════════════════════════════
#  Background MUSIC worker thread
# ══════════════════════════════════════════════════════════════════════════════

class MusicProcessor:
    """
    Runs MUSIC in a background thread.  Call push_snapshot() from the serial
    reader thread; read spectrum / peak from the main (display) thread.
    """

    def __init__(self, A_steer, valid_mask, valid_u, valid_v,
                 n_scan, num_sources, n_snapshots, update_every):
        self._A_steer      = A_steer
        self._valid_mask   = valid_mask
        self._valid_u      = valid_u
        self._valid_v      = valid_v
        self._n_scan       = n_scan
        self._num_sources  = num_sources
        self._n_snapshots  = n_snapshots
        self._update_every = update_every

        # Circular buffer — holds up to 4× the covariance window
        self._buf  = deque(maxlen=n_snapshots * 4)
        self._lock = threading.Lock()

        # Latest results (written by worker, read by display)
        self.spectrum_db = np.full((n_scan, n_scan), np.nan, dtype=np.float32)
        self.theta_deg   = float('nan')
        self.phi_deg     = float('nan')
        self.peak_u      = float('nan')
        self.peak_v      = float('nan')
        self.updates     = 0       # number of MUSIC updates completed
        self.snapshots   = 0       # total snapshots received
        self._res_lock   = threading.Lock()

        self._stop_event = threading.Event()
        self._thread     = threading.Thread(target=self._worker, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=3)

    def push_snapshot(self, phases: list):
        """
        Accept one 8-element phase list from the serial reader and convert it
        to a complex snapshot vector exp(1j * phases).
        """
        sv = np.exp(1j * np.array(phases, dtype=np.float32))
        with self._lock:
            self._buf.append(sv)
            self.snapshots += 1

    def get_result(self):
        """Return a copy of the latest result dict (thread-safe)."""
        with self._res_lock:
            return {
                'spectrum_db': self.spectrum_db.copy(),
                'theta_deg':   self.theta_deg,
                'phi_deg':     self.phi_deg,
                'peak_u':      self.peak_u,
                'peak_v':      self.peak_v,
                'updates':     self.updates,
                'snapshots':   self.snapshots,
            }

    def _worker(self):
        last_processed = 0
        while not self._stop_event.is_set():
            with self._lock:
                n = len(self._buf)

            # Wait until we have enough snapshots AND enough new ones
            if n < self._n_snapshots or (n - last_processed) < self._update_every:
                time.sleep(0.01)
                continue

            # Copy the most recent window
            with self._lock:
                window = list(self._buf)[-self._n_snapshots:]

            last_processed = n

            # Build snapshot matrix (Nr, N_snapshots)
            X = np.array(window).T   # (Nr, N_snapshots)

            try:
                spectrum_db, peak_idx = run_music(
                    X, self._A_steer, self._valid_mask,
                    self._n_scan, self._num_sources)
            except np.linalg.LinAlgError:
                continue   # degenerate covariance — wait for more data

            pu = float(self._valid_u[peak_idx])
            pv = float(self._valid_v[peak_idx])
            uv2 = pu ** 2 + pv ** 2
            theta = float(np.degrees(np.arcsin(np.sqrt(min(uv2, 1.0)))))
            phi   = float(np.degrees(np.arctan2(pv, pu)))

            with self._res_lock:
                self.spectrum_db = spectrum_db
                self.peak_u      = pu
                self.peak_v      = pv
                self.theta_deg   = theta
                self.phi_deg     = phi
                self.updates    += 1


# ══════════════════════════════════════════════════════════════════════════════
#  Simulation source (no hardware needed)
# ══════════════════════════════════════════════════════════════════════════════

def simulate_snapshots(processor, theta_deg, phi_deg, snr_db, stop_event):
    """
    Feed synthetic snapshots into the processor at ~100 Hz.
    Generates a single plane-wave source at (theta, phi) with additive
    complex Gaussian noise.
    """
    theta = np.radians(theta_deg)
    phi   = np.radians(phi_deg)
    u = np.sin(theta) * np.cos(phi)
    v = np.sin(theta) * np.sin(phi)

    # True steering vector
    ax = np.exp(2j * np.pi * d * np.arange(Nx) * u)
    ay = np.exp(2j * np.pi * d * np.arange(Ny) * v)
    a_true = np.kron(ax, ay)   # (Nr,)

    noise_std = 10 ** (-snr_db / 20.0)
    rng = np.random.default_rng()

    print(f"[sim] Source at θ={theta_deg:.1f}°  φ={phi_deg:.1f}°  "
          f"u={u:.3f}  v={v:.3f}  SNR={snr_db} dB")

    while not stop_event.is_set():
        # Random complex signal amplitude
        s = rng.standard_normal() + 1j * rng.standard_normal()
        noise = noise_std * (rng.standard_normal(Nr) + 1j * rng.standard_normal(Nr))
        received = a_true * s + noise

        # Extract phase of each element (equivalent to what ESP32s report)
        phases = np.angle(received).tolist()
        processor.push_snapshot(phases)
        time.sleep(0.01)   # ~100 Hz


# ══════════════════════════════════════════════════════════════════════════════
#  Backend detection
# ══════════════════════════════════════════════════════════════════════════════

def _setup_backend(args):
    """
    Select an interactive matplotlib backend BEFORE pyplot is imported.

    Must be called once, before any `import matplotlib.pyplot` statement.
    If no interactive backend can be found (e.g. WSL2 without a display),
    automatically enables --save mode and falls back to Agg.

    Returns True if an interactive display is available.
    """
    if args.save:
        matplotlib.use('Agg')
        return False

    # Manual override
    if hasattr(args, 'backend') and args.backend:
        matplotlib.use(args.backend)
        return True

    # Auto-detect: try interactive backends in preference order
    for backend in ('TkAgg', 'Qt5Agg', 'wxAgg', 'MacOSX'):
        try:
            matplotlib.use(backend)
            import matplotlib.pyplot as _plt
            _plt.figure()
            _plt.close('all')
            return True
        except Exception:
            pass

    # No interactive backend found — fall back to headless save mode
    matplotlib.use('Agg')
    print("[display] No interactive display found — switching to --save mode automatically.")
    print("          On WSL2: install WSLg or start an X server (e.g. VcXsrv).")
    print("          Frames will be saved as music_frame_NNNN.png\n")
    args.save = True
    return False


# ══════════════════════════════════════════════════════════════════════════════
#  Live display
# ══════════════════════════════════════════════════════════════════════════════

def build_figure(n_scan, vmin_db):
    """Create the matplotlib figure and return handles needed for animation."""
    import matplotlib.pyplot as plt  # backend must already be set by _setup_backend()
    fig, axes = plt.subplots(1, 2, figsize=(13, 6),
                             gridspec_kw={'width_ratios': [3, 1]})
    fig.patch.set_facecolor('#1a1a2e')

    # ── Left: 2D MUSIC heatmap ────────────────────────────────────────────────
    ax_map = axes[0]
    ax_map.set_facecolor('#16213e')

    placeholder = np.full((n_scan, n_scan), np.nan)
    im = ax_map.imshow(
        placeholder.T,                 # imshow: rows=y, cols=x → transpose
        extent=[-1, 1, -1, 1],
        origin='lower',
        aspect='equal',
        cmap='plasma',
        vmin=vmin_db,
        vmax=0,
        interpolation='bilinear',
    )
    cb = plt.colorbar(im, ax=ax_map, fraction=0.046, pad=0.04)
    cb.set_label('dB  (normalized to peak)', color='white', fontsize=9)
    cb.ax.yaxis.set_tick_params(color='white')
    plt.setp(cb.ax.yaxis.get_ticklabels(), color='white')

    # Unit circle = edge of visible hemisphere
    circle = plt.Circle((0, 0), 1, fill=False, color='#aaaaaa',
                         linestyle='--', linewidth=0.8)
    ax_map.add_patch(circle)

    # Axis grid lines
    for val in [-0.5, 0, 0.5]:
        ax_map.axhline(val, color='#444466', linewidth=0.4, linestyle=':')
        ax_map.axvline(val, color='#444466', linewidth=0.4, linestyle=':')

    # Peak marker (initially off-screen)
    peak_dot, = ax_map.plot([], [], 'r+', markersize=16, markeredgewidth=2.5,
                            zorder=5, label='Peak')

    ax_map.set_xlim(-1.1, 1.1)
    ax_map.set_ylim(-1.1, 1.1)
    ax_map.set_xlabel('u = sin(θ)cos(φ)   [azimuth →]', color='white', fontsize=10)
    ax_map.set_ylabel('v = sin(θ)sin(φ)   [↑ elevation]', color='white', fontsize=10)
    ax_map.tick_params(colors='white')
    for spine in ax_map.spines.values():
        spine.set_edgecolor('#555577')

    title = ax_map.set_title('2D MUSIC — waiting for data...', color='white',
                              fontsize=11, pad=8)

    # ── Right: info panel ─────────────────────────────────────────────────────
    ax_info = axes[1]
    ax_info.set_facecolor('#16213e')
    ax_info.set_xlim(0, 1)
    ax_info.set_ylim(0, 1)
    ax_info.axis('off')

    info_text = ax_info.text(
        0.5, 0.95, '',
        transform=ax_info.transAxes,
        color='white', fontsize=10,
        va='top', ha='center',
        fontfamily='monospace',
        linespacing=1.8,
    )

    # Array diagram
    ax_info.text(0.5, 0.18, 'Node layout (Kronecker order)',
                 color='#aaaaaa', fontsize=7.5, ha='center', va='center')
    layout = (
        "  col 0  col 1\n"
        "   [0]    [4]\n"
        "   [1]    [5]\n"
        "   [2]    [6]\n"
        "   [3]    [7]"
    )
    ax_info.text(0.5, 0.09, layout,
                 color='#88aacc', fontsize=7, ha='center', va='center',
                 fontfamily='monospace')

    plt.tight_layout(pad=1.5)

    return fig, im, peak_dot, title, info_text


def _apply_result(result, im, peak_dot, title_obj, info_text, args):
    """Update all plot artists from a result dict. Returns True if data was ready."""
    spectrum = result['spectrum_db']
    updates  = result['updates']
    snaps    = result['snapshots']

    if updates > 0:
        im.set_data(spectrum.T)   # transpose: imshow rows=y, cols=x
        im.set_clim(vmin=args.vmin, vmax=0)

        pu = result['peak_u']
        pv = result['peak_v']
        th = result['theta_deg']
        ph = result['phi_deg']

        peak_dot.set_data([pu], [pv])
        title_obj.set_text(
            f'2D MUSIC — 2×4 array   θ = {th:.1f}°   φ = {ph:.1f}°'
        )
        info_text.set_text(
            f"ESTIMATE\n"
            f"────────────────\n"
            f"θ (elevation) : {th:+6.1f}°\n"
            f"φ (azimuth)   : {ph:+6.1f}°\n"
            f"u             : {pu:+.3f}\n"
            f"v             : {pv:+.3f}\n"
            f"\n"
            f"STATS\n"
            f"────────────────\n"
            f"Snapshots rx  : {snaps}\n"
            f"MUSIC updates : {updates}\n"
            f"Window size   : {args.snapshots}\n"
            f"Sources (K)   : {args.sources}\n"
            f"Grid          : {args.scan}×{args.scan}\n"
        )
        return True
    else:
        buf_pct = min(100, int(100 * snaps / args.snapshots))
        title_obj.set_text(f'2D MUSIC — filling buffer... {buf_pct}%')
        info_text.set_text(
            f"Waiting for data\n\n"
            f"Snapshots: {snaps}/{args.snapshots}\n"
            f"Need {args.snapshots} to start."
        )
        return False


def launch_display(processor, args, stop_event):
    """Set up matplotlib backend, build the figure, and run the update loop."""
    # Backend must be selected before pyplot is imported anywhere.
    # _setup_backend() may flip args.save=True if no display is found.
    _setup_backend(args)
    import matplotlib.pyplot as plt   # safe to import now

    fig, im, peak_dot, title_obj, info_text = build_figure(args.scan, args.vmin)

    if args.save:
        # ── Headless save mode: manual loop, no GUI needed ────────────────────
        print("[display] Save mode — frames written to music_frame_NNNN.png")
        frame = 0
        try:
            while not stop_event.is_set():
                result = processor.get_result()
                _apply_result(result, im, peak_dot, title_obj, info_text, args)
                fname = f"music_frame_{frame:04d}.png"
                fig.savefig(fname, dpi=100, facecolor=fig.get_facecolor())
                print(f"\r[display] Saved {fname}  "
                      f"(snaps={result['snapshots']}, updates={result['updates']})",
                      end='', flush=True)
                frame += 1
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            plt.close(fig)
            stop_event.set()
    else:
        # ── Interactive live display ───────────────────────────────────────────
        # plt.ion() + plt.pause() is more reliable than FuncAnimation across
        # backends: pause() pumps the GUI event loop and flushes draw_idle().
        plt.ion()
        plt.show(block=False)
        try:
            while not stop_event.is_set():
                result = processor.get_result()
                _apply_result(result, im, peak_dot, title_obj, info_text, args)
                fig.canvas.draw_idle()   # queue redraw
                plt.pause(0.2)           # pump GUI event loop + execute redraw
                if not plt.fignum_exists(fig.number):
                    break                # user closed the window
        except KeyboardInterrupt:
            pass
        finally:
            plt.close(fig)
            stop_event.set()


# ══════════════════════════════════════════════════════════════════════════════
#  Serial reader bridge
# ══════════════════════════════════════════════════════════════════════════════

def serial_bridge(ns, processor, stop_event):
    """
    Pull snapshots from NucleoSerial and push into the MUSIC processor.
    Runs in its own thread.
    """
    print("[bridge] Serial bridge started")
    while not stop_event.is_set():
        snap = ns.get_snapshot(timeout=0.1)
        if snap is None:
            continue
        processor.push_snapshot(snap['phases'])
    print("[bridge] Serial bridge stopped")


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(
        description='Live 2D MUSIC DoA processor for the EECS 430 Team 4 array.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--port',      default='/dev/ttyACM0',
                   help='Serial port (default: /dev/ttyACM0)')
    p.add_argument('--baud',      type=int, default=115200,
                   help='Baud rate (default: 115200)')
    p.add_argument('--sources',   type=int, default=1,
                   help='Number of signal sources K (default: 1)')
    p.add_argument('--snapshots', type=int, default=64,
                   help='Covariance window size N (default: 64)')
    p.add_argument('--update',    type=int, default=16,
                   help='Recompute MUSIC every N new snapshots (default: 16)')
    p.add_argument('--scan',      type=int, default=100,
                   help='u-v grid resolution (default: 100)')
    p.add_argument('--vmin',      type=float, default=-30,
                   help='Heatmap lower colour limit in dB (default: -30)')
    p.add_argument('--simulate',  action='store_true',
                   help='Generate synthetic data (no hardware needed)')
    p.add_argument('--theta',     type=float, default=30,
                   help='Simulated source elevation in degrees (default: 30)')
    p.add_argument('--phi',       type=float, default=45,
                   help='Simulated source azimuth in degrees (default: 45)')
    p.add_argument('--snr',       type=float, default=20,
                   help='Simulated SNR in dB (default: 20)')
    p.add_argument('--save',      action='store_true',
                   help='Save frames as PNGs instead of live display')
    p.add_argument('--backend',   default=None,
                   help='Matplotlib backend override, e.g. TkAgg, Qt5Agg, wxAgg')
    return p.parse_args()


def main():
    args = parse_args()

    # ── Validate sources vs array size ────────────────────────────────────────
    if args.sources >= Nr:
        print(f"[error] --sources ({args.sources}) must be less than "
              f"the number of array elements ({Nr})")
        sys.exit(1)

    if args.snapshots <= Nr:
        print(f"[warn] --snapshots ({args.snapshots}) should be > Nr ({Nr}) "
              f"for a full-rank covariance. Increasing to {Nr + 4}.")
        args.snapshots = Nr + 4

    # ── Precompute steering matrix ────────────────────────────────────────────
    print(f"[init] Building steering matrix ({args.scan}×{args.scan} grid)...")
    A_steer, valid_mask, valid_u, valid_v, u_scan, v_scan = \
        build_steering_matrix(Nx, Ny, d, args.scan)
    n_valid = int(valid_mask.sum())
    print(f"[init] Steering matrix: {n_valid} valid points out of "
          f"{args.scan**2} total  ({100*n_valid/args.scan**2:.0f}%)")

    # ── Create MUSIC processor ────────────────────────────────────────────────
    processor = MusicProcessor(
        A_steer, valid_mask, valid_u, valid_v,
        n_scan=args.scan,
        num_sources=args.sources,
        n_snapshots=args.snapshots,
        update_every=args.update,
    )
    processor.start()

    stop_event = threading.Event()

    # ── Connect data source ───────────────────────────────────────────────────
    if args.simulate:
        print(f"[init] Simulation mode: θ={args.theta}°  φ={args.phi}°  "
              f"SNR={args.snr} dB")
        sim_thread = threading.Thread(
            target=simulate_snapshots,
            args=(processor, args.theta, args.phi, args.snr, stop_event),
            daemon=True,
        )
        sim_thread.start()
        ns = None
    else:
        print(f"[init] Connecting to Nucleo on {args.port} @ {args.baud} baud...")
        try:
            from serialRead import NucleoSerial
            ns = NucleoSerial(port=args.port, baudrate=args.baud)
            ns.start()
            ns.send_command('run')
            print("[init] Sent 'run' command to Nucleo — streaming started")
        except Exception as e:
            print(f"[error] Could not open serial port: {e}")
            print("        Check --port, or use --simulate for testing without hardware.")
            processor.stop()
            sys.exit(1)

        bridge_thread = threading.Thread(
            target=serial_bridge,
            args=(ns, processor, stop_event),
            daemon=True,
        )
        bridge_thread.start()

    # ── Launch display (blocks until window closed or Ctrl+C) ────────────────
    print(f"[init] Starting display  "
          f"(window={args.snapshots} snaps, K={args.sources} sources, "
          f"grid={args.scan}×{args.scan})")
    print("[init] Close the plot window or press Ctrl+C to exit.\n")

    try:
        launch_display(processor, args, stop_event)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[shutdown] Stopping...")
        stop_event.set()
        processor.stop()
        if ns is not None:
            ns.send_command('stop')
            ns.stop()
        print("[shutdown] Done.")


if __name__ == '__main__':
    main()
