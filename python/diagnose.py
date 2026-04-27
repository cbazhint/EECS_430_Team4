#!/usr/bin/env python3
"""
diagnose.py — CSI/MUSIC Pipeline Health Diagnostic
EECS 430 Team 4

Collects raw phase snapshots from the Nucleo and renders a 6-panel figure that
immediately reveals whether the signal is coherent, whether calibration ran, and
whether the RF switch has crosstalk between the calibration and antenna paths.

Usage
-----
  python diagnose.py                        # auto-detect port, 200 snapshots
  python diagnose.py --port /dev/ttyACM0   # explicit port
  python diagnose.py --n 500               # collect more snapshots
  python diagnose.py --save diag.png       # save PNG instead of live display
  python diagnose.py --no-run              # Nucleo already streaming, skip 'run'
  python diagnose.py --simulate            # synthetic coherent data (no hardware)

Crosstalk test
--------------
Run once with switch on antenna path (normal), once after sending 'hi' (cal path).
Compare panel 1 (phase time series) and panel 4 (covariance heatmap).
  Identical → RF switch not isolating paths (crosstalk confirmed)
  Different  → switch works fine

Panel guide
-----------
  1  Phase time series    : stable = coherent signal; random = noise/frame-mix
  2  Phase diff vs node 1 : constant = fixed HW offsets; random = incoherent
  3  Phase histogram      : Gaussian = good 802.11 signal; flat = no signal
  4  Covariance |R| map   : off-diag ≈ 1 = rank-1, MUSIC can work
  5  Eigenvalue spectrum  : one dominant eigenvalue = coherent source
  6  Packet counts        : dead nodes, cal_mask distribution
"""

import sys
import time
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from collections import Counter

NODE_COLORS = [
    '#e74c3c', '#e67e22', '#f1c40f', '#2ecc71',
    '#1abc9c', '#3498db', '#9b59b6', '#ec407a',
]
Nr = 8


# ── Data collection ───────────────────────────────────────────────────────────

def collect_serial(port, n_snapshots, send_run):
    from serialRead import NucleoSerial, auto_detect_port

    if port is None:
        port = auto_detect_port()
        if port:
            print(f"[diag] Auto-detected port: {port}")
        else:
            print("[diag] ERROR: No serial port found. Use --port or --simulate.")
            sys.exit(1)

    print(f"[diag] Connecting to {port}...")
    ns = NucleoSerial(port=port)
    ns.start()
    time.sleep(1.0)

    print("[diag] Type Nucleo commands and press Enter before collection starts:")
    print("[diag]   cal    — run calibration (wait for it to finish before proceeding)")
    print("[diag]   run    — start streaming")
    print("[diag]   status — print correction vector")
    print("[diag] Press Enter with no command when ready to collect.\n")

    valid = {'run', 'stop', 'cal', 'status', 'verify', 'ant', 'nrf', 'hi', 'lo'}
    while True:
        try:
            line = input("[diag cmd]> ").strip()
        except EOFError:
            break
        if line == '':
            break
        if line not in valid:
            print(f"[diag] Unknown: '{line}'. Valid: {', '.join(sorted(valid))}")
            continue
        ns.send_command(line)
        print(f"[diag] → '{line}' sent")
        if line == 'cal':
            print("[diag]   Waiting 30 s for calibration to complete...")
            time.sleep(30)
            print("[diag]   Done waiting. Press Enter to collect, or send another command.")

    if send_run:
        ns.send_command('run')
        print("[diag] Sent 'run'")
        time.sleep(0.5)

    phases_buf = []
    masks_buf  = []
    print(f"[diag] Collecting {n_snapshots} snapshots...")
    last_print = time.time()

    while len(phases_buf) < n_snapshots:
        snap = ns.get_snapshot(timeout=0.5)
        if snap is None:
            if time.time() - last_print > 3.0:
                print(f"[diag]   {len(phases_buf)}/{n_snapshots} — waiting for packets...")
                last_print = time.time()
            continue
        phases_buf.append(snap['phases'])
        masks_buf.append(snap.get('cal_mask', 0xFF))
        if time.time() - last_print > 2.0:
            print(f"[diag]   {len(phases_buf)}/{n_snapshots}")
            last_print = time.time()

    ns.send_command('stop')
    ns.stop()
    print(f"[diag] Collection complete.")
    return np.array(phases_buf, dtype=np.float32), np.array(masks_buf, dtype=np.uint8)


def collect_simulate(n_snapshots):
    """Synthetic coherent source at (u=0.3, v=0.2) with phase noise."""
    print("[diag] Simulate mode — generating coherent synthetic data")
    rng = np.random.default_rng(42)
    d = 0.5
    Nx, Ny = 2, 4
    u, v = 0.3, 0.2
    ax = np.exp(2j * np.pi * d * np.arange(Nx) * u)
    ay = np.exp(2j * np.pi * d * np.arange(Ny) * v)
    a_true = np.kron(ax, ay)
    true_phases = np.angle(a_true)

    noise_std = 0.15
    phases = (true_phases[np.newaxis, :] +
              noise_std * rng.standard_normal((n_snapshots, Nr))).astype(np.float32)
    # Simulate random carrier phase per snapshot (common to all nodes — should cancel)
    carrier = rng.uniform(-np.pi, np.pi, n_snapshots)
    phases += carrier[:, np.newaxis]
    # Wrap to [-π, π]
    phases = np.arctan2(np.sin(phases), np.cos(phases))

    masks = np.full(n_snapshots, 0x3F, dtype=np.uint8)  # nodes 0-5 alive (simulate 0,7 dead)
    return phases, masks


# ── Analysis ──────────────────────────────────────────────────────────────────

def build_covariance(phases, masks):
    """
    Build snapshot matrix X (Nr, N) and covariance R = X X^H / N.
    Dead/uncalibrated nodes (mask bit = 0) → 0+0j column entry.
    Returns X, R.
    """
    N = len(phases)
    X = np.zeros((Nr, N), dtype=np.complex64)
    for k in range(N):
        for i in range(Nr):
            if (masks[k] >> i) & 1:
                X[i, k] = np.exp(1j * float(phases[k, i]))
    R = (X @ X.conj().T) / N
    return X, R


# ── Rendering ─────────────────────────────────────────────────────────────────

def render(phases, masks, save_path=None):
    N = len(phases)
    X, R = build_covariance(phases, masks)

    # Eigenvalues (ascending) — use eigh for Hermitian guarantee
    eigvals = np.linalg.eigvalsh(R.astype(np.complex128))
    eigvals = np.sort(np.abs(eigvals))[::-1]   # descending

    # Normalised covariance magnitude for heatmap
    diag_sqrt = np.sqrt(np.abs(np.diag(R)))
    with np.errstate(divide='ignore', invalid='ignore'):
        denom = np.outer(diag_sqrt, diag_sqrt)
        R_norm = np.where(denom > 0, np.abs(R) / denom, 0.0)

    coherence_snr = float(eigvals[0] / eigvals[1]) if eigvals[1] > 1e-12 else float('inf')

    # Per-node alive mask (node considered dead if cal_mask bit is always 0)
    alive = np.array([(masks >> i).astype(bool).any() for i in range(Nr)])

    # Phase differences relative to first alive node
    ref_node = int(np.argmax(alive)) if alive.any() else 0
    phase_diff = np.zeros_like(phases)
    for i in range(Nr):
        raw = phases[:, i] - phases[:, ref_node]
        phase_diff[:, i] = np.arctan2(np.sin(raw), np.cos(raw))

    # Cal mask distribution
    mask_counts = Counter(int(m) for m in masks)
    all_zero_pct = 100 * mask_counts.get(0, 0) / N

    # Packet count per node (how many snapshots had this node live)
    pkt_counts = np.array([(masks >> i).astype(bool).sum() for i in range(Nr)])

    # ── Figure ────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(20, 12))
    fig.patch.set_facecolor('#0d1117')
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.35)

    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[1, 0])
    ax3 = fig.add_subplot(gs[0, 1])
    ax4 = fig.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[0, 2])
    ax6 = fig.add_subplot(gs[1, 2])

    _style_ax(ax1, 'Phase time series (raw, each node)', 'snapshot #', 'phase (rad)')
    _style_ax(ax2, f'Phase diff vs node {ref_node} (HW offset visibility)', 'snapshot #', 'Δphase (rad)')
    _style_ax(ax3, 'Phase histogram per node', 'phase (rad)', 'count')
    _style_ax(ax5, 'Eigenvalue spectrum  (log scale)', 'eigenvalue rank', 'magnitude')

    # ── Panel 1: phase time series ────────────────────────────────────────────
    idx = np.arange(N)
    for i in range(Nr):
        style = '-' if alive[i] else ':'
        lw = 1.0 if alive[i] else 0.5
        ax1.plot(idx, phases[:, i], style, color=NODE_COLORS[i],
                 linewidth=lw, alpha=0.8, label=f'N{i}')
    ax1.set_ylim(-np.pi - 0.2, np.pi + 0.2)
    ax1.axhline( np.pi, color='#444', linewidth=0.5, linestyle='--')
    ax1.axhline(-np.pi, color='#444', linewidth=0.5, linestyle='--')
    ax1.legend(ncol=4, fontsize=7, loc='upper right',
               facecolor='#1a1a2e', labelcolor='white', framealpha=0.7)

    # ── Panel 2: phase difference ─────────────────────────────────────────────
    for i in range(Nr):
        if i == ref_node or not alive[i]:
            continue
        std = float(np.std(phase_diff[:, i]))
        ax2.plot(idx, phase_diff[:, i], '-', color=NODE_COLORS[i],
                 linewidth=0.8, alpha=0.8, label=f'N{i}  σ={std:.2f}r')
    ax2.set_ylim(-np.pi - 0.2, np.pi + 0.2)
    ax2.legend(ncol=3, fontsize=7, loc='upper right',
               facecolor='#1a1a2e', labelcolor='white', framealpha=0.7)

    # ── Panel 3: phase histograms ─────────────────────────────────────────────
    bins = np.linspace(-np.pi, np.pi, 17)
    for i in range(Nr):
        if not alive[i]:
            continue
        ax3.hist(phases[:, i], bins=bins, alpha=0.5,
                 color=NODE_COLORS[i], label=f'N{i}', density=True)
    flat_level = 1.0 / (2 * np.pi)
    ax3.axhline(flat_level, color='white', linewidth=0.8, linestyle='--',
                label='uniform (no signal)')
    ax3.legend(ncol=4, fontsize=7, loc='upper right',
               facecolor='#1a1a2e', labelcolor='white', framealpha=0.7)
    ax3.set_xlim(-np.pi, np.pi)

    # ── Panel 4: covariance magnitude heatmap ────────────────────────────────
    ax4.set_facecolor('#16213e')
    im = ax4.imshow(R_norm, vmin=0, vmax=1, cmap='plasma', aspect='equal',
                    interpolation='nearest')
    cb = plt.colorbar(im, ax=ax4, fraction=0.046, pad=0.04)
    cb.set_label('|R[i,j]| / √(R[i,i]·R[j,j])', color='white', fontsize=8)
    cb.ax.yaxis.set_tick_params(color='white')
    plt.setp(cb.ax.yaxis.get_ticklabels(), color='white')
    ax4.set_title('Covariance |R| (normalised)\noff-diag ≈ 1 = coherent, ≈ 0 = noise',
                  color='white', fontsize=9, pad=6)
    ax4.set_xlabel('node j', color='white', fontsize=8)
    ax4.set_ylabel('node i', color='white', fontsize=8)
    ax4.tick_params(colors='white')
    ax4.set_xticks(range(Nr))
    ax4.set_yticks(range(Nr))
    for i in range(Nr):
        for j in range(Nr):
            ax4.text(j, i, f'{R_norm[i, j]:.2f}', ha='center', va='center',
                     color='black' if R_norm[i, j] > 0.5 else 'white', fontsize=6)

    # ── Panel 5: eigenvalue spectrum ─────────────────────────────────────────
    ax5.bar(range(1, Nr + 1), eigvals + 1e-12,
            color=[NODE_COLORS[i] for i in range(Nr)], alpha=0.85, edgecolor='black')
    ax5.set_yscale('log')
    ax5.set_xticks(range(1, Nr + 1))
    ax5.set_xticklabels([f'λ{i+1}' for i in range(Nr)], color='white', fontsize=8)
    ratio_str = f'{coherence_snr:.1f}×' if np.isfinite(coherence_snr) else '∞'
    ax5.set_title(f'Eigenvalue spectrum\nλ₁/λ₂ = {ratio_str}  (>10× = coherent source)',
                  color='white', fontsize=9, pad=6)

    # ── Panel 6: packet counts + cal_mask summary ─────────────────────────────
    ax6.set_facecolor('#16213e')
    bars = ax6.bar(range(Nr), pkt_counts,
                   color=[NODE_COLORS[i] for i in range(Nr)], alpha=0.85,
                   edgecolor='black')
    ax6.axhline(N, color='white', linewidth=0.8, linestyle='--', label=f'total={N}')
    ax6.set_xticks(range(Nr))
    ax6.set_xticklabels([f'N{i}' for i in range(Nr)], color='white', fontsize=8)
    ax6.set_title('Packets per node + cal_mask distribution',
                  color='white', fontsize=9, pad=6)
    ax6.set_xlabel('node', color='white', fontsize=8)
    ax6.set_ylabel('snapshots with live data', color='white', fontsize=8)
    ax6.tick_params(colors='white')
    ax6.legend(fontsize=7, facecolor='#1a1a2e', labelcolor='white', framealpha=0.7)

    # Cal mask text summary inside panel 6
    top5 = mask_counts.most_common(5)
    lines = ['cal_mask distribution:']
    for m, cnt in top5:
        lines.append(f'  0x{m:02X}  ({cnt}/{N}  {100*cnt/N:.0f}%)')
    if all_zero_pct > 5:
        lines.append(f'\n  ⚠  {all_zero_pct:.0f}% zero-mask → cal not run!')
    summary = '\n'.join(lines)
    ax6.text(0.97, 0.97, summary, transform=ax6.transAxes,
             color='#aaccff', fontsize=7.5, va='top', ha='right',
             fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='#0d1117', alpha=0.7))

    # ── Global title ─────────────────────────────────────────────────────────
    alive_list = [str(i) for i in range(Nr) if alive[i]]
    dead_list  = [str(i) for i in range(Nr) if not alive[i]]
    title = (f'DoA Pipeline Diagnostics  —  N={N} snapshots  |  '
             f'alive: [{", ".join(alive_list)}]  '
             f'dead: [{", ".join(dead_list)}]  |  '
             f'coherence λ₁/λ₂ = {ratio_str}')
    fig.suptitle(title, color='white', fontsize=11, y=0.98)

    # ── Text summary to stdout ────────────────────────────────────────────────
    print('\n' + '='*60)
    print('DIAGNOSTIC SUMMARY')
    print('='*60)
    print(f'Snapshots collected : {N}')
    print(f'Alive nodes         : {alive_list}')
    print(f'Dead nodes          : {dead_list}')
    print(f'Coherence λ₁/λ₂     : {ratio_str}')
    if coherence_snr < 3:
        print('  → INCOHERENT — MUSIC will not work (no dominant eigenvalue)')
    elif coherence_snr < 10:
        print('  → MARGINAL coherence — MUSIC may produce noisy results')
    else:
        print('  → COHERENT — MUSIC should work')
    print(f'cal_mask = 0x00 rate: {all_zero_pct:.0f}%')
    if all_zero_pct > 20:
        print('  → WARNING: most packets have cal_mask=0 — calibration not run?')
    for i in range(Nr):
        if alive[i]:
            diff_std = float(np.std(phase_diff[:, i])) if i != ref_node else 0.0
            print(f'  Node {i}: {pkt_counts[i]:4d} pkts  '
                  f'phase_diff_σ={diff_std:.3f} rad'
                  + ('  (reference)' if i == ref_node else
                     '  ← STABLE' if diff_std < 0.3 else ''))
        else:
            print(f'  Node {i}:    0 pkts  (dead)')
    print('='*60 + '\n')

    if save_path:
        fig.savefig(save_path, dpi=120, facecolor=fig.get_facecolor(),
                    bbox_inches='tight')
        print(f'[diag] Saved to {save_path}')
    else:
        plt.tight_layout(rect=[0, 0, 1, 0.97])
        plt.show()

    plt.close(fig)


def _style_ax(ax, title, xlabel, ylabel):
    ax.set_facecolor('#16213e')
    ax.set_title(title, color='white', fontsize=9, pad=6)
    ax.set_xlabel(xlabel, color='white', fontsize=8)
    ax.set_ylabel(ylabel, color='white', fontsize=8)
    ax.tick_params(colors='white')
    for sp in ax.spines.values():
        sp.set_edgecolor('#555577')


# ── Entry point ───────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='CSI/MUSIC pipeline diagnostic for EECS 430 Team 4.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--port',     default=None,
                   help='Serial port (default: auto-detect)')
    p.add_argument('--n',        type=int, default=200,
                   help='Number of snapshots to collect (default: 200)')
    p.add_argument('--save',     default=None, metavar='FILE',
                   help='Save figure to FILE instead of showing interactively')
    p.add_argument('--no-run',   action='store_true',
                   help="Don't send 'run' command (Nucleo already streaming)")
    p.add_argument('--simulate', action='store_true',
                   help='Generate synthetic coherent data (no hardware needed)')
    p.add_argument('--backend',  default=None,
                   help='Matplotlib backend override (e.g. TkAgg, Qt5Agg)')
    return p.parse_args()


def main():
    args = parse_args()

    if args.backend:
        matplotlib.use(args.backend)
    elif args.save:
        matplotlib.use('Agg')
    else:
        for backend in ('TkAgg', 'Qt5Agg', 'wxAgg', 'MacOSX'):
            try:
                matplotlib.use(backend)
                import matplotlib.pyplot as _p
                _p.figure(); _p.close('all')
                break
            except Exception:
                pass

    if args.simulate:
        phases, masks = collect_simulate(args.n)
    else:
        phases, masks = collect_serial(args.port, args.n, not args.no_run)

    render(phases, masks, save_path=args.save)


if __name__ == '__main__':
    main()
