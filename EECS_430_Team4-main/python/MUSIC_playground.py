import numpy as np
import matplotlib.pyplot as plt


def steering_vector_2d(phi, theta, Nx, Ny, d=0.5):
    u = np.sin(theta) * np.cos(phi)
    v = np.sin(theta) * np.sin(phi)
    ax = np.exp(2j * np.pi * d * np.arange(Nx) * u)
    ay = np.exp(2j * np.pi * d * np.arange(Ny) * v)
    return np.kron(ax, ay)  # Kronecker product → length Nx*Ny


# ── Array parameters ──────────────────────────────────────────────────────────
Nx = 8   # columns (azimuth,  x-direction, cos(phi))
Ny = 2   # rows    (elevation, y-direction, sin(phi))
Nr = Nx * Ny   # 16 total elements
d  = 0.5
N  = 1000

# ── True signal directions: each signal needs BOTH theta and phi ──────────────
signals = [
    ( 20 * np.pi/180,  -30 * np.pi/180),   # (theta, phi)
    ( 40 * np.pi/180,   60 * np.pi/180),
    ( 35 * np.pi/180,  120 * np.pi/180),
]
num_expected_signals = len(signals)

# ── Build received signal matrix X: (Nr x N) ─────────────────────────────────
X = np.zeros((Nr, N), dtype=complex)
for theta_s, phi_s in signals:
    sig = np.random.randn(N) + 1j * np.random.randn(N)
    a   = steering_vector_2d(phi_s, theta_s, Nx, Ny, d)
    X  += np.outer(a, sig)
noise_power = 0.01
X += noise_power * (np.random.randn(Nr, N) + 1j * np.random.randn(Nr, N))

# ── MUSIC: covariance → eigendecomposition → noise subspace ──────────────────
R             = np.cov(X)
w, v          = np.linalg.eig(R)
eig_val_order = np.argsort(np.abs(w))          # ascending magnitude
v             = v[:, eig_val_order]
V = np.zeros((Nr, Nr - num_expected_signals), dtype=np.complex64)
for i in range(Nr - num_expected_signals):
    V[:, i] = v[:, i]

# ── 2D scan in direction-cosine (u, v) space ─────────────────────────────────
# u = sin(theta)*cos(phi)  →  Nx columns / azimuth
# v = sin(theta)*sin(phi)  →  Ny rows    / elevation
# Physical region: u² + v² ≤ 1  (hemisphere in front of array)
n_scan = 200
u_scan = np.linspace(-1, 1, n_scan)
v_scan = np.linspace(-1, 1, n_scan)

results = np.full((n_scan, n_scan), np.nan)
for i, u_i in enumerate(u_scan):
    for j, v_j in enumerate(v_scan):
        if u_i**2 + v_j**2 > 1:    # outside physical hemisphere → skip
            continue
        # Build steering vector directly from u, v
        a_x = np.exp(2j * np.pi * d * np.arange(Nx) * u_i)
        a_y = np.exp(2j * np.pi * d * np.arange(Ny) * v_j)
        a   = np.kron(a_x, a_y).reshape(-1, 1)
        metric = 1 / (a.conj().T @ V @ V.conj().T @ a)
        results[j, i] = 10 * np.log10(np.abs(metric.squeeze()))

results -= np.nanmax(results)   # normalize to 0 dB peak

# ── Plot 2D heatmap ───────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(7, 7))
im = ax.imshow(results,
               extent=[-1, 1, -1, 1],
               origin='lower', aspect='auto',
               vmin=-20, cmap='viridis')
plt.colorbar(im, ax=ax, label='dB (normalized)')

# Mark true signal locations with red crosses
for theta_s, phi_s in signals:
    u_t = np.sin(theta_s) * np.cos(phi_s)
    v_t = np.sin(theta_s) * np.sin(phi_s)
    ax.plot(u_t, v_t, 'r+', markersize=14, markeredgewidth=2, label=f'θ={np.degrees(theta_s):.0f}° φ={np.degrees(phi_s):.0f}°')

# Unit circle = edge of physical hemisphere
ax.add_patch(plt.Circle((0, 0), 1, fill=False, color='white', linestyle='--', linewidth=0.8))

ax.set_xlabel('u = sin(θ)cos(φ)  [azimuth]')
ax.set_ylabel('v = sin(θ)sin(φ)  [elevation]')
ax.set_title(f'2D MUSIC — {Ny}×{Nx} array,  {num_expected_signals} signals')
ax.legend(loc='upper right', fontsize=8)
plt.tight_layout()
plt.savefig('music_plot.png')
print("Saved music_plot.png")
