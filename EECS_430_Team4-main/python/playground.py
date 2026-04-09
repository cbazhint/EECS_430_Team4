import numpy as np
import matplotlib.pyplot as plt 

sample_rate = 1e6
N = 10000 #number of samples to simulate

#Create a tone to act as a transmitter

t = np.arange(N) / sample_rate 
f_tone = 0.02e6
tx = np.exp(2j * np.pi * f_tone * t)

d = 0.5 # half wavelength spacing
Nr_rows = 2  # number of antenna rows
Nr_cols = 8  # number of antenna columns
Nr = Nr_rows * Nr_cols  # 16 total elements

theta_deg = 20  # elevation angle
phi_deg   = 45  # azimuth angle
theta = theta_deg / 180 * np.pi
phi   = phi_deg   / 180 * np.pi

# 1D steering vector along columns (8 elements, azimuth)
s_col = np.exp(2j * np.pi * np.arange(Nr_cols) * d * np.sin(theta) * np.cos(phi))
# 1D steering vector along rows (2 elements, elevation)
s_row = np.exp(2j * np.pi * np.arange(Nr_rows) * d * np.sin(theta) * np.sin(phi))

# Full 2D steering vector via Kronecker product (16 elements)
s = np.kron(s_row, s_col)
print(s)

s = s.reshape(-1, 1)   # (16, 1) column vector
print(s.shape)
tx = tx.reshape(1, -1) # (1, N) row vector
print(tx.shape)

X = s @ tx  # (16, N)

# Plot first 4 antennas to keep it readable
for i in range(4):
    plt.plot(np.asarray(X[i,:]).squeeze().real[0:200], label=f"Ant {i}")
plt.legend()
plt.savefig("plot.png")
#print("Saved plot.png")



# For a 2D array, scan in direction cosine (u-v) space:
#   u = sin(theta)*cos(phi)  -> driven by columns (Nr_cols elements)
#   v = sin(theta)*sin(phi)  -> driven by rows    (Nr_rows elements)
# This is linear/unambiguous. Scanning phi/theta directly causes non-monotonic
# sin mappings that fold the beam pattern back on itself (creating blobs).

u_true = np.sin(theta) * np.cos(phi)  # true column direction cosine
v_true = np.sin(theta) * np.sin(phi)  # true row direction cosine

u_scan = np.linspace(-1, 1, 1000)
v_scan = np.linspace(-1, 1, 1000)

# Column (azimuth) cut: scan u, fix v at true value
results_u = []
for u_i in u_scan:
    w_col = np.exp(2j * np.pi * d * np.arange(Nr_cols) * u_i)
    w_row = np.exp(2j * np.pi * d * np.arange(Nr_rows) * v_true)
    w = np.kron(w_row, w_col)
    X_weighted = w.conj() @ X
    results_u.append(10 * np.log10(np.var(X_weighted)))
results_u = np.array(results_u)
results_u -= np.max(results_u)

# Row (elevation) cut: scan v, fix u at true value
results_v = []
for v_i in v_scan:
    w_col = np.exp(2j * np.pi * d * np.arange(Nr_cols) * u_true)
    w_row = np.exp(2j * np.pi * d * np.arange(Nr_rows) * v_i)
    w = np.kron(w_row, w_col)
    X_weighted = w.conj() @ X
    results_v.append(10 * np.log10(np.var(X_weighted)))
results_v = np.array(results_v)
results_v -= np.max(results_v)

# Polar scans: phi azimuth (-pi to pi), theta elevation (-pi/2 to pi/2 only)
phi_scan   = np.linspace(-np.pi, np.pi, 1000)
theta_scan = np.linspace(-np.pi/2, np.pi/2, 1000)  # physical range only

results_phi = []
for phi_i in phi_scan:
    w_col = np.exp(2j * np.pi * d * np.arange(Nr_cols) * np.sin(theta) * np.cos(phi_i))
    w_row = np.exp(2j * np.pi * d * np.arange(Nr_rows) * np.sin(theta) * np.sin(phi_i))
    w = np.kron(w_row, w_col)
    X_weighted = w.conj() @ X
    results_phi.append(10 * np.log10(np.var(X_weighted)))
results_phi = np.array(results_phi)
results_phi -= np.max(results_phi)

results_theta = []
for theta_i in theta_scan:
    w_col = np.exp(2j * np.pi * d * np.arange(Nr_cols) * np.sin(theta_i) * np.cos(phi))
    w_row = np.exp(2j * np.pi * d * np.arange(Nr_rows) * np.sin(theta_i) * np.sin(phi))
    w = np.kron(w_row, w_col)
    X_weighted = w.conj() @ X
    results_theta.append(10 * np.log10(np.var(X_weighted)))
results_theta = np.array(results_theta)
results_theta -= np.max(results_theta)

fig = plt.figure(figsize=(12, 8))

ax1 = fig.add_subplot(2, 2, 1)
ax1.plot(u_scan, results_u)
ax1.axvline(u_true, color='r', linestyle='--', label=f'True u={u_true:.3f}')
ax1.set_xlabel('u = sin(θ)cos(φ)  [columns / azimuth]')
ax1.set_ylabel('dB')
ax1.set_title('Column Cut (Azimuth)')
ax1.legend()

ax2 = fig.add_subplot(2, 2, 2)
ax2.plot(v_scan, results_v)
ax2.axvline(v_true, color='r', linestyle='--', label=f'True v={v_true:.3f}')
ax2.set_xlabel('v = sin(θ)sin(φ)  [rows / elevation]')
ax2.set_ylabel('dB')
ax2.set_title('Row Cut (Elevation) — broad due to Nr_rows=2')
ax2.legend()

ax3 = fig.add_subplot(2, 2, 3, projection='polar')
ax3.plot(phi_scan, results_phi - results_phi.min())
ax3.set_title('Azimuth (φ) Polar', pad=15)

ax4 = fig.add_subplot(2, 2, 4, projection='polar')
ax4.plot(theta_scan, results_theta - results_theta.min())
ax4.set_title('Elevation (θ) Polar', pad=15)

plt.tight_layout()
plt.savefig("plot2.png")
print("Saved plot2.png")
