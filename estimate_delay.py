"""Estimate PI+motor delay from real car data via cross-correlation and phase analysis."""
import csv
import numpy as np
from scipy import signal as sig

# Load data
with open("data/realcar/balance_disturb_2.csv") as f:
    rows = list(csv.DictReader(f))

N = len(rows)
dt = 0.01  # 100Hz
theta_1 = np.array([float(r["theta_1"]) for r in rows])
theta_2 = np.array([float(r["theta_2"]) for r in rows])
thdot_L = np.array([float(r["theta_L_dot"]) for r in rows])
thdot_R = np.array([float(r["theta_R_dot"]) for r in rows])
u_L = np.array([float(r["u_L"]) for r in rows])
u_R = np.array([float(r["u_R"]) for r in rows])
TargetVal_L = np.array([float(r["TargetVal_L"]) for r in rows])
TargetVal_R = np.array([float(r["TargetVal_R"]) for r in rows])
PWM_L = np.array([float(r["PWM_L"]) for r in rows])
PWM_R = np.array([float(r["PWM_R"]) for r in rows])

# Remove mean (focus on AC/dynamic response)
u_L_ac = u_L - u_L.mean()
u_R_ac = u_R - u_R.mean()
thdot_L_ac = thdot_L - thdot_L.mean()
thdot_R_ac = thdot_R - thdot_R.mean()
tv_L_ac = TargetVal_L - TargetVal_L.mean()
pwm_L_ac = PWM_L - PWM_L.mean()

print("=" * 70)
print("1. Cross-correlation: u (desired accel) -> theta_dot (wheel velocity)")
print("=" * 70)

for name, u_ac, th_ac in [("Left", u_L_ac, thdot_L_ac), ("Right", u_R_ac, thdot_R_ac)]:
    # Cross-correlation
    corr = np.correlate(th_ac, u_ac, mode='full')
    lags = np.arange(-N + 1, N) * dt
    peak_idx = np.argmax(np.abs(corr))
    peak_lag = lags[peak_idx]

    # thdot responds AFTER u, so peak should be at positive lag (u leads, thdot lags)
    print(f"\n  {name} wheel:")
    print(f"    Peak correlation at lag = {peak_lag*1000:.1f} ms (positive = u leads thdot)")
    print(f"    Peak correlation coefficient = {corr[peak_idx] / (np.std(th_ac)*np.std(u_ac)*N):.3f}")

    # Find the first significant peak (positive lag)
    pos_corr = corr[N-1:]  # lags >= 0
    pos_lags = lags[N-1:]
    # Find all peaks
    peaks, props = sig.find_peaks(pos_corr, height=np.max(pos_corr)*0.3, distance=5)
    if len(peaks) > 0:
        for p in peaks[:3]:
            print(f"    Peak at {pos_lags[p]*1000:.0f} ms, corr={pos_corr[p]/(np.std(th_ac)*np.std(u_ac)*N):.3f}")

print()
print("=" * 70)
print("2. TargetVal -> theta_dot (PI tracking delay)")
print("=" * 70)

for name, tv_ac, th_ac in [("Left", tv_L_ac, thdot_L_ac)]:
    corr = np.correlate(th_ac, tv_ac, mode='full')
    lags = np.arange(-N + 1, N) * dt
    # thdot should lag TargetVal
    pos_corr = corr[N-1:]
    pos_lags = lags[N-1:]
    peaks, props = sig.find_peaks(pos_corr, height=np.max(pos_corr)*0.3, distance=5)
    if len(peaks) > 0:
        for p in peaks[:3]:
            print(f"  Peak at {pos_lags[p]*1000:.0f} ms, corr={pos_corr[p]/(np.std(th_ac)*np.std(tv_ac)*N):.3f}")

print()
print("=" * 70)
print("3. Phase analysis at dominant oscillation frequency (~1 Hz)")
print("=" * 70)

# Use FFT to find phase at dominant frequency
freqs = np.fft.rfftfreq(N, dt)
for name, x, y in [("u_L -> thdot_L", u_L_ac, thdot_L_ac),
                    ("u_R -> thdot_R", u_R_ac, thdot_R_ac),
                    ("TargetVal_L -> thdot_L", tv_L_ac, thdot_L_ac),
                    ("theta_1 -> u_L (LQR reaction)", theta_1, u_L_ac)]:
    X = np.fft.rfft(x)
    Y = np.fft.rfft(y)
    # Cross-spectrum
    cross = X * np.conj(Y)

    # Find dominant frequency in 0.5-3 Hz range
    mask = (freqs >= 0.5) & (freqs <= 3.0)
    f_band = freqs[mask]
    # Use magnitude of Y as weight
    mag = np.abs(Y[mask])
    # Find peak frequency
    peak_f = f_band[np.argmax(mag)]

    # Phase at peak frequency
    idx = np.argmin(np.abs(freqs - peak_f))
    phase = np.angle(cross[idx], deg=True)
    delay_ms = (phase / 360) * (1000 / peak_f)

    print(f"\n  {name}:")
    print(f"    Peak frequency = {peak_f:.2f} Hz")
    print(f"    Phase shift = {phase:.1f}°")
    print(f"    Equivalent delay = {delay_ms:.1f} ms")
    if phase < 0:
        print(f"    Interpretation: y LAGS x by {-delay_ms:.1f} ms")

print()
print("=" * 70)
print("4. Impulse response estimate (FIR from u -> thdot)")
print("=" * 70)

# Fit a simple FIR model: thdot[k] = sum_i h[i] * u[k-i]
# Use first half for fit, second half for validation
split = N // 2
# Build Toeplitz matrix for 50 lags (500ms)
n_lags = 50
U = np.zeros((split - n_lags, n_lags))
for i in range(n_lags):
    U[:, i] = u_L_ac[n_lags - 1 - i:split - 1 - i]

y = thdot_L_ac[n_lags:split]

# Ridge regression
h_fir = np.linalg.solve(U.T @ U + 0.1 * np.eye(n_lags), U.T @ y)

# Find peak of impulse response
peak_idx = np.argmax(np.abs(h_fir))
print(f"\n  FIR impulse response (u_L -> thdot_L):")
print(f"  Peak at lag = {peak_idx * dt * 1000:.0f} ms")
print(f"  Peak value = {h_fir[peak_idx]:.6f}")

# Time to rise to 50% of cumulative energy
cum_energy = np.cumsum(h_fir**2)
total = cum_energy[-1]
t50_idx = np.argmax(cum_energy > 0.5 * total)
t90_idx = np.argmax(cum_energy > 0.9 * total)
print(f"  50% energy at {t50_idx * dt * 1000:.0f} ms")
print(f"  90% energy at {t90_idx * dt * 1000:.0f} ms")

# Effective delay: centroid of |h|
centroid = np.sum(np.arange(n_lags) * np.abs(h_fir)) / np.sum(np.abs(h_fir))
print(f"  Centroid of |h| = {centroid * dt * 1000:.0f} ms (effective delay)")

print()
print("=" * 70)
print("5. LQR theoretical vs actual phase (theta_1 -> u)")
print("=" * 70)

# In ideal sim: u = -K @ x, so u and theta_1 should have specific phase
# Let's compute the theoretical u from theta_1 alone (ignoring other states)
# and compare with actual u
K13 = -5492.4061  # theta_1 coefficient
u_theory = -K13 * theta_1  # what u_L would be if only theta_1 mattered

corr = np.correlate(u_L_ac, u_theory - u_theory.mean(), mode='full')
lags = np.arange(-N + 1, N) * dt
peak_idx = np.argmax(np.abs(corr))
print(f"  u_L vs K13*theta_1 lag = {lags[peak_idx]*1000:.1f} ms")
print(f"  (Positive = u_L lags behind theoretical, indicating PI filtering)")

print()
print("=" * 70)
print("Summary")
print("=" * 70)
print("""The delay estimated from cross-correlation and phase analysis represents
the total latency from LQR command to actual wheel velocity change.
This includes: PI loop response time + motor electrical/mechanical time constant
+ encoder measurement delay + any software pipeline delay.""")
