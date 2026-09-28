import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from pathlib import Path

# =========================
# PATHS
# =========================
BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR.parent / "logs" / "4_objects"

ft_file = LOG_DIR / "clean_ft_values.csv"

# =========================
# LOAD DATA
# =========================
ft = pd.read_csv(ft_file)

print("Columns:", ft.columns.tolist())

# =========================
# FIND Ty COLUMN
# =========================
ty_col = None
for c in ft.columns:
    lc = c.lower()
    if "ty" in lc or ("torque" in lc and "y" in lc):
        ty_col = c
        break

if ty_col is None:
    raise ValueError("Ty column not found")

print("Using Ty column:", ty_col)

signal = ft[ty_col].values

# =========================
# SAMPLING RATE (EDIT IF KNOWN)
# =========================
fs = 100.0  # Hz

N = len(signal)

# =========================
# FFT
# =========================
yf = fft(signal)
xf = fftfreq(N, 1/fs)

# Positive frequencies only
idx = xf > 0
xf = xf[idx]
yf = np.abs(yf[idx])

# =========================
# PLOT FFT
# =========================
plt.figure(figsize=(8,4))
plt.plot(xf, yf)
plt.xlim(0, 60)
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.title("FFT of Ty")
plt.grid(True)
plt.show()

# =========================
# FIND DOMINANT FREQUENCIES
# =========================
top_idx = np.argsort(yf)[-5:]   # top 5 peaks

print("\nTop frequency components:")
for i in reversed(top_idx):
    print(f"Freq: {xf[i]:.2f} Hz  |  Magnitude: {yf[i]:.3f}")