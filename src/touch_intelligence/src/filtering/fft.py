import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

# -----------------------------
# 1. Load the data file
# -----------------------------
# Replace with your file path
file_path = "logs/Star_18_02_10_14/clean_ft_values.csv"

# Read large dataset efficiently
data = pd.read_csv(file_path)

# -----------------------------
# 2. Extract ty column
# -----------------------------
ty = data['ty'].values

# Remove NaN if present
ty = ty[~np.isnan(ty)]

# -----------------------------
# 3. Define sampling rate
# -----------------------------
# IMPORTANT: change according to your data
fs = 1000   # Sampling frequency (Hz)

N = len(ty)

# -----------------------------
# 4. Perform FFT
# -----------------------------
fft_values = np.fft.fft(ty)

# Frequency axis
freq = np.fft.fftfreq(N, d=1/fs)

# Take only positive frequencies
positive = freq > 0

freq = freq[positive]
fft_magnitude = np.abs(fft_values[positive])

# Create dataframe with results
fft_results = pd.DataFrame({
    "frequency": freq,
    "magnitude": fft_magnitude
})

# Save file in same folder as script
output_path = os.path.join(os.path.dirname(__file__), "fft_results.csv")

#fft_results.to_csv(output_path, index=False)

print(f"FFT results saved to: {output_path}")
# -----------------------------
# 5. Plot FFT spectrum
# -----------------------------
plt.figure(figsize=(10,6))

plt.plot(freq, fft_magnitude)
plt.title("FFT Spectrum of ty")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.grid(True)

plt.tight_layout()
plt.show()