import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from mpl_toolkits.mplot3d import Axes3D

# ====================================
# 1. LOAD DATA (ORIGINAL DATA SAFE)
# ====================================

df = pd.read_csv("logs/session_logs/combined_output.csv")

Fz = df["fz"].copy().values
Ty = df["ty"].copy().values
TM = df["time"].copy().values
PX = df["px"].copy().values
PY = df["py"].copy().values
PZ = df["pz"].copy().values

print("Total samples:", len(Fz))

# ====================================
# 2. SMOOTH FOR DETECTION ONLY
# ====================================

Fz_s = pd.Series(Fz).rolling(2, center=True).mean()
Fz_s = Fz_s.fillna(method="bfill").fillna(method="ffill").values

# ====================================
# 3. DETECT POSITIVE PEAKS IN Fz
# ====================================

std_fz = np.std(Fz_s)
MIN_PEAK_VALUE = 6.0
fz_peaks, _ = find_peaks(
    Fz_s,
    prominence=2.0 * std_fz,
    distance=5,
    height=MIN_PEAK_VALUE
)

print("Positive Fz peaks found:", len(fz_peaks))

# ====================================
# 4. FIND RISE STARTS
# ====================================

def find_rise_start(signal, peak_idx):
    peak_val = signal[peak_idx]
    baseline_level = 0.3 * peak_val

    i = peak_idx
    while i > 0 and signal[i] > baseline_level:
        i -= 1

    return i


rise_starts = []

for p in fz_peaks:
    rise_starts.append(find_rise_start(Fz_s, p))

# ====================================
# 5. SAVE OUTPUT TABLE
# ====================================

results = []

for rise, peak in zip(rise_starts, fz_peaks):

    row = {
        "rise_timestep": rise,
        "peak_timestep": peak,

        "Time_at_rise": TM[rise],
        "Time_at_peak": TM[peak],

        "Fz_at_rise": Fz[rise],
        "Fz_at_peak": Fz[peak],

        "Ty_at_rise": Ty[rise],
        "Ty_at_peak": Ty[peak],

        "PX_at_peak": PX[peak], 
        "PY_at_peak": PY[peak], 
        "PZ_at_peak": PZ[peak], 
    }

    results.append(row)

results_df = pd.DataFrame(results)
results_df.to_csv("logs/session_logs/Fz_events_output.csv", index=False)

print("Saved: Fz_events_output.csv")

# ====================================
# 5B. CREATE UPDATED_X FILE
# ====================================

# avoid divide-by-zero
delta_fz = results_df["Fz_at_peak"] - results_df["Fz_at_rise"]

# compute updated_x
results_df["updated_x"] = (
    results_df["PX_at_peak"]
    - 1.5*(results_df["Ty_at_peak"] - results_df["Ty_at_rise"]) / delta_fz
)

# keep only required columns
updated_xyz = results_df[
    ["updated_x", "PY_at_peak", "PZ_at_peak"]
].copy()

# rename columns if you want clean names
updated_xyz.columns = ["updated_x", "py", "pz"]

# save
updated_xyz.to_csv(
    "logs/session_logs/updated_xyz.csv",
    index=False
)

print("Saved: updated_xyz.csv")
# ====================================
# 6. PLOT TY + FZ TOGETHER
# ====================================

plt.figure(figsize=(12,6))

# ORIGINAL signals
plt.plot(Fz, label="Fz (original)", linewidth=1)
plt.plot(Ty, label="Ty (original)", linewidth=1, alpha=0.7)

# Fz peaks
plt.scatter(
    fz_peaks,
    Fz[fz_peaks],
    color="red",
    label="Fz Peaks",
    zorder=3
)

# Rise starts
plt.scatter(
    rise_starts,
    Fz[rise_starts],
    color="green",
    label="Rise Starts",
    zorder=3
)

plt.title("Fz and Ty with Fz Peaks + Rise Starts")
plt.xlabel("Time step")
plt.ylabel("Signal value")
plt.legend()
plt.grid(True)

plt.show()
# ====================================
# 7. 3D SCATTER PLOT (updated_x, py, pz)
# ====================================



fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')

ax.scatter(
    updated_xyz["updated_x"],
    updated_xyz["py"],
    updated_xyz["pz"],
    s=40,
    alpha=0.8
)

ax.set_title("3D Scatter: updated_x vs py vs pz")
ax.set_xlabel("updated_x")
ax.set_ylabel("py")
ax.set_zlabel("pz")

# ---------- MAKE AXES A CUBE ----------
x = updated_xyz["updated_x"].values
y = updated_xyz["py"].values
z = updated_xyz["pz"].values

CUBE_SCALE = 1.5 

max_range = (np.array([
    x.max()-x.min(),
    y.max()-y.min(),
    z.max()-z.min()
]).max() / 2.0) * CUBE_SCALE

mid_x = (x.max()+x.min()) * 0.5
mid_y = (y.max()+y.min()) * 0.5
mid_z = (z.max()+z.min()) * 0.5

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)
# --------------------------------------

plt.show()