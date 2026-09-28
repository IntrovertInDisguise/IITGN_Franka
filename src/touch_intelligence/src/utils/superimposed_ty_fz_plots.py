import pandas as pd
import matplotlib.pyplot as plt
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

# =========================
# AUTO FIND COLUMNS
# =========================
ty_col = None
fz_col = None

for c in ft.columns:
    lc = c.lower()

    if ty_col is None and ("ty" in lc or ("torque" in lc and "y" in lc)):
        ty_col = c

    if fz_col is None and ("fz" in lc or ("force" in lc and "z" in lc)):
        fz_col = c

if ty_col is None:
    raise ValueError("Ty column not found")

if fz_col is None:
    raise ValueError("Fz column not found")

print("Using Ty:", ty_col)
print("Using Fz:", fz_col)

# =========================
# SUPERIMPOSED PLOT
# =========================
plt.figure(figsize=(10,5))

plt.plot(ft[fz_col], label="Fz (N)", linewidth=1.5)
plt.plot(ft[ty_col], label="Ty (Nm)", linewidth=1.5)

plt.xlabel("Sample index")
plt.ylabel("Value")
plt.title("Superimposed Ty and Fz")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()