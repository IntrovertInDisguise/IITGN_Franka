import pandas as pd
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

print("Available columns:", ft.columns.tolist())

# =========================
# AUTO FIND Ty AND Fz
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
# EXTRACT DATA
# =========================
out = ft[[ty_col, fz_col]].copy()

# Rename for consistency
out.columns = ["Ty", "Fz"]

# =========================
# SAVE
# =========================
output_file = LOG_DIR / "Ty_Fz_only.csv"
out.to_csv(output_file, index=False)

print("Saved:", output_file)