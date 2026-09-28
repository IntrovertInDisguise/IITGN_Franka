import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# =========================
# PATHS
# =========================
BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR.parent / "logs" / "4_objects"

ft_file = LOG_DIR / "clean_ft_values.csv"
pose_file = LOG_DIR / "current_pose.csv"

# =========================
# LOAD CSV
# =========================
ft = pd.read_csv(ft_file)
pose = pd.read_csv(pose_file)

print("FT columns:", ft.columns.tolist())
print("Pose columns:", pose.columns.tolist())

# =========================
# AUTO COLUMN DETECTION
# =========================
def find_force_z(cols):
    for c in cols:
        lc = c.lower()
        if ("fz" in lc) or ("force" in lc and "z" in lc):
            return c
    raise ValueError("Could not find Fz column")

def find_axis(cols, axis):
    for c in cols:
        if c.lower() == axis:
            return c
    for c in cols:
        if axis in c.lower():
            return c
    raise ValueError(f"Could not find {axis} column")

fz_col = find_force_z(ft.columns)

x_col = find_axis(pose.columns, "x")
y_col = find_axis(pose.columns, "y")
z_col = find_axis(pose.columns, "z")

print("Using Fz:", fz_col)
print("Using pose:", x_col, y_col, z_col)

# =========================
# ALIGN LENGTH
# =========================
n = min(len(ft), len(pose))
ft = ft.iloc[:n].reset_index(drop=True)
pose = pose.iloc[:n].reset_index(drop=True)

# =========================
# TOUCH POINTS (Fz > 5)
# =========================
touch_mask = ft[fz_col] > 5.0
touch_points = pose[touch_mask]

print("Number of touch points:", len(touch_points))

# =========================
# 3D GRAPH
# =========================
fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection="3d")

ax.scatter(
    touch_points[x_col],
    touch_points[y_col],
    touch_points[z_col],
    s=12
)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Touch Points (Fz > 5N)")

plt.tight_layout()
plt.show()