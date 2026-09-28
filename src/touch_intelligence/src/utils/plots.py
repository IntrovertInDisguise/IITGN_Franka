import os
import ast
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ==========================================================
# Helper: Compute Relative Time
# ==========================================================
def compute_time(df):
    time = df["sec"].to_numpy() + df["nanosec"].to_numpy() * 1e-9
    return time - time[0]


# ==========================================================
# Locate CSV directory
# ==========================================================
current_dir = os.path.dirname(os.path.abspath(__file__))
base_path = os.path.abspath(os.path.join(current_dir, "../logs/session_logs"))

ft_path = os.path.join(base_path, "clean_ft_values.csv")
pose_path = os.path.join(base_path, "current_pose.csv")
joint_path = os.path.join(base_path, "joint_states.csv")


# ==========================================================
# Create plots folder inside session_logs
# ==========================================================
plots_dir = os.path.join(base_path, "plots")

if not os.path.exists(plots_dir):
    os.mkdir(plots_dir)


# ==========================================================
# CLEAN FT VALUES
# ==========================================================
ft_df = pd.read_csv(ft_path)
ft_time = compute_time(ft_df)

fx = ft_df["fx"].to_numpy()
fy = ft_df["fy"].to_numpy()
fz = ft_df["fz"].to_numpy()
tx = ft_df["tx"].to_numpy()
ty = ft_df["ty"].to_numpy()
tz = ft_df["tz"].to_numpy()


# ---- Forces (3 vertical subplots) ----
fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

axs[0].plot(ft_time, fx)
axs[0].set_ylabel("Fx (N)")
axs[0].grid()

axs[1].plot(ft_time, fy)
axs[1].set_ylabel("Fy (N)")
axs[1].grid()

axs[2].plot(ft_time, fz)
axs[2].set_ylabel("Fz (N)")
axs[2].set_xlabel("Time (s)")
axs[2].grid()

fig.suptitle("Forces vs Time")
fig.tight_layout()
fig.savefig(os.path.join(plots_dir, "forces.png"))
plt.close(fig)


# ---- Torques (3 vertical subplots) ----
fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

axs[0].plot(ft_time, tx)
axs[0].set_ylabel("Tx (Nm)")
axs[0].grid()

axs[1].plot(ft_time, ty)
axs[1].set_ylabel("Ty (Nm)")
axs[1].grid()

axs[2].plot(ft_time, tz)
axs[2].set_ylabel("Tz (Nm)")
axs[2].set_xlabel("Time (s)")
axs[2].grid()

fig.suptitle("Torques vs Time")
fig.tight_layout()
fig.savefig(os.path.join(plots_dir, "torques.png"))
plt.close(fig)


# ==========================================================
# CURRENT POSE (Position)
# ==========================================================
pose_df = pd.read_csv(pose_path)

px = pose_df["px"].to_numpy()
py = pose_df["py"].to_numpy()
pz = pose_df["pz"].to_numpy()

pose_time = np.arange(len(px))

fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

axs[0].plot(pose_time, px)
axs[0].set_ylabel("Px (m)")
axs[0].grid()

axs[1].plot(pose_time, py)
axs[1].set_ylabel("Py (m)")
axs[1].grid()

axs[2].plot(pose_time, pz)
axs[2].set_ylabel("Pz (m)")
axs[2].set_xlabel("Time Step")
axs[2].grid()

fig.suptitle("End-Effector Position vs Time")
fig.tight_layout()
fig.savefig(os.path.join(plots_dir, "pose_position.png"))
plt.close(fig)


# ==========================================================
# JOINT STATES
# ==========================================================
joint_df = pd.read_csv(joint_path)
joint_time = compute_time(joint_df)

positions = np.array(joint_df["positions"].apply(ast.literal_eval).to_list())
velocities = np.array(joint_df["velocities"].apply(ast.literal_eval).to_list())
efforts = np.array(joint_df["efforts"].apply(ast.literal_eval).to_list())

num_joints = positions.shape[1]


# ---- Joint Positions ----
fig, axs = plt.subplots(num_joints, 1, figsize=(8, 3*num_joints), sharex=True)

for i in range(num_joints):
    axs[i].plot(joint_time, positions[:, i])
    axs[i].set_ylabel(f"J{i+1} (rad)")
    axs[i].grid()

axs[-1].set_xlabel("Time (s)")
fig.suptitle("Joint Positions vs Time")
fig.tight_layout()
fig.savefig(os.path.join(plots_dir, "joint_positions.png"))
plt.close(fig)


# ---- Joint Velocities ----
fig, axs = plt.subplots(num_joints, 1, figsize=(8, 3*num_joints), sharex=True)

for i in range(num_joints):
    axs[i].plot(joint_time, velocities[:, i])
    axs[i].set_ylabel(f"J{i+1} (rad/s)")
    axs[i].grid()

axs[-1].set_xlabel("Time (s)")
fig.suptitle("Joint Velocities vs Time")
fig.tight_layout()
fig.savefig(os.path.join(plots_dir, "joint_velocities.png"))
plt.close(fig)


# ---- Joint Efforts ----
fig, axs = plt.subplots(num_joints, 1, figsize=(8, 3*num_joints), sharex=True)

for i in range(num_joints):
    axs[i].plot(joint_time, efforts[:, i])
    axs[i].set_ylabel(f"J{i+1} (Nm)")
    axs[i].grid()

axs[-1].set_xlabel("Time (s)")
fig.suptitle("Joint Efforts vs Time")
fig.tight_layout()
fig.savefig(os.path.join(plots_dir, "joint_efforts.png"))
plt.close(fig)


print("All plots saved in:", plots_dir)
