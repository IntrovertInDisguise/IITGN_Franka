import pandas as pd

# File paths
file1 = "logs/session_logs/clean_ft_values.csv"
file2 = "logs/session_logs/current_pose.csv"

# Load CSVs
df1 = pd.read_csv(file1)
df2 = pd.read_csv(file2)

# --- Combine dataframes ---
# Option 1: combine side-by-side (same number of rows)
combined_df = pd.concat([df1, df2], axis=1)

# --- Create time column ---
# time = sec + nanosec * 1e-9
combined_df["time"] = (
    combined_df["sec"] + combined_df["nanosec"] * 1e-9
)

# --- Save output ---
output_file = "logs/session_logs/combined_output.csv"
combined_df.to_csv(output_file, index=False)

print(f"Combined CSV saved to: {output_file}")