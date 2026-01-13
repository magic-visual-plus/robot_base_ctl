import pandas as pd
import numpy as np
import re
import os
import matplotlib
import matplotlib.pyplot as plt

csv_path = "/opt/project/robot_base_ctl/data_20251219_201132_100/data_20251219_201132_100/control_data.csv"
out_dir = "/opt/project/robot_base_ctl/viz_out"
os.makedirs(out_dir, exist_ok=True)

print("Matplotlib backend:", matplotlib.get_backend())
print("Reading:", csv_path)

df = pd.read_csv(csv_path)[0:270]  # 只看一部分数据，避免太密集
print("Loaded CSV. shape =", df.shape)
print("Columns =", list(df.columns))

def parse_tensor(v):
    # robust: handles tensor(0.), tensor(-1.23), "0.123", 0.123
    if isinstance(v, (int, float, np.floating)):
        return float(v)
    s = str(v)
    m = re.search(r"tensor\(([-+0-9.eE]+)\)", s)
    if m:
        return float(m.group(1))
    m2 = re.search(r"([-+0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)", s)
    return float(m2.group(1)) if m2 else np.nan

need = ["base_x", "base_y", "base_z", "base_pitch"]
for c in need:
    if c not in df.columns:
        raise RuntimeError(f"Missing column: {c}. Available: {list(df.columns)}")
    df[c] = df[c].map(parse_tensor)

x = df["base_x"].to_numpy()
y = df["base_y"].to_numpy()
z = df["base_z"].to_numpy()
yaw = df["base_pitch"].to_numpy()
idx = np.arange(len(df))

print("Ranges:")
print("  x:", np.nanmin(x), "to", np.nanmax(x))
print("  y:", np.nanmin(y), "to", np.nanmax(y))
print("  z:", np.nanmin(z), "to", np.nanmax(z))
print(" yaw:", np.nanmin(yaw), "to", np.nanmax(yaw))

# ========= 1) 轨迹 + yaw（时间渐变颜色） =========
plt.figure(figsize=(7,7))

sc = plt.scatter(
    x, y,
    c=idx,                 # 用时间 / sample index 上色
    cmap="viridis",        # 颜色映射：早->晚
    s=8                    # 点大小，别太大
)

# yaw 箭头（仍然稀疏画）
step = max(1, len(x)//30)
plt.quiver(
    x[::step], y[::step],
    np.cos(yaw[::step]), np.sin(yaw[::step]),
    angles="xy", scale_units="xy", scale=18,
    color="red", alpha=0.7
)

plt.axis("equal")
plt.grid(True)
plt.title("Trajectory with time-colored path (yaw=base_pitch)")
plt.xlabel("base_x")
plt.ylabel("base_y")

cbar = plt.colorbar(sc)
cbar.set_label("time (sample index)")

p1 = os.path.join(out_dir, "traj_xy_time_colored.png")
plt.savefig(p1, dpi=200, bbox_inches="tight")
print("Saved:", p1)
