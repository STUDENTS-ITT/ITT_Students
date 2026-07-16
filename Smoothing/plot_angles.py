import numpy as np
import matplotlib.pyplot as plt
import sys
import os

def find_data_folder():
    if os.path.exists('output.txt'):
        return '.'
    for root, dirs, files in os.walk('.'):
        if 'output.txt' in files:
            return root
    return None

colors = {'raw': '#2196F3', 'smooth': '#FF5722', 'rmse': '#4CAF50'}
SKIP = 0

def running_rmse(raw, smooth):
    n = len(raw)
    rmse = np.zeros(n)
    M2 = 0.0
    m = raw[0]
    rmse[0] = 0.0
    for i in range(1, n):
        delta = raw[i] - m
        m = m + delta / (i + 1)
        M2 = M2 + delta * (raw[i] - m)
        rmse[i] = np.sqrt(M2 / (i + 1))
    return rmse

if len(sys.argv) > 1:
    folder = sys.argv[1]
else:
    folder = find_data_folder()
    if folder is None:
        print("Ошибка: папка с IMU.txt и Nav.dat не найдена")
        sys.exit(1)

output_file = os.path.join(folder, 'output.txt')
if not os.path.exists(output_file):
    print(f"Ошибка: {output_file} не найден. Сначала запустите main.cpp")
    sys.exit(1)

data = np.loadtxt(output_file, skiprows=2)
t = data[SKIP:, 0]
pitch_r = data[SKIP:, 7]
roll_r  = data[SKIP:, 8]
yaw_r   = data[SKIP:, 9]
pitch_s = data[SKIP:, 16]
roll_s  = data[SKIP:, 17]
yaw_s   = data[SKIP:, 18]

fig, axes = plt.subplots(6, 1, figsize=(12, 14), sharex=True)
fig.suptitle(f'Angles vs Time (rad) — {folder}', fontsize=14, fontweight='bold')

labels = ['Pitch', 'Roll', 'Yaw']
raw    = [pitch_r, roll_r, yaw_r]
smooth = [pitch_s, roll_s, yaw_s]
rmse   = [running_rmse(r, s) for r, s in zip(raw, smooth)]

for i, (name, r, s, rms) in enumerate(zip(labels, raw, smooth, rmse)):
    ax = axes[2 * i]
    ax.plot(t, r, color=colors['raw'], lw=0.3, alpha=0.5, label=f'{name} raw')
    ax.plot(t, s, color=colors['smooth'], lw=0.8, label=f'{name} smoothed')
    ax.set_ylabel(f'{name} (rad)')
    ax.legend(fontsize=9, loc='upper right')
    ax.grid(True, alpha=0.3)

    ax2 = axes[2 * i + 1]
    ax2.plot(t, rms, color=colors['rmse'], lw=0.8)
    ax2.set_ylabel(f'{name} RMSE (rad)')
    ax2.grid(True, alpha=0.3)

axes[-2].set_xlabel('Time (s)')
axes[-1].set_xlabel('Time (s)')

plt.tight_layout()
out_path = os.path.join(folder, 'angles.png')
plt.savefig(out_path, dpi=150)
print(f'Saved {out_path}')
