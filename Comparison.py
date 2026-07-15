import math
import random
import matplotlib.pyplot as plt
import time


def boxcar(data, win):
    half = win // 2
    m = len(data)
    avg = []
    for i in range(m):
        start = max(0, i - half)
        end = min(half + i + 1, m)
        sum_array = sum(data[start:end])
        aver = sum_array / len(data[start:end])
        avg.append(aver)
    return avg

def boxcar_reflect(data, win):
    half = win // 2
    n = len(data)
    reflected = data[::-1] + data + data[::-1]
    offset = len(data)

    avg = []
    for i in range(n):
        start = offset + i - half
        end = start + win
        avg.append(sum(reflected[start:end]) / win)

    return avg

def progressive(data):
    result_list = []
    total = 0.0
    for i, value in enumerate(data):
         total += value
         result_list.append(total / (i + 1))
    return result_list

def hybrid(data, win):
    n = len(data)

    box = boxcar(data, win)
    prog = progressive(data)

    out = []
    for i in range(n):
        if i < win:
            out.append(box[i])
        elif i < 2 * win:
            z = (i -win) / win
            k = (1.0 - z) * box[i] + z * prog[i]
            out.append(k)
        else:
            out.append(prog[i])
    return out

def hybrid_reflect(data, win):
    n = len(data)

    box = boxcar_reflect(data, win)
    prog = progressive(data)

    out = []
    for i in range(n):
        if i < win:
            out.append(box[i])
        elif i < 2 * win:
            z = (i - win) / win
            k = (1.0 - z) * box[i] + z * prog[i]
            out.append(k)
        else:
            out.append(prog[i])
    return out


def gaussian(data, win):
    half = win // 2
    sigma = win / 6.0
    n = len(data)
    k = [math.exp(-0.5*(i - half)**2 / sigma**2) for i in range(win)]
    k = [x/sum(k) for x in k]
    p = [data[i] for i in range(half-1,-1,-1)] + data + [data[i] for i in range(n-1,n-half-1,-1)]
    return [sum(k[j]*p[i+j] for j in range(win)) for i in range(n)]

n_points = 1000000
true_g=9.81
noise_std=0.3
#data = [random.randint(1, 100) for _ in range(n_points)]
data = []
for i in range(n_points):
    value = true_g + random.gauss(0,noise_std)
    if random.random() < 0.05:
        value += random.choice([-2.0, 2.0])
    data.append(round(value, 2))

win = 11

results = {}
times = {}

methods_with_win = [
    ("Boxcar", boxcar,),
    ("Boxcar with reflection", boxcar_reflect,),
    ("Hybrid", hybrid),
    ("Hybrid with reflection", hybrid_reflect),
    ("Gaussian",gaussian),]

for name, func in methods_with_win:
    start = time.perf_counter()
    result = func(data, win)
    end = time.perf_counter()
    results[name] = result
    times[name] = end - start

methods_without_win = [
    ("Progressive", progressive),
]

for name, func in methods_without_win:
    start = time.perf_counter()
    result = func(data)
    end = time.perf_counter()
    results[name] = result
    times[name] = end - start

rmsd_results = {}
errors_dict = {}

for name in results:
    smoothed = results[name]

    squared_errors = [(g - true_g) ** 2 for g in smoothed]

    rmsd = math.sqrt(sum(squared_errors) / len(squared_errors))
    rmsd_results[name] = rmsd

    errors_dict[name] = squared_errors


fig, axes = plt.subplots(3, 2, figsize=(14, 12))
axes = axes.flatten()

all_methods = methods_with_win + methods_without_win

for idx, (name, func) in enumerate(all_methods):
    ax = axes[idx]

    ax.plot(data, 'b-', alpha=0.3, label='Исходные данные', linewidth=1)

    smoothed = results[name]
    ax.plot(smoothed, 'r-', linewidth=2, label=name)

    ax.set_title(f'{name}\n Время: {times[name]:.6f} сек', fontsize=10)
    ax.set_xlabel('Индекс')
    ax.set_ylabel('Значение')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)

plt.tight_layout()
plt.savefig('comparison.png', dpi = 150, bbox_inches='tight')
plt.close()

fig, axes = plt.subplots(3, 2, figsize=(14, 12))
axes = axes.flatten()

for idx, (name, func) in enumerate(all_methods):
    ax = axes[idx]

    errors = errors_dict[name]
    ax.plot(errors, 'r-', linewidth= 1.5, label=f'{name}')

    mean_error = sum(errors) / len (errors)
    ax.axhline(y=mean_error, color='blue', linestyle='--', alpha = 0.5, label=f'Средняя ошибка: {mean_error:.4f}')

    ax.set_title(f'{name}\nRMSD: {rmsd_results[name]:.4f} м/с', fontsize=10)
    ax.set_xlabel('Номер измерения')
    ax.set_ylabel("Квадрат ошибки")
    ax.grid(True, alpha=0.3)
    ax.legend()

plt.tight_layout()
plt.savefig('comparison_errors.png', dpi=150, bbox_inches='tight')
plt.close()

print(data)
print("=" * 70)
print(f"Размер окна: {win}")
print(f"Количество точек: {n_points}")
print("=" * 70)
print(f"{'Метод':<25} {'Среднее':<10} {'Мин':<10} {'Макс':<10}")
print("-" * 70)
for name, func in all_methods:
    smoothed = results[name]
    print(f"{name:<25} {sum(smoothed)/len(smoothed):<10.2f} {min(smoothed):<10.2f} {max(smoothed):<10.2f}")
print("=" * 70)