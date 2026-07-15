import math
import random
import matplotlib.pyplot as plt

def boxcar(win, data):
    half = win // 2
    m = len(data)
    avg = []
    for i in range(m):
        start = max(0, i + 1 -half)
        end = min(half + i, m)
        sum_array = sum(data[start:end])
        aver = sum_array / len(data[start:end])
        avg.append(aver)
    return avg

def progressive(data):
    result = []
    total = 0.0
    for i, value in enumerate(data):
         total += value
         result.append(total / (i + 1))
    return result

def hybrid(data, win):
    n = len(data)

    box = boxcar(win, data)
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

def gaussian(data, win):
    half = win // 2
    sigma = win / 6.0
    n = len(data)
    k = [math.exp(-0.5*(i - half)**2 / sigma**2) for i in range(win)]
    k = [x/sum(k) for x in k]
    p = [data[i] for i in range(half-1,-1,-1)] + data + [data[i] for i in range(n-1,n-half-1,-1)]
    return [sum(k[j]*p[i+j] for j in range(win)) for i in range(n)]


data = [random.randint(1, 100) for _ in range(100)]
win = 11

box_result = boxcar(win, data)
prog_result = progressive(data)
hybrid_result = hybrid(data, win)
gauss_result = gaussian(data, win)

x = list(range(len(data)))

fig, axes = plt.subplots(3, 2, figsize=(14, 12))

axes[0, 0].plot(x, data, color='gray', alpha=0.5, linewidth=1, label='Исходные данные')
axes[0, 0].plot(x, box_result, linewidth=2, label='Boxcar')
axes[0, 0].plot(x, prog_result, linewidth=2, label='Progressive')
axes[0, 0].plot(x, hybrid_result, linewidth=2, label='Hybrid')
axes[0, 0].plot(x, gauss_result, linewidth=2, label='Gaussian')
axes[0, 0].set_title('Все методы сглаживания')
axes[0, 0].legend()
axes[0, 0].grid(True, alpha=0.3)

axes[0, 1].plot(x, data, color='gray', alpha=0.5, linewidth=1, label='Данные')
axes[0, 1].plot(x, box_result, linewidth=2, label='Boxcar')
axes[0, 1].set_title('Boxcar (W={})'.format(win))
axes[0, 1].legend()
axes[0, 1].grid(True, alpha=0.3)

axes[1, 0].plot(x, data, color='gray', alpha=0.5, linewidth=1, label='Данные')
axes[1, 0].plot(x, prog_result, linewidth=2, label='Progressive')
axes[1, 0].set_title('Progressive')
axes[1, 0].legend()
axes[1, 0].grid(True, alpha=0.3)

axes[1, 1].plot(x, data, color='gray', alpha=0.5, linewidth=1, label='Данные')
axes[1, 1].plot(x, hybrid_result, linewidth=2, label='Hybrid')
axes[1, 1].set_title('Hybrid (W={})'.format(win))
axes[1, 1].legend()
axes[1, 1].grid(True, alpha=0.3)

axes[2, 0].plot(x, data, color='gray', alpha=0.5, linewidth=1, label='Данные')
axes[2, 0].plot(x, gauss_result, linewidth=2, label='Gaussian')
axes[2, 0].set_title('Gaussian (W={})'.format(win))
axes[2, 0].legend()
axes[2, 0].grid(True, alpha=0.3)

axes[2, 1].axis('off')

plt.tight_layout()
plt.savefig('comparison_plot.png', dpi=150)
plt.show()
