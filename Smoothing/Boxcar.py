import random
def boxcar(win:int, data):
    half = win // 2
    n = len(data)
    avg = []
    for i in range(n):
        start = max(0, i + 1 -half)
        end = min(half + i, n)
        sum_array = sum(data[start:end])
        aver = sum_array / len(data[start:end])
        avg.append(aver)
    return avg
data = [random.randint(1, 100) for _ in range(100)]

smooth = boxcar(4, data)
with open("data.txt", "w") as f:
    for i in range(len(data)):
        f.write(f"{data[i]}\t{smooth[i]}\n")
print(smooth)