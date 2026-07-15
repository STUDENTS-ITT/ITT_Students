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

data = [0, 0, 10, 0, 0]
print(hybrid(data, 5))