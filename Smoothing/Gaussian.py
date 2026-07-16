import math
import random

def gaussian(data, win):
    half = win // 2
    sigma = win / 6.0
    n = len(data)
    k = [math.exp(-0.5*(i - half)**2 / sigma**2) for i in range(win)]
    k = [x/sum(k) for x in k]
    p = [data[i] for i in range(half-1,-1,-1)] + data + [data[i] for i in range(n-1,n-half-1,-1)]
    return [sum(k[j]*p[i+j] for j in range(win)) for i in range(n)]

data = [0, 0, 10, 0, 0]
print(data)
print(gaussian(data, 5))