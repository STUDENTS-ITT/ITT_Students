import random

def progressive (data):
    result = []
    total = 0.0
    for i, value in enumerate(data):
         total += value
         result.append(total / (i + 1))
    return result

data = [0, 0, 10, 0, 0]
print(data)
print(progressive(data))