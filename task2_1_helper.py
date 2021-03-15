import math

def weight(predicted, actual):
    distance = abs(predicted[0] - actual[0]) + abs(predicted[1] - actual[1])
    angle = predicted[2] - actual[2]
    print(distance, angle)
    return "{:.2e}".format(math.exp(-((distance**2)/(2*(0.5**2))) + ((angle**2)/(2*(0.3**2)))))
predictions = [
    (0, 5, 2.33),
    (3, 2, 6.12),
    (2, 2, 3.56),
    (5, 0, 2.22),
    (5, 5, 1.23)
    ]
actuals = [
    (6, 0, 3.22),
    (3, 4, 4.32),
    (5, 6, 0.23),
    (1, 6, 5.13),
    (1, 3, 2.56)
]

for actual in actuals:
    print( (0, 5, 2.33), actual)
    print(weight( (0, 5, 2.33), actual))
