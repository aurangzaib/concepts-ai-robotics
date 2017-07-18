import numpy as np

initial_x = -1
initial_y = 2

move1x = 2
move1y = 2

move2x = 3
move2y = 3

Z0x = 4
Z0y = 6

Z1x = 2
Z1y = 4

Z2x = -1
Z2y = 1

Omega = np.matrix([[1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])

Xix = np.matrix([[initial_x], [0], [0], [0]])
Xiy = np.matrix([[initial_y], [0], [0], [0]])

Omega += np.matrix([[1, -1, 0, 0], [-1, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
Xix += np.matrix([[-move1x], [move1x], [0], [0]])
Xiy += np.matrix([[-move1y], [move1y], [0], [0]])

Omega += np.matrix([[0, 0, 0, 0], [0, 1, -1, 0], [0, -1, 1, 0], [0, 0, 0, 0]])
Xix += np.matrix([[0], [-move2x], [move2x], [0]])
Xiy += np.matrix([[0], [-move2y], [move2y], [0]])

Omega += np.matrix([[1, 0, 0, -1], [0, 0, 0, 0], [0, 0, 0, 0], [-1, 0, 0, 1]])
Xix += np.matrix([[-Z0x], [0], [0], [Z0x]])
Xiy += np.matrix([[-Z0y], [0], [0], [Z0y]])

Omega += np.matrix([[0, 0, 0, 0], [0, 1, 0, -1], [0, 0, 0, 0], [0, -1, 0, 1]])
Xix += np.matrix([[0], [- Z1x], [0], [Z1x]])
Xiy += np.matrix([[0], [- Z1y], [0], [Z1y]])

Omega += np.matrix([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, -1], [0, 0, -1, 1]])
Xix += np.matrix([[0], [0], [-Z2x], [Z2x]])
Xiy += np.matrix([[0], [0], [-Z2y], [Z2y]])

mu_x = Omega.getI() * Xix
mu_y = Omega.getI() * Xiy
mu = []
for index in range(len(mu_x)):
    mu.append(mu_x[index])
    mu.append(mu_y[index])

for m in mu:
    print m


def a():
    data = [
        [[[[0, 5]], [[0, 5]]], [5]],
        [[[0], [2]], [3]]
    ]
    N = 3
    num_landmarks = 1
    matrix_length = N + num_landmarks
    omega = [[0 for col in range(matrix_length[0])] for row in range(matrix_length)]

    for index in range(1, matrix_length - 1):
        other = index - 1

        omega[index][other] += -1
        omega[other][index] += -1
        omega[other][other] += 1
        omega[index][index] += 1

        if len(data[index - 1][0]):
            landmark_index = data[index - 1][0][0]
            for _index in range(len(data[index - 1][0])):
                print
