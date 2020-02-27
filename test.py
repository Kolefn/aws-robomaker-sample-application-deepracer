DEG_TO_RAD = 3.1459 / 180.0

MAX_ROLL = 10.0 * DEG_TO_RAD
MAX_PITCH = 10.0 * DEG_TO_RAD
MAX_YAW_RATE = 45.0 * DEG_TO_RAD
MAX_THRUST = 30.0

normal_mins = [0, 0, 0, 0]
normal_maxs = [0, 0, 0, 1.0]
normal_incs = [1.0, 1.0, 1.0, 0.05]
normal_values = [[],[],[],[]]

for i in range(0, len(normal_values)):
    n_min = normal_mins[i]
    n_max = normal_maxs[i]
    inc = normal_incs[i]
    for j in range(0, int((n_max - n_min) / inc) + 1):
        normal_values[i].append(n_min+(j*inc))

from itertools import product

action_tuples = product(*normal_values)

actions = []
for action_tuple in action_tuples:
    action_list = list(action_tuple)
    roll = action_list[0] * MAX_ROLL
    pitch = action_list[1] * MAX_PITCH
    yaw_rate = action_list[2] * MAX_YAW_RATE
    thrust = action_list[3] * MAX_THRUST
    actions.append([roll, pitch, yaw_rate, thrust])

print(len(actions))