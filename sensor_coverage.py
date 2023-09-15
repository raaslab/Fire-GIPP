import numpy as np
import copy
import math
import random
import json
from fire_sim_functions import *
from csv import writer

# Displacements from a cell to its eight nearest neighbours
parameter_dict = {}
EMPTY, TREE, FIRE, SMOKE = 0, 1, 2, 3
velocity = 1
num_sensors = 50
sensor_distance = 5
border_size = 2

fire_spread_rate = 25

burning = False
wind = True
wind_direction = 90
search_direction = 180 + wind_direction
if search_direction >= 360:
    search_direction -= 360
wind_angle = 60
wind_edge_difference = 35
radius = 1
smoke_spread_radius = 7
search_radius = smoke_spread_radius + 1

probability_distance_effect = 1.1
probability_angle_effect = 0

forest_fraction = 1
# Probability of new tree growth per empty cell, and of lightning strike.
p, f = 0.0, 0.0001
# Forest size (number of cells in x and y directions).
nx, ny = 100, 100

parameter_dict['num_sensors'] = num_sensors
parameter_dict['sensor_distance'] = sensor_distance
parameter_dict['border_size'] = border_size
parameter_dict['wind_direction'] = wind_direction
parameter_dict['wind_angle'] = wind_angle
parameter_dict['wind_edge_difference'] = wind_edge_difference
parameter_dict['radius'] = radius
parameter_dict['smoke_spread_radius'] = smoke_spread_radius
parameter_dict['search_radius'] = search_radius
parameter_dict['probability_distance_effect'] = probability_distance_effect
parameter_dict['probability_angle_effect'] = probability_angle_effect
parameter_dict['forest_fraction'] = forest_fraction
parameter_dict['p'] = p
parameter_dict['f'] = f
parameter_dict['nx'] = nx
parameter_dict['ny'] = ny


neighbourhood = ()
for i in range((-1 * radius), radius + 1):
    for j in range((-1 * radius), radius + 1):
        if i == 0 and j == 0:
            continue
        tup = (i, j)
        neighbourhood = neighbourhood + (tup, )

smoke_neighbourhood = ()
for i in range((-1 * smoke_spread_radius), smoke_spread_radius + 1):
    for j in range((-1 * smoke_spread_radius), smoke_spread_radius + 1):
        if i == 0 and j == 0:
            continue
        tup = (i, j)
        smoke_neighbourhood = smoke_neighbourhood + (tup, )


count = 0

results_list = []
results_detected_list = []
for num_sensors in range(1, 201):
    print("Sensors Deployed %d" % num_sensors)
    data_list = []
    num_detected = 0
    for iteration in range(0, 10):
        print("Iteration %d" % (iteration+1))
        X = np.zeros((ny, nx))
        Xz1 = np.zeros((ny, nx))
        X[border_size:ny - border_size, border_size:nx - border_size] = np.random.random(
            size=(ny - (border_size * 2), nx - (border_size * 2))) < forest_fraction
        Xz1[border_size:ny - border_size, border_size:nx - border_size] = np.random.random(
            size=(ny - (border_size * 2), nx - (border_size * 2))) < forest_fraction
        maze = np.zeros((ny, nx))

        fire_y = 50
        fire_x = 50
        X[fire_y, fire_x] = FIRE

        sensor_coverage_map = np.zeros((ny, nx))

        sensor_locations = np.zeros((num_sensors, 2))
        for i in range(0, num_sensors):
            while True:
                y = random.randint(border_size, (ny - border_size - 1))
                x = random.randint(border_size, (nx - border_size - 1))
                if y != fire_y and x != fire_x and maze[y, x] == 0:
                    break
            if i > 0:
                while True:
                    sensor_not_near_others = True
                    for j in range(0, i):
                        p = sensor_locations[j]
                        q = [y, x]
                        dist = np.linalg.norm(p - q)
                        if dist < sensor_distance:
                            sensor_not_near_others = False
                            break

                    if sensor_not_near_others:
                        break
                    while True:
                        y = random.randint(border_size, (ny - border_size - 1))
                        x = random.randint(border_size, (nx - border_size - 1))
                        if y != fire_y and x != fire_x and maze[y, x] == 0:
                            break

            sensor_locations[i][0] = y
            sensor_locations[i][1] = x
            sensor_coverage_map[y, x] = 1

        sensor_burned = np.zeros(num_sensors)
        sensor_smoke_detected = np.zeros(num_sensors)

        # X[1:ny-1, 1:nx-1] = np.random.randint(0, 2, size=(ny-2, nx-2))

        burning = False
        stop = False
        previous_sum = np.sum(X)
        count = 0
        previous_sensors_smoke_detected = 0
        detected = False
        for i in range(0, 100):
            #print("Iteration: %d" % (i + 1))
            if stop:
                break
            X = iterate(X, velocity, ny, nx, border_size, neighbourhood, wind_direction, wind_angle, maze)
            for j in range(0, num_sensors):
                sensor_y = int(sensor_locations[j][0])
                sensor_x = int(sensor_locations[j][1])
                if X[sensor_y][sensor_x] == FIRE:
                    sensor_burned[j] = 1
            Xz1 = smoke_iterate(X, velocity, ny, nx, border_size, smoke_neighbourhood, wind_direction, wind_angle, maze)
            for j in range(0, num_sensors):
                sensor_y = int(sensor_locations[j][0])
                sensor_x = int(sensor_locations[j][1])
                if Xz1[sensor_y][sensor_x] == SMOKE:
                    sensor_smoke_detected[j] = 1

            current_sensors_smoke_detected = np.sum(sensor_smoke_detected)
            if current_sensors_smoke_detected > previous_sensors_smoke_detected:
                detected = True
                break
            previous_sensors_smoke_detected = current_sensors_smoke_detected

            #if np.sum(sensor_burned) > 0:
                #print("%d sensor(s) burned down" % np.sum(sensor_burned))

            if not burning:
                if previous_sum != np.sum(X):
                    burning = True
            else:
                if previous_sum == np.sum(X):
                    count += 1
            if count == 4:
                stop = True
            #print("Sum: %d" % np.sum(X))
            # print("Smoke Sum: %d" % np.sum(Xz1))
            previous_sum = np.sum(X)

        if detected:
            num_detected += 1
        borderless_grid = X[border_size:ny - border_size, border_size:nx - border_size]
        number_burned = np.count_nonzero(borderless_grid == EMPTY)
        number_burned += np.count_nonzero(borderless_grid == FIRE)
        data_list.append(number_burned)
        count += 1
    results_list.append(data_list)
    results_detected_list.append(num_detected)

num_results = np.asarray(results_list)
num_results_detected = np.asarray(results_detected_list)
np.save('results_sensor_coverage.npy', num_results)
np.save('results_sensor_coverage_detected.npy', num_results_detected)

print('Done')