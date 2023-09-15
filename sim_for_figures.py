import numpy as np
from matplotlib import pyplot as plt
from matplotlib import colors
import random
from fire_sim_functions import *

velocity = 1
num_sensors = 50
sensor_distance = 5
border_size = 2

fire_spread_rate = 25

burning = False
wind = True
wind_direction = 0
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


search_neighbourhood = ()
for i in range((-1 * search_radius), search_radius + 1):
    for j in range((-1 * search_radius), search_radius + 1):
        if i == 0 and j == 0:
            continue
        tup = (i, j)
        search_neighbourhood = search_neighbourhood + (tup, )

X = np.zeros((ny, nx))
Xz1 = np.zeros((ny, nx))
X[border_size:ny-border_size, border_size:nx-border_size] = np.random.random(size=(ny-(border_size*2), nx-(border_size*2))) < forest_fraction
Xz1[border_size:ny-border_size, border_size:nx-border_size] = np.random.random(size=(ny-(border_size*2), nx-(border_size*2))) < forest_fraction
fire_y = 50
fire_x = 50

X[fire_y, fire_x] = FIRE

search_area = np.zeros((ny, nx))

sensor_locations = np.zeros((num_sensors, 2))
sensor_coverage_map = np.zeros((ny, nx))

for i in range(0, num_sensors):
    while True:
        y = random.randint(border_size, (ny - border_size - 1))
        x = random.randint(border_size, (nx - border_size - 1))
        if y != fire_y and x != fire_x:
            break
    if i > 0:
        while True:
            sensor_not_near_others = True
            for j in range(0, i):
                p = sensor_locations[j]
                q = [y, x]
                dist = np.linalg.norm(p-q)
                if dist < sensor_distance:
                    sensor_not_near_others = False
                    break

            if sensor_not_near_others:
                break
            while True:
                y = random.randint(border_size, (ny - border_size - 1))
                x = random.randint(border_size, (nx - border_size - 1))
                if y != fire_y and x != fire_x:
                    break

    sensor_locations[i][0] = y
    sensor_locations[i][1] = x
    sensor_coverage_map[y, x] = 1
    '''
    X[y, x + 1] = SENSOR
    X[y, x - 1] = SENSOR
    X[y - 1, x - 1] = SENSOR
    X[y - 1, x] = SENSOR
    X[y - 1, x + 1] = SENSOR
    X[y + 1, x - 1] = SENSOR
    X[y + 1, x] = SENSOR
    X[y + 1, x + 1] = SENSOR
    '''

burning = False
stop = False
previous_sum = np.sum(X)
count = 0
previous_sensors_smoke_detected = 0
detected = False

sensor_burned = np.zeros(num_sensors)
sensor_smoke_detected = np.zeros(num_sensors)
maze = np.zeros((ny, nx))

for i in range(0, 100):
    print("Iteration: %d" % (i+1))
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
        print("%d sensor(s) detecting smoke" % current_sensors_smoke_detected)
        search_area = find_search_area(sensor_locations, sensor_smoke_detected, velocity, ny, nx, border_size, num_sensors, search_neighbourhood, probability_angle_effect, probability_distance_effect, wind_direction, wind_angle, maze)
        if np.sum(search_area) == 0:
            print('Search area 0, exiting')
            exit()
        np.save('smokeless_world.npy', X)
        np.save('world.npy', Xz1)
        np.save('sensor_map.npy', sensor_coverage_map)
        np.save('sensor_locations.npy', sensor_locations)
        np.save('search_area.npy', search_area)
        np.save('sensor_smoke_detected.npy', sensor_smoke_detected)
        detected = True
        break
    previous_sensors_smoke_detected = current_sensors_smoke_detected

    if np.sum(sensor_burned) > 0:
        print("%d sensor(s) burned down" % np.sum(sensor_burned))

    if not burning:
        if previous_sum != np.sum(X):
            burning = True
    else:
        if previous_sum == np.sum(X):
            count += 1
    if count == 10:
        stop = True
    print("Sum: %d" % np.sum(X))
    #print("Smoke Sum: %d" % np.sum(Xz1))
    previous_sum = np.sum(X)

if not detected:
    print("No sensor detected fire, exiting")
    exit()