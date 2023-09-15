#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import colors
import math
import random
import json

# Create a forest fire animation based on a simple cellular automaton model.
# The maths behind this code is described in the scipython blog article
# at https://scipython.com/blog/the-forest-fire-model/
# Christian Hill, January 2016.
# Updated January 2020.

# Displacements from a cell to its eight nearest neighbours
parameter_dict = {}
EMPTY, TREE, FIRE, SMOKE = 0, 1, 2, 3
velocity = 1
num_sensors = 50
sensor_distance = 5
border_size = 2

burning = False
wind = True
wind_direction = 200
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


search_neighbourhood = ()
for i in range((-1 * search_radius), search_radius + 1):
    for j in range((-1 * search_radius), search_radius + 1):
        if i == 0 and j == 0:
            continue
        tup = (i, j)
        search_neighbourhood = search_neighbourhood + (tup, )

'''
neighbourhood = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1),
                 (-2, 2), (-1, 2), (0, 2), (1, 2), (2, 2),
                 (-2, -1), (-2, 0), (-2, 1), (2, -1), (2, 0), (2, 1),
                 (-2, -2), (-1, -2), (0, -2), (1, -2), (2, -2))
'''


# Colours for visualization: brown for EMPTY, dark green for TREE and orange
# for FIRE. Note that for the colormap to work, this list and the bounds list
# must be one larger than the number of different values in the array.

#colors_list = [(0.2,0,0), (0,0.5,0), (1,0,0), 'orange']
#cmap = colors.ListedColormap(colors_list)
#bounds = [0,1,2,3]
#norm = colors.BoundaryNorm(bounds, cmap.N)



#search_direction = 180 + wind_direction
#if search_direction >= 360:
#    search_direction -= 360


def iterate(X):
    """Iterate the forest according to the forest-fire rules."""

    # The boundary of the forest is always empty, so only consider cells
    # indexed from 1 to nx-2, 1 to ny-2
    global wind

    for i in range(0, velocity):
        X1 = np.zeros((ny, nx))
        for ix in range(border_size, nx-border_size):
            for iy in range(border_size, ny-border_size):
            #    if X[iy, ix] == EMPTY and np.random.random() <= p:
            #        X1[iy, ix] = TREE
                if X[iy, ix] == TREE:
                    X1[iy, ix] = TREE
                    for dy,dx in neighbourhood:

                        if wind:
                            neighbour_direction = math.atan2(dy, dx)
                            #neighbour_direction += math.pi
                            neighbour_direction = math.degrees(neighbour_direction)
                            if neighbour_direction < 0:
                                neighbour_direction += 360
                            global wind_angle
                            global wind_direction
                            angle_difference = (neighbour_direction - wind_direction + 180) % 360 - 180
                            if angle_difference < -180:
                                angle_difference += 360
                            if abs(angle_difference) > wind_angle:
                                continue


                        # The diagonally-adjacent trees are further away, so
                        # only catch fire with a reduced probability:
                        #if abs(dx) == abs(dy) and np.random.random() < 0.573:
                        #    continue
                        #if ((wind_angle - abs(angle_difference)) < wind_edge_difference) and np.random.random() < 0.573:
                        #    continue
                        if X[iy+dy, ix+dx] == FIRE:
                            X1[iy, ix] = FIRE
                            break
                    else:
                        global burning
                        if np.random.random() <= f and not burning:
                            burning = True
                            X1[iy, ix] = FIRE
        X = X1
    return X1


def smoke_iterate(X, Xz1):
    """Iterate the forest according to the forest-fire rules."""

    # The boundary of the forest is always empty, so only consider cells
    # indexed from 1 to nx-2, 1 to ny-2
    global wind

    for i in range(0, velocity):
        X1 = np.zeros((ny, nx))
        for ix in range(border_size, nx-border_size):
            for iy in range(border_size, ny-border_size):
                X1[iy, ix] = X[iy, ix]
                if X[iy, ix] == TREE:
                    for dy, dx in smoke_neighbourhood:

                        if wind:
                            neighbour_direction = math.atan2(dy, dx)
                            #neighbour_direction += math.pi
                            neighbour_direction = math.degrees(neighbour_direction)
                            if neighbour_direction < 0:
                                neighbour_direction += 360
                            global wind_angle
                            global wind_direction
                            angle_difference = (neighbour_direction - wind_direction + 180) % 360 - 180
                            if angle_difference < -180:
                                angle_difference += 360
                            if abs(angle_difference) > wind_angle:
                                continue


                        # The diagonally-adjacent trees are further away, so
                        # only catch fire with a reduced probability:
                        #if abs(dx) == abs(dy) and np.random.random() < 0.573:
                        #    continue
                        #if ((wind_angle - abs(angle_difference)) < wind_edge_difference) and np.random.random() < 0.573:
                        #    continue
                        if (iy+dy) >= 100 or (ix+dx) >= 100:
                            continue
                        if X[iy+dy, ix+dx] == FIRE:
                            X1[iy, ix] = SMOKE
                            break
                        #if Xz1[iy+dy, ix+dx] == SMOKE:
                        #    X1[iy, ix] = SMOKE
                        #    break
        Xz1 = X1
    return X1



def find_search_area(sensor_locations, sensor_smoke_detected, sensor_burned):
    """Iterate the forest according to the forest-fire rules."""

    # The boundary of the forest is always empty, so only consider cells
    # indexed from 1 to nx-2, 1 to ny-2
    global wind

    for i in range(0, velocity):
        X1 = np.zeros((ny, nx))
        for i in range(0, num_sensors):
            if sensor_smoke_detected[i] > 0:
                y = int(sensor_locations[i][0])
                x = int(sensor_locations[i][1])
                X1[y, x] = 1
                for dy, dx in search_neighbourhood:
                    if ((y + dy) >= (ny - border_size)) or ((y + dy) < border_size) or ((x + dx) >= (nx - border_size)) or ((x + dx) < border_size):
                        continue
                    if wind:
                        neighbour_direction = math.atan2(dy, dx)
                        # neighbour_direction += math.pi
                        neighbour_direction = math.degrees(neighbour_direction)
                        if neighbour_direction < 0:
                            neighbour_direction += 360
                        global wind_angle
                        global wind_direction
                        angle_difference = (neighbour_direction - wind_direction + 180) % 360 - 180
                        if angle_difference < -180:
                            angle_difference += 360
                        if abs(angle_difference) > wind_angle:
                            continue

                        angle_rad_difference = abs(math.radians(angle_difference))
                        p = np.array((y, x))
                        q = np.array((y + dy, x + dx))
                        distance = np.linalg.norm(p - q)
                        if distance < 1:
                            distance = 1

                        X1[y + dy, x + dx] = 1 / ((1 + (probability_angle_effect * abs(angle_difference) / wind_angle)) * probability_distance_effect * distance)

    return X1


# The initial fraction of the forest occupied by trees.

# Initialize the forest grid.
X = np.zeros((ny, nx))
Xz1 = np.zeros((ny, nx))
search_area = np.zeros((ny, nx))
sensor_coverage_map = np.zeros((ny, nx))

sensor_locations = np.zeros((num_sensors, 2))
for i in range(0, num_sensors):
    y = random.randint(border_size, (ny - border_size))
    x = random.randint(border_size, (nx - border_size))
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
            y = random.randint(border_size, (ny - border_size))
            x = random.randint(border_size, (nx - border_size))

    sensor_locations[i][0] = y
    sensor_locations[i][1] = x
    sensor_coverage_map[y, x] = 1

sensor_burned = np.zeros(num_sensors)
sensor_smoke_detected = np.zeros(num_sensors)



#X[1:ny-1, 1:nx-1] = np.random.randint(0, 2, size=(ny-2, nx-2))
X[border_size:ny-border_size, border_size:nx-border_size] = np.random.random(size=(ny-(border_size*2), nx-(border_size*2))) < forest_fraction
Xz1[border_size:ny-border_size, border_size:nx-border_size] = np.random.random(size=(ny-(border_size*2), nx-(border_size*2))) < forest_fraction

burning = False
stop = False
previous_sum = np.sum(X)
count = 0
previous_sensors_smoke_detected = 0
for i in range(0, 100):
    print("Iteration: %d" % (i+1))
    if stop:
        break
    X = iterate(X)
    for j in range(0, num_sensors):
        sensor_y = int(sensor_locations[j][0])
        sensor_x = int(sensor_locations[j][1])
        if X[sensor_y][sensor_x] == FIRE:
            sensor_burned[j] = 1
    Xz1 = smoke_iterate(X, Xz1)
    for j in range(0, num_sensors):
        sensor_y = int(sensor_locations[j][0])
        sensor_x = int(sensor_locations[j][1])
        if Xz1[sensor_y][sensor_x] == SMOKE:
            sensor_smoke_detected[j] = 1

    current_sensors_smoke_detected = np.sum(sensor_smoke_detected)
    if current_sensors_smoke_detected > previous_sensors_smoke_detected:
        print("%d sensor(s) detecting smoke" % current_sensors_smoke_detected)
        search_area = find_search_area(sensor_locations, sensor_smoke_detected, sensor_burned)
        np.save(str(wind_direction) + '/world.npy', Xz1)
        np.save(str(wind_direction) + '/sensor_map.npy', sensor_coverage_map)
        np.save(str(wind_direction) + '/sensor_locations.npy', sensor_locations)
        np.save(str(wind_direction) + '/search_area.npy', search_area)
        with open(str(wind_direction) + "/parameters.txt", "w") as fp:
            json.dump(parameter_dict, fp)
        exit()
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

print('Done')