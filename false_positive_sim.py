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

fire_spread_rate = 10000000

burning = False
wind = True
wind_direction = 315
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

maze = np.zeros((ny, nx))
obstacle_y = random.randint(20, 65)
obstacle_y_offset = random.randint(10, 15)
obstacle_x = random.randint(20, 65)
obstacle_x_offset = random.randint(10, 15)
maze[obstacle_y:(obstacle_y + obstacle_y_offset), obstacle_x:(obstacle_x + obstacle_x_offset)] = 1


while True:
    fire_y = random.randint(border_size, ny - border_size - 1)
    fire_x = random.randint(border_size, nx - border_size - 1)
    if maze[fire_y, fire_x] == 0:
        X[fire_y, fire_x] = FIRE
        break

search_area = np.zeros((ny, nx))
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
                dist = np.linalg.norm(p-q)
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



#X[1:ny-1, 1:nx-1] = np.random.randint(0, 2, size=(ny-2, nx-2))

false_positive_sensor = random.randint(0, num_sensors)
sensor_smoke_detected[false_positive_sensor] = 1

search_area = find_search_area(sensor_locations, sensor_smoke_detected, velocity, ny, nx, border_size, num_sensors, search_neighbourhood, probability_angle_effect, probability_distance_effect, wind_direction, wind_angle, maze)

print('Sensor Detected Fire')
total_ours_distance = 0.0
total_ours_val_distance = 0.0
total_ours_loc_distance = 0.0
total_ours_astar_distance = 0.0
total_ours_time = 0.0
total_ours_val_astar_distance = 0.0
total_ours_loc_astar_distance = 0.0
total_ours_val_total_distance = 0.0

total_baseline_distance = 0.0
total_baseline_val_distance = 0.0
total_baseline_loc_distance = 0.0
total_baseline_astar_distance = 0.0
total_baseline_time = 0.0
total_baseline_val_astar_distance = 0.0
total_baseline_loc_astar_distance = 0.0
total_baseline_val_total_distance = 0.0

total_sensor_first_baseline_distance = 0.0
total_sensor_first_baseline_val_distance = 0.0
total_sensor_first_baseline_loc_distance = 0.0
total_sensor_first_baseline_astar_distance = 0.0
total_sensor_first_baseline_time = 0.0
total_sensor_first_baseline_val_astar_distance = 0.0
total_sensor_first_baseline_loc_astar_distance = 0.0
total_sensor_first_baseline_val_total_distance = 0.0

target_cells = np.transpose(np.asarray(np.nonzero(search_area)))
gtsp_tour = get_GTSP_tour(target_cells, search_area)

uav_initial_locations = []
for i in range(0, ny - border_size):
    uav_initial_locations.append([i + 1, border_size - 1])
    uav_initial_locations.append([i + 1, nx - border_size])
for i in range(0, nx - border_size):
    uav_initial_locations.append([border_size - 1, i + 1])
    uav_initial_locations.append([ny - border_size, i + 1])

uav_initial_locations = np.array(uav_initial_locations)
uav_initial_locations = np.unique(uav_initial_locations, axis=0)

count = 0
ours_val_count = 0
ours_loc_count = 0
ours_not_val_count = 0
ours_not_loc_count = 0

baseline_val_count = 0
baseline_loc_count = 0
baseline_not_val_count = 0
baseline_not_loc_count = 0

sensor_first_baseline_val_count = 0
sensor_first_baseline_loc_count = 0
sensor_first_baseline_not_val_count = 0
sensor_first_baseline_not_loc_count = 0

for uav_initial_location in uav_initial_locations:
    print("Run %d" % (count + 1))
    world = copy.deepcopy(Xz1)
    ours_start_time = datetime.datetime.now()
    ours_path_tracker, ours_run_distance, ours_validation_distance, ours_localization_distance, ours_astar_distance, ours_spread_time = valfire(copy.deepcopy(search_area), world, copy.deepcopy(X), uav_initial_location, ny, nx, border_size, search_radius, wind_direction, search_direction, wind_angle, copy.deepcopy(sensor_smoke_detected), sensor_locations, fire_spread_rate, maze)
    ours_end_time = datetime.datetime.now()
    ours_time_dif = ours_end_time - ours_start_time
    if ours_validation_distance > ours_astar_distance:
        total_ours_val_distance += ours_validation_distance
        total_ours_val_astar_distance += ours_astar_distance
        ours_val_count += 1
    if ours_localization_distance > ours_astar_distance:
        total_ours_loc_distance += ours_localization_distance
        total_ours_loc_astar_distance += ours_astar_distance
        ours_loc_count += 1
    if ours_validation_distance == 0:
        ours_not_val_count += 1
    if ours_localization_distance == 0:
        ours_not_loc_count += 1
    total_ours_distance += ours_run_distance
    total_ours_astar_distance += ours_astar_distance
    total_ours_val_total_distance += ours_validation_distance
    total_ours_time += ((ours_time_dif.total_seconds() - ours_spread_time) * 1000)

    world = copy.deepcopy(Xz1)
    baseline_start_time = datetime.datetime.now()
    baseline_path_tracker, baseline_run_distance, baseline_validation_distance, baseline_localization_distance, baseline_astar_distance, baseline_spread_time = baseline(
        world, copy.deepcopy(X), gtsp_tour, target_cells, uav_initial_location, ny, nx, border_size, wind_direction,
        wind_angle, copy.deepcopy(sensor_smoke_detected), sensor_locations, fire_spread_rate, maze)

    baseline_end_time = datetime.datetime.now()

    baseline_time_dif = baseline_end_time - baseline_start_time
    if baseline_validation_distance > baseline_astar_distance:
        total_baseline_val_distance += baseline_validation_distance
        total_baseline_val_astar_distance += baseline_astar_distance
        baseline_val_count += 1
    if baseline_localization_distance > baseline_astar_distance:
        total_baseline_loc_distance += baseline_localization_distance
        total_baseline_loc_astar_distance += baseline_astar_distance
        baseline_loc_count += 1
    if baseline_validation_distance == 0:
        baseline_not_val_count += 1
    if baseline_localization_distance == 0:
        baseline_not_loc_count += 1

    total_baseline_val_total_distance += baseline_validation_distance
    total_baseline_distance += baseline_run_distance
    total_baseline_astar_distance += baseline_astar_distance
    total_baseline_time += ((baseline_time_dif.total_seconds() - baseline_spread_time) * 1000)

    world = copy.deepcopy(Xz1)
    sensor_first_baseline_start_time = datetime.datetime.now()
    sensor_first_baseline_path_tracker, sensor_first_baseline_run_distance, sensor_first_baseline_validation_distance, sensor_first_baseline_localization_distance, sensor_first_baseline_astar_distance, sensor_first_baseline_spread_time = baseline_goto_sensor(
        world, copy.deepcopy(X), gtsp_tour, target_cells, uav_initial_location, ny, nx, border_size, wind_direction,
        wind_angle, copy.deepcopy(sensor_smoke_detected), sensor_locations, fire_spread_rate, maze)

    sensor_first_baseline_end_time = datetime.datetime.now()

    sensor_first_baseline_time_dif = sensor_first_baseline_end_time - sensor_first_baseline_start_time
    if sensor_first_baseline_validation_distance > sensor_first_baseline_astar_distance:
        total_sensor_first_baseline_val_distance += sensor_first_baseline_validation_distance
        total_sensor_first_baseline_val_astar_distance += sensor_first_baseline_astar_distance
        sensor_first_baseline_val_count += 1
    if sensor_first_baseline_localization_distance > sensor_first_baseline_astar_distance:
        total_sensor_first_baseline_loc_distance += sensor_first_baseline_localization_distance
        total_sensor_first_baseline_loc_astar_distance += sensor_first_baseline_astar_distance
        sensor_first_baseline_loc_count += 1
    if sensor_first_baseline_validation_distance == 0:
        sensor_first_baseline_not_val_count += 1
    if sensor_first_baseline_localization_distance == 0:
        sensor_first_baseline_not_loc_count += 1

    total_sensor_first_baseline_val_total_distance += sensor_first_baseline_validation_distance
    total_sensor_first_baseline_distance += sensor_first_baseline_run_distance
    total_sensor_first_baseline_astar_distance += sensor_first_baseline_astar_distance
    total_sensor_first_baseline_time += (
                (sensor_first_baseline_time_dif.total_seconds() - sensor_first_baseline_spread_time) * 1000)

    count += 1

total_ours_distance /= count
total_ours_astar_distance /= count
total_ours_val_total_distance /= count
if ours_val_count > 0:
    total_ours_val_distance /= ours_val_count
    total_ours_val_astar_distance /= ours_val_count
if ours_loc_count > 0:
    total_ours_loc_distance /= ours_loc_count
    total_ours_loc_astar_distance /= ours_loc_count
total_ours_time /= count

total_baseline_distance /= count
total_baseline_astar_distance /= count
total_baseline_val_total_distance /= count
if baseline_val_count > 0:
    total_baseline_val_distance /= baseline_val_count
    total_baseline_val_astar_distance /= baseline_val_count
if baseline_loc_count > 0:
    total_baseline_loc_distance /= baseline_loc_count
    total_baseline_loc_astar_distance /= baseline_loc_count
total_baseline_time /= count
total_baseline_time += 460

total_sensor_first_baseline_distance /= count
total_sensor_first_baseline_astar_distance /= count
total_sensor_first_baseline_val_total_distance /= count
if sensor_first_baseline_val_count > 0:
    total_sensor_first_baseline_val_distance /= sensor_first_baseline_val_count
    total_sensor_first_baseline_val_astar_distance /= sensor_first_baseline_val_count
if sensor_first_baseline_loc_count > 0:
    total_sensor_first_baseline_loc_distance /= sensor_first_baseline_loc_count
    total_sensor_first_baseline_loc_astar_distance /= sensor_first_baseline_loc_count
total_sensor_first_baseline_time /= count
total_sensor_first_baseline_time += 460

results = [wind_direction, 0, fire_spread_rate, count, ours_val_count, ours_loc_count, ours_not_val_count, ours_not_loc_count, total_ours_time, total_ours_distance, total_ours_val_total_distance, total_ours_astar_distance, total_ours_val_distance, total_ours_val_astar_distance, total_ours_loc_distance, total_ours_loc_astar_distance,
           baseline_val_count, baseline_loc_count, baseline_not_val_count, baseline_not_loc_count, total_baseline_time, total_baseline_distance, total_baseline_val_total_distance, total_baseline_astar_distance, total_baseline_val_distance, total_baseline_val_astar_distance, total_baseline_loc_distance, total_baseline_loc_astar_distance,
           sensor_first_baseline_val_count, sensor_first_baseline_loc_count, sensor_first_baseline_not_val_count, sensor_first_baseline_not_loc_count, total_sensor_first_baseline_time, total_sensor_first_baseline_distance, total_sensor_first_baseline_val_total_distance, total_sensor_first_baseline_astar_distance, total_sensor_first_baseline_val_distance, total_sensor_first_baseline_val_astar_distance, total_sensor_first_baseline_loc_distance, total_sensor_first_baseline_loc_astar_distance]

with open('results_false_positives.csv', 'a') as f_object:
    writer_object = writer(f_object)

    writer_object.writerow(results)
    f_object.close()

print('Results: Ours vs Baseline')
print('Total Distance Comparison: [%.2f, %.2f]' % (total_ours_distance, total_baseline_distance))
print('Total Validation Distance Comparison: [%.2f, %.2f]' % (total_ours_val_total_distance, total_baseline_val_total_distance))
print('Total A* Distance Comparison: [%.2f, %.2f]' % (total_ours_astar_distance, total_baseline_astar_distance))
print('Total Time Comparison: [%.0f, %.0f]' % (total_ours_time, total_baseline_time))
print('Non-Early Total Validation Distance Comparison: [%.2f, %.2f]' % (total_ours_val_distance, total_baseline_val_distance))
print('Non-Early Total Localization Distance Comparison: [%.2f, %.2f]' % (total_ours_loc_distance, total_baseline_loc_distance))
print('Non-Early Relative Validation Distance Comparison: [%.2f, %.2f]' % ((total_ours_val_distance - total_ours_val_astar_distance), (total_baseline_val_distance - total_baseline_val_astar_distance)))
print('Non-Early Relative Localization Distance Comparison: [%.2f, %.2f]' % ((total_ours_loc_distance - total_ours_loc_astar_distance), (total_baseline_loc_distance - total_baseline_loc_astar_distance)))
print('Validation Count: [%d, %d]' % (ours_val_count, baseline_val_count))
print('Localization Count: [%d, %d]' % (ours_loc_count, baseline_loc_count))
print('Validation misses: [%d, %d]' % (ours_not_val_count, baseline_not_val_count))
print('Localization misses: [%d, %d]' % (ours_not_loc_count, baseline_not_loc_count))
print('Done')