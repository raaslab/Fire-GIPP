import numpy as np
from matplotlib import pyplot as plt
from matplotlib import colors
import random
from fire_sim_functions import *
import copy

EMPTY, TREE, FIRE, SMOKE = 0, 1, 2, 3
velocity = 1
num_sensors = 50
sensor_distance = 5
border_size = 2

fire_spread_rate = 100000000000

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
maze = np.zeros((ny, nx))

X = np.load('smokeless_world.npy')
Xz1 = np.load('world.npy')
sensor_coverage_map = np.load('sensor_map.npy')
sensor_locations = np.load('sensor_locations.npy')
search_area = np.load('search_area.npy')
sensor_smoke_detected = np.load('sensor_smoke_detected.npy')

#Xz1[border_size:ny-border_size, border_size:nx-border_size] = np.random.random(size=(ny-(border_size*2), nx-(border_size*2))) < forest_fraction

path_tracker = np.zeros((ny, nx))
for i in range(0, ny):
    for j in range(0, nx):
        path_tracker[i, j] = np.nan
        #if search_area[i, j] > 0:
        #    Xz1[i, j] = SEARCH
        #else:
         #   search_area[i, j] = np.nan

for i in range(0, sensor_smoke_detected.shape[0]):
    y = int(sensor_locations[i][0])
    x = int(sensor_locations[i][1])
    #if sensor_smoke_detected[i] == 1:
     #   Xz1[y, x] = ALERT
    #else:
     #   Xz1[y, x] = SENSOR

uav_initial_location = [98, 30]
world = copy.deepcopy(Xz1)
ours_path_tracker, ours_run_distance, ours_validation_distance, ours_localization_distance, ours_astar_distance, ours_spread_time, ours_search_area = valfire(copy.deepcopy(search_area), world, copy.deepcopy(X), uav_initial_location, ny, nx, border_size, search_radius, wind_direction, search_direction, wind_angle, copy.deepcopy(sensor_smoke_detected), sensor_locations, fire_spread_rate, maze)
'''
search_area_distances = np.zeros((ny, nx))
spread_time = 0.0
for i in range(0, ny):
    for j in range(0, nx):
        if search_area[i, j] > 0:
            p = np.array((i, j))
            q = np.array((uav_initial_location[0], uav_initial_location[1]))
            distance = np.linalg.norm(p - q)
            search_area_distances[i, j] = distance

closest_search_area_point = np.asarray(
    np.where(search_area_distances == np.min(search_area_distances[np.nonzero(search_area_distances)])))
np.save('closest_search_area_point.npy', closest_search_area_point)
#maze = np.zeros((ny, nx))

start = (uav_initial_location[0], uav_initial_location[1])
if closest_search_area_point.size > 2:
    end = (closest_search_area_point[0, 0], closest_search_area_point[1, 0])

else:
    end = (closest_search_area_point[0], closest_search_area_point[1])

np.save('start_point.npy', start)
np.save('end_point.npy', end)
path = astar(maze, start, end)


for node in path:
    y = node[0]
    x = node[1]
    path_tracker[y, x] = Xz1[y, x]
'''
colors_list = ['green', 'gray', 'orange', 'gray']
cmap = colors.ListedColormap(colors_list)

view = np.zeros((ny, nx))
for i in range(0, ny):
    for j in range(0, nx):
        view[i, j] = np.nan
        if ours_path_tracker[i, j] > 0:
            view[i, j] = Xz1[i, j]

orig_map = plt.cm.get_cmap('hot')
reversed_map = orig_map.reversed()
#plt.imshow(Xz1, cmap=cmap)
plt.imshow(ours_search_area, cmap=orig_map)
plt.imshow(view, cmap=cmap)
plt.show()

print('Done')
