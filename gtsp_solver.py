import numpy as np
import os
from astar import *
import json
import random
import datetime

EMPTY, TREE, FIRE, SMOKE = 0, 1, 2, 3

#uav_initial_location = np.load(str(sim_config)+'/uav_initial_location.npy')



'''
dimension = target_cells.shape[0]
y = search_area[target_cells[0][0], target_cells[0][1]]
# visiting_cells = 3
f = open("temp.gtsp", "w")
f.write("NAME: Temp\n")
f.write("TYPE: AGTSP\n")
f.write("COMMENT: temporary GTSP file\n")
f.write("DIMENSION: %d\n" % (dimension))
f.write("GTSP_SETS: %d\n" % dimension)
f.write("EDGE_WEIGHT_TYPE: EXPLICIT\n")
f.write("EDGE_WEIGHT_FORMAT: FULL_MATRIX\n")
f.write("EDGE_WEIGHT_SECTION\n")

count = 0

for i in range(0, dimension):
    for j in range(0, dimension):
        if i == j:
            f.write("999999 ")
        else:
            p = np.array((target_cells[i][0], target_cells[i][1]))
            q = np.array((target_cells[j][0], target_cells[j][1]))
            distance = np.linalg.norm(p - q)
            rounded_edge_cost = int(distance * 100)
            f.write("%d " % rounded_edge_cost)
    f.write("\n")

f.write("GTSP_SET_SECTION:\n")

for i in range(0, dimension):
    f.write("%d %d -1\n" % ((i + 1), (i + 1)))

f.close()

os.system(
    "/home/user/catkin_ws/src/gtsp/src/GLNScmd.jl /home/user/Code/WildFire/temp.gtsp -mode=fast -output=/home/user/Code/WildFire/tour.txt")

gtsp_tour_file = open('tour.txt', 'r')
Lines = gtsp_tour_file.readlines()

for line in Lines:
    x = line.split(':')
    if x[0].replace(' ', '') == "Tour":
        x[1] = x[1].replace(' ', '')[1:-1]
        tour = x[1].split(',')

gtsp_tour = np.asarray(tour)
gtsp_tour = gtsp_tour.astype(int)
gtsp_tour -= 1
'''


def baseline(world, gtsp_tour, target_cells, uav_initial_location, ny, nx):

    min_distance = 100000
    for i in range(0, gtsp_tour.shape[0]):
        p = np.array((uav_initial_location[0], uav_initial_location[1]))
        grid_index = gtsp_tour[i]
        q = np.array((target_cells[grid_index][0], target_cells[grid_index][1]))
        distance = np.linalg.norm(p - q)
        if distance < min_distance:
            min_distance = distance
            gtsp_shift = i

    gtsp_tour = np.roll(gtsp_tour, -gtsp_shift)

    maze = np.zeros((ny, nx))
    distance_traveled = 0
    validation_distance = 0
    localization_distance = 0
    astar_distance = 0
    path_tracker = np.zeros((ny, nx))

    current_position = uav_initial_location
    validated = False
    first_tour = True
    for i in range(0, gtsp_tour.shape[0]):
        index = gtsp_tour[i]
        start = (current_position[0], current_position[1])
        end = (target_cells[index][0], target_cells[index][1])
        path = astar(maze, start, end)

        for item in path[1:]:
            p = np.array((current_position[0], current_position[1]))
            q = np.array((item[0], item[1]))
            distance_traveled += np.linalg.norm(p - q)
            path_tracker[item[0], item[1]] = 1
            current_position = item
            if world[current_position[0], current_position[1]] == FIRE:
                localization_distance = distance_traveled
                if first_tour:
                    astar_distance = distance_traveled
                if not validated:
                    validation_distance = distance_traveled
                    print('Fire validated! Total distance: %.2f' % validation_distance)
                print('Fire located! Total distance: %.2f' % localization_distance)
                return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance
            elif not validated and (world[current_position[0], current_position[1]] == SMOKE or world[current_position[0], current_position[1]] == EMPTY):
                validated = True
                validation_distance = distance_traveled
                print('Fire validated! Total distance: %.2f' % validation_distance)
        if first_tour:
            astar_distance = distance_traveled
            first_tour = False
