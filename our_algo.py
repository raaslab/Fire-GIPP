import numpy as np
import json
from astar import *
import math
import random
import datetime

EMPTY, TREE, FIRE, SMOKE = 0, 1, 2, 3


def update_search_area_probabilities(search_area, neighborhood, current_position, y_size, x_size, border, direction, spread):
    for dy, dx in neighborhood:
        if ((current_position[0] + dy) >= (y_size - border)) or ((current_position[0] + dy) < border) or (
                (current_position[1] + dx) >= (x_size - border)) or (
                (current_position[1] + dx) < border):
            continue
        if search_area[current_position[0] + dy, current_position[1] + dx] == 0:
            continue

        neighbour_direction = math.atan2(dy, dx)
        # neighbour_direction += math.pi
        neighbour_direction = math.degrees(neighbour_direction)
        if neighbour_direction < 0:
            neighbour_direction += 360

        angle_difference = (neighbour_direction - direction + 180) % 360 - 180
        if angle_difference < -180:
            angle_difference += 360
        if abs(angle_difference) > spread:
            continue
        search_area[current_position[0] + dy, current_position[1] + dx] = 0
    return search_area


def valfire(search_area, world, uav_initial_location, ny, nx, border_size, search_radius, wind_direction, search_direction, wind_angle):

    search_area_distances = np.zeros((ny, nx))
    for i in range(0, ny):
        for j in range(0, nx):
            if search_area[i, j] > 0:
                p = np.array((i, j))
                q = np.array((uav_initial_location[0], uav_initial_location[1]))
                distance = np.linalg.norm(p - q)
                search_area_distances[i, j] = distance

    closest_search_area_point = np.asarray(np.where( search_area_distances==np.min(search_area_distances[np.nonzero(search_area_distances)])))
    np.save('closest_search_area_point.npy', closest_search_area_point)
    maze = np.zeros((ny, nx))

    start = (uav_initial_location[0], uav_initial_location[1])
    if closest_search_area_point.size > 2:
        end = (closest_search_area_point[0, 0], closest_search_area_point[1, 0])

    else:
        end = (closest_search_area_point[0], closest_search_area_point[1])

    np.save('start_point.npy', start)
    np.save('end_point.npy', end)
    path = astar(maze, start, end)

    path_tracker = np.zeros((ny, nx))

    current_position = path[0]
    distance_traveled = 0
    astar_distance = 0
    validation_distance = 0
    localization_distance = 0

    neighbourhood = ()
    for i in range((-1 * 1), 1 + 1):
        for j in range((-1 * 1), 1 + 1):
            if i == 0 and j == 0:
                continue
            tup = (i, j)
            neighbourhood = neighbourhood + (tup, )

    probability_check_neighborhood = ()
    for i in range((-1 * search_radius), search_radius + 1):
        for j in range((-1 * search_radius), search_radius + 1):
            if i == 0 and j == 0:
                continue
            tup = (i, j)
            probability_check_neighborhood = probability_check_neighborhood + (tup, )

    validated = False
    located = False
    for item in path[1:]:
        p = np.array((current_position[0], current_position[1]))
        q = np.array((item[0], item[1]))
        distance_traveled += np.linalg.norm(p - q)
        path_tracker[item[0], item[1]] = 1
        current_position = item
        if world[item[0], item[1]] == FIRE:
            print('Fire located and validated! Total distance: %.2f' % distance_traveled)
            astar_distance = distance_traveled
            if not validated:
                validation_distance = distance_traveled
            localization_distance = distance_traveled
            return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance
        elif world[item[0], item[1]] == SMOKE or world[item[0], item[1]] == EMPTY:
            if not validated:
                print('Fire validated! Total distance: %.2f' % distance_traveled)
                validation_distance = distance_traveled
                validated = True
            continue
            #break

        '''
        for dy, dx in probability_check_neighborhood:
            if ((item[0] + dy) >= (ny - border_size)) or ((item[0] + dy) < border_size) or ((item[1] + dx) >= (nx - border_size)) or (
                    (item[1] + dx) < border_size):
                continue
            if search_area[item[0] + dy, item[1] + dx] == 0:
                continue
    
            if wind:
                neighbour_direction = math.atan2(dy, dx)
                # neighbour_direction += math.pi
                neighbour_direction = math.degrees(neighbour_direction)
                if neighbour_direction < 0:
                    neighbour_direction += 360
    
                angle_difference = (neighbour_direction - wind_direction + 180) % 360 - 180
                if angle_difference < -180:
                    angle_difference += 360
                if abs(angle_difference) > wind_angle:
                    continue
                search_area[item[0] + dy, item[1] + dx] = 0
        '''

    astar_distance = distance_traveled

    next_position = current_position
    while not validated:
        max_neighbor_probability = 0
        for dy, dx in neighbourhood:
            neighbor_probability = search_area[current_position[0] + dy, current_position[1] + dx]
            if neighbor_probability > max_neighbor_probability:
                max_neighbor_probability = neighbor_probability
                next_position = (current_position[0] + dy, current_position[1] + dx)
        if next_position == current_position:
            print('Stuck')

                    # break
        p = np.array((current_position[0], current_position[1]))
        q = np.array((next_position[0], next_position[1]))
        distance_traveled += np.linalg.norm(p - q)
        current_position = next_position
        path_tracker[current_position[0], current_position[1]] = 1
        if world[current_position[0], current_position[1]] == FIRE:
            print('Fire validated and located! Total distance: %.2f' % distance_traveled)
            validation_distance = distance_traveled
            localization_distance = distance_traveled
            return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance
        elif world[current_position[0], current_position[1]] == SMOKE:
            print('Fire validated! Total distance: %.2f' % distance_traveled)
            search_area[current_position[0], current_position[1]] = 0
            validated = True
            search_area = update_search_area_probabilities(search_area, probability_check_neighborhood, current_position, ny, nx, border_size, search_direction, wind_angle)
            '''
            for dy, dx in probability_check_neighborhood:
                if ((current_position[0] + dy) >= (ny - border_size)) or ((current_position[0] + dy) < border_size) or (
                        (current_position[1] + dx) >= (nx - border_size)) or (
                        (current_position[1] + dx) < border_size):
                    continue
                if search_area[current_position[0] + dy, current_position[1] + dx] == 0:
                    continue

                if wind:
                    neighbour_direction = math.atan2(dy, dx)
                    # neighbour_direction += math.pi
                    neighbour_direction = math.degrees(neighbour_direction)
                    if neighbour_direction < 0:
                        neighbour_direction += 360

                    angle_difference = (neighbour_direction - search_direction + 180) % 360 - 180
                    if angle_difference < -180:
                        angle_difference += 360
                    if abs(angle_difference) > wind_angle:
                        continue
                    search_area[current_position[0] + dy, current_position[1] + dx] = 0
            '''
        elif world[current_position[0], current_position[1]] == EMPTY:
            print('Fire validated! Total distance: %.2f' % distance_traveled)
            search_area[current_position[0], current_position[1]] = 0
            validated = True
            search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                           current_position, ny, nx, border_size, wind_direction,
                                                           wind_angle)
            '''
            for dy, dx in probability_check_neighborhood:
                if ((current_position[0] + dy) >= (ny - border_size)) or ((current_position[0] + dy) < border_size) or (
                        (current_position[1] + dx) >= (nx - border_size)) or (
                        (current_position[1] + dx) < border_size):
                    continue
                if search_area[current_position[0] + dy, current_position[1] + dx] == 0:
                    continue

                if wind:
                    neighbour_direction = math.atan2(dy, dx)
                    # neighbour_direction += math.pi
                    neighbour_direction = math.degrees(neighbour_direction)
                    if neighbour_direction < 0:
                        neighbour_direction += 360

                    angle_difference = (neighbour_direction - wind_direction + 180) % 360 - 180
                    if angle_difference < -180:
                        angle_difference += 360
                    if abs(angle_difference) > wind_angle:
                        continue
                    search_area[current_position[0] + dy, current_position[1] + dx] = 0
            '''
        else:
            search_area[current_position[0], current_position[1]] = 0
            search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                           current_position, ny, nx, border_size, wind_direction,
                                                           wind_angle)
            '''
            for dy, dx in probability_check_neighborhood:
                if ((current_position[0] + dy) >= (ny - border_size)) or ((current_position[0] + dy) < border_size) or (
                        (current_position[1] + dx) >= (nx - border_size)) or (
                        (current_position[1] + dx) < border_size):
                    continue
                if search_area[current_position[0] + dy, current_position[1] + dx] == 0:
                    continue
    
                if wind:
                    neighbour_direction = math.atan2(dy, dx)
                    # neighbour_direction += math.pi
                    neighbour_direction = math.degrees(neighbour_direction)
                    if neighbour_direction < 0:
                        neighbour_direction += 360
    
                    angle_difference = (neighbour_direction - wind_direction + 180) % 360 - 180
                    if angle_difference < -180:
                        angle_difference += 360
                    if abs(angle_difference) > wind_angle:
                        continue
                    search_area[current_position[0] + dy, current_position[1] + dx] = 0
            '''
        #print('Add code to validate fire once a* finished')

    validation_distance = distance_traveled


    while not located:
        max_neighbor_probability = 0
        for dy, dx in neighbourhood:
            neighbor_probability = search_area[current_position[0] + dy, current_position[1] + dx]
            if neighbor_probability > max_neighbor_probability:
                max_neighbor_probability = neighbor_probability
                next_position = (current_position[0] + dy, current_position[1] + dx)
        if next_position == current_position:
            print('Stuck')
            search_area_distances = np.zeros((ny, nx))
            for i in range(0, ny):
                for j in range(0, nx):
                    if search_area[i, j] > 0:
                        p = np.array((i, j))
                        q = np.array((uav_initial_location[0], uav_initial_location[1]))
                        distance = np.linalg.norm(p - q)
                        search_area_distances[i, j] = distance

            closest_search_area_point = np.asarray(np.where(search_area_distances == np.min(search_area_distances[np.nonzero(search_area_distances)])))
            start = (current_position[0], current_position[1])
            if closest_search_area_point.size > 2:
                end = (closest_search_area_point[0, 0], closest_search_area_point[1, 0])

            else:
                end = (closest_search_area_point[0], closest_search_area_point[1])

            path = astar(maze, start, end)
            for item in path[1:]:
                p = np.array((current_position[0], current_position[1]))
                q = np.array((item[0], item[1]))
                distance_traveled += np.linalg.norm(p - q)
                path_tracker[item[0], item[1]] = 1
                current_position = item
                if world[current_position[0], current_position[1]] == FIRE:
                    print('Fire located! Total distance: %.2f' % distance_traveled)
                    localization_distance = distance_traveled
                    return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance
                elif world[current_position[0], current_position[1]] == SMOKE:
                    search_area[current_position[0], current_position[1]] = 0
                    search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                                   current_position, ny, nx, border_size,
                                                                   search_direction,
                                                                   90)
                    '''
                    for dy, dx in probability_check_neighborhood:
                        if ((current_position[0] + dy) >= (ny - border_size)) or (
                                (current_position[0] + dy) < border_size) or (
                                (current_position[1] + dx) >= (nx - border_size)) or (
                                (current_position[1] + dx) < border_size):
                            continue
                        if search_area[current_position[0] + dy, current_position[1] + dx] == 0:
                            continue

                        if wind:
                            neighbour_direction = math.atan2(dy, dx)
                            # neighbour_direction += math.pi
                            neighbour_direction = math.degrees(neighbour_direction)
                            if neighbour_direction < 0:
                                neighbour_direction += 360

                            angle_difference = (neighbour_direction - search_direction + 180) % 360 - 180
                            if angle_difference < -180:
                                angle_difference += 360
                            if abs(angle_difference) > 90:
                                continue
                            search_area[current_position[0] + dy, current_position[1] + dx] = 0
                    '''
                elif world[current_position[0], current_position[1]] == EMPTY:
                    search_area[current_position[0], current_position[1]] = 0
                    search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                                   current_position, ny, nx, border_size,
                                                                   wind_direction,
                                                                   90)
                    '''
                    for dy, dx in probability_check_neighborhood:
                        if ((current_position[0] + dy) >= (ny - border_size)) or (
                                (current_position[0] + dy) < border_size) or (
                                (current_position[1] + dx) >= (nx - border_size)) or (
                                (current_position[1] + dx) < border_size):
                            continue
                        if search_area[current_position[0] + dy, current_position[1] + dx] == 0:
                            continue

                        if wind:
                            neighbour_direction = math.atan2(dy, dx)
                            # neighbour_direction += math.pi
                            neighbour_direction = math.degrees(neighbour_direction)
                            if neighbour_direction < 0:
                                neighbour_direction += 360

                            angle_difference = (neighbour_direction - wind_direction + 180) % 360 - 180
                            if angle_difference < -180:
                                angle_difference += 360
                            if abs(angle_difference) > 90:
                                continue
                            search_area[current_position[0] + dy, current_position[1] + dx] = 0
                    '''
                else:
                    search_area[current_position[0], current_position[1]] = 0
                    search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                                   current_position, ny, nx, border_size,
                                                                   wind_direction,
                                                                   wind_angle)
                    '''
                    for dy, dx in probability_check_neighborhood:
                        if ((current_position[0] + dy) >= (ny - border_size)) or (
                                (current_position[0] + dy) < border_size) or (
                                (current_position[1] + dx) >= (nx - border_size)) or (
                                (current_position[1] + dx) < border_size):
                            continue
                        if search_area[current_position[0] + dy, current_position[1] + dx] == 0:
                            continue

                        if wind:
                            neighbour_direction = math.atan2(dy, dx)
                            # neighbour_direction += math.pi
                            neighbour_direction = math.degrees(neighbour_direction)
                            if neighbour_direction < 0:
                                neighbour_direction += 360

                            angle_difference = (neighbour_direction - wind_direction + 180) % 360 - 180
                            if angle_difference < -180:
                                angle_difference += 360
                            if abs(angle_difference) > wind_angle:
                                continue
                            search_area[current_position[0] + dy, current_position[1] + dx] = 0
                    '''
            continue
        p = np.array((current_position[0], current_position[1]))
        q = np.array((next_position[0], next_position[1]))
        distance_traveled += np.linalg.norm(p - q)
        current_position = next_position
        path_tracker[current_position[0], current_position[1]] = 1
        if world[current_position[0], current_position[1]] == FIRE:
            print('Fire located! Total distance: %.2f' % distance_traveled)
            localization_distance = distance_traveled
            return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance
        elif world[current_position[0], current_position[1]] == SMOKE:
            search_area[current_position[0], current_position[1]] = 0
            search_area[current_position[0], current_position[1]] = 0
            search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                           current_position, ny, nx, border_size,
                                                           search_direction,
                                                           90)
            '''
            for dy, dx in probability_check_neighborhood:
                if ((current_position[0] + dy) >= (ny - border_size)) or ((current_position[0] + dy) < border_size) or (
                        (current_position[1] + dx) >= (nx - border_size)) or (
                        (current_position[1] + dx) < border_size):
                    continue
                if search_area[current_position[0] + dy, current_position[1] + dx] == 0:
                    continue

                if wind:
                    neighbour_direction = math.atan2(dy, dx)
                    # neighbour_direction += math.pi
                    neighbour_direction = math.degrees(neighbour_direction)
                    if neighbour_direction < 0:
                        neighbour_direction += 360

                    angle_difference = (neighbour_direction - search_direction + 180) % 360 - 180
                    if angle_difference < -180:
                        angle_difference += 360
                    if abs(angle_difference) > 90:
                        continue
                    search_area[current_position[0] + dy, current_position[1] + dx] = 0
            '''
        elif world[current_position[0], current_position[1]] == EMPTY:
            search_area[current_position[0], current_position[1]] = 0
            search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                           current_position, ny, nx, border_size,
                                                           wind_direction,
                                                           90)
            '''
            for dy, dx in probability_check_neighborhood:
                if ((current_position[0] + dy) >= (ny - border_size)) or ((current_position[0] + dy) < border_size) or (
                        (current_position[1] + dx) >= (nx - border_size)) or (
                        (current_position[1] + dx) < border_size):
                    continue
                if search_area[current_position[0] + dy, current_position[1] + dx] == 0:
                    continue

                if wind:
                    neighbour_direction = math.atan2(dy, dx)
                    # neighbour_direction += math.pi
                    neighbour_direction = math.degrees(neighbour_direction)
                    if neighbour_direction < 0:
                        neighbour_direction += 360

                    angle_difference = (neighbour_direction - wind_direction + 180) % 360 - 180
                    if angle_difference < -180:
                        angle_difference += 360
                    if abs(angle_difference) > 90:
                        continue
                    search_area[current_position[0] + dy, current_position[1] + dx] = 0
            '''
        else:
            search_area[current_position[0], current_position[1]] = 0
            search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                           current_position, ny, nx, border_size,
                                                           wind_direction,
                                                           wind_angle)
            '''
            for dy, dx in probability_check_neighborhood:
                if ((current_position[0] + dy) >= (ny - border_size)) or ((current_position[0] + dy) < border_size) or (
                        (current_position[1] + dx) >= (nx - border_size)) or (
                        (current_position[1] + dx) < border_size):
                    continue
                if search_area[current_position[0] + dy, current_position[1] + dx] == 0:
                    continue

                if wind:
                    neighbour_direction = math.atan2(dy, dx)
                    # neighbour_direction += math.pi
                    neighbour_direction = math.degrees(neighbour_direction)
                    if neighbour_direction < 0:
                        neighbour_direction += 360

                    angle_difference = (neighbour_direction - wind_direction + 180) % 360 - 180
                    if angle_difference < -180:
                        angle_difference += 360
                    if abs(angle_difference) > wind_angle:
                        continue
                    search_area[current_position[0] + dy, current_position[1] + dx] = 0
            '''

    return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance


