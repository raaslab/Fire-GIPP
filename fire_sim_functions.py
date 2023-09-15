import numpy as np
import math
import os
import json
from astar import *
import random
import datetime
import copy

EMPTY, TREE, FIRE, SMOKE = 0, 1, 2, 3

neighbourhood = ()
for i in range((-1 * 1), 1 + 1):
    for j in range((-1 * 1), 1 + 1):
        if i == 0 and j == 0:
            continue
        tup = (i, j)
        neighbourhood = neighbourhood + (tup,)

smoke_spread_radius = 7
smoke_neighbourhood = ()
for i in range((-1 * smoke_spread_radius), smoke_spread_radius + 1):
    for j in range((-1 * smoke_spread_radius), smoke_spread_radius + 1):
        if i == 0 and j == 0:
            continue
        tup = (i, j)
        smoke_neighbourhood = smoke_neighbourhood + (tup, )


def iterate(X, velocity, ny, nx, border_size, neighbourhood, wind_direction, wind_angle, maze):
    """Iterate the forest according to the forest-fire rules."""

    # The boundary of the forest is always empty, so only consider cells
    # indexed from 1 to nx-2, 1 to ny-2

    for i in range(0, velocity):
        X1 = np.zeros((ny, nx))
        for ix in range(border_size, nx-border_size):
            for iy in range(border_size, ny-border_size):
            #    if X[iy, ix] == EMPTY and np.random.random() <= p:
            #        X1[iy, ix] = TREE
                if X[iy, ix] == TREE:
                    X1[iy, ix] = TREE
                    if maze[iy, ix] == 0:
                        for dy,dx in neighbourhood:

                            neighbour_direction = math.atan2(dy, dx)
                            #neighbour_direction += math.pi
                            neighbour_direction = math.degrees(neighbour_direction)
                            if neighbour_direction < 0:
                                neighbour_direction += 360
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
        X = X1
    return X1


def smoke_iterate(X, velocity, ny, nx, border_size, smoke_neighbourhood, wind_direction, wind_angle, maze):
    """Iterate the forest according to the forest-fire rules."""

    # The boundary of the forest is always empty, so only consider cells
    # indexed from 1 to nx-2, 1 to ny-2
    for i in range(0, velocity):
        X1 = np.zeros((ny, nx))
        for ix in range(border_size, nx-border_size):
            for iy in range(border_size, ny-border_size):
                X1[iy, ix] = X[iy, ix]
                if X[iy, ix] == TREE and maze[iy, ix] == 0:
                    for dy, dx in smoke_neighbourhood:
                        neighbour_direction = math.atan2(dy, dx)
                        #neighbour_direction += math.pi
                        neighbour_direction = math.degrees(neighbour_direction)
                        if neighbour_direction < 0:
                            neighbour_direction += 360
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
    return X1


def find_search_area(sensor_locations, sensor_smoke_detected, velocity, ny, nx, border_size, num_sensors, search_neighbourhood, probability_angle_effect, probability_distance_effect, wind_direction, wind_angle, maze):
    """Iterate the forest according to the forest-fire rules."""

    # The boundary of the forest is always empty, so only consider cells
    # indexed from 1 to nx-2, 1 to ny-2

    for i in range(0, velocity):
        X1 = np.zeros((ny, nx))
        for i in range(0, num_sensors):
            if sensor_smoke_detected[i] > 0:
                y = int(sensor_locations[i][0])
                x = int(sensor_locations[i][1])
                X1[y, x] = 1
                for dy, dx in search_neighbourhood:
                    if ((y + dy) >= (ny - border_size)) or ((y + dy) < border_size) or ((x + dx) >= (nx - border_size)) or ((x + dx) < border_size) or maze[y + dy, x + dx] == 1:
                        continue
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

                    angle_rad_difference = abs(math.radians(angle_difference))
                    p = np.array((y, x))
                    q = np.array((y + dy, x + dx))
                    distance = np.linalg.norm(p - q)
                    if distance < 1:
                        distance = 1

                    X1[y + dy, x + dx] = 1 / ((1 + (probability_angle_effect * abs(angle_difference) / wind_angle)) * probability_distance_effect * distance)

    return X1


def update_search_area_probabilities(search_area, neighborhood, current_position, y_size, x_size, border, direction,
                                     spread):
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


def valfire(search_area, world, smokeless_world, uav_initial_location, ny, nx, border_size, search_radius, wind_direction,
            search_direction, wind_angle, sensor_smoke_detected, sensor_locations, fire_spread_rate, maze):
    original_search_area = copy.deepcopy(search_area)
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

    path_tracker = np.zeros((ny, nx))

    current_position = path[0]
    distance_traveled = 0
    astar_distance = 0
    validation_distance = 0
    localization_distance = 0

    probability_check_neighborhood = ()
    for i in range((-1 * search_radius), search_radius + 1):
        for j in range((-1 * search_radius), search_radius + 1):
            if i == 0 and j == 0:
                continue
            tup = (i, j)
            probability_check_neighborhood = probability_check_neighborhood + (tup,)

    validated = False
    located = False
    iteration_count = 0
    for item in path[1:]:
        iteration_count += 1
        if iteration_count % fire_spread_rate == 0:
            print("Fire spreading")
            start_time = datetime.datetime.now()
            smokeless_world = iterate(smokeless_world, 1, ny, nx, border_size, neighbourhood, wind_direction, wind_angle, maze)
            world = smoke_iterate(smokeless_world, 1, ny, nx, border_size, smoke_neighbourhood, wind_direction, wind_angle, maze)
            current_sensors_smoke_detected = np.sum(sensor_smoke_detected)
            for j in range(0, sensor_locations.shape[0]):
                sensor_y = int(sensor_locations[j][0])
                sensor_x = int(sensor_locations[j][1])
                if world[sensor_y][sensor_x] == SMOKE:
                    sensor_smoke_detected[j] = 1
            if np.sum(sensor_smoke_detected) > current_sensors_smoke_detected:
                print('Update search area')
            end_time = datetime.datetime.now()
            spread_time += ((end_time - start_time).total_seconds())
            if world[item[0], item[1]] == FIRE:
                print('Fire located and validated! Total distance: %.2f' % distance_traveled)
                astar_distance = distance_traveled
                if not validated:
                    validation_distance = distance_traveled
                localization_distance = distance_traveled
                return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
            elif world[item[0], item[1]] == SMOKE or world[item[0], item[1]] == EMPTY:
                if not validated:
                    print('Fire validated! Total distance: %.2f' % distance_traveled)
                    validation_distance = distance_traveled
                    validated = True
            elif world[item[0], item[1]] == TREE:
                for sensor_index in np.where(sensor_smoke_detected == 1):
                    sensor_index = sensor_index[0]
                    if item[0] == sensor_locations[sensor_index][0] and item[1] == sensor_locations[sensor_index][1]:
                        print('False positive detected')
                        astar_distance = distance_traveled
                        if not validated:
                            validation_distance = distance_traveled
                        localization_distance = distance_traveled
                        return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time

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
            return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
        elif world[item[0], item[1]] == SMOKE or world[item[0], item[1]] == EMPTY:
            if not validated:
                print('Fire validated! Total distance: %.2f' % distance_traveled)
                validation_distance = distance_traveled
                validated = True
            continue
            # break
        elif world[item[0], item[1]] == TREE:
            for sensor_index in np.where(sensor_smoke_detected == 1):
                sensor_index = sensor_index[0]
                if item[0] == sensor_locations[sensor_index][0] and item[1] == sensor_locations[sensor_index][1]:
                    print('False positive detected')
                    astar_distance = distance_traveled
                    if not validated:
                        validation_distance = distance_traveled
                    localization_distance = distance_traveled
                    return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area

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
        iteration_count += 1
        if iteration_count % fire_spread_rate == 0:
            print("Fire spreading")
            start_time = datetime.datetime.now()
            smokeless_world = iterate(smokeless_world, 1, ny, nx, border_size, neighbourhood, wind_direction,
                                      wind_angle, maze)
            world = smoke_iterate(smokeless_world, 1, ny, nx, border_size, smoke_neighbourhood, wind_direction,
                                  wind_angle, maze)
            current_sensors_smoke_detected = np.sum(sensor_smoke_detected)
            for j in range(0, sensor_locations.shape[0]):
                sensor_y = int(sensor_locations[j][0])
                sensor_x = int(sensor_locations[j][1])
                if world[sensor_y][sensor_x] == SMOKE:
                    sensor_smoke_detected[j] = 1
            if np.sum(sensor_smoke_detected) > current_sensors_smoke_detected:
                print('Update search area')
            end_time = datetime.datetime.now()
            spread_time += ((end_time - start_time).total_seconds())
            if world[current_position[0], current_position[1]] == FIRE:
                print('Fire validated and located! Total distance: %.2f' % distance_traveled)
                validation_distance = distance_traveled
                localization_distance = distance_traveled
                return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
            elif world[current_position[0], current_position[1]] == SMOKE:
                print('Fire validated! Total distance: %.2f' % distance_traveled)
                search_area[current_position[0], current_position[1]] = 0
                validated = True
                search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                               current_position, ny, nx, border_size, search_direction,
                                                               wind_angle)
                break
            elif world[current_position[0], current_position[1]] == EMPTY:
                print('Fire validated! Total distance: %.2f' % distance_traveled)
                search_area[current_position[0], current_position[1]] = 0
                validated = True
                search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                               current_position, ny, nx, border_size, wind_direction,
                                                               wind_angle)
                break
            else:
                for sensor_index in np.where(sensor_smoke_detected == 1):
                    sensor_index = sensor_index[0]
                    if current_position[0] == sensor_locations[sensor_index][0] and current_position[1] == sensor_locations[sensor_index][1]:
                        print('False positive detected')
                        validation_distance = distance_traveled
                        localization_distance = distance_traveled
                        return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
                search_area[current_position[0], current_position[1]] = 0
                search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                               current_position, ny, nx, border_size, wind_direction,
                                                               wind_angle)


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
            if np.sum(search_area_distances) == 0:
                return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area

            closest_search_area_point = np.asarray(
                np.where(search_area_distances == np.min(search_area_distances[np.nonzero(search_area_distances)])))
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
                    print('Fire validated and located! Total distance: %.2f' % distance_traveled)
                    validation_distance = distance_traveled
                    localization_distance = distance_traveled
                    return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
                elif world[current_position[0], current_position[1]] == SMOKE:
                    print('Fire validated! Total distance: %.2f' % distance_traveled)
                    search_area[current_position[0], current_position[1]] = 0
                    validated = True
                    search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                                   current_position, ny, nx, border_size,
                                                                   search_direction,
                                                                   wind_angle)
                elif world[current_position[0], current_position[1]] == EMPTY:
                    print('Fire validated! Total distance: %.2f' % distance_traveled)
                    search_area[current_position[0], current_position[1]] = 0
                    validated = True
                    search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                                   current_position, ny, nx, border_size,
                                                                   wind_direction,
                                                                   wind_angle)
                else:
                    for sensor_index in np.where(sensor_smoke_detected == 1):
                        sensor_index = sensor_index[0]
                        if current_position[0] == sensor_locations[sensor_index][0] and current_position[1] == \
                                sensor_locations[sensor_index][1]:
                            print('False positive detected')
                            validation_distance = distance_traveled
                            localization_distance = distance_traveled
                            return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
                    search_area[current_position[0], current_position[1]] = 0
                    search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                                   current_position, ny, nx, border_size,
                                                                   wind_direction,
                                                                   wind_angle)
            continue
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
            return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
        elif world[current_position[0], current_position[1]] == SMOKE:
            print('Fire validated! Total distance: %.2f' % distance_traveled)
            search_area[current_position[0], current_position[1]] = 0
            validated = True
            search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                           current_position, ny, nx, border_size, search_direction,
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
            for sensor_index in np.where(sensor_smoke_detected == 1):
                sensor_index = sensor_index[0]
                if current_position[0] == sensor_locations[sensor_index][0] and current_position[1] == \
                        sensor_locations[sensor_index][1]:
                    print('False positive detected')
                    validation_distance = distance_traveled
                    localization_distance = distance_traveled
                    return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
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
        # print('Add code to validate fire once a* finished')

    validation_distance = distance_traveled

    while not located:
        iteration_count += 1
        if iteration_count % fire_spread_rate == 0:
            print("Fire spreading")
            start_time = datetime.datetime.now()
            smokeless_world = iterate(smokeless_world, 1, ny, nx, border_size, neighbourhood, wind_direction,
                                      wind_angle, maze)
            world = smoke_iterate(smokeless_world, 1, ny, nx, border_size, smoke_neighbourhood, wind_direction,
                                  wind_angle, maze)
            current_sensors_smoke_detected = np.sum(sensor_smoke_detected)
            for j in range(0, sensor_locations.shape[0]):
                sensor_y = int(sensor_locations[j][0])
                sensor_x = int(sensor_locations[j][1])
                if world[sensor_y][sensor_x] == SMOKE:
                    sensor_smoke_detected[j] = 1
            if np.sum(sensor_smoke_detected) > current_sensors_smoke_detected:
                print('Update search area')
            end_time = datetime.datetime.now()
            spread_time += ((end_time - start_time).total_seconds())
            if world[current_position[0], current_position[1]] == FIRE:
                print('Fire located! Total distance: %.2f' % distance_traveled)
                localization_distance = distance_traveled
                return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
            elif world[current_position[0], current_position[1]] == SMOKE:
                search_area[current_position[0], current_position[1]] = 0
                search_area[current_position[0], current_position[1]] = 0
                search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                               current_position, ny, nx, border_size,
                                                               search_direction,
                                                               90)
            elif world[current_position[0], current_position[1]] == EMPTY:
                search_area[current_position[0], current_position[1]] = 0
                search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                               current_position, ny, nx, border_size,
                                                               wind_direction,
                                                               90)
            else:
                for sensor_index in np.where(sensor_smoke_detected == 1):
                    sensor_index = sensor_index[0]
                    if current_position[0] == sensor_locations[sensor_index][0] and current_position[1] == \
                            sensor_locations[sensor_index][1]:
                        print('False positive detected')
                        localization_distance = distance_traveled
                        return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
                search_area[current_position[0], current_position[1]] = 0
                search_area = update_search_area_probabilities(search_area, probability_check_neighborhood,
                                                               current_position, ny, nx, border_size,
                                                               wind_direction,
                                                               wind_angle)

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
            if np.sum(search_area_distances) == 0:
                return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area

            closest_search_area_point = np.asarray(
                np.where(search_area_distances == np.min(search_area_distances[np.nonzero(search_area_distances)])))
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
                    return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
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
                    for sensor_index in np.where(sensor_smoke_detected == 1):
                        sensor_index = sensor_index[0]
                        if current_position[0] == sensor_locations[sensor_index][0] and current_position[1] == \
                                sensor_locations[sensor_index][1]:
                            print('False positive detected')
                            localization_distance = distance_traveled
                            return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
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
            return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
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
            for sensor_index in np.where(sensor_smoke_detected == 1):
                sensor_index = sensor_index[0]
                if current_position[0] == sensor_locations[sensor_index][0] and current_position[1] == \
                        sensor_locations[sensor_index][1]:
                    print('False positive detected')
                    localization_distance = distance_traveled
                    return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area
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

    return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time, search_area


def baseline(world, smokeless_world, gtsp_tour, target_cells, uav_initial_location, ny, nx, border_size, wind_direction, wind_angle, sensor_smoke_detected, sensor_locations, fire_spread_rate, maze):

    min_distance = 100000
    if gtsp_tour is None:
        print('Should not get here')
    for i in range(0, gtsp_tour.shape[0]):
        p = np.array((uav_initial_location[0], uav_initial_location[1]))
        grid_index = gtsp_tour[i]
        q = np.array((target_cells[grid_index][0], target_cells[grid_index][1]))
        distance = np.linalg.norm(p - q)
        if distance < min_distance:
            min_distance = distance
            gtsp_shift = i

    gtsp_tour = np.roll(gtsp_tour, -gtsp_shift)
    spread_time = 0.0
    #maze = np.zeros((ny, nx))
    distance_traveled = 0
    validation_distance = 0
    localization_distance = 0
    astar_distance = 0
    path_tracker = np.zeros((ny, nx))

    current_position = uav_initial_location
    validated = False
    first_tour = True
    iteration_count = 0
    if gtsp_tour is None:
        print('Should not get here')
    for i in range(0, gtsp_tour.shape[0]):
        index = gtsp_tour[i]
        start = (current_position[0], current_position[1])
        end = (target_cells[index][0], target_cells[index][1])
        path = astar(maze, start, end)

        for item in path[1:]:
            iteration_count += 1
            if iteration_count % fire_spread_rate == 0:
                print("Fire spreading")
                start_time = datetime.datetime.now()
                smokeless_world = iterate(smokeless_world, 1, ny, nx, border_size, neighbourhood, wind_direction,
                                          wind_angle, maze)
                world = smoke_iterate(smokeless_world, 1, ny, nx, border_size, smoke_neighbourhood, wind_direction,
                                      wind_angle, maze)
                current_sensors_smoke_detected = np.sum(sensor_smoke_detected)

                for j in range(0, sensor_locations.shape[0]):
                    sensor_y = int(sensor_locations[j][0])
                    sensor_x = int(sensor_locations[j][1])
                    if world[sensor_y][sensor_x] == SMOKE:
                        sensor_smoke_detected[j] = 1
                if np.sum(sensor_smoke_detected) > current_sensors_smoke_detected:
                    print('Update search area')
                end_time = datetime.datetime.now()
                spread_time += ((end_time - start_time).total_seconds())
                if world[current_position[0], current_position[1]] == FIRE:
                    localization_distance = distance_traveled
                    if first_tour:
                        astar_distance = distance_traveled
                    if not validated:
                        validation_distance = distance_traveled
                        print('Fire validated! Total distance: %.2f' % validation_distance)
                    print('Fire located! Total distance: %.2f' % localization_distance)
                    return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time
                elif not validated and (world[current_position[0], current_position[1]] == SMOKE or world[
                    current_position[0], current_position[1]] == EMPTY):
                    validated = True
                    validation_distance = distance_traveled
                    print('Fire validated! Total distance: %.2f' % validation_distance)
                elif world[current_position[0], current_position[1]] == TREE:
                    for sensor_index in np.where(sensor_smoke_detected == 1):
                        sensor_index = sensor_index[0]
                        if current_position[0] == sensor_locations[sensor_index][0] and current_position[1] == \
                                sensor_locations[sensor_index][1]:
                            print('False positive detected')
                            localization_distance = distance_traveled
                            if first_tour:
                                astar_distance = distance_traveled
                            if not validated:
                                validation_distance = distance_traveled
                            return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time

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
                return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time
            elif not validated and (world[current_position[0], current_position[1]] == SMOKE or world[current_position[0], current_position[1]] == EMPTY):
                validated = True
                validation_distance = distance_traveled
                print('Fire validated! Total distance: %.2f' % validation_distance)
            elif world[current_position[0], current_position[1]] == TREE:
                for sensor_index in np.where(sensor_smoke_detected == 1):
                    sensor_index = sensor_index[0]
                    if current_position[0] == sensor_locations[sensor_index][0] and current_position[1] == \
                            sensor_locations[sensor_index][1]:
                        print('False positive detected')
                        localization_distance = distance_traveled
                        if first_tour:
                            astar_distance = distance_traveled
                        if not validated:
                            validation_distance = distance_traveled
                        return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time
        if first_tour:
            astar_distance = distance_traveled
            first_tour = False

    return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time


def baseline_goto_sensor(world, smokeless_world, gtsp_tour, target_cells, uav_initial_location, ny, nx, border_size, wind_direction, wind_angle, sensor_smoke_detected, sensor_locations, fire_spread_rate, maze):

    min_distance = 100000
    if gtsp_tour is None:
        print('Should not get here')
    for j in range(0, sensor_smoke_detected.shape[0]):
        if sensor_smoke_detected[j] == 1:
            for i in range(0, gtsp_tour.shape[0]):
                p = np.array((uav_initial_location[0], uav_initial_location[1]))
                grid_index = gtsp_tour[i]
                if sensor_locations[j][0] == target_cells[grid_index][0] and sensor_locations[j][1] == target_cells[grid_index][1]:
                    q = np.array((target_cells[grid_index][0], target_cells[grid_index][1]))
                    distance = np.linalg.norm(p - q)
                    if distance < min_distance:
                        min_distance = distance
                        gtsp_shift = i
                        print('Found sensor location in GTSP tour')

    gtsp_tour = np.roll(gtsp_tour, -gtsp_shift)
    spread_time = 0.0
    #maze = np.zeros((ny, nx))
    distance_traveled = 0
    validation_distance = 0
    localization_distance = 0
    astar_distance = 0
    path_tracker = np.zeros((ny, nx))

    current_position = uav_initial_location
    validated = False
    first_tour = True
    iteration_count = 0
    if gtsp_tour is None:
        print('Should not get here')
    for i in range(0, gtsp_tour.shape[0]):
        index = gtsp_tour[i]
        start = (current_position[0], current_position[1])
        end = (target_cells[index][0], target_cells[index][1])
        path = astar(maze, start, end)

        for item in path[1:]:
            iteration_count += 1
            if iteration_count % fire_spread_rate == 0:
                print("Fire spreading")
                start_time = datetime.datetime.now()
                smokeless_world = iterate(smokeless_world, 1, ny, nx, border_size, neighbourhood, wind_direction,
                                          wind_angle, maze)
                world = smoke_iterate(smokeless_world, 1, ny, nx, border_size, smoke_neighbourhood, wind_direction,
                                      wind_angle, maze)
                current_sensors_smoke_detected = np.sum(sensor_smoke_detected)

                for j in range(0, sensor_locations.shape[0]):
                    sensor_y = int(sensor_locations[j][0])
                    sensor_x = int(sensor_locations[j][1])
                    if world[sensor_y][sensor_x] == SMOKE:
                        sensor_smoke_detected[j] = 1
                if np.sum(sensor_smoke_detected) > current_sensors_smoke_detected:
                    print('Update search area')
                end_time = datetime.datetime.now()
                spread_time += ((end_time - start_time).total_seconds())
                if world[current_position[0], current_position[1]] == FIRE:
                    localization_distance = distance_traveled
                    if first_tour:
                        astar_distance = distance_traveled
                    if not validated:
                        validation_distance = distance_traveled
                        print('Fire validated! Total distance: %.2f' % validation_distance)
                    print('Fire located! Total distance: %.2f' % localization_distance)
                    return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time
                elif not validated and (world[current_position[0], current_position[1]] == SMOKE or world[
                    current_position[0], current_position[1]] == EMPTY):
                    validated = True
                    validation_distance = distance_traveled
                    print('Fire validated! Total distance: %.2f' % validation_distance)
                elif world[current_position[0], current_position[1]] == TREE:
                    for sensor_index in np.where(sensor_smoke_detected == 1):
                        sensor_index = sensor_index[0]
                        if current_position[0] == sensor_locations[sensor_index][0] and current_position[1] == \
                                sensor_locations[sensor_index][1]:
                            print('False positive detected')
                            localization_distance = distance_traveled
                            if first_tour:
                                astar_distance = distance_traveled
                            if not validated:
                                validation_distance = distance_traveled
                            return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time

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
                return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time
            elif not validated and (world[current_position[0], current_position[1]] == SMOKE or world[current_position[0], current_position[1]] == EMPTY):
                validated = True
                validation_distance = distance_traveled
                print('Fire validated! Total distance: %.2f' % validation_distance)
            elif world[current_position[0], current_position[1]] == TREE:
                for sensor_index in np.where(sensor_smoke_detected == 1):
                    sensor_index = sensor_index[0]
                    if current_position[0] == sensor_locations[sensor_index][0] and current_position[1] == \
                            sensor_locations[sensor_index][1]:
                        print('False positive detected')
                        localization_distance = distance_traveled
                        if first_tour:
                            astar_distance = distance_traveled
                        if not validated:
                            validation_distance = distance_traveled
                        return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time
        if first_tour:
            astar_distance = distance_traveled
            first_tour = False

    return path_tracker, distance_traveled, validation_distance, localization_distance, astar_distance, spread_time


def get_GTSP_tour(target_cells, search_area):
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

    os.system("/home/user/catkin_ws/src/gtsp/src/GLNScmd.jl /home/user/Code/WildFire/temp.gtsp -mode=fast -output=/home/user/Code/WildFire/tour.txt")

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
    return gtsp_tour