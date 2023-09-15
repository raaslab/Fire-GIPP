import numpy as np
import json
import random

wind_direction = 45
world = np.load(str(wind_direction) + '/world.npy')
search_area = np.load(str(wind_direction) + '/search_area.npy')
sensor_locations = np.load(str(wind_direction) + '/sensor_locations.npy')
sensor_map = np.load(str(wind_direction) + '/sensor_map.npy')

with open(str(wind_direction) + "/parameters.txt", "r") as fp:
    # Load the dictionary from the file
    parameters_dict = json.load(fp)

border_size = parameters_dict['border_size']
nx = parameters_dict['nx']
ny = parameters_dict['ny']

uav_initial_y = 0
uav_initial_x = 0

rand_axis = random.randint(0,1)
if rand_axis == 0:
    uav_initial_y = random.choice([(border_size - 1), (ny - border_size)])
    uav_initial_x = random.randint(border_size, (nx - 1 - border_size))
elif rand_axis == 1:
    uav_initial_x = random.choice([(border_size - 1), (nx - border_size)])
    uav_initial_y = random.randint(border_size, (ny - 1 - border_size))
else:
    print("Error, shouldn't get here")

uav_initial_location = np.array(([uav_initial_y, uav_initial_x]))
np.save(str(wind_direction) + '/uav_initial_location.npy', uav_initial_location)

print('Done')