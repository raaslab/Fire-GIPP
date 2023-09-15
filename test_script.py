from our_algo import *
from gtsp_solver import *

sim_config = 200

def get_GTSP_tour(target_cells):
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



with open(str(sim_config) + "/parameters.txt", "r") as fp:
    # Load the dictionary from the file
    parameters_dict = json.load(fp)

border_size = parameters_dict['border_size']
nx = parameters_dict['nx']
ny = parameters_dict['ny']
search_radius = parameters_dict['search_radius']
wind_direction = parameters_dict['wind_direction']
search_direction = 180 + wind_direction
if search_direction >= 360:
    search_direction -= 360

wind_angle = parameters_dict['wind_angle']

search_area = np.load(str(sim_config) + '/search_area.npy')
world = np.load(str(sim_config) + '/world.npy')
target_cells = np.transpose(np.asarray(np.nonzero(search_area)))
gtsp_tour = get_GTSP_tour(target_cells)

total_ours_distance = 0.0
total_ours_val_distance = 0.0
total_ours_loc_distance = 0.0
total_ours_time = 0.0
total_ours_val_astar_distance = 0.0
total_ours_loc_astar_distance = 0.0
total_baseline_distance = 0.0
total_baseline_val_distance = 0.0
total_baseline_loc_distance = 0.0
total_baseline_time = 0.0
total_baseline_val_astar_distance = 0.0
total_baseline_loc_astar_distance = 0.0

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
baseline_val_count = 0
baseline_loc_count = 0
for uav_initial_location in uav_initial_locations:
    print("Run %d" % (count+1))
    search_area = np.load(str(sim_config) + '/search_area.npy')
    world = np.load(str(sim_config) + '/world.npy')
    ours_start_time = datetime.datetime.now()
    ours_path_tracker, ours_run_distance, ours_validation_distance, ours_localization_distance, ours_astar_distance = valfire(search_area, world, uav_initial_location, ny, nx, border_size, search_radius, wind_direction, search_direction, wind_angle)
    ours_end_time = datetime.datetime.now()
    ours_time_dif = ours_end_time - ours_start_time
    if ours_validation_distance > ours_astar_distance:
        total_ours_val_distance += (ours_validation_distance - ours_astar_distance)
        total_ours_val_astar_distance += ours_astar_distance
        ours_val_count += 1
    if ours_localization_distance > ours_astar_distance:
        total_ours_loc_distance += (ours_localization_distance - ours_astar_distance)
        total_ours_loc_astar_distance += ours_astar_distance
        ours_loc_count += 1
    total_ours_distance += ours_run_distance
    total_ours_time += (ours_time_dif.microseconds / 1000)

    baseline_start_time = datetime.datetime.now()
    baseline_path_tracker, baseline_run_distance, baseline_validation_distance, baseline_localization_distance, baseline_astar_distance = baseline(world, gtsp_tour, target_cells, uav_initial_location, ny, nx)
    baseline_end_time = datetime.datetime.now()

    baseline_time_dif = baseline_end_time - baseline_start_time
    if baseline_validation_distance > baseline_astar_distance:
        total_baseline_val_distance += (baseline_validation_distance - baseline_astar_distance)
        total_baseline_val_astar_distance += baseline_astar_distance
        baseline_val_count += 1
    if baseline_localization_distance > baseline_astar_distance:
        total_baseline_loc_distance += (baseline_localization_distance - baseline_astar_distance)
        total_baseline_loc_astar_distance += baseline_astar_distance
        baseline_loc_count += 1

    total_baseline_distance += baseline_run_distance
    total_baseline_time += (baseline_time_dif.microseconds / 1000)

    count += 1

total_ours_distance /= count
total_ours_val_distance /= ours_val_count
total_ours_loc_distance /= ours_loc_count
total_ours_time /= count

total_baseline_distance /= count
total_baseline_val_distance /= baseline_val_count
total_baseline_loc_distance /= baseline_loc_count
total_baseline_time /= count
total_baseline_time += 460

print('Results: Ours vs Baseline')
print('Total Distance Comparison: [%.2f, %.2f]' % (total_ours_distance, total_baseline_distance))
print('Total Time Comparison: [%.0f, %.0f]' % (total_ours_time, total_baseline_time))
print('Validation Distance Comparison: [%.2f, %.2f]' % (total_ours_val_distance, total_baseline_val_distance))
print('Localization Distance Comparison: [%.2f, %.2f]' % (total_ours_loc_distance, total_baseline_loc_distance))
print('Validation Count: [%d, %d]' % (ours_val_count, baseline_val_count))
print('Localization Count: [%d, %d]' % (ours_loc_count, baseline_loc_count))

print('Done')
