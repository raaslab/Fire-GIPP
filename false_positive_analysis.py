import numpy as np

results = np.loadtxt('results_false_positives.csv', delimiter=',', skiprows=1)

total_runs = np.sum(results[:, 3])
non_early_validations = np.sum(results[:, 4])

ours_total_distance = np.mean(results[:, 9])
baseline_total_distance = np.mean(results[:, 21])
sensor_first_baseline_total_distance = np.mean(results[:, 33])

ours_time = np.mean(results[:, 8])
baseline_time = np.mean(results[:, 20])
sensor_first_baseline_time = np.mean(results[:, 32])

ours_non_early_distance = results[:, 12]
temp = results[:, 4]
ours_non_early_distance = np.multiply(ours_non_early_distance, temp)
ours_non_early_distance /= np.sum(temp)
ours_non_early_distance = np.sum(ours_non_early_distance)

baseline_non_early_distance = results[:, 24]
temp = results[:, 16]
baseline_non_early_distance = np.multiply(baseline_non_early_distance, temp)
baseline_non_early_distance /= np.sum(temp)
baseline_non_early_distance = np.sum(baseline_non_early_distance)

ours_non_early_distance_relative = results[:, 12] - results[:, 13]
temp = results[:, 4]
ours_non_early_distance_relative = np.multiply(ours_non_early_distance_relative, temp)
ours_non_early_distance_relative /= np.sum(temp)
ours_non_early_distance_relative = np.sum(ours_non_early_distance_relative)

baseline_non_early_distance_relative = results[:, 24] - results[:, 25]
temp = results[:, 16]
baseline_non_early_distance_relative = np.multiply(baseline_non_early_distance_relative, temp)
baseline_non_early_distance_relative /= np.sum(temp)
baseline_non_early_distance_relative = np.sum(baseline_non_early_distance_relative)

print('Total Runs: %d' % total_runs)
print('Non-Early Validations: %d' % non_early_validations)
print('Results: [Ours, Baseline, Sensor First Baseline]')
print('Total Computational Time (ms): [%.2f, %.2f, %.2f]' % (ours_time, baseline_time, sensor_first_baseline_time))
print('Total Distance: [%.2f, %.2f, %.2f]' % (ours_total_distance, baseline_total_distance, sensor_first_baseline_total_distance))
print('Non-Early Total Distance: [%.2f, %.2f, N/A]' % (ours_non_early_distance, baseline_non_early_distance))
print('Non-Early Relative Distance: [%.2f, %.2f, N/A]' % (ours_non_early_distance_relative, baseline_non_early_distance_relative))
print('Done')