import numpy as np

results = np.loadtxt('results.csv', delimiter=',', skiprows=1)

total_runs = np.sum(results[:, 3])
non_early_validations = np.sum(results[:, 4])
non_early_localizations = np.sum(results[:, 5])
non_early_sensor_first_localizations = np.sum(results[:, 29])

ours_total_distance = np.mean(results[:, 9])
baseline_total_distance = np.mean(results[:, 21])
sensor_first_baseline_total_distance = np.mean(results[:, 33])

ours_total_validation_distance = np.mean(results[:, 10])
baseline_total_validation_distance = np.mean(results[:, 22])
sensor_first_baseline_total_validation_distance = np.mean(results[:, 34])

ours_time = np.mean(results[:, 8])
baseline_time = np.mean(results[:, 20])
sensor_first_baseline_time = np.mean(results[:, 32])

ours_non_early_distance = results[:, 14]
temp = results[:, 5]
ours_non_early_distance = np.multiply(ours_non_early_distance, temp)
ours_non_early_distance /= np.sum(temp)
ours_non_early_distance = np.sum(ours_non_early_distance)

baseline_non_early_distance = results[:, 26]
temp = results[:, 17]
baseline_non_early_distance = np.multiply(baseline_non_early_distance, temp)
baseline_non_early_distance /= np.sum(temp)
baseline_non_early_distance = np.sum(baseline_non_early_distance)

sensor_first_baseline_non_early_distance = results[:, 38]
temp = results[:, 29]
sensor_first_baseline_non_early_distance = np.multiply(sensor_first_baseline_non_early_distance, temp)
sensor_first_baseline_non_early_distance /= np.sum(temp)
sensor_first_baseline_non_early_distance = np.sum(sensor_first_baseline_non_early_distance)

ours_non_early_distance_relative = results[:, 14] - results[:, 15]
temp = results[:, 5]
ours_non_early_distance_relative = np.multiply(ours_non_early_distance_relative, temp)
ours_non_early_distance_relative /= np.sum(temp)
ours_non_early_distance_relative = np.sum(ours_non_early_distance_relative)

baseline_non_early_distance_relative = results[:, 26] - results[:, 27]
temp = results[:, 17]
baseline_non_early_distance_relative = np.multiply(baseline_non_early_distance_relative, temp)
baseline_non_early_distance_relative /= np.sum(temp)
baseline_non_early_distance_relative = np.sum(baseline_non_early_distance_relative)

sensor_first_baseline_non_early_distance_relative = results[:, 38] - results[:, 39]
temp = results[:, 29]
sensor_first_baseline_non_early_distance_relative = np.multiply(sensor_first_baseline_non_early_distance_relative, temp)
sensor_first_baseline_non_early_distance_relative /= np.sum(temp)
sensor_first_baseline_non_early_distance_relative = np.sum(sensor_first_baseline_non_early_distance_relative)

ours_non_early_validation_distance = results[:, 12]
temp = results[:, 4]
ours_non_early_validation_distance = np.multiply(ours_non_early_validation_distance, temp)
ours_non_early_validation_distance /= np.sum(temp)
ours_non_early_validation_distance = np.sum(ours_non_early_validation_distance)

baseline_non_early_validation_distance = results[:, 24]
temp = results[:, 16]
baseline_non_early_validation_distance = np.multiply(baseline_non_early_validation_distance, temp)
baseline_non_early_validation_distance /= np.sum(temp)
baseline_non_early_validation_distance = np.sum(baseline_non_early_validation_distance)

ours_non_early_validation_distance_relative = results[:, 12] - results[:, 13]
temp = results[:, 4]
ours_non_early_validation_distance_relative = np.multiply(ours_non_early_validation_distance_relative, temp)
ours_non_early_validation_distance_relative /= np.sum(temp)
ours_non_early_validation_distance_relative = np.sum(ours_non_early_validation_distance_relative)

baseline_non_early_validation_distance_relative = results[:, 24] - results[:, 25]
temp = results[:, 16]
baseline_non_early_validation_distance_relative = np.multiply(baseline_non_early_validation_distance_relative, temp)
baseline_non_early_validation_distance_relative /= np.sum(temp)
baseline_non_early_validation_distance_relative = np.sum(baseline_non_early_validation_distance_relative)

print('Total Runs: %d' % total_runs)
print('Non-Early Validations: %d' % non_early_validations)
print('Non-Early Localizations: %d' % non_early_localizations)
print('Non-Early Sensor-First Localizations: %d' % non_early_sensor_first_localizations)
print('Results: [Ours, Baseline, Sensor First Baseline]')
print('Total Computational Time (ms): [%.2f, %.2f, %.2f]' % (ours_time, baseline_time, sensor_first_baseline_time))
print('Total Validation Distance: [%.2f, %.2f, %.2f]' % (ours_total_validation_distance, baseline_total_validation_distance, sensor_first_baseline_total_validation_distance))
print('Total Localization Distance: [%.2f, %.2f, %.2f]' % (ours_total_distance, baseline_total_distance, sensor_first_baseline_total_distance))
print('Non-Early Total Validation Distance: [%.2f, %.2f, N/A]' % (ours_non_early_validation_distance, baseline_non_early_validation_distance))
print('Non-Early Relative Validation Distance: [%.2f, %.2f, N/A]' % (ours_non_early_validation_distance_relative, baseline_non_early_validation_distance_relative))
print('Non-Early Total Localization Distance: [%.2f, %.2f, %.2f]' % (ours_non_early_distance, baseline_non_early_distance, sensor_first_baseline_non_early_distance))
print('Non-Early Relative Localization Distance: [%.2f, %.2f, %.2f]' % (ours_non_early_distance_relative, baseline_non_early_distance_relative, sensor_first_baseline_non_early_distance_relative))
print('Done')