import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick

sensor_coverage_results = np.load('results_sensor_coverage.npy')
sensor_coverage_detected_results = np.load('results_sensor_coverage_detected.npy')

averages = np.mean(sensor_coverage_results, axis=1)
averages /= (96 * 96)
averages *= 100

detection_percent = sensor_coverage_detected_results / 10
detection_percent *= 100

x = np.arange(1, (averages.shape[0] + 1))
x_percent = x / 10000
x_percent *= 100

fig, ax1 = plt.subplots()

color = 'tab:red'
ax1.set_xlabel('Sensor Coverage', fontweight='bold', fontsize=12)
ax1.set_ylabel('(%) Grid Burned', color=color, fontweight='bold', fontsize=12)
ax1.plot(x_percent, averages, color=color)
ax1.tick_params(axis='y', labelcolor=color)
ax1.set_xticklabels(ax1.get_xticks(), rotation=45, weight='bold', size=12)
ax1.set_yticklabels(ax1.get_yticks(), weight='bold', size=12)
ax1.xaxis.set_major_formatter(mtick.PercentFormatter())
ax1.yaxis.set_major_formatter(mtick.PercentFormatter())

ax2 = ax1.twinx()
color = 'tab:blue'
ax2.set_ylabel('(%) Fires Detected', color=color, fontweight='bold', fontsize=12)
ax2.plot(x_percent, detection_percent, color=color)
ax2.tick_params(axis='y', labelcolor=color)
ax2.set_yticklabels(ax2.get_yticks(), weight='bold', size=12)
ax2.yaxis.set_major_formatter(mtick.PercentFormatter())

fig.tight_layout()
plt.show()
'''
plt.title("Line graph")
plt.xlabel("Number of Sensors")
plt.ylabel("Y axis")
plt.plot(x, averages, color ="red")
plt.show()
'''
print('Done')