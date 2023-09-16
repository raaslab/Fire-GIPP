# Fire-GIPP

Run fire simulation:
```
python3 run_sim.py
```

Parameters:
```
num_sensors: 				Number of sensors deployed
sensor_distance: 			Minimum distance between every sensor

border_size: 				Empty border around grid
ny, nx:					Grid dimensions


fire_spread_rate:			How fast fire spreads compared to UAV movement
wind_direction:				Direction of wind
wind_angle:				Angle at which fire and smoke spread

radius:					Depth of neighbors to check for fire
smoke_spread_radius:			Depth of neighbors to check for smoke

probability_distance_effect: 		Effect distance has on search area probability
probability_angle_effect:		Effect angle has on search area probability
```
