#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import colors
import math

# Create a forest fire animation based on a simple cellular automaton model.
# The maths behind this code is described in the scipython blog article
# at https://scipython.com/blog/the-forest-fire-model/
# Christian Hill, January 2016.
# Updated January 2020.

# Displacements from a cell to its eight nearest neighbours
velocity = 1
neighbourhood = ()
depth = 1
for i in range((-1 * depth), depth + 1):
    for j in range((-1 * depth), depth + 1):
        if i == 0 and j == 0:
            continue
        tup = (i, j)
        neighbourhood = neighbourhood + (tup, )
EMPTY, TREE, FIRE = 0, 1, 2
# Colours for visualization: brown for EMPTY, dark green for TREE and orange
# for FIRE. Note that for the colormap to work, this list and the bounds list
# must be one larger than the number of different values in the array.
colors_list = [(0.2,0,0), (0,0.5,0), (1,0,0), 'orange']
cmap = colors.ListedColormap(colors_list)
bounds = [0,1,2,3]
norm = colors.BoundaryNorm(bounds, cmap.N)

burning = False
wind = True
wind_direction = 180
wind_angle = 60
wind_edge_difference = 35

def iterate(X):
    """Iterate the forest according to the forest-fire rules."""

    # The boundary of the forest is always empty, so only consider cells
    # indexed from 1 to nx-2, 1 to ny-2
    global wind

    for i in range(0, velocity):
        X1 = np.zeros((ny, nx))
        for ix in range(2, nx-2):
            for iy in range(2, ny-2):
                if X[iy, ix] == EMPTY and np.random.random() <= p:
                    X1[iy, ix] = TREE
                if X[iy, ix] == TREE:
                    X1[iy, ix] = TREE
                    for dy,dx in neighbourhood:

                        if wind:
                            neighbour_direction = math.atan2(dy, dx)
                            #neighbour_direction += math.pi
                            neighbour_direction = math.degrees(neighbour_direction)
                            if neighbour_direction < 0:
                                neighbour_direction += 360
                            global wind_angle
                            global wind_direction
                            angle_difference = (neighbour_direction - wind_direction + 180) % 360 - 180
                            if angle_difference < -180:
                                angle_difference += 360
                            if abs(angle_difference) > wind_angle:
                                continue


                        # The diagonally-adjacent trees are further away, so
                        # only catch fire with a reduced probability:
                        #if abs(dx) == abs(dy) and np.random.random() < 0.573:
                        #    continue
                        if ((wind_angle - abs(angle_difference)) < wind_edge_difference) and np.random.random() < 0.573:
                            continue
                        if X[iy+dy, ix+dx] == FIRE:
                            X1[iy, ix] = FIRE
                            break
                    else:
                        global burning
                        if np.random.random() <= f and not burning:
                            burning = True
                            X1[iy, ix] = FIRE
        X = X1
    print("Sum: %d" % np.sum(X1))
    return X1


# The initial fraction of the forest occupied by trees.
forest_fraction = 1
# Probability of new tree growth per empty cell, and of lightning strike.
p, f = 0.0, 0.0001
# Forest size (number of cells in x and y directions).
nx, ny = 100, 100
# Initialize the forest grid.
X  = np.zeros((ny, nx))
#X[1:ny-1, 1:nx-1] = np.random.randint(0, 2, size=(ny-2, nx-2))
X[2:ny-2, 2:nx-2] = np.random.random(size=(ny-4, nx-4)) < forest_fraction

fig = plt.figure(figsize=(25/3, 6.25))
ax = fig.add_subplot(111)
ax.set_axis_off()
im = ax.imshow(X, cmap=cmap, norm=norm)#, interpolation='nearest')


# The animation function: called to produce a frame for each generation.
def animate(i):
    im.set_data(animate.X)
    animate.X = iterate(animate.X)
# Bind our grid to the identifier X in the animate function's namespace.
animate.X = X


# Interval between frames (ms).
interval = 250
anim = animation.FuncAnimation(fig, animate, interval=interval, frames=5)
plt.show()
