import numpy as np

ny, nx = 100, 100

maze = np.zeros((ny, nx))
maze[10:20, 10:20] = 1
print(maze[10:20,10:20])
print('Done')