from cgshop2021_pyutils import InstanceDatabase
from cgshop2021_pyutils import Solution, SolutionStep, SolutionZipWriter, Direction
from cgshop2021_pyutils import Instance

import numpy as np
import matplotlib.pyplot as plt

# Visualize a cgshop2021_pyutils Solution object by displaying a sequence
# of matplotlib plots
#
# Input:
# - solution: a cgshop2021_pyutils Solution object
# - colors: to be passed to as the cmap argument of imshow (from matplotlib)
# - min_x, max_x, min_y, max_y: define the bounding box of the solution
#
# Note: There are a few bugs in this code: the axes are not labled properly,
#       not all grid lines show up, and there may be issues with the
#       bounding box code
def draw_soln(solution, colors, min_x, max_x, min_y, max_y, timestep = 1):
    i = solution.instance
    steps = solution.steps
    pos = np.array(i.start)
    BUFFER = 1 # arbitrary, maybe helps avoid visualization bugs
    min_x, max_x, min_y, max_y = (min_x - 1 - BUFFER, max_x + 1 + BUFFER,
                                  min_y - 1 - BUFFER, max_y + 1 + BUFFER)
    draw_pos(pos, i.obstacles, i.target, colors, min_x, max_x, min_y, max_y)
    plt.show()
    for t, step in enumerate(steps):
        for robot_idx, move_dir in step._directions.items():
            pos[robot_idx] += np.array(move_dir.value)
        if t % timestep == 0:
            draw_pos(pos, i.obstacles, i.target, colors, min_x, max_x, min_y, max_y)
            plt.show()
    draw_pos(pos, i.obstacles, i.target, colors, min_x, max_x, min_y, max_y) 


# Plots the current state of all robots
#
# Input:
# - positions: a numpy array of (x, y) coordinates of robots to draw
# - obstacles: a numpy array of (x, y) coordinates
# - colors: a colormap to pass to matplotlib
# - min_x, max_x, min_y, max_y: a bounding box for the grid to display
#
# Returns: a matplotlib axes object
def draw_pos(positions, obstacles, targets, colors, min_x, max_x, min_y, max_y):
    fig, ax = plt.subplots()

    # Visualization code inspired by https://stackoverflow.com/a/55520277
    cells = np.zeros((max_x - min_y, max_y - min_y))
    for (i, (x, y)) in enumerate(positions):
        cells[x - min_x, y - min_y] = i + 1 # We ignore 0 when plotting

    obstacle_cells = np.zeros((max_x - min_y, max_y - min_y))
    for (i, (x, y)) in enumerate(obstacles):
        obstacle_cells[x - min_x, y - min_y] = 100 # We ignore 0 when plotting

    masked_cells = np.ma.array(cells, mask = (cells==0))
    ax.imshow(masked_cells.T, cmap = colors, origin = "lower", vmin = 0)
    masked_obstacle_cells = np.ma.array(obstacle_cells, mask = (obstacle_cells==0))
    ax.imshow(masked_obstacle_cells.T, cmap = "Reds", origin = "lower", vmin = 0)
    #ax.scatter([x-min_x for (x, y) in targets], [y-min_y for (x, y) in targets], cmap = colors, c = [x for x in range(len(targets))])
    ax.set_xticks(np.arange(max_x-min_x+1)-0.5 + min_x, minor=True)
    ax.set_yticks(np.arange(max_y-min_y+1)-0.5 + min_y, minor=True)
    ax.grid(which="minor")
    ax.tick_params(which="minor", size=0)