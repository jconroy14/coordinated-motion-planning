using PyCall
pushfirst!(PyVector(pyimport("sys")."path"), "..")

# Python for loading instance
py"""
import cgshop2021_pyutils

def load_instance():
    idb = cgshop2021_pyutils.InstanceDatabase('../cgshop_2021_instances.zip')
    instance_list = list(idb)
    i = instance_list[194] # 10 robots (smallest instance)
    return i
"""

i = py"load_instance"()
obstacles = i[:obstacles]
targets   = i[:target]

println(obstacles)
println(targets)



## Python for visualization
py"""
import matplotlib.pyplot as plt
import numpy as np
from cgshop2021_pyutils import InstanceDatabase, Instance
from cgshop2021_pyutils import Solution, SolutionStep, SolutionZipWriter, Direction, validate
from robot_visualization import draw_soln, draw_pos

def draw_and_show_pos(positions, obstacles, targets, colors, min_x, max_x, min_y, max_y):
    draw_pos(positions, obstacles, targets, colors, min_x, max_x, min_y, max_y)
    plt.show()

# Convert a tuple (delta x, delta y) to a cgshop2021_pyutils Direction
def move_to_direction(move):
    if   np.equal(move, (0,   1)).all():
        return Direction.NORTH
    elif np.equal(move, (0,  -1)).all():
        return Direction.SOUTH
    elif np.equal(move, ( 1,  0)).all():
        return Direction.EAST
    elif np.equal(move, (-1,  0)).all():
        return Direction.WEST
    else:
        return None

# Convert all_paths (list of paths) to a cgshop2021_pyutils Solution
def check_solution(inst, all_paths):
    solution_arr = []
    for robot_idx, path in enumerate(all_paths):
        # print('-------')
        # print('Path: ', path)
        moves = [np.array(path[i+1]) - np.array(path[i]) for i in range(len(path) - 1)]
        
        for t, move in enumerate(moves):
            if t < len(solution_arr):
                solution_arr[t].append((robot_idx, move))
            else:
                solution_arr.append([(robot_idx, move)])
    
    solution = s = Solution(inst)
    for step in solution_arr:
        soln_step = SolutionStep()
        for robot_idx, move in step:
            direction = move_to_direction(move)
            if direction is None: pass
            else: soln_step[robot_idx] = direction
        solution.add_step(soln_step)

    print(solution)
    print('Makespan:', solution.makespan)
    print('Sum:', solution.total_moves)

    validate(solution)

    min_x = min([min([x for (x, y) in path]) for path in all_paths])
    max_x = max([max([x for (x, y) in path]) for path in all_paths])
    min_y = min([min([y for (x, y) in path]) for path in all_paths])
    max_y = max([max([y for (x, y) in path]) for path in all_paths])

    draw_soln(solution, 'Greens', min_x, max_x, min_y, max_y)

    return solution

"""

py"draw_and_show_pos"(i[:start], i[:target], i[:start], "Greens", -1, 10, -1, 10)

all_paths = [[(3, 0), (2, 0), (1, 0), (0, 0), (0, 1), (0, 2), (0, 3), (1, 3), (1, 4), (1, 5), (1, 6), (2, 6)],
[(2, 4), (2, 5)],
[(0, 3), (0, 2), (-1, 2), (-1, 1), (-1, 0), (0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (4, -1), (5, -1), (6, -1), (7, -1), (8, -1), (9, -1), (10, -1), (10, 0), (10, 1), (10, 2), (9, 2)],
[(8, 4), (9, 4), (9, 5), (9, 6), (9, 7), (9, 8), (9, 9), (8, 9)],
[(2, 9), (1, 9), (0, 9)],
[(3, 9), (3, 8), (2, 8), (2, 7), (2, 6), (1, 6), (1, 5), (0, 5), (0, 4), (0, 3)],
[(2, 7), (2, 8), (1, 8), (1, 9), (2, 9), (3, 9), (4, 9), (5, 9), (6, 9), (7, 9), (7, 8), (8, 8), (9, 8), (9, 7), (9, 6)],
[(0, 9), (0, 8), (0, 7), (0, 6)],
[(2, 0), (1, 0), (0, 0), (-1, 0), (-1, -1), (-2, -1), (-2, 0), (-2, 1), (-2, 2), (-2, 3), (-2, 4), (-1, 4), (-1, 5), (-1, 6), (-1, 7), (-1, 8), (0, 8), (1, 8), (2, 8), (3, 8), (4, 8), (4, 9), (5, 9), (6, 9), (7, 9), (7, 8)],
[(1, 5), (1, 4), (1, 3), (0, 3), (-1, 3), (-1, 2), (-1, 1), (0, 1), (0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (4, -1), (5, -1), (6, -1), (7, -1), (8, -1), (9, -1), (10, -1), (10, 0), (10, 1), (10, 2), (10, 3), (9, 3)]]


py"check_solution"(i, all_paths)