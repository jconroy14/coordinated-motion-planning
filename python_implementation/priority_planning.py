from cgshop2021_pyutils import InstanceDatabase, Instance
from cgshop2021_pyutils import Solution, SolutionStep, SolutionZipWriter, Direction, validate

import numpy as np
import matplotlib.pyplot as plt

from timeit import default_timer as timer

from robot_visualization import draw_soln, draw_pos
from astar import astar

#########################################
## Path Planning for Multiple Robots ####
#########################################

# Plan paths for multiple robots, following a "priority planning" scheme.
# We plan paths for the robots one at a time (in arbitrary order),
# treating previously planned robots as moving obstacles for later robots.
# We find a path (not necessarily optimal) for each individual robot using
# the A* algorithm.
#
# Input: a cgshop2021_pyutils instance
# Returns: a tuple
# - min_x, max_x, min_y, max_y define the bounding box of robot positions
#                              in the solution (helpful for visualization)
# - soln is a cgshop2021_pyutils Solution object
def priority_planning(inst):
    start = timer()
    all_paths = []
    for n, (start_pos, end_pos) in enumerate(zip(inst.start, inst.target)):
        print("Planning robot", n)
        curr_robot_path = astar(start_pos, end_pos,
                                inst.obstacles, all_paths)
        print(curr_robot_path)
        all_paths.append(curr_robot_path)

    print("All paths")
    print(all_paths)
    print("-----------------")

    min_x = min([min([x for (x, y) in path]) for path in all_paths])
    max_x = max([max([x for (x, y) in path]) for path in all_paths])
    min_y = min([min([y for (x, y) in path]) for path in all_paths])
    max_y = max([max([y for (x, y) in path]) for path in all_paths])

    end = timer()
    print("EXECUTION TIME:", (end-start) * 1000, "(ms)")

    soln = convert_paths_into_soln(inst, all_paths)
    return (min_x, max_x, min_y, max_y, soln)


#############################################
## Utility functions for Priority Planning ##
#############################################

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
def convert_paths_into_soln(inst, all_paths):
    solution_arr = []
    for robot_idx, path in enumerate(all_paths):
        print("-------")
        print("Path: ", path)
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
    print("Makespan:", solution.makespan)
    print("Sum:", solution.total_moves)
    return solution

#################
## Main Method ##
#################

if __name__ == '__main__':    
    # This is a rather inefficient way to load in an instance using the
    # cgshop2021_pyutils library. The primary benefit of using this library
    # as opposed to manually parsing the json files is the predefined "validate"
    # function, which checks the feasibility of solutions to a given instance.
    idb = InstanceDatabase("cgshop_2021_instances.zip")
    instance_list = list(idb)
    i = instance_list[194] # 10 robots (smallest instance)

    # For testing bug in code
    # i.target[-2] = (1,0)
    path =  astar((0,0), (9, 2), i.obstacles, i.target)
    print(path)
    print('------')

    min_x, max_x, min_y, max_y, soln = priority_planning(i)
    
    draw_soln(soln, "Greens", min_x, max_x, min_y, max_y)
    validate(soln)