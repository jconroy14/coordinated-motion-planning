using PyCall
using BenchmarkTools
include("astar.jl")

MAX_ITERATION = 5000

pushfirst!(PyVector(pyimport("sys")."path"), "..")

# Higher priority = larger value
function convert_target_to_priority(target, max_x, max_y)
    x, y = target

    if max_x <= max_y
        return max_y * x + y
    else
        return max_x * y + x
    end
end

# The "priority" input is a dictionary that maps robot index to a number
# Robots with larger priority values will be planned first
function plan_with_priority(start, target, obstacles, priority, iteration_num = 0)
    robot_list = [(robot_num, start_pos, target_pos) for (robot_num, (start_pos, target_pos)) in collect(enumerate(zip(start, target)))]
    
    # Sort robots by priority and plan paths
    sorted_robots = sort(robot_list, by = robot -> -priority[robot[1]])
    all_paths = []
    robot_nums = []
    for (i, (robot_num, start_pos, target_pos)) in enumerate(sorted_robots)
        curr_robot_path = astar(start_pos, target_pos, obstacles, all_paths)
        if isnothing(curr_robot_path)
            if iteration_num == MAX_ITERATION
                println("ERROR: Solver failed")
                return nothing
            end
            println("Planned $i robots")
            println("-------------------")
            println("Iteration $iteration_num")
            #println("Attempting with new priority")
            #println("Robot $robot_num will be planned first")
            priority[robot_num] = maximum(values(priority)) + 1
            return plan_with_priority(start, target, obstacles, priority, iteration_num + 1)
        end

        pathlength = size(curr_robot_path)[1]
        insert!(all_paths, length(all_paths) + 1, curr_robot_path)
        insert!(robot_nums, length(robot_nums) + 1, robot_num)

        #println("-------")
        # println("Planning robot $robot_num")
        # println("Start: $start_pos")
        # println("Target: $target_pos")
        # println("Path: $curr_robot_path")
        # println("Path length: $pathlength")
    end

    # Reorder paths back to correspond to robot index
    reordered_paths = [path for (path, num) in sort(collect(zip(all_paths, robot_nums)), by = x -> x[2])]
    println("-----------")
    println("FOUND SOLUTION")
    println(reordered_paths)
    return reordered_paths
end

function priority_planning(inst)
    start     = [collect(pos) for pos in inst[:start]]
    target    = [collect(pos) for pos in inst[:target]]
    obstacles = [collect(pos) for pos in inst[:obstacles]]

    target_max_x = maximum([x for (x, y) in target])
    target_max_y = maximum([y for (x, y) in target])
    #priority  = Dict([[i, convert_target_to_priority(target_pos, target_max_x, target_max_y)] for (i, target_pos) in collect(enumerate(target))])
    priority = Dict([[i, manhattan_dist(start_pos, target_pos)] for (i, (start_pos, target_pos)) in collect(enumerate(zip(start, target)))])

    return plan_with_priority(start, target, obstacles, priority, 1)
end

py"""
from cgshop2021_pyutils import InstanceDatabase, Instance
from cgshop2021_pyutils import Solution, SolutionStep, SolutionZipWriter, Direction, validate

import numpy as np
import matplotlib.pyplot as plt

from robot_visualization import draw_soln, draw_pos

# Load instances
def load_instance():
    idb = InstanceDatabase('../cgshop_2021_instances.zip')
    instance_list = list(idb)
    i = instance_list[194] # 10 robots (smallest instance, solver works)
    #i = instance_list[52] # 80 robots  (solver works)
    #i = instance_list[92] # 63 robots  (solver fails)
    #i = instance_list[0] # 160 robots   (solver fails)
    return i

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
    elif np.equal(move, (0, 0)).all():
        return None
    else:
        print('*****************************')
        print('** ERROR: Robot teleported **')
        print('*****************************')
        assert False

# Convert all_paths (list of paths) to a cgshop2021_pyutils Solution
def convert_paths_into_soln(inst, all_paths):
    solution_arr = []
    for robot_idx, path in enumerate(all_paths):
        #print('-------')
        #print('Path: ', path)
        moves = [np.array(path[i+1]) - np.array(path[i]) for i in range(len(path) - 1)]
        
        if robot_idx == 74:
            print('robot 74 path:', path)
            print('robot 74 moves:', moves)

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
            if direction is not None:
                soln_step[robot_idx] = direction
        solution.add_step(soln_step)

    print(solution)
    print('Makespan:', solution.makespan)
    print('Sum:', solution.total_moves)
    return solution

def visualize_all_paths(inst, all_paths):
    min_x = min([min([x for (x, y) in path]) for path in all_paths])
    max_x = max([max([x for (x, y) in path]) for path in all_paths])
    min_y = min([min([y for (x, y) in path]) for path in all_paths])
    max_y = max([max([y for (x, y) in path]) for path in all_paths])

    soln = convert_paths_into_soln(inst, all_paths)

    draw_soln(soln, 'Greens', min_x, max_x, min_y, max_y)
    validate(soln)
    
"""

println("Loading instance")
inst = py"load_instance"()

println("Planning paths")
#all_paths = [[collect(pos)] for pos in inst[:start]]
all_paths = priority_planning(inst)
println("-------")
println(all_paths)

println("Visualizing paths")
py"visualize_all_paths"(inst, all_paths)



#@benchmark priority_planning(inst)