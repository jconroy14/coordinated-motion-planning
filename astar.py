import numpy as np
import matplotlib.pyplot as plt
import heapq

from robot_visualization import draw_soln, draw_pos

##########################
### A* Star Algorithm ####
##########################

# Represents the "state" (position, etc.) of a robot in the A* algorithm
class state:
    # pos: tuple of (x, y)
    # cost: minimum time required to reach position from start node
    # end_pos: tuple of (x, y)
    # parent: tuple of (x, y) representing the location the robot came from
    def __init__(self, pos, cost, end_pos, parent):
        self.pos = pos
        self.h = manhattan_dist(pos, end_pos)
        self.cost = cost
        self.f = self.h + self.cost
        self.parent = parent
    
    def __eq__(self, other):
        return self.pos == other.pos

    def __ge__(self, other):
        return self.f >= other.f

    def __gt__(self, other):
        return self.f > other.f

# This implementation of the A* algorithm finds a heuristic path for a single
# robot, given a set of static obstacles and a set of moving obstacles.
# The heuristic used by A* is Manhattan distance.
#
# Input:
# - start_pos: tuple (x, y)
# - end_pos: tuple (x, y)
# - obstacles: list of tuples (x, y)
# - all_paths: list of moving obstacle paths,
#              where each path is represented by a list L of tuples
#              such that the obstacle is at location L[t] at time t
# Returns: a path between start_pos and end_pos
#          (represented as a list of tuples as described above)
#          or None if no path could be found
#
# Note: This implementation does not make the robot continue to
#       dodge moving obstacles after it reaches its target position.
#       If the path of a moving obstacle takes it through end_pos
#       after the robot has reached end_pos, there will be a collision.
#       TODO: This should be fixed.
def astar(start_pos, end_pos, obstacles, all_paths = []):
    open_heap = []
    closed = []
    start_state = state(start_pos, 0, end_pos, None)
    heapq.heappush(open_heap, start_state)

    initial_thing = True
    while open_heap:
        curr_node = heapq.heappop(open_heap)
        neighbors = get_neighbors(curr_node.pos, curr_node.cost, obstacles, all_paths)

        for neighbor_pos in neighbors:
            if neighbor_pos == end_pos:
                print("found target with cost", curr_node.cost + 1)
                path = []
                parent = curr_node
                while parent is not None:
                    path.append(parent)
                    parent = parent.parent
                path.reverse()
                path_positions = [state.pos for state in path]
                path_positions.append(end_pos)
                return path_positions
            else:
                neighbor_cost = curr_node.cost + 1
                neighbor = state(neighbor_pos, neighbor_cost, end_pos, curr_node)
                if neighbor in open_heap:
                    index = open_heap.index(neighbor)
                    if open_heap[index].f <= neighbor.f:
                        continue
                if neighbor in closed:
                    index = closed.index(neighbor)
                    if closed[index].f <= neighbor.f:
                        continue
                    else:
                        closed = closed.pop(index)
                heapq.heappush(open_heap, neighbor)
        closed.append(curr_node)

    print("No valid path found")
    return None

##############################
## Utility functions for A* ##
##############################

def manhattan_dist(start_pos, end_pos):
    x1, y1 = start_pos
    x2, y2 = end_pos
    return abs(x1 - x2) + abs(y1 - y2)

# Returns the valid neighboring squares for a robot at position 'pos' at time t
def get_neighbors(pos, t, obstacles, all_paths):
    all_moves = [[0, 1], [0, -1], [1, 0], [-1, 0]]
    neighbors = [tuple(np.array(pos) + move) for move in all_moves if
                   valid_move(t, obstacles, all_paths, pos, move)]
    return neighbors
    
# Checks if a single move (ie. going N/S/E/W) will collide with any obstacles
# Note that 'start_pos' is a tuple (x, y)
#       and 'move' is a list [x, y]
def valid_move(t, obstacles, all_paths, start_pos, move):
    end_pos = tuple(np.array(start_pos) + move)

    # Check if we run into any static obstacles
    if end_pos in obstacles: return False

    # Check if we would run into any moving obstacles
    for path in all_paths:
        if end_pos == position(path, t + 1):
            return False
        elif end_pos == position(path, t):
            conflicting_move_x  = position(path, t + 1)[0] - position(path, t)[0]
            conflicting_move_y  = position(path, t + 1)[1] - position(path, t)[1]
            if move[0] == conflicting_move_x and move[1] == conflicting_move_y:
                pass # Allow robots "sliding" together
            else:
                return False

    # Check if any moving obstacles would run into us
    for path in all_paths:
        if start_pos == position(path, t + 1):
            conflicting_move_x = position(path, t + 1)[0] - position(path, t)[0]
            conflicting_move_y = position(path, t + 1)[1] - position(path, t)[1]
            if move[0] == conflicting_move_x and move[1] == conflicting_move_y:
                pass
            else:
                return False

    return True


# Returns the position of a robot along the given path at time t
# Note that 'path' must be a nonempty array.
def position(path, t):
    if t < 0:
        return path[0]
    elif t < len(path):
        return path[t]
    else:
        return path[-1]