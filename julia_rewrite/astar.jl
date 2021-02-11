using DataStructures

struct state
    pos # array of [x, y]
    cost # int, minimum time required to reach position from start node
    end_pos # array of [x, y]
    parent_state # array of [x, y] representing the location the robot came from
    h
    f

    state(pos, cost, end_pos, parent_state,
        h = manhattan_dist(pos, end_pos),
        f = h + cost) = new(pos, cost, end_pos, parent_state, h, f)
end

# a = state([0,0], 1, [10,10], nothing)

# Implementation of A* algorithm
# Adapted from pseudocode here, with modifications for our setting:
#   https://mat.uab.cat/~alseda/MasterOpt/AStar-Algorithm.pdf
function astar(start_pos::Array{Int64,1}, target_pos::Array{Int64,1}, obstacles, all_paths = [], start_time = 1, verbose = false)
    final_time = maximum(append!([length(path) for path in all_paths], 1))

    open_heap_positions = PriorityQueue()
    open_pos_to_state = Dict{Array{Int64,1}, state}()
    closed_pos_to_state = Dict{Array{Int64,1}, state}()

    enqueue!(open_heap_positions, start_pos, start_time)
    start_state = state(start_pos, start_time, target_pos, nothing)    
    open_pos_to_state[start_pos] = start_state

    iteration = 1
    while !isempty(open_heap_positions)
        if iteration > 10000
            println("Exhausted allowed repetitions - no path found")
            return nothing
        end
        iteration += 1

        curr_pos = dequeue!(open_heap_positions)
        curr_node = open_pos_to_state[curr_pos]
        delete!(open_pos_to_state, curr_pos)

        neighbors = get_neighbors(curr_node.pos, curr_node.cost, obstacles, all_paths)

        for neighbor_pos in neighbors
            if neighbor_pos == target_pos
                path = Array{state,1}()
                parent_node = curr_node
                while !isnothing(parent_node)
                    insert!(path, length(path) + 1, parent_node)
                    parent_node = parent_node.parent_state
                end
                reverse!(path)
                path_positions = [state.pos for state in path]

                # Modify the path to move out of the way of others after
                # reaching target.
                # Note that at the current, the path has not yet reached the
                # target node. This means that we must always run the body of
                # the while loop at least once
                curr_time = curr_node.cost

                while true
                    # Note: we check neighbors of time stamp curr_time + 1
                    #   (looking ahead further into the future than is
                    #   strictly necessary) to give the robot more options
                    #   when deciding where to "sidestep".
                    if (target_pos in get_neighbors(curr_pos, curr_time + 1, obstacles, all_paths))
                        insert!(path_positions, length(path_positions) + 1, target_pos)
                        curr_pos = target_pos

                        curr_time += 1
                        if curr_time >= final_time
                            return path_positions
                        end
                    else
                        # Select arbitrary neighbor to sidestep to
                        # We check each possibility and accept the first valid path
                        for sidestep_pos in get_neighbors(curr_pos, curr_time, obstacles, all_paths)
                            path_to_append = astar(sidestep_pos, target_pos, obstacles, all_paths, curr_time + 1, true)
                            if !isnothing(path_to_append)
                                append!(path_positions, path_to_append)
                                return path_positions
                            end
                        end
                        println("No valid path found (for sidestepping)")
                        return nothing
                    end
                end
            else
                neighbor_cost = curr_node.cost + 1
                neighbor = state(neighbor_pos, neighbor_cost, target_pos, curr_node)
                if neighbor_pos in keys(open_pos_to_state)
                    old_f = open_pos_to_state[neighbor_pos].f
                    if old_f <= neighbor.f
                        continue
                    else
                        delete!(open_heap_positions, neighbor_pos)
                        enqueue!(open_heap_positions, neighbor_pos, neighbor.f)
                        open_pos_to_state[neighbor_pos] = neighbor
                    end
                elseif neighbor_pos in keys(closed_pos_to_state)
                    old_f = closed_pos_to_state[neighbor_pos].f
                    if old_f <= neighbor.f
                        continue
                    else
                        delete!(closed_pos_to_state, neighbor_pos)
                        enqueue!(open_heap_positions, neighbor_pos, neighbor.f)
                        open_pos_to_state[neighbor_pos] = neighbor
                    end
                else
                    enqueue!(open_heap_positions, neighbor_pos, neighbor.f)
                    open_pos_to_state[neighbor_pos] = neighbor
                end
            end
        end
        closed_pos_to_state[curr_pos] = curr_node
    end

    println("No valid path found")
    return nothing
end

##############################
## Utility functions for A* ##
##############################

function manhattan_dist(start_pos, target_pos)
    x1, y1 = start_pos
    x2, y2 = target_pos
    return (abs(x1 - x2) + abs(y1 - y2))
end

function get_neighbors(pos, t, obstacles, all_paths)
    all_moves = [[0, 1], [0, -1], [1, 0], [-1, 0], [0,0]]
    neighbors = [pos + move for move in all_moves if
                   valid_move(t, obstacles, all_paths, pos, move)]
    return neighbors
end

# Checks if a single move (ie. going N/S/E/W) will collide with any obstacles
# Note that 'start_pos' is a tuple (x, y)
#       and 'move' is a list [x, y]
function valid_move(t, obstacles, all_paths, start_pos, move)
    end_pos = start_pos + move

    # Check if we run into any static obstacles
    if end_pos in obstacles
        return false
    end

    # Check if we would run into any moving obstacles
    for path in all_paths
        if end_pos == position(path, t + 1)
            return false
        elseif end_pos == position(path, t)
            conflicting_move_x  = position(path, t + 1)[1] - position(path, t)[1]
            conflicting_move_y  = position(path, t + 1)[2] - position(path, t)[2]
            if move[1] == conflicting_move_x && move[2] == conflicting_move_y
                # Do nothing - allows robots "sliding" together
            else
                return false
            end
        end
    end

    # Check if any moving obstacles would run into us
    for path in all_paths
        if start_pos == position(path, t + 1)
            conflicting_move_x = position(path, t + 1)[1] - position(path, t)[1]
            conflicting_move_y = position(path, t + 1)[2] - position(path, t)[2]
            if move[1] == conflicting_move_x && move[2] == conflicting_move_y
                # Do nothing
            else
                return false
            end
        end
    end

    return true
end

# Returns the position of a robot along the given path at time t
# Note that 'path' must be a nonempty array.
function position(path, t)
    if t < 1
        path[1]
    elseif t <= size(path)[1]
        path[t]
    else
        path[end]
    end
end