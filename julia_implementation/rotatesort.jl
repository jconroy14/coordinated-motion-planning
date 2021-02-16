using Profile
using PyCall
pushfirst!(PyVector(pyimport("sys")."path"), "..")
pushfirst!(PyVector(pyimport("sys")."path"), "../python_implementation")

include("priority_planning.jl")

INST_INDEX = 0
EXPANSION = 2

py"""
import cgshop2021_pyutils

def load_instance(index):
    idb = cgshop2021_pyutils.InstanceDatabase('../cgshop_2021_instances.zip')
    #idb = cgshop2021_pyutils.InstanceDatabase('../test_instances.zip')
    instance_list = list(idb)
    i = instance_list[index] # 10 robots (smallest instance)
    return i

def save_paths(inst, all_paths):
    soln = convert_paths_into_soln(inst, all_paths)
    validate(soln)

    filename = '../output/' + inst.name + '.zip'
    with SolutionZipWriter(filename) as szw:
        szw.add_solution(soln)

    print('Solution saved to ' + filename)

def validate_paths(inst, all_paths):
    soln = convert_paths_into_soln(inst, all_paths)
    validate(soln)
"""

@enum MOVE UP DOWN LEFT RIGHT

function swap(mesh, order_to_robot, all_paths, col1, col2, row1, row2)
    robot1 = order_to_robot[mesh[col1, row1]]
    robot2 = order_to_robot[mesh[col2, row2]]
    if row1 == row2
        # Assume that col1 + 1 = col2
        if robot1 != -1
            robot1_pos = all_paths[robot1][end]
            robot1_moves = [robot1_pos + [0, -1],
                            robot1_pos + [1, -1],
                            robot1_pos + [2, -1],
                            robot1_pos + [2, 0]]
            all_paths[robot1] = append!(all_paths[robot1], robot1_moves)
        end
        if robot2 != -1
            robot2_pos = all_paths[robot2][end]
            robot2_moves = [robot2_pos + [-1, 0],
                            robot2_pos + [-2, 0],
                            robot2_pos + [-2, 0],
                            robot2_pos + [-2, 0]]
            all_paths[robot2] = append!(all_paths[robot2], robot2_moves)
        end
    else
        # Assume that row1 + 1 = row2
        # and col1 = col2
        if robot1 != -1
            robot1_pos = all_paths[robot1][end]
            robot1_moves = [robot1_pos + [-1, 0],
                            robot1_pos + [-1, 1],
                            robot1_pos + [-1, 2],
                            robot1_pos + [0, 2]]
            all_paths[robot1] = append!(all_paths[robot1], robot1_moves)
        end

        if robot2 != -1
            robot2_pos = all_paths[robot2][end]
            robot2_moves = [robot2_pos + [0, -1],
                            robot2_pos + [0, -2],
                            robot2_pos + [0, -2],
                            robot2_pos + [0, -2]]
            all_paths[robot2] = append!(all_paths[robot2], robot2_moves)
        end
    end
    temp = mesh[col1, row1]
    mesh[col1, row1] = mesh[col2, row2]
    mesh[col2, row2] = temp
    return mesh, all_paths
end

function sort_column(mesh, col, order_to_robot, all_paths)
    len = size(mesh)[2]
    sorted = false
    robots_in_column = setdiff([order_to_robot[order] for order in mesh[col, :]], [-1])

    while !sorted
        sorted = true
        for i in 2:2:(len-1)
            if mesh[col, i] > mesh[col, i + 1]
                mesh, all_paths = swap(mesh, order_to_robot, all_paths, col, col, i, i + 1)
                sorted = false
            end
        end
        all_paths[robots_in_column] = hold_until_all_complete(all_paths[robots_in_column])

        for i in 1:2:(len-1)
            if mesh[col, i] > mesh[col, i + 1]
                mesh, all_paths = swap(mesh, order_to_robot, all_paths, col, col, i, i + 1)
                sorted = false
            end
        end
        all_paths[robots_in_column] = hold_until_all_complete(all_paths[robots_in_column])
    end
    return mesh, all_paths
end

function sort_row(mesh, row, order_to_robot, all_paths, to_right = true)
    len = size(mesh)[1]
    sorted = false
    robots_in_row = setdiff([order_to_robot[order] for order in mesh[:, row]], [-1])

    while !sorted
        sorted = true
        for i in 2:2:(len-1)
            if ((to_right && (mesh[i, row] > mesh[i + 1, row])) ||
                (!to_right && (mesh[i, row] < mesh[i + 1, row])))
                mesh, all_paths = swap(mesh, order_to_robot, all_paths, i, i + 1, row, row)
                sorted = false
            end
        end
        all_paths[robots_in_row] = hold_until_all_complete(all_paths[robots_in_row])

        for i in 1:2:(len-1)
            if ((to_right && (mesh[i, row] > mesh[i + 1, row])) ||
                (!to_right && (mesh[i, row] < mesh[i + 1, row])))
                mesh, all_paths = swap(mesh, order_to_robot, all_paths, i, i + 1, row, row)
                sorted = false
            end
        end
        all_paths[robots_in_row] = hold_until_all_complete(all_paths[robots_in_row])

    end
    return mesh, all_paths
end

function rotate_row(mesh, row, numsteps, order_to_robot, all_paths)
    old_row = copy(mesh[:,row])
    rowlen = size(old_row)[1]
    for i in 1:rowlen
        previous_row_index = (i - numsteps - 1 + rowlen) % rowlen + 1
        robot_order = old_row[previous_row_index]
        robot_index = order_to_robot[robot_order]
        mesh[i, row] = robot_order

        # Move from previous_row_index to i
        if robot_index != -1
            moves = []
            curr_pos = previous_row_index

            p = all_paths[robot_index][end]
            n = curr_pos - i

            if curr_pos < i
                moves = [all_paths[robot_index][end] + [0, -1]]
                for _ in 1:(2 * max(numsteps, rowlen - numsteps))
                    moves = append!(moves, [moves[end]])
                end
                while curr_pos < i
                    moves = append!(moves, [moves[end] + [1, 0]])
                    moves = append!(moves, [moves[end] + [1, 0]])
                    curr_pos += 1
                end
                moves = append!(moves, [moves[end] + [0, 1]])
            elseif curr_pos > i
                moves = [all_paths[robot_index][end] + [-1, 0]]
                moves = append!(moves, [moves[end] + [-1, 0]])
                curr_pos += -1

                while curr_pos > i
                    moves = append!(moves, [moves[end] + [-1, 0]])
                    moves = append!(moves, [moves[end] + [-1, 0]])
                    curr_pos += -1
                end
            end
            all_paths[robot_index] = append!(all_paths[robot_index], moves)

        end
    end
    return mesh, all_paths
end

function rotate_col(mesh, col, numsteps, order_to_robot, all_paths)
    old_col = copy(mesh[col, :])
    collen = size(old_col)[1]
    for i in 1:collen
        previous_col_index = (i - numsteps - 1 + collen) % collen + 1
        robot_order = old_col[previous_col_index]
        robot_index = order_to_robot[robot_order]
        mesh[col, i] = robot_order

        # Move from previous_col_index to i
        if robot_index != -1
            moves = []
            curr_pos = previous_col_index

            if curr_pos < i
                moves = [all_paths[robot_index][end] + [-1, 0]]
                for _ in 1:(2 * max(numsteps, collen - numsteps))
                    moves = append!(moves, [moves[end]])
                end
                while curr_pos < i
                    moves = append!(moves, [moves[end] + [0, 1]])
                    moves = append!(moves, [moves[end] + [0, 1]])
                    curr_pos += 1
                end
                moves = append!(moves, [moves[end] + [1, 0]])
            elseif curr_pos > i
                moves = [all_paths[robot_index][end] + [0, -1]]
                moves = append!(moves, [moves[end] + [0, -1]])
                curr_pos += -1

                while curr_pos > i
                    moves = append!(moves, [moves[end] + [0, -1]])
                    moves = append!(moves, [moves[end] + [0, -1]])
                    curr_pos += -1
                end
            end
            all_paths[robot_index] = append!(all_paths[robot_index], moves)
        end
    end
    return mesh, all_paths
end

# function balance_vertical(mesh, first_col, last_col)
#     for col in first_col:last_col
#         mesh = sort_column(mesh, col)
#     end
#     for row in 1:num_rows
#         mesh = rotate_row(mesh, row, row % (last_col - first_col), first_col, last_col)
# end

function balance_vertical(mesh, order_to_robot, all_paths)
    println("balancing vertical")

    num_columns = size(mesh)[1]
    num_rows = size(mesh)[2]
    robots_in_mesh = setdiff([order_to_robot[order] for order in mesh], [-1])

    for col in 1:num_columns
        mesh, all_paths = sort_column(mesh, col, order_to_robot, all_paths)
    end
    all_paths[robots_in_mesh] = hold_until_all_complete(all_paths[robots_in_mesh])

    for row in 1:num_rows
        mesh, all_paths = rotate_row(mesh, row, row % num_columns, order_to_robot, all_paths)
    end
    all_paths[robots_in_mesh] = hold_until_all_complete(all_paths[robots_in_mesh])

    for col in 1:num_columns
        mesh, all_paths = sort_column(mesh, col, order_to_robot, all_paths)
    end
    all_paths[robots_in_mesh] = hold_until_all_complete(all_paths[robots_in_mesh])

    return mesh, all_paths
end

function balance_horizontal(mesh, order_to_robot, all_paths)
    num_columns = size(mesh)[1]
    num_rows = size(mesh)[2]
    robots_in_mesh = setdiff([order_to_robot[order] for order in mesh], [-1])

    for row in 1:num_rows
        mesh, all_paths = sort_row(mesh, row, order_to_robot, all_paths)
    end
    all_paths[robots_in_mesh] = hold_until_all_complete(all_paths[robots_in_mesh])

    for col in 1:num_columns
        mesh, all_paths = rotate_col(mesh, col, col % num_rows, order_to_robot, all_paths)
    end
    all_paths[robots_in_mesh] = hold_until_all_complete(all_paths[robots_in_mesh])

    for row in 1:num_rows
        mesh, all_paths = sort_row(mesh, row, order_to_robot, all_paths)
    end
    all_paths[robots_in_mesh] = hold_until_all_complete(all_paths[robots_in_mesh])

    return mesh, all_paths
end

function unblock(mesh, order_to_robot, all_paths)
    num_columns = size(mesh)[1]
    num_rows = size(mesh)[2]
    blocksize = floor(Int64, sqrt(num_columns))

    for row in 1:num_rows
        mesh, all_paths = rotate_row(mesh, row, (row * blocksize) % num_columns, order_to_robot, all_paths)
    end
    all_paths = hold_until_all_complete(all_paths)

    for col in 1:num_columns
        mesh, all_paths = sort_column(mesh, col, order_to_robot, all_paths)
    end
    all_paths = hold_until_all_complete(all_paths)

    return mesh, all_paths
end

function shear(mesh, order_to_robot, all_paths)
    num_columns = size(mesh)[1]
    num_rows = size(mesh)[2]

    for row in 1:num_rows
        sort_to_right = (row % 2 == 0)
        mesh, all_paths = sort_row(mesh, row, order_to_robot, all_paths, sort_to_right)
    end
    all_paths = hold_until_all_complete(all_paths)

    for col in 1:num_columns
        mesh, all_paths = sort_column(mesh, col, order_to_robot, all_paths)
    end
    all_paths = hold_until_all_complete(all_paths)

    return mesh, all_paths
end

function hold_until_all_complete(all_paths)
    if size(all_paths)[1] == 0
        return all_paths
    end

    last_time = maximum([size(path)[1] for path in all_paths])
    for (i, path) in enumerate(all_paths)
        while size(path)[1] < last_time
            all_paths[i] = append!(path, [path[end]])
        end
    end
    return all_paths
end

function assert_same_length(all_paths)
    if size(all_paths)[1] > 0
        last_time = size(all_paths[1])[1]
        for path in all_paths
            @assert last_time == size(path)[1]
        end
    end
end

function astar_if_possible(all_paths, expanded_target, obstacles = [])
    println("Attempting astar... ")
    curr_positions = [path[end] for path in all_paths]
    priority = Dict([[i, manhattan_dist(start_pos, target_pos)] for (i, (start_pos, target_pos)) in collect(enumerate(zip(curr_positions, expanded_target)))])
    paths_to_expanded_target = plan_with_priority(curr_positions, expanded_target, obstacles, priority, 0, 2, false)
    if isnothing(paths_to_expanded_target)
        println("Failed")
        return nothing
    else
        println("Success")
        return [append!(path1, path2) for (path1, path2) in zip(all_paths, paths_to_expanded_target)]
    end
end

function rotate_sort(mesh, order_to_robot, all_paths, inst, expanded_target)
    assert_same_length(all_paths)
    paths_to_expanded_target = astar_if_possible(all_paths, expanded_target)
    if !isnothing(paths_to_expanded_target)
        return mesh, paths_to_expanded_target
    end

    println("Original mesh")
    show(stdout, "text/plain", mesh)
    println("\n")

    num_columns = size(mesh)[1]
    num_rows = size(mesh)[2]

    # 1. Balance each vertical slice
    blocksize = floor(Int64, sqrt(num_columns))
    delim_lines = collect(1:blocksize:num_columns)
    chunks = [[delim_lines[i], delim_lines[i+1] - 1] for i in 1:(size(delim_lines)[1] - 1)]
    append!(chunks, [[delim_lines[end], num_columns]])

    for (a, b) in chunks
        curr_slice = mesh[a:b, :]
        balanced_slice, all_paths = balance_vertical(curr_slice, order_to_robot, all_paths)
        mesh[a:b, :] = balanced_slice
        # TODO: record robot moves
    end
    all_paths = hold_until_all_complete(all_paths)

    
    println("1. Balance each vertical slice")
    # show(stdout, "text/plain", mesh)
    # println("\n")
    assert_same_length(all_paths)
    paths_to_expanded_target = astar_if_possible(all_paths, expanded_target)
    if !isnothing(paths_to_expanded_target)
        return mesh, paths_to_expanded_target
    end


    # 2. Unblock the mesh
    mesh, all_paths = unblock(mesh, order_to_robot, all_paths)
    println("2. Unblock the mesh")
    # show(stdout, "text/plain", mesh)
    # println("\n")
    assert_same_length(all_paths)
    paths_to_expanded_target = astar_if_possible(all_paths, expanded_target)
    if !isnothing(paths_to_expanded_target)
        return mesh, paths_to_expanded_target
    end

    # 3. Balance each horizontal slice on its side
    delim_lines = collect(1:blocksize:num_rows)
    chunks = [[delim_lines[i], delim_lines[i+1] - 1] for i in 1:(size(delim_lines)[1] - 1)]
    append!(chunks, [[delim_lines[end], num_rows]])

    for (a, b) in chunks
        curr_slice = mesh[:, a:b]
        balanced_slice, all_paths = balance_horizontal(curr_slice, order_to_robot, all_paths)
        mesh[:, a:b] = balanced_slice
    end
    all_paths = hold_until_all_complete(all_paths)


    println("3. Balance each horizontal slice")
    # show(stdout, "text/plain", mesh)
    # println("\n")
    assert_same_length(all_paths)
    paths_to_expanded_target = astar_if_possible(all_paths, expanded_target)
    if !isnothing(paths_to_expanded_target)
        return mesh, paths_to_expanded_target
    end

    # 4. Unblock the mesh
    mesh, all_paths = unblock(mesh, order_to_robot, all_paths)
    println("4. Unblock the mesh")
    # show(stdout, "text/plain", mesh)
    # println("\n")
    assert_same_length(all_paths)
    paths_to_expanded_target = astar_if_possible(all_paths, expanded_target)
    if !isnothing(paths_to_expanded_target)
        return mesh, paths_to_expanded_target
    end

    # 5. Shear the mesh three times
    mesh, all_paths = shear(mesh, order_to_robot, all_paths)
    mesh, all_paths = shear(mesh, order_to_robot, all_paths)
    mesh, all_paths = shear(mesh, order_to_robot, all_paths)
    println("5. Shear the mesh three times")
    # show(stdout, "text/plain", mesh)
    # println("\n")
    assert_same_length(all_paths)
    paths_to_expanded_target = astar_if_possible(all_paths, expanded_target)
    if !isnothing(paths_to_expanded_target)
        return mesh, paths_to_expanded_target
    end

    # 6. Sort all rows to the right
    for row in 1:num_rows
        mesh, all_paths = sort_row(mesh, row, order_to_robot, all_paths)
    end
    all_paths = hold_until_all_complete(all_paths)

    println("6. Sort all rows to the right")
    # show(stdout, "text/plain", mesh)
    # println("\n")
    assert_same_length(all_paths)

    return mesh, all_paths
end


# how do we represent a mesh?
# 0. Dict mapping targetPriority -> robot (or empty space...)
# 1. 2D Array of positions (compressed, positions act as unique ID of robot)
#    When sorting eg. a column
#    - Sort the column of the array, keeping track of all moves for each ID
#    - Translate those moves into actual robot paths

function robot_rotate_sort(inst)
    ### Construct list of positions ###
    start     = [collect(pos) for pos in inst[:start]]
    target    = [collect(pos) for pos in inst[:target]]
    obstacles = [collect(pos) for pos in inst[:obstacles]]

    ### Decide sorted order ###
    # Lexicographical order: primarily determined by y, then by x
    max_x = maximum([x for (x, y) in vcat(start, target)])
    max_y = maximum([y for (x, y) in vcat(start, target)])
    ncols = max_x + 1
    nrows = max_y + 1

    robot_to_order = Dict(robot_num => pos_to_order(x, y, ncols, nrows)
                          for (robot_num, (x, y)) in enumerate(target))
    order_to_robot = Dict(pos_to_order(x, y, ncols, nrows) => robot_num
                          for (robot_num, (x, y)) in enumerate(target))
    startpos_to_order = Dict(start_pos => robot_to_order[i] for (i, start_pos) in enumerate(start))

    for i in 0:(ncols*nrows - 1)
        if !(i in keys(order_to_robot))
            order_to_robot[i] = -1
        end
    end
    
    ### Construct nxn compressed matrix ###
    mesh = fill(-1, (ncols, nrows))

    used_orders = Set()
    for x in 0:(ncols-1)
        for y in 0:(nrows-1)
            if [x,y] in keys(startpos_to_order)
                mesh[x+1, y+1] = startpos_to_order[[x,y]]
                union!(used_orders, startpos_to_order[[x,y]])
            end
        end
    end
    unused_orders = setdiff(Set(0:(ncols*nrows-1)), used_orders)

    for x in 1:ncols
        for y in 1:nrows
            if mesh[x, y] == -1
                order_to_add = collect(unused_orders)[1]
                mesh[x, y] = order_to_add
                delete!(unused_orders, order_to_add)
            end
        end
    end

    show(stdout, "text/plain", mesh)
    println("")

    ### Send each robot to expanded position ###
    expanded_start = [[floor(Int64, x * EXPANSION), floor(Int64, y * EXPANSION)]
                        for (x, y) in start]
    priority = [pos_to_order(x, y, max_x, max_y) for (x, y) in start]
    expansion_paths = nothing #plan_with_priority(start, expanded_start, [], priority, 0, 10)
    if isnothing(expansion_paths)
        expansion_paths = expand(start)
    end
    t = size(expansion_paths[1])[1]
    println("Solution takes $t steps")

    ### Sort compressed matrix ###
    println("----SORTING----")
    expanded_target = [[floor(Int64, x * EXPANSION), floor(Int64, y * EXPANSION)]
                        for (x, y) in target]

    mesh, up_to_middle_paths = rotate_sort(mesh, order_to_robot, expansion_paths, inst, expanded_target)

    robot_mesh = [order_to_robot[order] for order in mesh]
    println("Finished sorting (expanded verion)")
    # println("Robot positions in mesh:")
    # show(stdout, "text/plain", robot_mesh)
    # println("")
    # println("Current robot positions:")
    # for (i, path) in enumerate(up_to_middle_paths)
    #     #println("   $path")
    #     pos = path[end]
    #     curr_target = expanded_target[i]
    #     println("    Robot $i is at $pos and should be at $curr_target")
    # end
    
    # middle_paths = plan_with_priority(expanded_start, expanded_target, [], priority)
    # up_to_middle_paths = [append!(path1, path2) for (path1, path2) in zip(expansion_paths, middle_paths)]
    # t = size(middle_paths[1])[1]
    # println("Solution takes $t steps")

    ### Go to target positions
    priority = [-pos_to_order(x, y, max_x, max_y) for (x, y) in target]
    compression_paths = nothing #plan_with_priority(expanded_target, target, [], priority, 0, 10)
    if isnothing(compression_paths)
        compression_paths = compress(inst, expanded_target)
    end
    t = size(compression_paths[1])[1]
    println("Solution takes $t steps")

    all_paths = [append!(path1, path2) for (path1, path2) in zip(up_to_middle_paths, compression_paths)]
    return all_paths

end

function order_to_pos(index, max_x, max_y)
    x = index % max_x
    y = floor(Int64, (index - x)/max_x)
    return [x, y]
end

function pos_to_order(x, y, max_x, max_y)
    return max_x * y + x
end

function expand(start)
    all_paths = [[[x, y]] for (x, y) in start]
    for (i, (x, y)) in enumerate(start)
        for _ in 1:y
            all_paths[i] = append!(all_paths[i], [all_paths[i][end] + [0, 1]])
        end
    end
    all_paths = hold_until_all_complete(all_paths)

    for (i, (x, y)) in enumerate(start)
        for _ in 1:x
            all_paths[i] = append!(all_paths[i], [all_paths[i][end] + [1, 0]])
        end
    end
    all_paths = hold_until_all_complete(all_paths)

    m = maximum([size(path)[1] for path in all_paths])
    println("Steps to expand: $m")
    return(all_paths)
end

function compress(inst, expanded_pos)
    all_paths = [[[x, y]] for (x, y) in expanded_pos]
    for (i, (x, y)) in enumerate(expanded_pos)
        for _ in 1:(y/2)
            all_paths[i] = append!(all_paths[i], [all_paths[i][end] + [0, -1]])
        end
    end
    all_paths = hold_until_all_complete(all_paths)

    for (i, (x, y)) in enumerate(expanded_pos)
        for _ in 1:(x/2)
            all_paths[i] = append!(all_paths[i], [all_paths[i][end] + [-1, 0]])
        end
    end
    all_paths = hold_until_all_complete(all_paths)

    m = maximum([size(path)[1] for path in all_paths])
    println("Steps to compress: $m")

    return all_paths
end

#without_obstacles = [0, 3, 4, 7, 11, 12, 13, 14, 15, 18, 19, 21, 22, 25, 26, 31, 34, 36, 37, 38, 39, 40, 42, 47, 49, 51, 52, 53, 57, 59, 62, 65, 67, 71, 72, 74, 75, 76, 77, 78, 84, 85, 86, 87, 89, 90, 93, 96, 97, 98, 100, 101, 102, 103, 104, 108, 111, 112, 114, 116, 117, 118, 120, 121, 122, 125, 126, 127, 129, 131, 132, 135, 140, 141, 142, 143, 144, 145, 146, 149, 150, 151, 154, 157, 158, 159, 160, 162, 165, 167, 169, 171, 173, 174, 176, 180, 183, 188, 192, 195, 196, 198, 202]
without_obstacles = [7, 11, 13, 14, 15, 18, 19, 21, 22, 25, 26, 31, 34, 36, 37, 38, 39, 40, 42, 47, 49, 51, 52, 53, 57, 59, 62, 65, 67, 71, 72, 74, 75, 76, 77, 78, 84, 85, 86, 87, 89, 90, 93, 96, 97, 98, 100, 101, 102, 103, 104, 108, 111, 112, 114, 116, 117, 118, 120, 121, 122, 125, 126, 127, 129, 131, 132, 135, 140, 141, 142, 143, 144, 145, 146, 149, 150, 151, 154, 157, 158, 159, 160, 162, 165, 167, 169, 171, 173, 174, 176, 180, 183, 188, 192, 195, 196, 198, 202]
total_to_run = size(without_obstacles)[1]
for (i, curr_index) in enumerate(without_obstacles)
    inst = py"load_instance"(curr_index)
    println("Obstacles:")
    println(inst[:obstacles])
    all_paths = robot_rotate_sort(inst)
    #py"validate_paths"(inst, all_paths)
    py"save_paths"(inst, all_paths)
    
    instname = inst[:name]
    println("*********************************************************")
    println("Found solution for instance $i/$total_to_run")
    println("*********************************************************")
end

#py"visualize_all_paths"(inst, all_paths, 10)
