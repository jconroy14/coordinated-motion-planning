using PyCall
pushfirst!(PyVector(pyimport("sys")."path"), "..")
pushfirst!(PyVector(pyimport("sys")."path"), "../python_implementation")

include("priority_planning.jl")

py"""
import cgshop2021_pyutils

def load_instance():
    idb = cgshop2021_pyutils.InstanceDatabase('../cgshop_2021_instances.zip')
    #idb = cgshop2021_pyutils.InstanceDatabase('../test_instances.zip')
    instance_list = list(idb)
    i = instance_list[0] # 10 robots (smallest instance)
    return i
"""

@enum MOVE UP DOWN LEFT RIGHT

function swap(mesh, col1, col2, row1, row2)
    temp = mesh[col1, row1]
    mesh[col1, row1] = mesh[col2, row2]
    mesh[col2, row2] = temp
    return mesh
end

function sort_column(mesh, col)
    len = size(mesh)[2]
    sorted = false
    while !sorted
        sorted = true
        for i in 2:2:(len-1)
            if mesh[col, i] > mesh[col, i + 1]
                mesh = swap(mesh, col, col, i, i + 1)
                sorted = false
            end
        end

        for i in 1:2:(len-1)
            if mesh[col, i] > mesh[col, i + 1]
                mesh = swap(mesh, col, col, i, i + 1)
                sorted = false
            end
        end
    end
    return mesh
end

function sort_row(mesh, row, to_right = true)
    len = size(mesh)[1]
    sorted = false
    while !sorted
        sorted = true
        for i in 2:2:(len-1)
            if ((to_right && (mesh[i, row] > mesh[i + 1, row])) ||
                (!to_right && (mesh[i, row] < mesh[i + 1, row])))
                mesh = swap(mesh, i, i + 1, row, row)
                sorted = false
            end
        end

        for i in 1:2:(len-1)
            if ((to_right && (mesh[i, row] > mesh[i + 1, row])) ||
                (!to_right && (mesh[i, row] < mesh[i + 1, row])))
                mesh = swap(mesh, i, i + 1, row, row)
                sorted = false
            end
        end
    end
    return mesh
end

function rotate_row(mesh, row, numsteps)
    old_row = copy(mesh[:,row])
    rowlen = size(old_row)[1]
    for i in 1:rowlen
        new_value = old_row[(i - numsteps - 1 + rowlen) % rowlen + 1]
        mesh[i, row] = new_value
    end
    return mesh
end

function rotate_col(mesh, col, numsteps)
    old_col = copy(mesh[col, :])
    collen = size(old_col)[1]
    for i in 1:collen
        new_value = old_col[(i - numsteps - 1 + collen) % collen + 1]
        mesh[col, i] = new_value
    end
    return mesh
end

# function balance_vertical(mesh, first_col, last_col)
#     for col in first_col:last_col
#         mesh = sort_column(mesh, col)
#     end
#     for row in 1:num_rows
#         mesh = rotate_row(mesh, row, row % (last_col - first_col), first_col, last_col)
# end

function balance_vertical(mesh)
    num_columns = size(mesh)[1]
    num_rows = size(mesh)[2]

    for col in 1:num_columns
        mesh = sort_column(mesh, col)
    end
    for row in 1:num_rows
        mesh = rotate_row(mesh, row, row % num_columns)
    end
    for col in 1:num_columns
        mesh = sort_column(mesh, col)
    end

    return mesh
end

function balance_horizontal(mesh)
    num_columns = size(mesh)[1]
    num_rows = size(mesh)[2]

    for row in 1:num_rows
        mesh = sort_row(mesh, row)
    end
    for col in 1:num_columns
        mesh = rotate_col(mesh, col, col % num_rows)
    end
    for row in 1:num_rows
        mesh = sort_row(mesh, row)
    end
    return mesh
end

function unblock(mesh)
    num_columns = size(mesh)[1]
    num_rows = size(mesh)[2]
    blocksize = floor(Int64, sqrt(num_columns))
    
    for row in 1:num_rows
        mesh = rotate_row(mesh, row, (row * blocksize) % num_columns)
    end
    for col in 1:num_columns
        mesh = sort_column(mesh, col)
    end
    return mesh
end

function shear(mesh)
    num_columns = size(mesh)[1]
    num_rows = size(mesh)[2]

    for row in 1:num_rows
        sort_to_right = (row % 2 == 0)
        mesh = sort_row(mesh, row, sort_to_right)
    end
    for col in 1:num_columns
        mesh = sort_column(mesh, col)
    end
    return mesh
end

function rotate_sort(mesh)
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
        balanced_slice = balance_vertical(curr_slice)
        mesh[a:b, :] = balanced_slice
        # TODO: record robot moves
    end
    
    println("1. Balance each vertical slice")
    show(stdout, "text/plain", mesh)
    println("\n")

    # 2. Unblock the mesh
    mesh = unblock(mesh)
    println("2. Unblock the mesh")
    show(stdout, "text/plain", mesh)
    println("\n")

    # 3. Balance each horizontal slice on its side
    delim_lines = collect(1:blocksize:num_rows)
    chunks = [[delim_lines[i], delim_lines[i+1] - 1] for i in 1:(size(delim_lines)[1] - 1)]
    append!(chunks, [[delim_lines[end], num_rows]])
    println("horizontal chunks: $chunks")

    println("\n Processing Horizontal Chunks:")
    for (a, b) in chunks
        println("\n After processing Chunk [$a, $b]:")
        curr_slice = mesh[:, a:b]
        # show(stdout, "text/plain", curr_slice)
        # println("--")
        balanced_slice = balance_horizontal(curr_slice)
        # show(stdout, "text/plain", balanced_slice)
        mesh[:, a:b] = balanced_slice
        show(stdout, "text/plain", mesh)
    end

    println("3. Balance each horizontal slice")
    show(stdout, "text/plain", mesh)
    println("\n")

    # 4. Unblock the mesh
    mesh = unblock(mesh)
    println("4. Unblock the mesh")
    show(stdout, "text/plain", mesh)
    println("\n")


    # 5. Shear the mesh three times
    mesh = shear(shear(shear(mesh)))
    println("5. Shear the mesh three times")
    show(stdout, "text/plain", mesh)
    println("\n")


    # 6. Sort all rows to the right
    for row in 1:num_rows
        mesh = sort_row(mesh, row)
    end
    println("6. Sort all rows to the right")
    show(stdout, "text/plain", mesh)
    println("\n")

    return mesh
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
    startrobot_to_order = Dict(robot_num => pos_to_order(x, y, ncols, nrows)
                          for (robot_num, (x, y)) in enumerate(start))

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
    expanded_start = [pos * 3 for pos in start]
    priority = [pos_to_order(x, y, max_x, max_y) for (x, y) in start]

    #expansion_paths = plan_with_priority(start, expanded_start, [], priority)

    ### Sort compressed matrix ###
    println("----SORTING----")
    mesh = rotate_sort(mesh)
    show(stdout, "text/plain", mesh)


    ### Go to target positions
    #compression_paths = plan_with_priority(expanded_start, target, [], priority)
    #all_paths = [append!(path1, path2) for (path1, path2) in zip(expansion_paths, compression_paths)]
    #return all_paths

end

function order_to_pos(index, max_x, max_y)
    x = index % max_x
    y = floor(Int64, (index - x)/max_x)
    return [x, y]
end

function pos_to_order(x, y, max_x, max_y)
    return max_x * y + x
end



robot_rotate_sort(py"load_instance"())