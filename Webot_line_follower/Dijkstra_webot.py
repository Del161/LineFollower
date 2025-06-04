import heapq
import matplotlib.pyplot as plt
import numpy as np


def create_grid():
    grid = np.array([
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0]
        ])
    return grid

def create_costs():
    costs = np.array([
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,]
    ])
    return costs
# the costs are how much effor it takes to traverse an area
# turns cost more time and thus have a higher cost
# this can also be used to avoid obstacles


def plot_grid(grid, path):
    """
    Plots the grid with optional path overlay.
    Args:
    - grid (2D array): The environment.
    - path (list of tuples): The path to overlay (optional).
    """
    costs = create_costs()
    plt.figure(figsize=(5, 5))

    # Plot the grid with inverted colors: 0 is black, 1 is white 
    # plt.imshow(grid, cmap="Greys", origin="upper")
    plt.imshow(grid, cmap="gray", origin="upper")

    # Anootate costs
    rows, cols = grid.shape
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 0:
                plt.text(j, i, f'{costs[i][j]}', ha='center', va='center', color=('lightgray'), fontsize=10)

    if path:
        i = []
        j = []
        for (x, y) in path:
            plt.scatter(y, x, color="red")
            i.append(x)
            j.append(y)
        plt.plot(j,i, 'r')
    plt.title("Map of the Environment")
    # plt.grid(True)
    plt.show()

def dijkstra(grid, start, goal):
    
    # creates the grid and costs using previous functions
    grid = create_grid()
    costs = create_costs()
    
    rows, cols = grid.shape
    visited = set()
    distances = {start: 0}  # Distance to the start node is 0
    parents = {start: None}  # A parent is the node that preceeds the current one: used to reconstruct the path

    priority_queue = []  # Priority queue ensures that nodes are processed in order of increasing distance
    heapq.heappush(priority_queue, (0, start))  # Initializes queue with the start node and distance to start

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        #  If a node has already been visited, it means the shortest distance to that node has been determined.
        if current_node in visited:
            continue 
        visited.add(current_node)

        # If the current node is the goal node, then the path has been found.
        if current_node == goal:
            break

        # Gets the coordinates of each neighboring cell
        for direction in directions:
            neighbor = (current_node[0] + direction[0], current_node[1] + direction[1])
            # Can only move within the boundaries of the world, and if there's no obstacle
            if (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] == 0):
                distance = current_distance + costs[neighbor]  # Use cost from the `costs` array

                # Updates the shortest known distance to a neighboring node and prepares it for further exploration
                if neighbor not in distances or distance < distances[neighbor]:
                    distances[neighbor] = distance  # Updates the shortest known distance to the neighbor
                    parents[neighbor] = current_node # Makes the current node be the parent of the neighbor to reconstruct the shortest path
                    heapq.heappush(priority_queue, (distance, neighbor))  # Adds the neighbor to the priority queue with its updated distance as the priority
    # Reconstruct path from the goal to the start
    path = []
    node = goal
    while node is not None:
        if node not in parents:
            print(f"Error: Node {node} has no parent. Path reconstruction failed.")
            return []
        path.append(node)
        node = parents[node]  # Gets the parent of the node
    path.reverse()  # Reverse the path to make it from start to the goal node
    return path