# ============================================================
# COMPONENT A - PART 2: MOTION PLANNING AND PATHFINDING
# Dijkstra's Algorithm and A* Algorithm
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
import heapq
import time


# -------------------------------------------------
# CREATE OCCUPANCY GRID
# -------------------------------------------------

# 0 = free space
# 1 = obstacle

grid = np.zeros((10, 10), dtype=int)

# Add obstacles
grid[2:5, 3] = 1
grid[5, 3:7] = 1
grid[2:6, 7] = 1


# -------------------------------------------------
# START AND GOAL
# -------------------------------------------------

start = (8, 1)
goal = (1, 8)

grid[start] = 0
grid[goal] = 0


# -------------------------------------------------
# POSSIBLE MOVEMENTS
# -------------------------------------------------

# Robot can move:
# up, down, left, right

directions = [
    (-1, 0),
    (1, 0),
    (0, -1),
    (0, 1)
]


# -------------------------------------------------
# DIJKSTRA'S ALGORITHM
# -------------------------------------------------

def dijkstra(grid, start, goal):

    queue = [(0, start)]

    distances = {start: 0}
    previous = {}

    nodes_explored = 0

    while queue:

        current_distance, current = heapq.heappop(queue)

        nodes_explored += 1

        # Stop when goal is reached
        if current == goal:
            break

        # Check neighbouring cells
        for direction in directions:

            new_row = current[0] + direction[0]
            new_col = current[1] + direction[1]

            neighbour = (new_row, new_col)

            # Check grid boundaries
            if (
                0 <= new_row < grid.shape[0]
                and 0 <= new_col < grid.shape[1]
            ):

                # Only travel through free cells
                if grid[neighbour] == 0:

                    new_distance = current_distance + 1

                    if (
                        neighbour not in distances
                        or new_distance < distances[neighbour]
                    ):

                        distances[neighbour] = new_distance
                        previous[neighbour] = current

                        heapq.heappush(
                            queue,
                            (new_distance, neighbour)
                        )

    # Reconstruct final path
    path = []

    if goal in distances:

        current = goal

        while current != start:
            path.append(current)
            current = previous[current]

        path.append(start)
        path.reverse()

    return path, nodes_explored


# -------------------------------------------------
# A* HEURISTIC
# -------------------------------------------------

def heuristic(position, goal):

    # Manhattan distance
    # Suitable because movement is limited to
    # up, down, left and right.

    return (
        abs(position[0] - goal[0])
        + abs(position[1] - goal[1])
    )


# -------------------------------------------------
# A* ALGORITHM
# -------------------------------------------------

def astar(grid, start, goal):

    # Queue stores:
    # (estimated total cost, current position)

    queue = [(heuristic(start, goal), start)]

    # Actual cost from start
    g_cost = {start: 0}

    previous = {}

    nodes_explored = 0

    while queue:

        current_f, current = heapq.heappop(queue)

        nodes_explored += 1

        # Stop when goal is reached
        if current == goal:
            break

        # Check neighbouring cells
        for direction in directions:

            new_row = current[0] + direction[0]
            new_col = current[1] + direction[1]

            neighbour = (new_row, new_col)

            # Check grid boundaries
            if (
                0 <= new_row < grid.shape[0]
                and 0 <= new_col < grid.shape[1]
            ):

                # Only travel through free cells
                if grid[neighbour] == 0:

                    new_g_cost = g_cost[current] + 1

                    if (
                        neighbour not in g_cost
                        or new_g_cost < g_cost[neighbour]
                    ):

                        g_cost[neighbour] = new_g_cost
                        previous[neighbour] = current

                        # A* formula:
                        # f(n) = g(n) + h(n)

                        f_cost = (
                            new_g_cost
                            + heuristic(neighbour, goal)
                        )

                        heapq.heappush(
                            queue,
                            (f_cost, neighbour)
                        )

    # Reconstruct final path
    path = []

    if goal in g_cost:

        current = goal

        while current != start:
            path.append(current)
            current = previous[current]

        path.append(start)
        path.reverse()

    return path, nodes_explored


# -------------------------------------------------
# RUN DIJKSTRA AND MEASURE EXECUTION TIME
# -------------------------------------------------

start_time = time.perf_counter()

dijkstra_path, dijkstra_nodes = dijkstra(
    grid,
    start,
    goal
)

dijkstra_time = time.perf_counter() - start_time


# -------------------------------------------------
# RUN A* AND MEASURE EXECUTION TIME
# -------------------------------------------------

start_time = time.perf_counter()

astar_path, astar_nodes = astar(
    grid,
    start,
    goal
)

astar_time = time.perf_counter() - start_time


# -------------------------------------------------
# RESULTS
# -------------------------------------------------

print("Occupancy Grid:")
print(grid)

print("\nStart:", start)
print("Goal:", goal)


# -------------------------------------------------
# DIJKSTRA RESULTS
# -------------------------------------------------

print("\n----- DIJKSTRA RESULTS -----")

print("Path:", dijkstra_path)
print("Path Cost:", len(dijkstra_path) - 1)
print("Nodes Explored:", dijkstra_nodes)

print(
    "Execution Time:",
    round(dijkstra_time * 1000, 4),
    "ms"
)


# -------------------------------------------------
# A* RESULTS
# -------------------------------------------------

print("\n----- A* RESULTS -----")

print("Path:", astar_path)
print("Path Cost:", len(astar_path) - 1)
print("Nodes Explored:", astar_nodes)

print(
    "Execution Time:",
    round(astar_time * 1000, 4),
    "ms"
)


# -------------------------------------------------
# ALGORITHM COMPARISON
# -------------------------------------------------

print("\n----- ALGORITHM COMPARISON -----")

print(
    "Dijkstra Path Cost:",
    len(dijkstra_path) - 1
)

print(
    "A* Path Cost:",
    len(astar_path) - 1
)

print(
    "Dijkstra Nodes Explored:",
    dijkstra_nodes
)

print(
    "A* Nodes Explored:",
    astar_nodes
)

print(
    "Dijkstra Execution Time:",
    round(dijkstra_time * 1000, 4),
    "ms"
)

print(
    "A* Execution Time:",
    round(astar_time * 1000, 4),
    "ms"
)


# -------------------------------------------------
# VISUALISE BOTH PATHS
# -------------------------------------------------

plt.imshow(grid)

# Start
plt.scatter(
    start[1],
    start[0],
    s=150,
    marker="o",
    label="Start"
)

# Goal
plt.scatter(
    goal[1],
    goal[0],
    s=150,
    marker="*",
    label="Goal"
)


# -------------------------------------------------
# DIJKSTRA PATH
# -------------------------------------------------

if dijkstra_path:

    dijkstra_rows = [
        position[0]
        for position in dijkstra_path
    ]

    dijkstra_cols = [
        position[1]
        for position in dijkstra_path
    ]

    plt.plot(
        dijkstra_cols,
        dijkstra_rows,
        marker="o",
        label="Dijkstra Path"
    )


# -------------------------------------------------
# A* PATH
# -------------------------------------------------

if astar_path:

    astar_rows = [
        position[0]
        for position in astar_path
    ]

    astar_cols = [
        position[1]
        for position in astar_path
    ]

    plt.plot(
        astar_cols,
        astar_rows,
        marker="x",
        linestyle="--",
        label="A* Path"
    )


# -------------------------------------------------
# GRAPH SETTINGS
# -------------------------------------------------

plt.title("Dijkstra vs A* Path Planning")

plt.xlabel("X Position")
plt.ylabel("Y Position")

plt.xticks(range(10))
plt.yticks(range(10))

plt.grid()

plt.legend()

plt.savefig(
    "outputs/path_planning_comparison.png",
    dpi=300,
    bbox_inches="tight"
)

plt.show()