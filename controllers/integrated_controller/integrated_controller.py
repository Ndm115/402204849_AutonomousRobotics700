# ============================================================
# COMPONENT A - INTEGRATED AUTONOMOUS NAVIGATION
#
# A* PATH PLANNING
# MULTIPLE NAVIGATION GOALS
# PID PATH TRACKING
# PROXIMITY OBSTACLE AVOIDANCE
# TRAJECTORY RECORDING
# ============================================================

from controller import Supervisor
import math
import csv
import heapq


# ============================================================
# ROBOT SETUP
# ============================================================

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

robot_node = robot.getFromDef("EPUCK")

if robot_node is None:
    raise RuntimeError("ERROR: DEF EPUCK was not found.")

print("Robot node EPUCK found successfully")


# ============================================================
# MOTORS
# ============================================================

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


# ============================================================
# PROXIMITY SENSORS
# ============================================================

sensors = []

for i in range(8):
    sensor = robot.getDevice("ps" + str(i))
    sensor.enable(timestep)
    sensors.append(sensor)

print("Proximity sensors enabled")


# ============================================================
# CAMERA
# ============================================================

camera = robot.getDevice("camera")
camera.enable(timestep)

print("Camera enabled")
print("Camera resolution:", camera.getWidth(), "x", camera.getHeight())


# ============================================================
# MOVEMENT SETTINGS
# ============================================================

MAX_SPEED = 6.28
FORWARD_SPEED = 2.0
TURN_SPEED = 1.5

WAYPOINT_TOLERANCE = 0.035
GOAL_TOLERANCE = 0.04

TURN_THRESHOLD = 0.25


# ============================================================
# OBSTACLE AVOIDANCE SETTINGS
# ============================================================

OBSTACLE_ENTER = 350
OBSTACLE_EXIT = 180

avoiding_obstacle = False
avoid_direction = 0

escape_steps = 0
ESCAPE_DURATION = 15


# ============================================================
# PID SETTINGS
# ============================================================

Kp = 1.5
Ki = 0.01
Kd = 0.2

integral = 0.0
previous_error = 0.0


# ============================================================
# INITIAL POSITION
# ============================================================

initial_position = robot_node.getPosition()

start_x = initial_position[0]
start_y = initial_position[1]

print("\nInitial position:")
print("X =", round(start_x, 3))
print("Y =", round(start_y, 3))


# ============================================================
# OCCUPANCY GRID
# ============================================================

# Smaller cells give A* better resolution.
CELL_SIZE = 0.025

# 41 x 41 grid:
# approximately -0.50 m to +0.50 m
GRID_SIZE = 41
GRID_CENTRE = GRID_SIZE // 2

# Physical clearance around obstacles.
SAFETY_MARGIN = 0.055

grid = []

for row in range(GRID_SIZE):
    new_row = []

    for col in range(GRID_SIZE):
        new_row.append(0)

    grid.append(new_row)


# ============================================================
# WORLD / GRID CONVERSION
# ============================================================

def world_to_grid(x, y):
    col = round(x / CELL_SIZE) + GRID_CENTRE
    row = GRID_CENTRE - round(y / CELL_SIZE)
    return row, col


def grid_to_world(cell):
    row, col = cell

    x = (col - GRID_CENTRE) * CELL_SIZE
    y = (GRID_CENTRE - row) * CELL_SIZE
    return round(x, 3), round(y, 3)


# ============================================================
# ADD PHYSICAL BOX TO GRID
# ============================================================

def add_box_obstacle(centre_x, centre_y, size_x, size_y):
    minimum_x = centre_x - size_x / 2 - SAFETY_MARGIN
    maximum_x = centre_x + size_x / 2 + SAFETY_MARGIN
    minimum_y = centre_y - size_y / 2 - SAFETY_MARGIN
    maximum_y = centre_y + size_y / 2 + SAFETY_MARGIN

    for row in range(GRID_SIZE):
        for col in range(GRID_SIZE):
            x, y = grid_to_world((row, col))

            if minimum_x <= x <= maximum_x and minimum_y <= y <= maximum_y:
                grid[row][col] = 1


# ============================================================
# ACTUAL WEBOTS OBSTACLES
# ============================================================

# BOX 1
# translation -0.20 0.22 0.10
# size         0.15 0.25 0.20

add_box_obstacle(-0.20, 0.22, 0.15, 0.25)


# BOX 2
# translation -0.18 -0.25 0.10
# size         0.25 0.12 0.20

add_box_obstacle(-0.18, -0.25, 0.25, 0.12)


# BOX 3
# translation 0.15 0.05 0.10
# size        0.20 0.20 0.20

add_box_obstacle(0.15, 0.05, 0.20, 0.20)


# ============================================================
# A* MOVEMENT
# ============================================================

directions = [
    (-1, 0),
    (1, 0),
    (0, -1),
    (0, 1)
]


# ============================================================
# HEURISTIC
# ============================================================

def heuristic(position, goal):
    return abs(position[0] - goal[0]) + abs(position[1] - goal[1])


# ============================================================
# A* ALGORITHM
# ============================================================

def astar(grid, start, goal):
    queue = [(heuristic(start, goal), start)]
    g_cost = {start: 0}
    previous = {}
    visited = set()
    nodes_explored = 0

    while queue:
        current_f, current = heapq.heappop(queue)

        if current in visited:
            continue

        visited.add(current)
        nodes_explored += 1

        if current == goal:
            break

        for direction in directions:
            new_row = current[0] + direction[0]
            new_col = current[1] + direction[1]
            neighbour = (new_row, new_col)

            if 0 <= new_row < GRID_SIZE and 0 <= new_col < GRID_SIZE:
                if grid[new_row][new_col] == 0:
                    new_cost = g_cost[current] + 1

                    if neighbour not in g_cost or new_cost < g_cost[neighbour]:
                        g_cost[neighbour] = new_cost
                        previous[neighbour] = current
                        f_cost = new_cost + heuristic(neighbour, goal)
                        heapq.heappush(queue, (f_cost, neighbour))

    path = []

    if goal not in g_cost:
        return [], nodes_explored

    current = goal

    while current != start:
        path.append(current)
        current = previous[current]

    path.append(start)
    path.reverse()

    return path, nodes_explored


# ============================================================
# SIMPLIFY A* PATH
# ============================================================

def simplify_path(points):
    if len(points) <= 2:
        return points

    simplified = [points[0]]

    for i in range(1, len(points) - 1):
        previous_point = points[i - 1]
        current_point = points[i]
        next_point = points[i + 1]

        direction_1 = (
            round(current_point[0] - previous_point[0], 3),
            round(current_point[1] - previous_point[1], 3)
        )

        direction_2 = (
            round(next_point[0] - current_point[0], 3),
            round(next_point[1] - current_point[1], 3)
        )

        if direction_1 != direction_2:
            simplified.append(current_point)

    simplified.append(points[-1])

    return simplified


# ============================================================
# NAVIGATION GOALS
# ============================================================

# IMPORTANT:
#
# These are DESTINATIONS, not manually defined paths.
#
# A* decides how to travel between them.
#
# The goals force the robot to navigate through more of
# the environment instead of simply travelling straight up.

navigation_goals = [
    # Lower-right region
    (0.32, -0.22),

    # Upper-left region
    (-0.38, 0.38),

    # Final destination
    (0.00, 0.32)
]


# ============================================================
# GENERATE COMPLETE A* ROUTE
# ============================================================

complete_waypoints = []
planning_start = (start_x, start_y)

total_nodes_explored = 0
total_path_cost = 0

print("\n======================================")
print("MULTI-GOAL A* PATH PLANNING")
print("======================================")

for goal_number, goal in enumerate(navigation_goals, start=1):
    start_cell = world_to_grid(planning_start[0], planning_start[1])
    goal_cell = world_to_grid(goal[0], goal[1])

    # Make sure exact start and goal cells are free.
    grid[start_cell[0]][start_cell[1]] = 0
    grid[goal_cell[0]][goal_cell[1]] = 0

    grid_path, nodes = astar(grid, start_cell, goal_cell)

    if not grid_path:
        raise RuntimeError("ERROR: A* could not reach navigation goal " + str(goal_number))

    total_nodes_explored += nodes
    total_path_cost += len(grid_path) - 1

    # Convert grid cells into Webots coordinates.
    world_path = []

    for cell in grid_path:
        world_path.append(grid_to_world(cell))

    # Simplify straight grid sections.
    simplified = simplify_path(world_path)

    # Use exact coordinates for the beginning and end.
    simplified[0] = planning_start
    simplified[-1] = goal

    # Avoid duplicating the first point when combining routes.
    if len(complete_waypoints) == 0:
        complete_waypoints.extend(simplified)
    else:
        complete_waypoints.extend(simplified[1:])

    print("\nGoal", goal_number, ":", goal)
    print("Start cell:", start_cell)
    print("Goal cell:", goal_cell)
    print("Nodes explored:", nodes)
    print("Path cost:", len(grid_path) - 1)
    print("Generated waypoints:")

    for point in simplified:
        print(" ", point)

    planning_start = goal


# ============================================================
# FINAL GENERATED ROUTE
# ============================================================

waypoints = complete_waypoints

print("\n======================================")
print("A* COMPLETE ROUTE")
print("======================================")

print("Total nodes explored:", total_nodes_explored)
print("Total path cost:", total_path_cost)
print("\nFinal A* generated waypoints:")

for i, point in enumerate(waypoints):
    print(i, (round(point[0], 3), round(point[1], 3)))


# ============================================================
# ANGLE NORMALISATION
# ============================================================

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


# ============================================================
# GET ROBOT HEADING
# ============================================================

def get_heading():
    orientation = robot_node.getOrientation()

    forward_x = orientation[0]
    forward_y = orientation[3]

    return math.atan2(forward_y, forward_x)


# ============================================================
# CONTROL VARIABLES
# ============================================================

current_waypoint = 1
trajectory = []
counter = 0


# ============================================================
# START CONTROLLER
# ============================================================

print("\n======================================")
print("INTEGRATED CONTROLLER STARTED")
print("======================================")

print("\nA* Waypoints:")

for i, point in enumerate(waypoints):
    print(i, point)


# ============================================================
# MAIN CONTROL LOOP
# ============================================================

while robot.step(timestep) != -1:

    # --------------------------------------------------------
    # POSITION
    # --------------------------------------------------------

    position = robot_node.getPosition()

    x = position[0]
    y = position[1]

    trajectory.append((x, y))


    # --------------------------------------------------------
    # HEADING
    # --------------------------------------------------------

    current_heading = get_heading()


    # --------------------------------------------------------
    # PROXIMITY SENSORS
    # --------------------------------------------------------

    values = []

    for sensor in sensors:
        values.append(sensor.getValue())

    front_left = max(values[6], values[7])
    front_right = max(values[0], values[1])
    max_front = max(front_left, front_right)


    # --------------------------------------------------------
    # COMPLETE ROUTE FINISHED
    # --------------------------------------------------------

    if current_waypoint >= len(waypoints):
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

        print("\n======================================")
        print("ALL A* NAVIGATION GOALS REACHED")
        print("======================================")

        break


    # --------------------------------------------------------
    # CURRENT A* WAYPOINT
    # --------------------------------------------------------

    target_x = waypoints[current_waypoint][0]
    target_y = waypoints[current_waypoint][1]

    dx = target_x - x
    dy = target_y - y

    distance_to_target = math.sqrt(dx * dx + dy * dy)


    # --------------------------------------------------------
    # WAYPOINT REACHED
    # --------------------------------------------------------

    if distance_to_target < WAYPOINT_TOLERANCE:
        print("\nA* waypoint", current_waypoint, "reached")

        current_waypoint += 1

        integral = 0.0
        previous_error = 0.0

        continue


    # --------------------------------------------------------
    # DESIRED HEADING
    # --------------------------------------------------------

    desired_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(desired_heading - current_heading)


    # --------------------------------------------------------
    # START OBSTACLE AVOIDANCE
    # --------------------------------------------------------

    if not avoiding_obstacle and escape_steps == 0 and max_front > OBSTACLE_ENTER:
        avoiding_obstacle = True

        # Turn away from stronger side.
        if front_left > front_right:
            avoid_direction = -1
        else:
            avoid_direction = 1

        integral = 0.0


    # --------------------------------------------------------
    # OBSTACLE AVOIDANCE
    # --------------------------------------------------------

    if avoiding_obstacle:
        if avoid_direction == 1:
            left_speed = -TURN_SPEED
            right_speed = TURN_SPEED
        else:
            left_speed = TURN_SPEED
            right_speed = -TURN_SPEED

        action = "OBSTACLE AVOIDANCE"

        if max_front < OBSTACLE_EXIT:
            avoiding_obstacle = False
            escape_steps = ESCAPE_DURATION

            integral = 0.0
            previous_error = heading_error


    # --------------------------------------------------------
    # CLEAR OBSTACLE
    # --------------------------------------------------------

    elif escape_steps > 0:
        left_speed = 1.5
        right_speed = 1.5

        escape_steps -= 1

        action = "CLEARING OBSTACLE"

        integral = 0.0
        previous_error = heading_error


    # --------------------------------------------------------
    # ALIGN ROBOT
    # --------------------------------------------------------

    elif abs(heading_error) > TURN_THRESHOLD:
        integral = 0.0
        previous_error = heading_error

        if heading_error > 0:
            left_speed = -TURN_SPEED
            right_speed = TURN_SPEED
        else:
            left_speed = TURN_SPEED
            right_speed = -TURN_SPEED

        action = "ALIGNING WITH A* WAYPOINT"


    # --------------------------------------------------------
    # PID PATH TRACKING
    # --------------------------------------------------------

    else:
        integral += heading_error
        integral = max(-10.0, min(10.0, integral))

        derivative = heading_error - previous_error
        correction = Kp * heading_error + Ki * integral + Kd * derivative

        previous_error = heading_error

        left_speed = FORWARD_SPEED - correction
        right_speed = FORWARD_SPEED + correction

        action = "PID TRACKING A* PATH"


    # --------------------------------------------------------
    # MOTOR LIMITS
    # --------------------------------------------------------

    left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
    right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))


    # --------------------------------------------------------
    # APPLY MOTOR SPEED
    # --------------------------------------------------------

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


    # --------------------------------------------------------
    # CONSOLE OUTPUT
    # --------------------------------------------------------

    counter += 1

    if counter >= 20:
        print("\n----- INTEGRATED MULTI-GOAL A* CONTROL -----")
        print("Position:", round(x, 3), round(y, 3))
        print("A* waypoint:", current_waypoint, "/", len(waypoints) - 1)
        print("Target:", round(target_x, 3), round(target_y, 3))
        print("Distance:", round(distance_to_target, 3))
        print("Current heading:", round(current_heading, 3))
        print("Desired heading:", round(desired_heading, 3))
        print("Heading error:", round(heading_error, 3))
        print("Front left:", round(front_left, 2))
        print("Front right:", round(front_right, 2))
        print("Maximum front:", round(max_front, 2))
        print("Avoidance active:", avoiding_obstacle)
        print("Left motor:", round(left_speed, 2))
        print("Right motor:", round(right_speed, 2))
        print("Action:", action)

        counter = 0


# ============================================================
# STOP MOTORS
# ============================================================

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


# ============================================================
# SAVE ACTUAL TRAJECTORY
# ============================================================

try:
    with open("../../trajectory.csv", "w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow(["x", "y"])

        for point in trajectory:
            writer.writerow([point[0], point[1]])

    print("\nTrajectory saved to trajectory.csv")

except Exception as error:
    print("\nCould not save trajectory:", error)


print("\nIntegrated multi-goal A* controller finished.")