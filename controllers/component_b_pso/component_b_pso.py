# ============================================================
# COMPONENT B - PART 2
# PARTICLE SWARM OPTIMISATION (PSO)
#
# Three e-puck robots cooperate as a swarm.
#
# Each robot:
# - Tracks its current position
# - Stores its personal-best position
# - Shares its best position with the other agents
# - Learns the global-best position found by the swarm
# - Moves toward the swarm solution
# - Avoids nearby robots
# - Settles inside a target region
#
# This demonstrates emergent swarm convergence without
# requiring all three physical robots to occupy one point.
# ============================================================

from controller import Supervisor
import math
import random


# ------------------------------------------------------------
# ROBOT SETUP
# ------------------------------------------------------------

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

robot_name = robot.getName()

print("\n======================================")
print("COMPONENT B - PARTICLE SWARM")
print("======================================")
print("Robot:", robot_name)


# ------------------------------------------------------------
# TARGET
# ------------------------------------------------------------

# Common optimisation target.

TARGET_X = 0.0
TARGET_Y = 0.0


# ------------------------------------------------------------
# SWARM SETTLING DISTANCES
# ------------------------------------------------------------

# The robots share the same target but stop at slightly
# different distances from it.
#
# This prevents several physical robots from attempting
# to occupy exactly the same location.

if robot_name == "ROBOT_1":
    SETTLE_DISTANCE = 0.05

elif robot_name == "ROBOT_2":
    SETTLE_DISTANCE = 0.09

elif robot_name == "ROBOT_3":
    SETTLE_DISTANCE = 0.13

else:
    SETTLE_DISTANCE = 0.10


# ------------------------------------------------------------
# ROBOT POSITION
# ------------------------------------------------------------

robot_node = robot.getSelf()


# ------------------------------------------------------------
# MOTORS
# ------------------------------------------------------------

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


# ------------------------------------------------------------
# PROXIMITY SENSORS
# ------------------------------------------------------------

proximity_sensors = []

for i in range(8):
    sensor = robot.getDevice("ps" + str(i))
    sensor.enable(timestep)
    proximity_sensors.append(sensor)

OBSTACLE_THRESHOLD = 150

print("Proximity sensors enabled")


# ------------------------------------------------------------
# COMMUNICATION
# ------------------------------------------------------------

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")

receiver.enable(timestep)

COMMUNICATION_CHANNEL = 1

emitter.setChannel(COMMUNICATION_CHANNEL)
receiver.setChannel(COMMUNICATION_CHANNEL)

print("Communication enabled")


# ------------------------------------------------------------
# PSO PARAMETERS
# ------------------------------------------------------------

# Inertia keeps some influence from previous movement.

INERTIA = 0.5

# Cognitive component attracts the robot toward
# its own personal best.

COGNITIVE = 1.0

# Social component attracts the robot toward
# the swarm global best.

SOCIAL = 1.5


# ------------------------------------------------------------
# MOVEMENT SETTINGS
# ------------------------------------------------------------

FORWARD_SPEED = 3.0
TURN_SPEED = 2.0

UPDATE_INTERVAL = 15


# ------------------------------------------------------------
# DISTANCE FUNCTION
# ------------------------------------------------------------

def distance_to_target(px, py):
    return math.sqrt((TARGET_X - px) ** 2 + (TARGET_Y - py) ** 2)


# ------------------------------------------------------------
# INITIAL POSITION
# ------------------------------------------------------------

position = robot_node.getPosition()

x = position[0]
y = position[1]


# ------------------------------------------------------------
# PERSONAL BEST
# ------------------------------------------------------------

personal_best_x = x
personal_best_y = y

personal_best_distance = distance_to_target(x, y)


# ------------------------------------------------------------
# GLOBAL BEST
# ------------------------------------------------------------

global_best_x = personal_best_x
global_best_y = personal_best_y

global_best_distance = personal_best_distance


# ------------------------------------------------------------
# PSO VELOCITY
# ------------------------------------------------------------

velocity_x = 0.0
velocity_y = 0.0


# ------------------------------------------------------------
# COUNTERS
# ------------------------------------------------------------

counter = 0
iteration = 0


# ------------------------------------------------------------
# MAIN LOOP
# ------------------------------------------------------------

while robot.step(timestep) != -1:
    counter += 1


    # --------------------------------------------------------
    # READ PROXIMITY SENSORS
    # --------------------------------------------------------

    sensor_values = []

    for sensor in proximity_sensors:
        sensor_values.append(sensor.getValue())

    # Front-right obstacle detection.

    right_obstacle = (
        sensor_values[0] > OBSTACLE_THRESHOLD
        or sensor_values[1] > OBSTACLE_THRESHOLD
    )

    # Front-left obstacle detection.

    left_obstacle = (
        sensor_values[6] > OBSTACLE_THRESHOLD
        or sensor_values[7] > OBSTACLE_THRESHOLD
    )


    # --------------------------------------------------------
    # PSO UPDATE
    # --------------------------------------------------------

    if counter >= UPDATE_INTERVAL:

        # ----------------------------------------------------
        # CURRENT POSITION
        # ----------------------------------------------------

        position = robot_node.getPosition()

        x = position[0]
        y = position[1]

        current_distance = distance_to_target(x, y)


        # ----------------------------------------------------
        # UPDATE PERSONAL BEST
        # ----------------------------------------------------

        if current_distance < personal_best_distance:
            personal_best_x = x
            personal_best_y = y
            personal_best_distance = current_distance


        # ----------------------------------------------------
        # SHARE PERSONAL BEST
        # ----------------------------------------------------

        message = (
            robot_name
            + "|"
            + str(personal_best_x)
            + "|"
            + str(personal_best_y)
            + "|"
            + str(personal_best_distance)
        )

        emitter.send(message.encode("utf-8"))


        # ----------------------------------------------------
        # RECEIVE OTHER AGENTS' BEST POSITIONS
        # ----------------------------------------------------

        while receiver.getQueueLength() > 0:
            received = receiver.getString()
            parts = received.split("|")

            if len(parts) == 4:
                sender = parts[0]

                if sender != robot_name:
                    received_x = float(parts[1])
                    received_y = float(parts[2])
                    received_distance = float(parts[3])

                    # Adopt a better solution discovered
                    # by another member of the swarm.

                    if received_distance < global_best_distance:
                        global_best_x = received_x
                        global_best_y = received_y
                        global_best_distance = received_distance

            receiver.nextPacket()


        # ----------------------------------------------------
        # CHECK OWN BEST
        # ----------------------------------------------------

        if personal_best_distance < global_best_distance:
            global_best_x = personal_best_x
            global_best_y = personal_best_y
            global_best_distance = personal_best_distance


        # ----------------------------------------------------
        # PSO VELOCITY UPDATE
        # ----------------------------------------------------

        r1 = random.random()
        r2 = random.random()

        velocity_x = (
            INERTIA * velocity_x
            + COGNITIVE * r1 * (personal_best_x - x)
            + SOCIAL * r2 * (global_best_x - x)
        )

        velocity_y = (
            INERTIA * velocity_y
            + COGNITIVE * r1 * (personal_best_y - y)
            + SOCIAL * r2 * (global_best_y - y)
        )


        # ----------------------------------------------------
        # DESIRED DIRECTION
        # ----------------------------------------------------

        desired_angle = math.atan2(velocity_y, velocity_x)


        # Read current robot orientation.

        orientation = robot_node.getOrientation()

        current_angle = math.atan2(orientation[3], orientation[0])

        angle_error = desired_angle - current_angle


        # Keep angle between -pi and +pi.

        while angle_error > math.pi:
            angle_error -= 2 * math.pi

        while angle_error < -math.pi:
            angle_error += 2 * math.pi


        # ----------------------------------------------------
        # MOVEMENT DECISION
        # ----------------------------------------------------

        # Each robot stops once it enters its assigned
        # settling distance around the common target.

        if current_distance <= SETTLE_DISTANCE:
            left_speed = 0.0
            right_speed = 0.0

            action = "SETTLED NEAR TARGET"


        # Avoid nearby robot/obstacle on the left.

        elif left_obstacle:
            left_speed = TURN_SPEED
            right_speed = -TURN_SPEED

            action = "AVOIDING LEFT OBSTACLE"


        # Avoid nearby robot/obstacle on the right.

        elif right_obstacle:
            left_speed = -TURN_SPEED
            right_speed = TURN_SPEED

            action = "AVOIDING RIGHT OBSTACLE"


        # Turn toward the direction selected by PSO.

        elif abs(angle_error) > 0.5:
            if angle_error > 0:
                left_speed = -TURN_SPEED
                right_speed = TURN_SPEED

            else:
                left_speed = TURN_SPEED
                right_speed = -TURN_SPEED

            action = "TURNING TOWARD SWARM BEST"


        # Correct heading: move forward.

        else:
            left_speed = FORWARD_SPEED
            right_speed = FORWARD_SPEED

            action = "MOVING TOWARD SWARM BEST"


        # ----------------------------------------------------
        # APPLY MOTOR SPEEDS
        # ----------------------------------------------------

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


        # ----------------------------------------------------
        # DISPLAY SWARM PROGRESS
        # ----------------------------------------------------

        iteration += 1

        print(
            robot_name,
            "- Iteration:", iteration,
            "- Distance:", round(current_distance, 3),
            "- Global Best:", round(global_best_distance, 3),
            "- Settle Distance:", SETTLE_DISTANCE,
            "- Action:", action
        )

        counter = 0