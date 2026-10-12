# ============================================================
# COMPONENT B - PSO DISTRIBUTED AREA COVERAGE
#
# Particle Swarm Optimisation is used by three autonomous
# e-puck robots for distributed area coverage.
#
# ROBOT_1 -> ZONE A
# ROBOT_2 -> ZONE B
# ROBOT_3 -> ZONE C
#
# Each robot:
# - Has an assigned coverage zone
# - Maintains a personal-best solution
# - Communicates its best solution to the swarm
# - Uses PSO cognitive/social/inertia terms
# - Moves toward its coverage objective
# - Avoids nearby obstacles/robots
# - Reports when its zone is covered
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
robot_node = robot.getSelf()

print("\n======================================")
print("COMPONENT B - PSO DISTRIBUTED COVERAGE")
print("======================================")
print("Robot:", robot_name)


# ------------------------------------------------------------
# DISTRIBUTED COVERAGE ZONES
# ------------------------------------------------------------

if robot_name == "ROBOT_1":
    zone_name = "ZONE A"
    target_x = -0.35
    target_y = 0.35

elif robot_name == "ROBOT_2":
    zone_name = "ZONE B"
    target_x = 0.35
    target_y = 0.35

elif robot_name == "ROBOT_3":
    zone_name = "ZONE C"
    target_x = 0.0
    target_y = -0.35

else:
    zone_name = "UNKNOWN"
    target_x = 0.0
    target_y = 0.0

print("Assigned:", zone_name)
print("Coverage target:", target_x, target_y)


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

CHANNEL = 1

emitter.setChannel(CHANNEL)
receiver.setChannel(CHANNEL)

print("Communication enabled")


# ------------------------------------------------------------
# PSO PARAMETERS
# ------------------------------------------------------------

# Inertia:
# Influence of previous movement.
INERTIA = 0.5

# Cognitive:
# Attraction toward the robot's personal best.
COGNITIVE = 1.0

# Social:
# Influence from the swarm's shared progress.
SOCIAL = 1.5


# ------------------------------------------------------------
# MOVEMENT SETTINGS
# ------------------------------------------------------------

FORWARD_SPEED = 3.0
TURN_SPEED = 2.0

TARGET_TOLERANCE = 0.05

UPDATE_INTERVAL = 15


# ------------------------------------------------------------
# DISTANCE FUNCTION
# ------------------------------------------------------------

def distance_to_zone(px, py):
    return math.sqrt((target_x - px) ** 2 + (target_y - py) ** 2)


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

personal_best_distance = distance_to_zone(x, y)


# ------------------------------------------------------------
# SWARM BEST DISTANCE
# ------------------------------------------------------------

# This stores the best coverage error reported by
# any member of the swarm.

swarm_best_distance = personal_best_distance


# ------------------------------------------------------------
# PSO VELOCITY
# ------------------------------------------------------------

velocity_x = 0.0
velocity_y = 0.0


# ------------------------------------------------------------
# VARIABLES
# ------------------------------------------------------------

counter = 0
iteration = 0

coverage_complete = False


# ------------------------------------------------------------
# MAIN LOOP
# ------------------------------------------------------------

while robot.step(timestep) != -1:
    counter += 1


    # --------------------------------------------------------
    # READ PROXIMITY SENSORS
    # --------------------------------------------------------

    values = []

    for sensor in proximity_sensors:
        values.append(sensor.getValue())

    right_obstacle = values[0] > OBSTACLE_THRESHOLD or values[1] > OBSTACLE_THRESHOLD
    left_obstacle = values[6] > OBSTACLE_THRESHOLD or values[7] > OBSTACLE_THRESHOLD


    # --------------------------------------------------------
    # PSO UPDATE
    # --------------------------------------------------------

    if counter >= UPDATE_INTERVAL:
        iteration += 1


        # ----------------------------------------------------
        # CURRENT POSITION
        # ----------------------------------------------------

        position = robot_node.getPosition()

        x = position[0]
        y = position[1]

        current_distance = distance_to_zone(x, y)


        # ----------------------------------------------------
        # UPDATE PERSONAL BEST
        # ----------------------------------------------------

        if current_distance < personal_best_distance:
            personal_best_x = x
            personal_best_y = y
            personal_best_distance = current_distance


        # ----------------------------------------------------
        # CHECK COVERAGE
        # ----------------------------------------------------

        if current_distance <= TARGET_TOLERANCE:
            coverage_complete = True
            status = "COVERED"

        else:
            coverage_complete = False
            status = "MOVING"


        # ----------------------------------------------------
        # BROADCAST PSO / COVERAGE INFORMATION
        # ----------------------------------------------------

        message = robot_name + "|" + zone_name + "|" + str(personal_best_distance) + "|" + status

        emitter.send(message.encode("utf-8"))


        # ----------------------------------------------------
        # RECEIVE SWARM INFORMATION
        # ----------------------------------------------------

        teammates_covered = 0

        while receiver.getQueueLength() > 0:
            received = receiver.getString()
            parts = received.split("|")

            if len(parts) == 4:
                sender = parts[0]
                received_zone = parts[1]
                received_best = float(parts[2])
                received_status = parts[3]

                if sender != robot_name:

                    # Update the best coverage error
                    # discovered by the swarm.

                    if received_best < swarm_best_distance:
                        swarm_best_distance = received_best

                    if received_status == "COVERED":
                        teammates_covered += 1

            receiver.nextPacket()


        # Own personal best can also become
        # the best swarm result.

        if personal_best_distance < swarm_best_distance:
            swarm_best_distance = personal_best_distance


        # ----------------------------------------------------
        # PSO UPDATE
        # ----------------------------------------------------

        r1 = random.random()
        r2 = random.random()


        # Cognitive component:
        # Return toward the best location this robot
        # has personally found.

        cognitive_x = COGNITIVE * r1 * (personal_best_x - x)
        cognitive_y = COGNITIVE * r1 * (personal_best_y - y)


        # Social component:
        # The swarm's shared progress influences how
        # strongly the robot moves toward its allocated
        # coverage objective.

        social_strength = max(0.2, min(1.0, swarm_best_distance))

        social_x = SOCIAL * r2 * social_strength * (target_x - x)
        social_y = SOCIAL * r2 * social_strength * (target_y - y)


        # Standard PSO-style velocity update.

        velocity_x = INERTIA * velocity_x + cognitive_x + social_x
        velocity_y = INERTIA * velocity_y + cognitive_y + social_y


        # ----------------------------------------------------
        # PSO DESIRED DIRECTION
        # ----------------------------------------------------

        desired_angle = math.atan2(velocity_y, velocity_x)


        # ----------------------------------------------------
        # CURRENT ROBOT ORIENTATION
        # ----------------------------------------------------

        orientation = robot_node.getOrientation()

        current_angle = math.atan2(orientation[3], orientation[0])

        angle_error = desired_angle - current_angle

        while angle_error > math.pi:
            angle_error -= 2 * math.pi

        while angle_error < -math.pi:
            angle_error += 2 * math.pi


        # ----------------------------------------------------
        # MOVEMENT DECISION
        # ----------------------------------------------------

        if coverage_complete:
            left_speed = 0.0
            right_speed = 0.0

            action = "ZONE COVERED"

        elif left_obstacle:
            left_speed = TURN_SPEED
            right_speed = -TURN_SPEED

            action = "PSO + AVOIDING OBSTACLE"

        elif right_obstacle:
            left_speed = -TURN_SPEED
            right_speed = TURN_SPEED

            action = "PSO + AVOIDING OBSTACLE"

        elif abs(angle_error) > 0.5:
            if angle_error > 0:
                left_speed = -TURN_SPEED
                right_speed = TURN_SPEED

            else:
                left_speed = TURN_SPEED
                right_speed = -TURN_SPEED

            action = "PSO TURNING TO COVERAGE ZONE"

        else:
            left_speed = FORWARD_SPEED
            right_speed = FORWARD_SPEED

            action = "PSO MOVING TO COVERAGE ZONE"


        # ----------------------------------------------------
        # APPLY MOTOR SPEED
        # ----------------------------------------------------

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


        # ----------------------------------------------------
        # OUTPUT
        # ----------------------------------------------------

        print(
            robot_name, "-", zone_name,
            "- Iteration:", iteration,
            "- Distance:", round(current_distance, 3),
            "- Personal Best:", round(personal_best_distance, 3),
            "- Swarm Best:", round(swarm_best_distance, 3),
            "- Status:", status,
            "- Action:", action
        )

        counter = 0