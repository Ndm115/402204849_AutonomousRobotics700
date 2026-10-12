# ============================================================
# COMPONENT B - DISTRIBUTED AREA COVERAGE
#
# Three autonomous e-puck robots cooperate to cover
# different areas of the shared environment.
#
# ROBOT_1 -> Zone A
# ROBOT_2 -> Zone B
# ROBOT_3 -> Zone C
#
# Each robot:
# - Has its own assigned coverage zone
# - Moves autonomously toward that zone
# - Avoids nearby robots/obstacles
# - Communicates its coverage status
# - Stops when its assigned zone is reached
# ============================================================

from controller import Supervisor
import math


# ------------------------------------------------------------
# ROBOT SETUP
# ------------------------------------------------------------

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

robot_name = robot.getName()
robot_node = robot.getSelf()

print("\n======================================")
print("COMPONENT B - DISTRIBUTED COVERAGE")
print("======================================")
print("Robot:", robot_name)


# ------------------------------------------------------------
# COVERAGE ZONES
# ------------------------------------------------------------

# Each robot is assigned a different area of the arena.
# The targets are deliberately separated so that the
# distributed coverage behaviour is clearly visible.

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
# MOVEMENT SETTINGS
# ------------------------------------------------------------

FORWARD_SPEED = 3.0
TURN_SPEED = 2.0

# Robot is considered to have covered its zone
# when it is within 5 cm of the target.

TARGET_TOLERANCE = 0.05

UPDATE_INTERVAL = 15


# ------------------------------------------------------------
# DISTANCE FUNCTION
# ------------------------------------------------------------

def distance_to_target(x, y):
    return math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)


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

    # Front-right sensors.

    right_obstacle = values[0] > OBSTACLE_THRESHOLD or values[1] > OBSTACLE_THRESHOLD

    # Front-left sensors.

    left_obstacle = values[6] > OBSTACLE_THRESHOLD or values[7] > OBSTACLE_THRESHOLD


    # --------------------------------------------------------
    # MAIN UPDATE
    # --------------------------------------------------------

    if counter >= UPDATE_INTERVAL:
        iteration += 1


        # ----------------------------------------------------
        # CURRENT POSITION
        # ----------------------------------------------------

        position = robot_node.getPosition()

        x = position[0]
        y = position[1]


        # ----------------------------------------------------
        # DISTANCE FROM ASSIGNED COVERAGE ZONE
        # ----------------------------------------------------

        distance = distance_to_target(x, y)


        # ----------------------------------------------------
        # CHECK COVERAGE
        # ----------------------------------------------------

        if distance <= TARGET_TOLERANCE:
            coverage_complete = True

        else:
            coverage_complete = False

        if coverage_complete:
            status = "COVERED"

        else:
            status = "MOVING"


        # ----------------------------------------------------
        # COMMUNICATE COVERAGE STATUS
        # ----------------------------------------------------

        message = robot_name + "|" + zone_name + "|" + status

        emitter.send(message.encode("utf-8"))


        # ----------------------------------------------------
        # RECEIVE TEAMMATE STATUS
        # ----------------------------------------------------

        received_messages = []

        while receiver.getQueueLength() > 0:
            received = receiver.getString()
            received_messages.append(received)
            receiver.nextPacket()


        # ----------------------------------------------------
        # CALCULATE DIRECTION TO COVERAGE ZONE
        # ----------------------------------------------------

        desired_angle = math.atan2(target_y - y, target_x - x)


        # ----------------------------------------------------
        # CURRENT ROBOT ORIENTATION
        # ----------------------------------------------------

        orientation = robot_node.getOrientation()

        current_angle = math.atan2(orientation[3], orientation[0])


        # ----------------------------------------------------
        # ANGLE ERROR
        # ----------------------------------------------------

        angle_error = desired_angle - current_angle

        # Keep the angle between -pi and +pi.

        while angle_error > math.pi:
            angle_error -= 2 * math.pi

        while angle_error < -math.pi:
            angle_error += 2 * math.pi


        # ----------------------------------------------------
        # MOVEMENT DECISION
        # ----------------------------------------------------

        # Priority 1:
        # Assigned coverage zone has been reached.

        if coverage_complete:
            left_speed = 0.0
            right_speed = 0.0

            action = "ZONE COVERED"


        # Priority 2:
        # Avoid another robot or obstacle on the left.

        elif left_obstacle:
            left_speed = TURN_SPEED
            right_speed = -TURN_SPEED

            action = "AVOIDING OBSTACLE"


        # Avoid another robot or obstacle on the right.

        elif right_obstacle:
            left_speed = -TURN_SPEED
            right_speed = TURN_SPEED

            action = "AVOIDING OBSTACLE"


        # Priority 3:
        # Turn toward the assigned coverage zone.

        elif abs(angle_error) > 0.5:
            if angle_error > 0:
                left_speed = -TURN_SPEED
                right_speed = TURN_SPEED

            else:
                left_speed = TURN_SPEED
                right_speed = -TURN_SPEED

            action = "TURNING TO ZONE"


        # Priority 4:
        # Correct direction - move toward coverage zone.

        else:
            left_speed = FORWARD_SPEED
            right_speed = FORWARD_SPEED

            action = "MOVING TO ZONE"


        # ----------------------------------------------------
        # APPLY MOTOR SPEEDS
        # ----------------------------------------------------

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


        # ----------------------------------------------------
        # CONSOLE OUTPUT
        # ----------------------------------------------------

        print(
            robot_name, "-", zone_name,
            "- Iteration:", iteration,
            "- Distance:", round(distance, 3),
            "- Status:", status,
            "- Action:", action
        )


        # Show information received from teammates.

        for received in received_messages:
            print(robot_name, "received:", received)

        counter = 0