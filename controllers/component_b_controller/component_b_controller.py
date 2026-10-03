# ============================================================
# COMPONENT B - PART 1
# MULTI-AGENT ROBOT CONTROLLER
#
# Three autonomous e-puck robots cooperate in a shared arena.
# Each robot has:
# - Its own identity
# - An assigned coverage role
# - Autonomous obstacle avoidance
# - Communication with the other agents
#
# Communication topology: All-to-All
# ============================================================

from controller import Robot


# ------------------------------------------------------------
# ROBOT SETUP
# ------------------------------------------------------------

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# All robots use this same controller.
# Their Webots names identify the individual agents.
robot_name = robot.getName()

print("\n======================================")
print("COMPONENT B - MULTI-AGENT SYSTEM")
print("======================================")
print("Robot:", robot_name)


# ------------------------------------------------------------
# AGENT ROLES
# ------------------------------------------------------------

# Each robot is responsible for a different coverage zone.

if robot_name == "ROBOT_1":
    role = "ZONE A COVERAGE"

elif robot_name == "ROBOT_2":
    role = "ZONE B COVERAGE"

elif robot_name == "ROBOT_3":
    role = "ZONE C COVERAGE"

else:
    role = "UNASSIGNED"

print("Role:", role)


# ------------------------------------------------------------
# MOTORS
# ------------------------------------------------------------

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Set motors to velocity-control mode.
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

print("Proximity sensors enabled")


# ------------------------------------------------------------
# COMMUNICATION DEVICES
# ------------------------------------------------------------

# Every e-puck contains an emitter and receiver.
# All robots use the same communication channel,
# creating an all-to-all communication topology.

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")

receiver.enable(timestep)

COMMUNICATION_CHANNEL = 1

emitter.setChannel(COMMUNICATION_CHANNEL)
receiver.setChannel(COMMUNICATION_CHANNEL)

print("Communication enabled on channel", COMMUNICATION_CHANNEL)


# ------------------------------------------------------------
# MOVEMENT SETTINGS
# ------------------------------------------------------------

FORWARD_SPEED = 3.0
TURN_SPEED = 2.0

OBSTACLE_THRESHOLD = 200


# ------------------------------------------------------------
# COMMUNICATION SETTINGS
# ------------------------------------------------------------

# Counter prevents the robots from flooding the console
# with messages every simulation step.

communication_counter = 0

COMMUNICATION_INTERVAL = 50


# ------------------------------------------------------------
# MAIN AUTONOMOUS LOOP
# ------------------------------------------------------------

while robot.step(timestep) != -1:

    # --------------------------------------------------------
    # READ PROXIMITY SENSORS
    # --------------------------------------------------------

    sensor_values = []

    for sensor in proximity_sensors:
        sensor_values.append(sensor.getValue())

    # Front/right sensors
    right_obstacle = (
        sensor_values[0] > OBSTACLE_THRESHOLD
        or sensor_values[1] > OBSTACLE_THRESHOLD
    )

    # Front/left sensors
    left_obstacle = (
        sensor_values[6] > OBSTACLE_THRESHOLD
        or sensor_values[7] > OBSTACLE_THRESHOLD
    )


    # --------------------------------------------------------
    # AUTONOMOUS OBSTACLE AVOIDANCE
    # --------------------------------------------------------

    if left_obstacle:

        # Obstacle detected on the left.
        # Turn towards the right.

        left_speed = TURN_SPEED
        right_speed = -TURN_SPEED

    elif right_obstacle:

        # Obstacle detected on the right.
        # Turn towards the left.

        left_speed = -TURN_SPEED
        right_speed = TURN_SPEED

    else:

        # No nearby obstacle.
        # Continue moving forward.

        left_speed = FORWARD_SPEED
        right_speed = FORWARD_SPEED


    # --------------------------------------------------------
    # APPLY MOTOR SPEEDS
    # --------------------------------------------------------

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


    # --------------------------------------------------------
    # BROADCAST AGENT STATUS
    # --------------------------------------------------------

    communication_counter += 1

    if communication_counter >= COMMUNICATION_INTERVAL:

        # Message contains the agent identity and its role.

        message = robot_name + "|" + role

        emitter.send(message.encode("utf-8"))

        print(robot_name, "sent:", message)

        communication_counter = 0


    # --------------------------------------------------------
    # RECEIVE MESSAGES FROM OTHER AGENTS
    # --------------------------------------------------------

    while receiver.getQueueLength() > 0:
        received_message = receiver.getString()

        # Ignore the robot's own broadcast.
        if not received_message.startswith(robot_name + "|"):
            print(robot_name, "received:", received_message)

        receiver.nextPacket()