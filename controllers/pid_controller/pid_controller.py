# Part 3 - PID Robot Control
# Webots e-puck PID obstacle avoidance

from controller import Robot

# -------------------------------------------------
# ROBOT SETUP
# -------------------------------------------------

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# -------------------------------------------------
# MOTORS
# -------------------------------------------------

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# -------------------------------------------------
# PROXIMITY SENSORS
# -------------------------------------------------

sensors = []

for i in range(8):
    sensor = robot.getDevice("ps" + str(i))
    sensor.enable(timestep)
    sensors.append(sensor)

print("PID controller started")

# -------------------------------------------------
# PID SETTINGS
# -------------------------------------------------

Kp = 0.008
Ki = 0.00001
Kd = 0.003

TARGET_DISTANCE = 200

BASE_SPEED = 3.0
MAX_SPEED = 6.28

integral = 0.0
previous_error = 0.0

# Escape mode prevents rapid forward/reverse switching
escape_counter = 0
ESCAPE_TIME = 20

counter = 0

# -------------------------------------------------
# MAIN LOOP
# -------------------------------------------------

while robot.step(timestep) != -1:

    # Read all 8 proximity sensors
    values = []

    for sensor in sensors:
        values.append(sensor.getValue())

    # -------------------------------------------------
    # SENSOR GROUPS
    # -------------------------------------------------

    # Right side/front
    right_obstacle = max(values[0], values[1], values[2])

    # Left side/front
    left_obstacle = max(values[5], values[6], values[7])

    # Strongest nearby obstacle
    measured_distance = max(left_obstacle, right_obstacle)

    # -------------------------------------------------
    # PID CALCULATION
    # -------------------------------------------------

    error = measured_distance - TARGET_DISTANCE

    # Only accumulate integral when an obstacle is nearby
    if measured_distance > TARGET_DISTANCE:
        integral += error
    else:
        integral = 0.0

    derivative = error - previous_error

    correction = (
        Kp * error
        + Ki * integral
        + Kd * derivative
    )

    previous_error = error

    # Limit PID correction
    correction = max(-3.0, min(3.0, correction))

    # -------------------------------------------------
    # ESCAPE DETECTION
    # -------------------------------------------------

    # If extremely close to something, begin escape mode
    if measured_distance > 800 and escape_counter == 0:
        escape_counter = ESCAPE_TIME

    # -------------------------------------------------
    # MOTOR CONTROL
    # -------------------------------------------------

    if escape_counter > 0:

        # Reverse and curve away from the stronger side
        if left_obstacle > right_obstacle:
            left_speed = -1.0
            right_speed = -3.0
        else:
            left_speed = -3.0
            right_speed = -1.0

        escape_counter -= 1
        action = "ESCAPING"

    elif measured_distance > TARGET_DISTANCE:

        # PID steering away from obstacle
        if left_obstacle > right_obstacle:
            left_speed = BASE_SPEED + correction
            right_speed = BASE_SPEED - correction
        else:
            left_speed = BASE_SPEED - correction
            right_speed = BASE_SPEED + correction

        action = "PID AVOIDANCE"

    else:

        # Clear path
        left_speed = BASE_SPEED
        right_speed = BASE_SPEED

        action = "MOVING FORWARD"

    # -------------------------------------------------
    # MOTOR LIMITS
    # -------------------------------------------------

    left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
    right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # -------------------------------------------------
    # CONSOLE OUTPUT
    # -------------------------------------------------

    counter += 1

    if counter >= 20:

        print("\n----- PID CONTROL -----")
        print("Left obstacle:", round(left_obstacle, 2))
        print("Right obstacle:", round(right_obstacle, 2))
        print("Maximum proximity:", round(measured_distance, 2))
        print("Error:", round(error, 2))
        print("PID correction:", round(correction, 2))
        print("Left motor:", round(left_speed, 2))
        print("Right motor:", round(right_speed, 2))
        print("Action:", action)

        counter = 0