# ============================================================
# COMPONENT B - MACHINE LEARNING NAVIGATION
#
# Uses the trained neural network from:
# machine_learning/navigation_model.pkl
#
# Inputs:
#   Left proximity value
#   Right proximity value
#
# Neural network outputs:
#   0 = FORWARD
#   1 = TURN LEFT
#   2 = TURN RIGHT
# ============================================================

from controller import Robot
import numpy as np
import pickle
import os


# ------------------------------------------------------------
# ROBOT SETUP
# ------------------------------------------------------------

robot = Robot()
timestep = int(robot.getBasicTimeStep())
robot_name = robot.getName()

print()
print("======================================")
print("COMPONENT B - ML NAVIGATION")
print("======================================")
print("Robot:", robot_name)


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

print("Proximity sensors enabled")


# ------------------------------------------------------------
# LOAD TRAINED NEURAL NETWORK
# ------------------------------------------------------------

controller_directory = os.path.dirname(os.path.abspath(__file__))

project_directory = os.path.abspath(
    os.path.join(controller_directory, "..", "..")
)

model_path = os.path.join(
    project_directory,
    "machine_learning",
    "navigation_model.pkl"
)

print("Loading neural network:", model_path)

with open(model_path, "rb") as file:
    model = pickle.load(file)

print("Trained neural network loaded")


# ------------------------------------------------------------
# MOVEMENT SETTINGS
# ------------------------------------------------------------

FORWARD_SPEED = 3.0
TURN_SPEED = 2.5

UPDATE_INTERVAL = 10

counter = 0
decision_number = 0


# ------------------------------------------------------------
# ACTION NAMES
# ------------------------------------------------------------

action_names = [
    "FORWARD",
    "TURN LEFT",
    "TURN RIGHT"
]


# ------------------------------------------------------------
# MAIN LOOP
# ------------------------------------------------------------

while robot.step(timestep) != -1:
    counter += 1

    if counter >= UPDATE_INTERVAL:
        decision_number += 1


        # ----------------------------------------------------
        # READ WEBOTS PROXIMITY SENSORS
        # ----------------------------------------------------

        sensor_values = []

        for sensor in proximity_sensors:
            sensor_values.append(sensor.getValue())


        # ----------------------------------------------------
        # CREATE LEFT / RIGHT ML INPUTS
        # ----------------------------------------------------

        # Sensors on the left/front-left side.
        left_value = max(sensor_values[5], sensor_values[6], sensor_values[7])

        # Sensors on the right/front-right side.
        right_value = max(sensor_values[0], sensor_values[1], sensor_values[2])


        # ----------------------------------------------------
        # MATCH TRAINING RANGE
        # ----------------------------------------------------

        # The network was trained using values from 0-400.
        # Limit real Webots values to that same range.

        left_value = min(left_value, 400.0)
        right_value = min(right_value, 400.0)


        # ----------------------------------------------------
        # NORMALISE INPUT
        # ----------------------------------------------------

        ml_input = np.array([
            [
                left_value / 400.0,
                right_value / 400.0
            ]
        ])


        # ----------------------------------------------------
        # NEURAL NETWORK DECISION
        # ----------------------------------------------------

        prediction = model.predict(ml_input)[0]


        # ----------------------------------------------------
        # APPLY PREDICTED ACTION
        # ----------------------------------------------------

        if prediction == 0:

            # FORWARD

            left_speed = FORWARD_SPEED
            right_speed = FORWARD_SPEED

        elif prediction == 1:

            # TURN LEFT

            left_speed = -TURN_SPEED
            right_speed = TURN_SPEED

        else:

            # TURN RIGHT

            left_speed = TURN_SPEED
            right_speed = -TURN_SPEED


        # ----------------------------------------------------
        # SEND ACTION TO MOTORS
        # ----------------------------------------------------

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


        # ----------------------------------------------------
        # OUTPUT
        # ----------------------------------------------------

        print(
            robot_name,
            "- Decision:", decision_number,
            "- Left:", round(left_value, 1),
            "- Right:", round(right_value, 1),
            "- NN Action:", action_names[prediction]
        )

        counter = 0