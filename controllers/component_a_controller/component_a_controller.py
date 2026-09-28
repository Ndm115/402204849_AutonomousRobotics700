# Component A - Sensor Fusion and Obstacle Avoidance

from controller import Robot
import cv2
import numpy as np

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

proximity_sensors = []

for i in range(8):
    sensor = robot.getDevice("ps" + str(i))
    sensor.enable(timestep)
    proximity_sensors.append(sensor)

print("Proximity sensors enabled")


# -------------------------------------------------
# CAMERA
# -------------------------------------------------

camera = robot.getDevice("camera")
camera.enable(timestep)

camera_width = camera.getWidth()
camera_height = camera.getHeight()

print("Camera enabled")
print("Camera resolution:", camera_width, "x", camera_height)


# -------------------------------------------------
# SETTINGS
# -------------------------------------------------

# Based on our proximity sensor testing
OBSTACLE_THRESHOLD = 200

# Camera threshold:
# If enough edges are visible, the camera has detected
# significant visual features in the environment.
CAMERA_EDGE_THRESHOLD = 100

FORWARD_SPEED = 3.0
TURN_SPEED = 2.0

counter = 0


# -------------------------------------------------
# MAIN LOOP
# -------------------------------------------------

while robot.step(timestep) != -1:

    # -------------------------------------------------
    # CAMERA IMAGE PROCESSING
    # -------------------------------------------------

    camera_image = camera.getImage()

    # Convert Webots image into NumPy array
    image = np.frombuffer(camera_image, np.uint8)
    image = image.reshape((camera_height, camera_width, 4))

    # Remove alpha channel
    image = image[:, :, :3]

    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect edges using Canny
    edges = cv2.Canny(gray_image, 50, 150)

    # Count edge pixels
    edge_count = cv2.countNonZero(edges)

    # Camera detection decision
    camera_detected = edge_count > CAMERA_EDGE_THRESHOLD


    # -------------------------------------------------
    # PROXIMITY SENSOR PROCESSING
    # -------------------------------------------------

    values = []

    for sensor in proximity_sensors:
        values.append(sensor.getValue())

    # Highest proximity reading
    max_proximity = max(values)

    # Proximity detection decision
    proximity_detected = max_proximity > OBSTACLE_THRESHOLD


    # -------------------------------------------------
    # SENSOR FUSION
    # -------------------------------------------------

    # Combine camera and proximity information into
    # one unified perception result.

    if proximity_detected and camera_detected:

        # Both sensors provide evidence
        fused_state = "HIGH OBSTACLE CONFIDENCE"

    elif proximity_detected and not camera_detected:

        # Proximity detects something nearby,
        # even though camera edges are weak
        fused_state = "NEARBY OBSTACLE"

    elif not proximity_detected and camera_detected:

        # Camera sees features but proximity sensors
        # indicate that nothing is immediately close
        fused_state = "DISTANT VISUAL FEATURE"

    else:

        # Neither modality detects anything important
        fused_state = "CLEAR"


    # -------------------------------------------------
    # AUTONOMOUS MOVEMENT
    # -------------------------------------------------

    # A nearby obstacle always takes priority for safety.
    if proximity_detected:

        left_motor.setVelocity(-TURN_SPEED)
        right_motor.setVelocity(TURN_SPEED)

    else:

        left_motor.setVelocity(FORWARD_SPEED)
        right_motor.setVelocity(FORWARD_SPEED)


    # -------------------------------------------------
    # CONSOLE OUTPUT
    # -------------------------------------------------

    counter += 1

    if counter >= 20:

        print("\n========== SENSOR FUSION ==========")

        # Camera information
        print("Camera edge pixels:", edge_count)

        if camera_detected:
            print("Camera: FEATURES DETECTED")
        else:
            print("Camera: NO SIGNIFICANT FEATURES")

        # Proximity information
        print("Maximum proximity:", round(max_proximity, 2))

        if proximity_detected:
            print("Proximity: OBSTACLE NEARBY")
        else:
            print("Proximity: CLEAR")

        # Combined result
        print("FUSED PERCEPTION:", fused_state)

        # Robot action
        if proximity_detected:
            print("ACTION: TURNING")
        else:
            print("ACTION: MOVING FORWARD")

        counter = 0