# ============================================================
# COMPONENT B - PART 2
# DISTRIBUTED AVERAGE CONSENSUS
#
# Three robots begin with different state values.
# They communicate their values to the other agents and
# repeatedly update toward the average value.
#
# Initial values:
# ROBOT_1 = 2.0
# ROBOT_2 = 4.0
# ROBOT_3 = 6.0
#
# Expected consensus value = 4.0
# ============================================================

from controller import Robot


# ------------------------------------------------------------
# ROBOT SETUP
# ------------------------------------------------------------

robot = Robot()
timestep = int(robot.getBasicTimeStep())

robot_name = robot.getName()


# ------------------------------------------------------------
# INITIAL AGENT STATES
# ------------------------------------------------------------

# Each robot starts with a different state value.

if robot_name == "ROBOT_1":
    state = 2.0

elif robot_name == "ROBOT_2":
    state = 4.0

elif robot_name == "ROBOT_3":
    state = 6.0

else:
    state = 0.0


print("\n======================================")
print("COMPONENT B - AVERAGE CONSENSUS")
print("======================================")

print("Robot:", robot_name)
print("Initial state:", state)


# ------------------------------------------------------------
# COMMUNICATION SETUP
# ------------------------------------------------------------

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")

receiver.enable(timestep)

COMMUNICATION_CHANNEL = 1

emitter.setChannel(COMMUNICATION_CHANNEL)
receiver.setChannel(COMMUNICATION_CHANNEL)

print("Communication enabled")


# ------------------------------------------------------------
# CONSENSUS SETTINGS
# ------------------------------------------------------------

# Consensus update rate.
#
# A value below 1 means the robot moves gradually toward
# the average rather than changing instantly.

CONSENSUS_RATE = 0.5

# Communication does not need to occur every simulation step.

UPDATE_INTERVAL = 20

counter = 0
iteration = 0


# ------------------------------------------------------------
# MAIN LOOP
# ------------------------------------------------------------

while robot.step(timestep) != -1:
    counter += 1


    # --------------------------------------------------------
    # BROADCAST CURRENT STATE
    # --------------------------------------------------------

    if counter >= UPDATE_INTERVAL:
        message = robot_name + "|" + str(state)

        emitter.send(message.encode("utf-8"))


        # ----------------------------------------------------
        # RECEIVE STATES FROM OTHER AGENTS
        # ----------------------------------------------------

        neighbour_states = []

        while receiver.getQueueLength() > 0:
            message = receiver.getString()
            parts = message.split("|")

            if len(parts) == 2:
                sender = parts[0]
                received_state = float(parts[1])

                # Do not include the robot's own broadcast.
                if sender != robot_name:
                    neighbour_states.append(received_state)

            receiver.nextPacket()


        # ----------------------------------------------------
        # AVERAGE CONSENSUS UPDATE
        # ----------------------------------------------------

        if len(neighbour_states) > 0:

            # Include the robot's own current state.
            all_states = [state] + neighbour_states

            average_state = sum(all_states) / len(all_states)

            # Move gradually toward the local average.
            state = state + CONSENSUS_RATE * (average_state - state)


        # ----------------------------------------------------
        # DISPLAY CONSENSUS PROGRESS
        # ----------------------------------------------------

        iteration += 1

        print(robot_name, "- Iteration:", iteration, "- State:", round(state, 4))

        counter = 0