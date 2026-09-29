# ============================================================
# COMPONENT A - PART 3: CONTROL SYSTEM DESIGN
# PID Gain Tuning and Response Evaluation
# ============================================================

import numpy as np
import matplotlib.pyplot as plt


# ------------------------------------------------------------
# PID SIMULATION
# ------------------------------------------------------------

def run_pid(Kp, Ki, Kd):

    target = 10.0
    dt = 0.1
    steps = 150

    position = 0.0
    velocity = 0.0

    integral = 0.0
    previous_error = 0.0

    times = []
    positions = []
    errors = []

    for i in range(steps):

        time = i * dt

        # Calculate error
        error = target - position

        # PID calculation
        integral += error * dt
        derivative = (error - previous_error) / dt

        control = (Kp * error + Ki * integral + Kd * derivative)

        previous_error = error

        # Simple robot response model
        acceleration = control - (0.5 * velocity)

        velocity += acceleration * dt
        position += velocity * dt

        times.append(time)
        positions.append(position)
        errors.append(error)

    return np.array(times), np.array(positions), np.array(errors)


# ------------------------------------------------------------
# PERFORMANCE METRICS
# ------------------------------------------------------------

def calculate_metrics(times, positions, target=10.0):

    # Rise time: time from 10% to 90% of target
    lower = 0.1 * target
    upper = 0.9 * target

    lower_index = np.where(positions >= lower)[0]
    upper_index = np.where(positions >= upper)[0]

    if len(lower_index) > 0 and len(upper_index) > 0:
        rise_time = times[upper_index[0]] - times[lower_index[0]]
    else:
        rise_time = np.nan

    # Percentage overshoot
    maximum = np.max(positions)

    overshoot = max(0, ((maximum - target) / target) * 100)

    # Settling time using +/- 2% tolerance
    tolerance = 0.02 * target
    settling_time = np.nan

    for i in range(len(positions)):

        remaining = positions[i:]

        if np.all(np.abs(remaining - target) <= tolerance):
            settling_time = times[i]
            break

    # Steady-state error
    steady_state_error = abs(target - positions[-1])

    return rise_time, overshoot, settling_time, steady_state_error


# ------------------------------------------------------------
# PID GAIN SETS
# ------------------------------------------------------------

low = (0.3, 0.001, 0.05)
medium = (0.8, 0.01, 0.2)
high = (1.5, 0.03, 0.4)


# ------------------------------------------------------------
# RUN TESTS
# ------------------------------------------------------------

t1, p1, e1 = run_pid(*low)
t2, p2, e2 = run_pid(*medium)
t3, p3, e3 = run_pid(*high)

m1 = calculate_metrics(t1, p1)
m2 = calculate_metrics(t2, p2)
m3 = calculate_metrics(t3, p3)


# ------------------------------------------------------------
# SYSTEM RESPONSE GRAPH
# ------------------------------------------------------------

plt.figure()

plt.plot(t1, p1, label="Low Gains")
plt.plot(t2, p2, label="Medium Gains")
plt.plot(t3, p3, label="High Gains")

plt.axhline(10.0, linestyle="--", label="Target")

plt.xlabel("Time (seconds)")
plt.ylabel("Position")
plt.title("PID Gain Tuning - System Response")

plt.legend()
plt.grid()
plt.tight_layout()

# Save graph for submission
plt.savefig("outputs/pid_response_comparison.png",dpi=300,bbox_inches="tight"
)

plt.show()


# ------------------------------------------------------------
# TRACKING ERROR GRAPH
# ------------------------------------------------------------

plt.figure()

plt.plot(t1, e1, label="Low Gains")
plt.plot(t2, e2, label="Medium Gains")
plt.plot(t3, e3, label="High Gains")

plt.axhline(0.0, linestyle="--")

plt.xlabel("Time (seconds)")
plt.ylabel("Tracking Error")
plt.title("PID Gain Tuning - Tracking Error")

plt.legend()
plt.grid()
plt.tight_layout()

# Save graph for submission
plt.savefig("outputs/pid_error_comparison.png",dpi=300,bbox_inches="tight"
)

plt.show()


# ------------------------------------------------------------
# PRINT PERFORMANCE RESULTS
# ------------------------------------------------------------

def print_results(name, gains, metrics):

    rise, overshoot, settling, error = metrics

    print("\n" + name)

    print("Kp =", gains[0], "Ki =", gains[1], "Kd =", gains[2])

    print("Rise time:", round(rise, 3), "seconds")
    print("Overshoot:", round(overshoot, 2), "%")

    if np.isnan(settling):
        print("Settling time: Did not settle within simulation")
    else:
        print("Settling time:", round(settling, 3), "seconds")

    print("Steady-state error:", round(error, 3))


print("\n========== PID PERFORMANCE COMPARISON ==========")

print_results("LOW GAINS", low, m1)
print_results("MEDIUM GAINS", medium, m2)
print_results("HIGH GAINS", high, m3)