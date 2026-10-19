import matplotlib.pyplot as plt

# Data from successful PSO distributed coverage test

robot1_iterations = [1, 2, 3, 4, 5, 6, 7, 8, 9]
robot1_distance = [0.180, 0.180, 0.180, 0.180, 0.152, 0.123, 0.095, 0.068, 0.042]

robot2_iterations = [1, 2, 3, 4, 5, 6, 7, 8]
robot2_distance = [0.180, 0.180, 0.153, 0.127, 0.101, 0.101, 0.073, 0.044]

robot3_iterations = [1, 2, 3, 4, 5]
robot3_distance = [0.100, 0.100, 0.100, 0.072, 0.046]


plt.plot(robot1_iterations, robot1_distance, marker="o", label="Robot 1 - Zone A")
plt.plot(robot2_iterations, robot2_distance, marker="o", label="Robot 2 - Zone B")
plt.plot(robot3_iterations, robot3_distance, marker="o", label="Robot 3 - Zone C")

# Coverage threshold
plt.axhline(
    y=0.05,
    linestyle="--",
    label="Coverage Threshold (0.05 m)"
)

plt.xlabel("Iteration")
plt.ylabel("Distance to Coverage Target (m)")
plt.title("PSO Distributed Coverage Convergence")

plt.legend()
plt.grid()

plt.savefig("outputs/pso_coverage_convergence.png", dpi=300,bbox_inches="tight")

plt.show()