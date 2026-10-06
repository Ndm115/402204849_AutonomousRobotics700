import matplotlib.pyplot as plt

# Consensus results from the Webots simulation

iterations = [1, 2, 3, 4, 5, 6, 7]

robot1 = [2.0, 3.0, 3.6667, 3.9444, 4.0185, 4.0216, 4.0113]
robot2 = [4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]
robot3 = [6.0, 5.0, 4.3333, 4.0556, 3.9815, 3.9784, 3.9887]

plt.plot(iterations, robot1, marker="o", label="Robot 1")
plt.plot(iterations, robot2, marker="o", label="Robot 2")
plt.plot(iterations, robot3, marker="o", label="Robot 3")

# Expected average consensus value
plt.axhline(
    y=4.0,
    linestyle="--",
    label="Consensus Value (4.0)"
)

plt.xlabel("Iteration")
plt.ylabel("Robot State")
plt.title("Multi-Agent Average Consensus Convergence")

plt.legend()
plt.grid()

plt.savefig(
    "outputs/consensus_convergence.png",
    dpi=300,
    bbox_inches="tight"
)

plt.show()