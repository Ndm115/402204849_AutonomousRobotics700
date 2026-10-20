# ============================================================
# COMPONENT B - MACHINE LEARNING
# Neural Network for Robot Navigation / Action Selection
#
# Inputs:
#   Left proximity sensor value
#   Right proximity sensor value
#
# Outputs:
#   0 = FORWARD
#   1 = TURN LEFT
#   2 = TURN RIGHT
#
# The trained neural network is saved so that it can later
# be loaded and used by the Webots robot controller.
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
import pickle

from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import (
    accuracy_score,
    confusion_matrix,
    ConfusionMatrixDisplay
)


# ------------------------------------------------------------
# CREATE TRAINING DATA
# ------------------------------------------------------------

np.random.seed(42)

samples = 1500

X = []
y = []

OBSTACLE_THRESHOLD = 200


for i in range(samples):

    # Simulated proximity sensor readings
    left_sensor = np.random.randint(0, 401)
    right_sensor = np.random.randint(0, 401)

    # --------------------------------------------------------
    # DESIRED ROBOT ACTION
    # --------------------------------------------------------

    # No obstacle -> move forward
    if (
        left_sensor < OBSTACLE_THRESHOLD
        and
        right_sensor < OBSTACLE_THRESHOLD
    ):

        action = 0

    # Obstacle mainly on right -> turn left
    elif right_sensor > left_sensor:

        action = 1

    # Obstacle mainly on left -> turn right
    else:

        action = 2


    X.append(
        [left_sensor, right_sensor]
    )

    y.append(action)


X = np.array(X)
y = np.array(y)


# ------------------------------------------------------------
# SPLIT TRAINING AND TEST DATA
# ------------------------------------------------------------

X_train, X_test, y_train, y_test = train_test_split(
    X,
    y,
    test_size=0.20,
    random_state=42,
    stratify=y
)


# ------------------------------------------------------------
# NORMALISE SENSOR VALUES
# ------------------------------------------------------------

# Convert sensor values from approximately 0-400
# into values between 0 and 1.

X_train = X_train / 400.0
X_test = X_test / 400.0


# ------------------------------------------------------------
# CREATE NEURAL NETWORK
# ------------------------------------------------------------

model = MLPClassifier(
    hidden_layer_sizes=(8,),
    activation="relu",
    solver="adam",
    max_iter=1000,
    random_state=42
)


# ------------------------------------------------------------
# TRAIN MODEL
# ------------------------------------------------------------

print()
print("======================================")
print("COMPONENT B - NEURAL NETWORK TRAINING")
print("======================================")

print("Training samples:", len(X_train))
print("Testing samples:", len(X_test))


model.fit(
    X_train,
    y_train
)


print("Training complete")


# ------------------------------------------------------------
# SAVE TRAINED NEURAL NETWORK
# ------------------------------------------------------------

with open(
    "machine_learning/navigation_model.pkl",
    "wb"
) as file:

    pickle.dump(
        model,
        file
    )


print(
    "Model saved: machine_learning/navigation_model.pkl"
)


# ------------------------------------------------------------
# TEST MODEL
# ------------------------------------------------------------

predictions = model.predict(
    X_test
)


accuracy = accuracy_score(
    y_test,
    predictions
)


print("--------------------------------------")

print(
    "Test Accuracy:",
    round(accuracy * 100, 2),
    "%"
)

print("--------------------------------------")


# ------------------------------------------------------------
# CONFUSION MATRIX
# ------------------------------------------------------------

matrix = confusion_matrix(
    y_test,
    predictions
)


display = ConfusionMatrixDisplay(
    confusion_matrix=matrix,
    display_labels=[
        "Forward",
        "Turn Left",
        "Turn Right"
    ]
)


display.plot()


plt.title(
    "Neural Network Navigation Confusion Matrix"
)


plt.savefig(
    "outputs/navigation_confusion_matrix.png",
    dpi=300,
    bbox_inches="tight"
)


plt.show()


# ------------------------------------------------------------
# TRAINING PERFORMANCE CURVE
# ------------------------------------------------------------

plt.figure()


plt.plot(
    model.loss_curve_
)


plt.xlabel(
    "Training Iteration"
)


plt.ylabel(
    "Loss"
)


plt.title(
    "Neural Network Training Loss"
)


plt.grid()


plt.savefig(
    "outputs/navigation_training_loss.png",
    dpi=300,
    bbox_inches="tight"
)


plt.show()


# ------------------------------------------------------------
# SIMPLE TEST EXAMPLES
# ------------------------------------------------------------

print()
print("Example navigation decisions:")


examples = np.array([
    [50, 60],
    [100, 350],
    [350, 100]
])


example_predictions = model.predict(
    examples / 400.0
)


action_names = [
    "FORWARD",
    "TURN LEFT",
    "TURN RIGHT"
]


for i in range(len(examples)):

    print(
        "Sensors:",
        examples[i],
        "->",
        action_names[
            example_predictions[i]
        ]
    )