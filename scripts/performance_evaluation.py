# ============================================================
# COMPONENT A - PART 4
# PERFORMANCE EVALUATION
# ============================================================

import csv
import math


# -------------------------------------------------
# READ ACTUAL ROBOT TRAJECTORY
# -------------------------------------------------

actual_points = []

with open("trajectory.csv", "r") as file:

    reader = csv.DictReader(file)

    for row in reader:

        actual_points.append(
            (
                float(row["x"]),
                float(row["y"])
            )
        )


# -------------------------------------------------
# FINAL A* PLANNED WAYPOINTS
# -------------------------------------------------

planned_points = [
    (0.000,  0.000),
    (0.000, -0.175),
    (0.325, -0.175),
    (0.325, -0.225),
    (0.325, -0.200),
    (0.200, -0.200),
    (0.200,  0.375),
    (-0.375, 0.375),
    (-0.375, 0.400),
    (-0.350, 0.400),
    (-0.350, 0.375),
    (-0.050, 0.375),
    (-0.050, 0.325),
    (0.000,  0.320)
]


# -------------------------------------------------
# PATH LENGTH
# -------------------------------------------------

def path_length(points):

    total = 0.0

    for i in range(1, len(points)):

        dx = points[i][0] - points[i - 1][0]
        dy = points[i][1] - points[i - 1][1]

        total += math.sqrt(
            dx * dx + dy * dy
        )

    return total


# -------------------------------------------------
# POINT-TO-SEGMENT DISTANCE
# -------------------------------------------------

def point_to_segment_distance(point, start, end):

    px, py = point

    x1, y1 = start
    x2, y2 = end

    dx = x2 - x1
    dy = y2 - y1

    if dx == 0 and dy == 0:

        return math.sqrt((px - x1) ** 2+ (py - y1) ** 2)

    t = (((px - x1) * dx) + ((py - y1) * dy)) / (dx * dx + dy * dy)

    t = max(0.0, min(1.0, t))

    nearest_x = x1 + t * dx
    nearest_y = y1 + t * dy

    return math.sqrt((px - nearest_x) ** 2 + (py - nearest_y) ** 2)


# -------------------------------------------------
# TRACKING ERROR
# -------------------------------------------------

tracking_errors = []

for actual_point in actual_points:

    segment_distances = []

    for i in range(
        len(planned_points) - 1
    ):

        distance = point_to_segment_distance(
            actual_point,
            planned_points[i],
            planned_points[i + 1]
        )

        segment_distances.append(distance)

    tracking_errors.append(
        min(segment_distances)
    )


# -------------------------------------------------
# CALCULATE PERFORMANCE METRICS
# -------------------------------------------------

planned_length = path_length(planned_points)

actual_length = path_length(actual_points)

average_error = (sum(tracking_errors) / len(tracking_errors))

maximum_error = max(tracking_errors)

path_difference = (actual_length - planned_length)

path_difference_percent = (abs(path_difference) / planned_length) * 100


# -------------------------------------------------
# PERFORMANCE EVALUATION TABLE
# -------------------------------------------------

print("\n======================================")
print("COMPONENT A - PERFORMANCE EVALUATION")
print("======================================")

print(f"{'Metric':<32} {'Result':>15}")

print("-" * 49)

print(f"{'Planned Path Length':<32}" f"{planned_length:>12.3f} m")

print(f"{'Actual Path Length':<32}"f"{actual_length:>12.3f} m")

print(f"{'Path Length Difference':<32}"f"{path_difference:>12.3f} m")

print(f"{'Path Length Difference (%)':<32}"f"{path_difference_percent:>12.2f} %")

print(f"{'Average Tracking Error':<32}"f"{average_error:>12.3f} m")

print(f"{'Maximum Tracking Error':<32}"f"{maximum_error:>12.3f} m")

print("-" * 49)


# -------------------------------------------------
# EVALUATION SUMMARY
# -------------------------------------------------

print("\nEvaluation Summary:")

print(
    "Tracking error is measured against the nearest "
    "point on the final A* planned path."
)

print(
    "Differences between planned and actual path length "
    "can occur because the robot performs curved turns, "
    "uses waypoint tolerance, and reacts to obstacles "
    "during closed-loop navigation."
)