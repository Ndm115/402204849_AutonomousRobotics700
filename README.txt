AUTONOMOUS SYSTEMS AND ROBOTICS 700
README


======================================================================
PROJECT OVERVIEW
======================================================================

This project contains the Webots simulations, Python scripts, trained machine
learning model, datasets and generated outputs for Component A and Component B
of the Autonomous Systems and Robotics 700 assignment.

Component A implements perception, path planning, PID control and integrated
autonomous navigation.

Component B implements a three-robot multi-agent system, consensus, Particle
Swarm Optimisation (PSO), distributed coverage and neural-network navigation.


======================================================================
SOFTWARE REQUIREMENTS
======================================================================

Webots Version:
R2025a

Python Version:
Python 3.13

Python Dependencies:
- numpy
- matplotlib
- opencv-python
- scikit-learn
- joblib

No ROS or Gazebo is used.


======================================================================
PROJECT STRUCTURE
======================================================================

402204849_AutonomousRobotics700/

controllers/
    component_a_controller/
    pid_controller/
    integrated_controller/

    component_b_controller/
    component_b_coverage/
    component_b_consensus/
    component_b_pso/
    component_b_pso_coverage/
    component_b_ml_navigation/

machine_learning/
    train_navigation_model.py
    navigation_model.pkl

scripts/
    path_planning.py
    pid_tuning.py
    trajectory_plot.py
    performance_evaluation.py
    consensus_evaluation.py
    pso_coverage_evaluation.py

worlds/
    component_a_world.wbt
    component_a_integrated.wbt
    component_b_multi_robot.wbt

outputs/
    path_planning_comparison.png
    pid_response_comparison.png
    pid_error_comparison.png
    planned_vs_actual_trajectory.png
    consensus_convergence.png
    pso_coverage_convergence.png
    navigation_training_loss.png
    navigation_confusion_matrix.png

trajectory.csv


======================================================================
COMPONENT A FILE GUIDE
======================================================================

A1 - SENSOR SIMULATION AND PERCEPTION PIPELINE

World:
    worlds/component_a_world.wbt

Controller:
    controllers/component_a_controller/

Purpose:
    Runs the e-puck perception system using the camera and proximity
    sensors. This controller contains the OpenCV image processing,
    sensor fusion and obstacle-avoidance implementation.


A2 - MOTION PLANNING AND PATHFINDING

Script:
    scripts/path_planning.py

Output:
    outputs/path_planning_comparison.png

Purpose:
    Implements and compares Dijkstra and A* path planning on the
    occupancy grid and generates the path-planning visualisation.


A3 - CONTROL SYSTEM DESIGN

Controller:
    controllers/pid_controller/

Script:
    scripts/pid_tuning.py

Outputs:
    outputs/pid_response_comparison.png
    outputs/pid_error_comparison.png

Purpose:
    The Webots controller demonstrates PID robot control.
    The Python script evaluates different PID gain configurations
    and generates the required response graphs.


A4 - SYSTEM INTEGRATION AND EVALUATION

World:
    worlds/component_a_integrated.wbt

Controller:
    controllers/integrated_controller/

Data:
    trajectory.csv

Scripts:
    scripts/trajectory_plot.py
    scripts/performance_evaluation.py

Output:
    outputs/planned_vs_actual_trajectory.png

Purpose:
    Runs the final integrated Component A system combining perception,
    A* path planning, PID path tracking and obstacle avoidance.

    trajectory.csv contains the recorded robot trajectory.

    trajectory_plot.py generates the planned-versus-actual path graph.

    performance_evaluation.py calculates the trajectory performance
    measurements.


======================================================================
COMPONENT B FILE GUIDE
======================================================================

B1 - MULTI-AGENT SIMULATION SETUP

World:
    worlds/component_b_multi_robot.wbt

Controllers:
    controllers/component_b_controller/
    controllers/component_b_coverage/

Purpose:
    Contains the three-e-puck multi-agent Webots environment.

    component_b_controller provides the base multi-agent communication
    and autonomous behaviour.

    component_b_coverage provides the distributed coverage behaviour.


B2 - CONSENSUS AND SWARM INTELLIGENCE

Controllers:
    controllers/component_b_consensus/
    controllers/component_b_pso/
    controllers/component_b_pso_coverage/

Scripts:
    scripts/consensus_evaluation.py
    scripts/pso_coverage_evaluation.py

Outputs:
    outputs/consensus_convergence.png
    outputs/pso_coverage_convergence.png

Purpose:
    component_b_consensus implements the consensus experiment.

    component_b_pso implements Particle Swarm Optimisation.

    component_b_pso_coverage applies PSO to the coverage experiment.

    The evaluation scripts generate the consensus and PSO convergence
    visualisations.


B3 - MACHINE LEARNING FOR NAVIGATION AND DECISION-MAKING

Training Script:
    machine_learning/train_navigation_model.py

Trained Model:
    machine_learning/navigation_model.pkl

Webots Controller:
    controllers/component_b_ml_navigation/

Outputs:
    outputs/navigation_training_loss.png
    outputs/navigation_confusion_matrix.png

Purpose:
    train_navigation_model.py trains and evaluates the navigation
    neural network and saves the trained model.

    navigation_model.pkl contains the trained model.

    component_b_ml_navigation loads the trained model and uses it for
    robot navigation inside Webots.


B4 - ETHICS, RISK AND RESPONSIBLE AUTONOMY

This section is addressed in the Discussion and Ethical Considerations
section of the submitted PDF report.


======================================================================
HOW TO RUN THE WEBOTS SIMULATIONS
======================================================================

1. Open Webots.

2. Select File -> Open World.

3. Open the required world from the worlds/ folder.

4. Press Play to start the simulation.

5. Use Reset Simulation before restarting an experiment if required.


COMPONENT A PERCEPTION:

Open:
    worlds/component_a_world.wbt

This runs the Component A perception controller.


COMPONENT A INTEGRATED NAVIGATION:

Open:
    worlds/component_a_integrated.wbt

This runs the final integrated autonomous navigation system.


COMPONENT B:

Open:
    worlds/component_b_multi_robot.wbt

This is the multi-agent environment used for the Component B
controllers.

The controller assigned to an e-puck determines which Component B
experiment is being run.


======================================================================
HOW TO RUN THE PYTHON SCRIPTS
======================================================================

Open Command Prompt or a terminal in the project root:

    402204849_AutonomousRobotics700

Run the required scripts using:


Component A - Path Planning:

    python scripts/path_planning.py


Component A - PID Evaluation:

    python scripts/pid_tuning.py


Component A - Trajectory Plot:

    python scripts/trajectory_plot.py


Component A - Performance Evaluation:

    python scripts/performance_evaluation.py


Component B - Consensus Evaluation:

    python scripts/consensus_evaluation.py


Component B - PSO Coverage Evaluation:

    python scripts/pso_coverage_evaluation.py


Component B - Neural Network Training/Evaluation:

    python machine_learning/train_navigation_model.py


Generated graphs are stored in:

    outputs/



CODE ALSO CAN BE OPENED BY OPENING THIS FOLDER IN VS CODE


======================================================================
DATA AND MODEL FILES
======================================================================

trajectory.csv

    Recorded trajectory from the integrated Component A Webots
    simulation.


machine_learning/navigation_model.pkl

    Trained neural-network model used by the Component B machine
    learning navigation controller.


======================================================================
EXPECTED RUNTIME
======================================================================

Python plotting and evaluation scripts should normally complete within
a few seconds.

Neural-network training time depends on the computer being used.

Webots simulations run until the relevant simulation is stopped or
completed. Runtime also depends on the selected Webots simulation speed.


======================================================================
HARDWARE REQUIREMENTS
======================================================================

No physical robot or specialised robotics hardware is required.

A computer capable of running Webots and Python 3.13 is sufficient.


======================================================================
IMPORTANT NOTES
======================================================================

- Keep the existing folder structure unchanged.
- Run Python scripts from the project root folder.
- The trained machine-learning model is already included.
- The recorded Component A trajectory is already included.
- Generated visualisations are available in the outputs/ folder.
- The accompanying PDF report contains the methodology, results,
  evaluation, theoretical discussion and ethical analysis.

======================================================================
END
======================================================================