# A_fish_like_robot_motion_planning
A project for control and motion planning of a nonholonomic robot while avoiding obstacles 

### Robot Model


### Motion Planning Algorithm
The algorithm integrates obstacle avoidance and heading control through a quadratic programming (QP) approach to compute the corrective torque needed to reach the target while avoiding obstacles. The process begins by calculating the desired angular velocity using a control law, which is then input into the cost function for optimization. The QP solver determines the corrective torque that minimizes the cost function, ensuring that the robot's angular velocity converges to the desired value.

The optimization problem is subject to three key constraints:

1) The robot's dynamics and non-holonomic constraints.
2) An inequality constraint ensuring obstacle avoidance.
3) An inequality constraint limiting the corrective torque (control input).
In this project, the described optimization problem is efficiently solved using a QP approach.   


## Overview
This project focuses on the design and implementation of a trajectory planning and control system for a non-holonomic mobile platform. The platform autonomously moves from point A to point B while avoiding obstacles in between, based on optimization-based techniques for cost function minimization.

## Objectives
- Design and implementing a controller as well a motion planning algorithm.
- Test the effectiveness and accuracy of the system in a simulation environment.

## Simulation Setup
This project is implemented using **MATLAB/Simulink 2023b**. You can run the simulation by executing the `motion_p2_obstacle_avoidance_with_plots.m` script in the `code/` folder.

## Results
The results of the simulation, including trajectories, control inputs, and performance metrics, are stored in the `results/` folder.

## Getting Started
1. Clone this repository to your local machine:
   ```bash
   git clone https://github.com/aliaahmaadi/Nonholonomic-Platform-Trajectory-Planning.git
