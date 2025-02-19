# A_fish_like_robot_motion_planning
A project for control and motion planning of a nonholonomic robot while avoiding obstacles 

### Robot Model
 The robot in this project is a swimming robot with nonholonomic constraints, modeled by the Chaplygin sleigh—a time-varying mathematical model. While parameter identification is beyond the scope of this work, more details can be found [here](https://link.springer.com/article/10.1007/s11071-019-05141-z). The robot's control input consists of a sinusoidal function (to generate velocity) and a corrective torque (to regulate the head angle). A key challenge in designing an autonomous control algorithm is minimizing the corrective control action, especially relative to the sinusoidal function. This limitation is crucial for independent velocity and head angle control while maintaining the validity of the identification process.
In this context, **quadratic programming (QP)** is a promising approach for head angle control under dynamic constraints, ensuring that corrective control remains within a predefined range.

### Motion Planning Algorithm
The algorithm integrates obstacle avoidance and heading control through QP approach to compute the corrective torque needed to reach the target while avoiding obstacles. The process begins by calculating the desired angular velocity using a control law, which is then input into the cost function for optimization. The QP solver determines the corrective torque that minimizes the cost function, ensuring that the robot's angular velocity converges to the desired value.

The optimization problem is subject to three key constraints:

1) The robot's dynamics and non-holonomic constraints.
2) An inequality constraint ensuring obstacle avoidance.
3) An inequality constraint limiting the corrective torque (control input).
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
