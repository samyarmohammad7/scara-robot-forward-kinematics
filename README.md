# SCARA Robot Forward Kinematics and MATLAB Simulation

## Overview

This project models and simulates a SCARA robotic manipulator with a Revolute-Revolute-Prismatic (RRP) joint configuration. The aim was to derive the robot's forward kinematics manually, validate the result using MATLAB Robotics Toolbox, and demonstrate the robot's motion through teaching mode and waypoint trajectory playback.

The project was completed as part of my Robotics and Artificial Intelligence degree at the University of Hertfordshire.

## Key Features

- SCARA RRP robot modelling
- Manual forward kinematics derivation
- Denavit-Hartenberg parameter implementation
- MATLAB Robotics Toolbox validation
- Interactive teaching mode using `teach()`
- Joint-space trajectory generation using `jtraj()`
- End-effector path visualisation
- Position tracking across X, Y and Z axes
- Validation of manual calculations against toolbox results

## Technologies Used

- MATLAB
- MATLAB Robotics Toolbox by Peter Corke
- Forward kinematics
- Denavit-Hartenberg parameters
- Joint-space trajectory planning
- 3D robot visualisation

## Project Structure

```text
src/
├── P2_main.m
├── build_scara_rtb.m
└── scara_fkine_manual.m

results/
└── P2_validation_table.csv

images/
├── Fig_P2_Home_Position.png
├── Fig_P2_Test_Trajectory.png
├── Fig_P2_Waypoint_Playback.png
├── Fig_P2_EndEffector_Path_3D.png
└── Fig_P2_EndEffector_Position_vs_Step.png
