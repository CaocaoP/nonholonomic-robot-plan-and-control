# Nonholonomic Robot Trajectory Planning and Control

This project provides implementations for trajectory planning and control systems of a non-holonomic platform. The robot autonomously navigates from point A to point B while avoiding obstacles in between.

## Table of Contents

- [Project Overview](#project-overview)
- [Setup Instructions](#setup-instructions)
- [Running the Simulations](#running-the-simulations)
- [Plotting the Results](#plotting-the-results)

## Project Overview

This project focuses on developing a trajectory planning and control system for a non-holonomic robot. The robot is designed to:
- Autonomously move between two points.
- Avoid obstacles encountered on its path.
- Use different control and planning methods to achieve efficient navigation.

## Setup Instructions

To run the simulation and plot the results, follow these steps:

1. **Add the Project Path**:  
   Add the `project_tmu` folder and all its subfolders to the MATLAB path to ensure all files can be accessed correctly:
   ```matlab
   addpath(genpath('project_tmu'))
   
2.**Install Dependencies:**
Ensure you have the necessary toolboxes installed in MATLAB, such as:
- Optimization Toolbox
- Navigation Toolbox
- Robotics System Toolbox
- Model Predictive Toolbox

## Running the Simulations
To experiment with different methods and setups, follow these steps:

1. **Open MATLAB and navigate to the project folder.**
2. **Run the 'Main_Simulation' file**
This script allows you to try different trajectory planning and control methods on the non-holonomic platform.

## Plotting the Results
To reproduce the figures shown in the report:

1. **Navigate to the ''figure_plot' folder.**
2. **Run the .m files inside**
This will generate the figures demonstrating the robot’s performance in trajectory planning and control.
