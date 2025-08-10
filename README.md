# Segway Trajectory Tracking Simulation

## Overview
This repository contains a MATLAB-based simulation for trajectory tracking of a Segway robot in 3D. The project models the Segway's nonlinear and linear dynamics, implements various controllers (PID and LQR), and visualizes the robot's motion as it follows a set of waypoints.

## File Structure

### Top-Level
- **rhs_eqns.m**: Derivation of nonlinear system dynamics and linearization.

### model_test/
- **animate.m**: Animation function for visualizing the Segway's trajectory.
- **controller.m**: Returns controller value (zero for model testing).
- **generate_waypoints.m**: Generates waypoints for the robot to follow.
- **main.m**: Main execution file for running the simulation.
- **myrhs_linear.m**: Computes rate of state change using linear dynamics.
- **myrhs.m**: Computes rate of state change using nonlinear dynamics.
- **params.m**: Returns a struct containing system parameters.
- **system_stability.m**: Contains root locus and bode plots of states with respect to control inputs.

### lqr/
- **animate.m**: Animation function (same as above).
- **generate_waypoints.m**: Waypoint generation (same as above).
- **myrhs_lqr.m**: Computes rate of change of state using nonlinear dynamics and LQR controller.
- **mywaypointcontroller.m**: Returns current target pose from the trajectory waypoints.
- **params.m**: System parameters (same as above).
- **plot_errors.m**: Plots trajectory tracking errors.

### pid/
- **animate.m**: Animation function (same as above).
- **controller.m**: Returns control inputs using the PID controller.
- **generate_waypoints.m**: Waypoint generation (same as above).
- **main.m**: Main execution file (same as above).
- **mypid.m**: Bode, root locus, and step plots of system states.
- **myrhs.m**: Computes rate of state change using nonlinear dynamics.
- **mywaypoint_controller.m**: Returns current target pose from waypoints.
- **params.m**: System parameters (same as above).
- **plot_errors.m**: Plots trajectory tracking errors.

## How to Run
1. Open MATLAB and navigate to the repository folder.
2. Use the `main.m` files in `model_test`, `lqr`, or `pid` folders to run simulations with different controllers.
3. Visualizations and analysis plots will be generated as specified in each script.

## Controllers
- **PID Controller**: Implements classic PID control for trajectory tracking.
- **LQR Controller**: Implements Linear Quadratic Regulator for optimal control.

## Visualization
- Animation scripts visualize the Segway's movement along the trajectory.
- Error and stability plots help analyze controller performance.

## Requirements
- MATLAB (tested on recent versions)
- Control System Toolbox (for transfer functions and plots)

## Authors
Tanay Raghunandan Srinivasa, Jia Bhargava
