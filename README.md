# Segway Trajectory Tracking Simulation

## Introduction
The goal of this project is to develop a 3-D trajectory tracking controller for a Segway in simulation. The work begins with a complete derivation of the Equations of Motion (EoM), followed by the design of PID controllers for balancing and steering, and LQR controllers for balancing and steering. Additionally, waypoint generators and controllers are implemented to update the target pose and trace the desired trajectory. This project was undertaken as part of the RO3003: Control, Autonomy, Planning, and Navigation course.

## Demos

### Trajectory Tracking of Sin Wave

https://github.com/user-attachments/assets/ea6afb2e-3468-484c-8e0a-522d5fc666c6

### Trajectory Tracking of Triangle Wave

https://github.com/user-attachments/assets/f8cc0550-59dc-426c-8381-95651eca95e8

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
