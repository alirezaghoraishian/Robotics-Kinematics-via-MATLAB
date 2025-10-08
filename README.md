# MATLAB Robotics Kinematics Scripts

This folder contains MATLAB scripts for symbolic kinematics, velocities, accelerations, and Jacobians of a 5-DOF robotic manipulator.

## Files

- `kinematics.m` : Main script. Computes link positions, velocities, accelerations, angular velocities, angular accelerations, and Jacobians. Plots end-effector trajectories.
- `Rot.m` : Function to generate homogeneous rotation matrices.
- `Trans.m` : Function to generate homogeneous translation matrices.

## How to Run

1. Open MATLAB.
2. Navigate to this folder.
3. Run `kinematics.m`. Make sure `Rot.m` and `Trans.m` are in the same folder.
4. The script will compute symbolic expressions and plot results for the end-effector.

## Notes

- Symbolic computations may take some time depending on MATLAB version.
- Modify link lengths or joint trajectories in `kinematics.m` to test different robots.
- Author: [Alireza Ghoraishian], Master’s student in Medical Engineering (Medical Robotics), FAU
