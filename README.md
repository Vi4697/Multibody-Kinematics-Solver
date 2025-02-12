# Multibody Mechanism Simulation with MATLAB and Adams

## Overview
This project provides a MATLAB-based simulation of a multibody mechanism with revolute and translational joints. The simulation includes **position, velocity, and acceleration analysis** of the system using kinematic equations and numerical solvers. Additionally, an **Adams model** is included in the repository for validation and comparison with the MATLAB results.

## Features
- Solves the **kinematics problem** (position, velocity, acceleration) for the mechanism.
- Uses **Newton-Raphson iteration** to solve the constraint equations.
- Computes **Jacobian matrices** for system constraints.
- Supports **time-dependent driving constraints** for selected joints.
- Provides **graphical visualization** of motion characteristics.
- Includes an **Adams model** for verification.

## Repository Structure
```
├── Acceleration.m        # Computes acceleration using system constraints
├── constraints.m         # Defines constraint equations for position analysis
├── data.m                # Stores mechanism dimensions and joint definitions
├── Jacobian.m            # Computes the Jacobian matrix for constraint equations
├── Jacobian_origin.m     # Alternative Jacobian calculation for debugging
├── NewtonRaphson.m       # Solves position constraints using the Newton-Raphson method
├── Rot.m                 # Computes 2D rotation matrices
├── sol_Problem_6.m       # Runs the full simulation and generates plots
├── test.m                # Validates constraints at the initial configuration
├── Velocity.m            # Computes velocity using constraint equations
└── Adams_Model/          # Contains the Adams model for verification
```

## Dependencies
- **MATLAB** (Tested on R2021a and later)
- **Adams** (For comparison with MATLAB results)

## How to Run
1. Open MATLAB and navigate to the project directory.
2. Run the main solver:
   ```matlab
   [T, Q, DQ, D2Q] = sol_Problem_6();
   ```
3. The script will:
   - Solve position, velocity, and acceleration.
   - Plot motion characteristics for selected points.
4. Use **test.m** to validate constraint satisfaction:
   ```matlab
   run('test.m')
   ```

## Adams Model Verification
1. Open the **adams** directory and load the simulation file in **Adams View**.
2. Run the simulation in Adams and compare results with MATLAB.
3. Ensure consistency between **position, velocity, and acceleration** outputs.
