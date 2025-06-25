## Codes

This folder contains all MATLAB scripts and the main Simulink project for the RBFNN-based control of the UR3 manipulator.

###  Main Files and Descriptions

| File Name                             | Description                                                                 |
|--------------------------------------|-----------------------------------------------------------------------------|
| `InputFunction.m`                    | Defines input trajectories or signals for the manipulator system.          |
| `RBF_SMC_6dof_Optimized.m`           | Implements **RBF-SMC with Global Approximation** strategy.                 |
| `RBF_SMC_Robust_6dof.m`              | Implements **RBF-SMC with Local Approximation** (Robust control version).  |
| `Sliding_Surface.m`                  | Defines sliding surface dynamics used in the SMC control law.              |
| `friction_disturbance_RBFNN.m`       | Models external disturbances and friction using RBFNN approximation.       |
| `UR3_Modelling_Control_Final.slx`    | **Simulink model** of the complete system including RBFNN, SMC, and plant. |

---
