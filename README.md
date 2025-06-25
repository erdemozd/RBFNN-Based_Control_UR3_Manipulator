# RBFNN-Based_Sliding_Mode_Control_UR3_Manipulator

Implementation of Radial Basis Function Neural Network (RBFNN) based system identification and sliding mode control strategies for the UR3 robotic manipulator.  
Includes model training, trajectory tracking, and adaptive control applications.

> **Note:** This project is developed using **MATLAB and Simulink (version 2023b)**.

---

## Required Toolboxes

To run and compile the project successfully, ensure the following MATLAB toolboxes are installed:

- Robotic System Toolbox  
- Simscape Multibody

---

## UR3 Manipulator Model

The UR3 manipulator used in this project is a **mechanical model**.  
To visualize the robot in simulation:

1. Navigate to the **"UR3 Manipulator"** Simulink block.
2. Insert the mechanical parts into the block.
3. Use the `.stl` files provided in the **`Mechanic Part Models`** folder.
4. Each Simscape part name must match the corresponding `.stl` filename.

---
