# Non-Holonomic Mobile Manipulator Obstacle Avoidance with Adaptive Prioritization

## Overview
This project implements an obstacle avoidance algorithm for a mobile manipulator system consisting of:
- A **6-DOF anthropomorphic robotic arm** (e.g., UR10)
- A **non-holonomic differential drive mobile base** (e.g., MiR100)

The algorithm dynamically adjusts movement prioritization between the manipulator and the base using a **weighted pseudo-inverse matrix**, allowing obstacle avoidance without predefined trajectory planning.

## Features
- **Real-time obstacle avoidance** in dynamic environments
- **Adaptive prioritization**:
  - Prioritizes base movement for long-distance navigation
  - Prioritizes manipulator movement for fine adjustments
- **Redundancy exploitation**: Uses null space projection to keep the end-effector fixed when necessary
- **Differential drive constraints**: Ensures feasible trajectories for a non-holonomic system

## Algorithm Details
1. **Kinematic Model**
   - The system consists of a **6-DOF manipulator** and a **differential drive base**.
   - Uses a **generalized Jacobian matrix** to integrate manipulator and base movements.

2. **Obstacle Avoidance Strategy**
   - Detects obstacles using a **safety region** defined by a radius **r**.
   - Generates **repulsive velocities** for collision avoidance.
   - Modifies movement using **closed-loop inverse kinematics (CLIK)**.

3. **Adaptive Prioritization**
   - **Weighted Pseudo-Inverse Control**:
     - Adjusts priority between base and manipulator dynamically.
     - Allows smooth transitions between coarse positioning and fine adjustments.
   - **Three Operational Zones**:
     - **Zone 1**: Base-only movement for long-distance travel.
     - **Zone 2**: Transition phase with increased manipulator involvement.
     - **Zone 3**: Full redundancy exploitation near the goal.

## Simulations
The algorithm has been tested in MATLAB, simulating various industrial scenarios:
- **Obstacle interference with the end-effector** → Manipulator prioritization
- **Obstacle blocking the mobile base** → Base movement with redundancy exploitation
- **Long-distance navigation** → Progressive activation of manipulator as the target approaches

## Requirements
- **MATLAB** (Tested on version R2022b)
- **Robotics Toolbox** for MATLAB
- **URDF model** of the mobile manipulator (UR10 + MiR100 or similar)

## Installation & Usage
1. Clone this repository:
   ```bash
   git clone https://github.com/mobile_manipulator_obstacle_avoidance.git
   cd mobile_manipulator_obstacle_avoidance
   ```
2. Open MATLAB and navigate to the project folder.
3. Run the simulation:
   ```matlab
   run main.m
   ```
   
## References
This algorithm is based on the research article:
> **F. Neri, G. Palmieri, M. Callegari**, "Non-Holonomic Mobile Manipulator Obstacle Avoidance with Adaptive Prioritization," *MDPI Robotics*, 2025.

## License
This project is licensed under the **Creative Commons Attribution (CC BY) 4.0** license. See [LICENSE](LICENSE) for details.

