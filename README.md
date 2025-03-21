# Non-Holonomic Mobile Manipulator Obstacle Avoidance with Adaptive Prioritization

## Overview
This project presents an advanced **obstacle avoidance algorithm** for a **mobile manipulator system**, integrating:
- A **6-DOF anthropomorphic robotic arm** (e.g., UR10)
- A **non-holonomic differential drive mobile base** (e.g., MiR100)

The algorithm dynamically adjusts movement prioritization between the manipulator and the base using a **weighted pseudo-inverse matrix**, allowing efficient obstacle avoidance without predefined trajectory planning.

## Key Features
- **Real-time obstacle avoidance** in dynamic environments
- **Adaptive prioritization**:
  - Prioritizes base movement for long-distance navigation
  - Prioritizes manipulator movement for fine adjustments
- **Redundancy exploitation**: Uses **null space projection** to maintain the end-effector's position when needed
- **Non-holonomic constraints compliance**: Ensures feasible trajectories for the differential drive system

## Algorithm Details
### 1. Kinematic Model
- The system consists of a **6-DOF robotic manipulator** mounted on a **differential drive base**.
- Uses a **generalized Jacobian matrix** to integrate manipulator and base movements.

### 2. Obstacle Avoidance Strategy
- Detects obstacles within a **safety region** defined by a radius **r**.
- Generates **repulsive velocities** to prevent collisions.
- Utilizes **closed-loop inverse kinematics (CLIK)** to adjust movement adaptively.

### 3. Adaptive Prioritization
- **Weighted Pseudo-Inverse Control**:
  - Dynamically adjusts priority between base and manipulator.
  - Ensures smooth transitions between coarse and fine adjustments.
- **Three Operational Zones**:
  - **Zone 1**: Base-dominant movement for long-distance travel.
  - **Zone 2**: Transition phase with increasing manipulator involvement.
  - **Zone 3**: Full redundancy exploitation near the goal.

## Simulations & Testing
The algorithm has been extensively tested in **MATLAB**, simulating various industrial scenarios:
- **Obstacle near the end-effector** → Manipulator prioritization.
- **Obstacle interfering with moving base** → Base maneuvering with redundancy exploitation.
- **Long-distance navigation** → Progressive activation of manipulator as the target approaches.

## Requirements
- **MATLAB** (Tested on version R2022b)
- **Robotics Toolbox for MATLAB**
- **URDF model** of the mobile manipulator (UR10 + MiR100 or similar)

## Installation & Usage
1. Clone the repository:
   ```bash
   git clone https://github.com/fedene97/mobile_manipulator_obstacle_avoidance.git
   cd mobile_manipulator_obstacle_avoidance
   ```
2. Open MATLAB and navigate to the project folder.
3. Run the simulation:
   ```matlab
   run Main.m
   ```

## References
This work is based on the following research article:
> **F. Neri, G. Palmieri, M. Callegari**, "Non-Holonomic Mobile Manipulator Obstacle Avoidance with Adaptive Prioritization," *MDPI Robotics*, 2025.

## License
This project is licensed under the **Creative Commons Attribution (CC BY) 4.0** license. See [LICENSE](LICENSE) for details.

