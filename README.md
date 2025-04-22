# Non-holonomic mobile manipulator obstacle avoidance with adaptive prioritization

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

## Required Files Download

To properly run the simulation, you need to download the following:

1. **`Matlab_Simulation` folder**: Clone or download this repository.
2. **Additional files**: Download the required files from the following link:  
   [Download additional files](https://univpm-my.sharepoint.com/:f:/g/personal/p019104_staff_univpm_it/EjArSEl71zRGuNXeg9USL3ABlbzB_3effC30kmyXSbDzFg?e=xJv9NQ)  

### Installation Instructions:

1. Download and place the `Matlab_Simulation` folder in your desired directory.
2. Open the provided link and download all available files.
3. Copy the downloaded files into the `Matlab_Simulation` folder, overwriting any existing files if necessary.
4. Open MATLAB and navigate to the `Matlab_Simulation` folder.
5. Run the `Main.m` script to start the simulation.

## References
This work is based on the following research article:
> **F. Neri, G. Palmieri, M. Callegari**, "Non-Holonomic Mobile Manipulator Obstacle Avoidance with Adaptive Prioritization," *MDPI Robotics*, 2025.

> **F. Neri, G. Palmieri, M. Callegari**,  
> *Non-Holonomic Mobile Manipulator Obstacle Avoidance with Adaptive Prioritization*,  
> **Robotics**, vol. 14, no. 4, art. 52, 2025.  
> [https://doi.org/10.3390/robotics14040052](https://doi.org/10.3390/robotics14040052)

### BibTeX

```bibtex
@article{neri2025non,
  title={Non-Holonomic Mobile Manipulator Obstacle Avoidance with Adaptive Prioritization},
  author={Neri, Federico and Palmieri, Giacomo and Callegari, Massimo},
  journal={Robotics},
  volume={14},
  number={4},
  pages={52},
  year={2025},
  publisher={MDPI}
}

## License
This project is licensed under the **Creative Commons Attribution (CC BY) 4.0** license. See [LICENSE](LICENSE) for details.

