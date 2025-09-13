# Actor-Critic Reinforcement Learning with IT2-TSK-Fuzzy ADRC for Mobile Robot Control

This repository contains a MATLAB implementation of an **Actor-Critic Reinforcement Learning-Based Active Disturbance Rejection Controller (ADRC)** for trajectory tracking of a mobile robot.  
The controller integrates:
- **Kinematic control** for trajectory generation
- **IT2-TSK fuzzy logic**-based actor for nonlinear control
- **Diagonal Recurrent Neural Network (DRNN)**-based critic for adaptive learning
- **Extended State Observer (ESO)** for disturbance estimation and rejection
- **Dynamic robot model** for simulation under uncertainties

---

## 🚀 Features
- Circular trajectory tracking with reference linear (`Vr`) and angular (`Wr`) velocities  
- ESO-based ADRC for both **linear** and **angular velocity channels**  
- Actor-Critic structure with:
  - **Actor:** Interval Type-2 TSK Fuzzy PD controller  
  - **Critic:** Diagonal Recurrent Neural Network (DRNN)  
- Robustness against disturbances and uncertainties in the dynamic model  
- Easily extendable for different trajectories or robot models  

---

## 📂 File Structure
- `main.m` → MATLAB script containing the full implementation (ADRC + IT2-TSK + DRNN)  
- The script is **self-contained** and does not require additional toolboxes beyond base MATLAB  

---

## ⚙️ How to Run
1. Clone this repository:
   ```bash
   git clone https://github.com/your-username/actor-critic-it2tsk-adrc.git
   cd actor-critic-it2tsk-adrc
2. Open MATLAB and run:

3. The simulation will:

Generate a circular reference trajectory

Run the proposed ADRC-based controller

Display robot state evolution and tracking performance
