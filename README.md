# GNC Framework for Autonomous Spacecraft Rendezvous & Docking in LEO

This repository contains the complete **Guidance, Navigation, and Control (GNC)** architecture developed in MATLAB and Simulink for autonomous rendezvous and docking maneuvers between a $10,000\text{ kg}$ chaser spacecraft and a target satellite in Low Earth Orbit (LEO, ~400 km altitude).

---

## 🚀 How to Run the Simulations / Come Eseguire le Simulazioni

To execute and visualize the simulations, open MATLAB and run/open the two primary simulator files:

### 1. Rendezvous Phase (Far-Field & Near-Field)
Open and run **`RendezvousSimulator`** (script `.m` or model `.slx`):
- Initializes workspace parameters and runs the **Far-Field** and **Near-Field Rendezvous** phases.
- **Guidance & Trajectory:** Clohessy-Wiltshire (CW) equations and Lambert solver for optimal impulse transfers.
- **Control:** **LQR** translational control with feedforward and **non-linear PD** attitude control (12-thruster RCS allocation with deadband & limit-cycle protection).
- **Navigation:** Multi-sensor **Kalman Filter (KF)** fusing GPS, LIDAR, and IMU.

### 2. Terminal Docking Phase
Open and run **`DockingSimulator`** (script `.m` or model `.slx`):
- Initializes workspace parameters and executes the **Terminal Docking** phase ($R \le 50\text{ m}$).
- **Control:** Discrete **Model Predictive Control (MPC)** with hard thruster constraints ($10\text{ N}$ max per axis) and dynamic *glideslope* deceleration profiling for soft contact ($\sim 0.02\text{ m/s}$).
- **Attitude & Navigation:** **Multiplicative Extended Kalman Filter (MEKF)** for 3D attitude and gyro bias estimation using Star Tracker, Gyro, and Optical sensors.

---

## 📌 Project Architecture Overview

### Mission Phases & Control Logic
- **Far-Field Rendezvous:** Impulse-based trajectory optimization solving Lambert's problem and CW dynamics to minimize $\Delta V$, total transfer time, and position error accumulation.
- **Near-Field Rendezvous:** LQR-guided approach from $5\text{ km}$ to a $-150\text{ m}$ hold point, strictly respecting the $100\text{ m}$ Keep-Out Zone (KOZ) and thruster saturation ($50\text{ N}$).
- **Terminal Docking:** Discrete MPC ($N_p=100, N_c=30, T_s=0.2\text{ s}$) with dynamic alignment along the target's LVLH docking axis and low contact velocity ($\sim 0.02\text{ m/s}$).
- **Attitude Control:** Continuous tracking of the LVLH reference frame quaternion to compensate for orbital pitching.

### Sensor Fusion (KF & MEKF)
- **Translational KF:** Fuses GPS ($1\text{ Hz}, \sigma=5\text{ m}$), LIDAR ($10\text{ Hz}$, range-dependent error), and Optical/Camera ($10\text{ Hz}, \sigma=0.03\text{ m}$) based on distance thresholds ($R \le 2\text{ km}$ and $R \le 50\text{ m}$).
- **MEKF:** Continuous unit quaternion propagation and gyro bias estimation using Star Tracker ($1\text{ Hz}$) and Gyroscope ($10\text{ Hz}$).

---

## 🛠️ Software Requirements & Toolboxes
- **MATLAB & Simulink** (R2023a or newer recommended)
- **Control System Toolbox**
- **Model Predictive Control Toolbox**

---

## 👥 Authors & Academic Context
Developed for the **Guidance, Navigation & Control (GNC)** course — Academic Year 2025/2026.
