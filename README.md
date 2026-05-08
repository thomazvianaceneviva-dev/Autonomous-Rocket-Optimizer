# Rocket Motor Optimizer

Technical documentation  
Geometric optimization pipeline for rocket motors

---

## Overview

Rocket Motor Optimizer is an automatic optimization system for experimental solid-propellant rocket motors.

The objective is to find a motor geometry that achieves a target apogee while maximizing ISP, respecting physical constraints.

The system combines:

- OpenMotor (internal ballistics)
- Custom 2D flight simulator with launch angle
- Optimization via Random Search + CMA-ES
- Interactive Streamlit dashboard

---

## Architecture

### Project Structure

| File / Folder | Description |
|---|---|
| optimizer.py | CLI optimization pipeline entry point |
| dashboard.py | Streamlit interactive dashboard |
| core/models.py | Data models (MotorDesign, EvalResult) |
| motor_sim/openmotor_sim.py | Internal ballistics (OpenMotor / motorlib) |
| flight/flight_simulator.py | 2D trajectory simulation with launch angle |
| motorlib/ | OpenMotor Python backend |
| optimizer_workspace/results/ | Logs and results |

---

### Data Flow

1. Generate motor design (5 parameters)
2. Run OpenMotor simulation (2-pass system)
3. Run 2D flight simulation with launch angle
4. Compute score (apogee + ISP trade-off)
5. Store result or visualize in dashboard

---

## Design Variables

All variables normalized in [0, 1]:

| Parameter | Min | Max | Unit |
|---|---|---|---|
| Grain outer diameter | 50 | 60 | mm |
| Grain length | 60 | 150 | mm |
| Core diameter | 15 | 40 | mm |
| Throat diameter | 10 | 20 | mm |
| Number of grains | 1 | 4 | - |

---

## Flight Simulation (2D with Launch Angle)

The flight model is now 2D (x, y).

### State Variables

- x(t), y(t)
- vx(t), vy(t)
- launch angle θ

---

### Equations of Motion

m(t) dvx/dt = F_thrust cos(θ) − F_drag_x  
m(t) dvy/dt = F_thrust sin(θ) − F_drag_y − m(t)g  

---

### Drag Model

F_drag = 0.5 · ρ(h) · Cd · A · |v| · v_vector

---

### Launch Angle

The user defines the launch angle:

- 0° horizontal flight
- 90° vertical flight
- intermediate ballistic trajectories

---

## Dashboard (Streamlit)

### Run Dashboard

streamlit run dashboard.py

---

### Features

1. Motor-only simulation
- OpenMotor execution
- Thrust curve visualization
- ISP and pressure analysis

2. Full flight simulation
- Motor + trajectory
- 2D flight visualization
- Launch angle control

3. Adjustable parameters
- Launch angle
- Total mass
- Drag coefficient (Cd)
- Target apogee

4. Simulation history
- Stored designs
- Results and scores
- Cached trajectories

5. Optimization mode
- Random Search
- CMA-ES
- Live monitoring

---

## Optimization Pipeline

### Phase 1: Random Search
- Full space exploration
- Stores all evaluations
- Selects top-K candidates

### Phase 2: CMA-ES
- Multivariate Gaussian search
- Learns parameter correlations
- Converges to optimum

---

## Objective Function

score = (apogee − target)^2 / target^2 − 0.005 · ISP

---

## Installation

### Requirements
- Python 3.11

### Install dependencies

pip install numpy scipy scikit-fmm scikit-image PyYAML cma pandas streamlit

---

### Virtual environment

python -m venv .venv
source .venv/bin/activate   (Mac/Linux)
.venv\Scripts\activate      (Windows)

---

## How to Run

### Dashboard mode

streamlit run dashboard.py

### CLI mode

python optimizer.py

---

## Output Files

optimizer_workspace/results/

- phase1_all.csv
- phase1_topk.csv
- phase2_cmaes.csv
- best_designs.csv

---

## Known Limitations

- No wind model
- No 3D dynamics
- Constant Cd
- Linear burn model
- Simplified aerodynamics
- No attitude dynamics

---

## Future Improvements

- Wind field simulation
- Full 3D rigid body dynamics
- Real-time CMA-ES tuning UI
- Trajectory animation
- OpenRocket integration
- Mission export system
- Adaptive constraints learning

---

## Summary

The system integrates:

- Rocket internal ballistics (OpenMotor)
- 2D flight simulation with launch angle
- Evolutionary optimization (CMA-ES)
- Streamlit dashboard interface
