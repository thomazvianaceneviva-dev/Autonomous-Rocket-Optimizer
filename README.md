# Rocket Motor Optimizer
**Technical Documentation**
Geometric optimization pipeline for rocket motors

---

## 1. Overview

Rocket Motor Optimizer is an automatic optimization pipeline for the geometry of experimental solid-propellant rocket motors. The objective is to find the geometric design that brings the rocket to the target apogee with maximum Isp, within defined physical constraints.

The system combines two simulators in sequence: OpenMotor (internal ballistics) and a custom trajectory simulator based on numerical integration of equations of motion. Optimization is conducted in two phases — Random Search followed by CMA-ES.

---

## 2. Project Architecture

### 2.1 File Structure

| File / Folder | Function |
|---|---|
| optimizer.py | Entry point — controls the full pipeline |
| core/models.py | Definition of the MotorDesign and EvalResult dataclasses |
| motor_sim/openmotor_sim.py | Internal ballistics simulation via motorlib (OpenMotor) |
| flight/flight_simulator.py | Trajectory simulation via numerical ODE integration |
| motorlib/ | Python library of OpenMotor (motor, nozzle, grains, etc.) |
| optimizer_workspace/results/ | Automatically generated result CSVs |

### 2.2 Data Flow

The pipeline follows this sequence for each evaluated design:

- The optimizer generates a MotorDesign with 5 geometric parameters
- simulate_openmotor() receives the design and runs 2 motorlib simulations
  - Pass 1: provisional expansion (4.0) to obtain real chamber pressure
  - Pass 2: nozzle with optimal expansion calculated from real pressure
- simulate_flight() receives the result and integrates the equation of motion
- The score is computed and returned to the optimizer (Random Search or CMA-ES)

---

## 3. Search Space

### 3.1 Optimized Variables

The optimizer varies 5 geometric motor parameters, all internally normalized to [0, 1]:

| Variable | Minimum | Maximum | Unit |
|---|---|---|---|
| Grain outer diameter | 50 | 60 | mm |
| Grain length | 60 | 150 | mm |
| Core diameter | 15 | 40 | mm |
| Throat diameter | 10 | 20 | mm |
| Number of grains | 1 | 4 | dimensionless |

### 3.2 Fixed Parameters

The following parameters are not optimized and must be manually adjusted in the code according to the project:

| Parameter | Current value | Where to change |
|---|---|---|
| Propellant | KNSB (Nakka) | openmotor_sim.py → _prop() |
| Propellant density | 1749.7 kg/m³ | openmotor_sim.py → _prop() |
| Maximum chamber pressure | 40 bar | openmotor_sim.py → MAX_PRES |
| Nozzle efficiency | 0.832 | openmotor_sim.py → _nozzle() |
| Total rocket mass | 4.079 kg | optimizer.py → TOTAL_MASS |
| Drag coefficient (Cd) | 0.85 | flight_simulator.py → CD |
| Target apogee | 1000 m | optimizer.py → TARGET_APOGEE |

---

## 4. Motor Simulation (OpenMotor)

### 4.1 Two-Pass Method

The motor simulation uses two passes to accurately compute optimal nozzle expansion without hardcoding chamber pressure:

- Pass 1: simulates with fixed expansion ratio (4.0) only to obtain real average chamber pressure
- Optimal expansion calculation: uses pressure from pass 1 and the motorlib function eRatioFromPRatio to compute the expansion ratio that equalizes exit pressure with ambient pressure (101325 Pa)
- Pass 2: simulates again with the optimal nozzle — this result is the one used

### 4.2 Nozzle Efficiency

The nozzle efficiency is set to **0.832** in the code, even though OpenMotor uses **0.95** for the same motor. These two values are equivalent in practice — they produce the same delivered ISP — because motorlib and OpenMotor apply the efficiency parameter differently inside their thrust coefficient models.

In motorlib, `getAdjustedThrustCoeff` multiplies four independent loss factors together:

```
Cf_delivered = divLoss × throatLoss × efficiency × (skinLoss × Cf_ideal + (1 − skinLoss))
```

For the nozzle geometry used (divAngle = 15°, throatLength = 5 mm, throat = 16 mm):

```
divLoss    = (1 + cos(15°)) / 2 = 0.983
throatLoss = 0.99 − 0.0333 × (0.005 / 0.016) = 0.980
skinLoss   = 0.99  (fixed constant)
```

OpenMotor reports Cf_ideal = 1.51 and Cf_delivered = 1.21 with efficiency = 0.95. Solving for the equivalent efficiency value that makes motorlib reproduce the same Cf_delivered:

```
efficiency = Cf_delivered / (divLoss × throatLoss × (skinLoss × Cf_ideal + (1 − skinLoss)))
           = 1.21 / (0.983 × 0.980 × (0.99 × 1.51 + 0.01))
           = 0.832
```

So **0.832 in motorlib = 0.95 in OpenMotor** — both produce Cf_delivered = 1.21 and the same ISP. The difference is purely how each program weights the efficiency factor relative to the other losses.

To recalibrate for a different motor design, use the Cf_ideal and Cf_delivered values reported by OpenMotor and apply the same formula above.

The ISP is computed directly from physics as `Impulse / (prop_mass × g0)`, which already incorporates all losses from the simulation. **Do not multiply the result by NOZZLE_EFFICIENCY again** — motorlib applies it internally via the thrust coefficient model.

### 4.3 Propellant Properties

The propellant used is **KNSB** based on Richard Nakka's formulation as entered in OpenMotor. The physical and thermochemical properties are:

| Property | Value | Unit |
|---|---|---|
| Density | 1749.7 | kg/m³ |
| Specific heat ratio (γ) | 1.13 | — |
| Combustion temperature | 1520 | K |
| Exhaust molar mass | 39.9 | g/mol |
| Burn rate coefficient (a) | 4.18 × 10⁻³ | m/(s·Pa^n) |
| Burn rate exponent (n) | 0.059 | — |
| Valid pressure range | 7.03 – 10.67 | MPa |

The narrow pressure range (70–107 bar) reflects the range over which the burn rate law was experimentally measured. Designs that operate outside this range will trigger a WARNING alert from motorlib — results remain usable but less accurate. The optimizer's MAX_PRES filter (40 bar) keeps chamber pressures well below the lower bound, so the burn rate is extrapolated by the closest tab. This was verified empirically to produce acceptable results for this propellant.

### 4.4 Filters

Invalid designs are discarded in three layers, from cheapest to most expensive:

- Geometric: core_diam ≥ outer_diam discarded before any simulation
- Pressure: peak_pressure > MAX_PRES (40 bar) discarded after pass 1
- Alerts: any ERROR-level alert from motorlib discarded after pass 2

Discarded designs receive score = 1e9, signaling CMA-ES to avoid that region of the space.

---

## 5. Flight Simulation (Trajectory)

### 5.1 Equation of Motion

The trajectory is computed by numerically integrating the 1D (vertical axis) equation of motion:

```
m(t) · dv/dt = F_thrust(t) − F_drag(v, h) − m(t) · g
```

Where each term is calculated as:

- F_thrust(t): interpolated from the motorlib thrust curve, or constant thrust as fallback
- F_drag(v, h) = 0.5 · ρ(h) · Cd · A_ref · v · |v| — keeps the sign so drag always opposes motion
- m(t): decreases linearly from TOTAL_MASS to dry_mass during burn_time, constant afterward

### 5.2 Atmospheric Model

Air density ρ(h) uses the standard ISA atmospheric model (troposphere up to 11 km), accounting for temperature and pressure drop with altitude. This is important because drag decreases as the rocket ascends.

### 5.3 Variable Mass

The total rocket mass is always 4.079 kg (taken from a hypothetical mass table of each subsystem of the "millennium"). Propellant mass comes directly from the motorlib result (getPropellantMass()) and varies per design. Therefore:

- dry_mass = 4.079 − prop_mass (varies per design)
- The rocket starts at 4.079 kg and ends the burn at dry_mass kg

### 5.4 Numerical Integration

scipy.integrate.solve_ivp with RK45 method is used with maximum step of 50 ms and tolerances rtol=1e-6, atol=1e-8. Integration stops when vertical velocity reaches zero — this is the apogee.

---

## 6. Optimization Pipeline

### 6.1 Phase 1 — Random Search

Generates N_RANDOM random designs within the limits defined in LOW and HIGH. All are evaluated and saved in phase1_all.csv. The TOP_K best (lowest score) are passed to phase 2. The goal is to map the search space and prevent CMA-ES from starting in a poor region.

### 6.2 Phase 2 — CMA-ES

CMA-ES (Covariance Matrix Adaptation Evolution Strategy) is initialized with the mean of the TOP_K designs from phase 1. At each generation it samples CMA_POPSIZE candidates from a multivariate Gaussian distribution, evaluates all, and adapts the distribution toward the best ones. Over generations, the distribution learns the geometry of the search space — including correlations between variables — and converges to the optimum.

### 6.3 Objective Function (Score)

The score is computed as:

```
score = (apogee − target)² / target² − 0.005 · Isp
```

Minimizing this value means: approaching the target apogee (1000 m) and preferring designs with higher Isp. The weight 0.005 can be adjusted — higher values give more importance to Isp relative to apogee.

---

## 7. Requirements and Installation

### 7.1 Required Software

| Software | Version | Notes |
|---|---|---|
| Python | 3.11.x | Do not use 3.12+ due to incompatibility with scikit-fmm |

### 7.2 Python Dependencies

Install with pip inside a virtual environment (.venv):

```
pip install numpy scipy scikit-fmm scikit-image PyYAML cma pandas
```

### 7.3 Note on Cython Extension

The original motorlib uses a Cython-compiled extension (`mathlib`) for performance. This extension was replaced with a pure Python fallback due to compatibility issues with Microsoft C++ Build Tools on some systems.

If you want to restore the original Cython performance, you can revert the mathlib module to its compiled version and run:

```
python setup.py build_ext --inplace
```

This requires Microsoft C++ Build Tools 2019+ on Windows, or Xcode Command Line Tools on Mac. The pure Python version is functionally identical — only slower for large simulations.

### 7.4 Configure Virtual Environment in VS Code

`Cmd+Shift+P` (Mac) or `Ctrl+Shift+P` (Windows) → Python: Select Interpreter → select the interpreter with (.venv) in the path.

---

## 8. How to Run

### 8.1 First Execution

Open the VS Code integrated terminal and activate the virtual environment:

**Mac:**
```
source .venv/bin/activate
```

**Windows:**
```
.venv\Scripts\Activate.ps1
```

Then run the optimizer:

```
python optimizer.py
```

The program will request two inputs:

- Phase 1 population (random search): number of random designs to evaluate — recommended 200 for testing, 1000+ for real optimization
- Top K for CMA-ES: how many designs to pass from phase 1 to phase 2 — recommended 20 to 40

### 8.2 Results

Results are automatically saved in optimizer_workspace/results/:

- phase1_all.csv — all designs evaluated in random search with score
- phase1_topk.csv — TOP_K best selected for CMA-ES
- phase2_cmaes.csv — best design of each CMA-ES generation
- best_designs.csv — final top 10 designs

### 8.3 Parameters to Adjust Before Running

The following values in optimizer.py should be reviewed before each optimization run:

- TARGET_APOGEE: target apogee in meters
- TOTAL_MASS: total rocket mass in kg
- LOW and HIGH: search space bounds in mm — must reflect construction constraints
- CMA_MAX_ITER: maximum number of CMA-ES generations — more generations = more accurate but slower
- CMA_POPSIZE: candidates per generation — default 24, reduce for quick tests

---

## 9. Known Limitations

- Cd is fixed at 0.85 — it does not vary with Mach number or rocket geometry. For designs very different from the reference rocket (Lyly.ork), this value should be updated. This assumption is only acceptable due to low Mach numbers.
- Dry mass is computed as TOTAL_MASS − prop_mass. This assumes total mass is always 4.079 kg regardless of motor size. Designs with much larger or smaller motors than the reference will have inaccurate dry mass.
- Trajectory integration is 1D (vertical axis). Wind, rotation, and aerodynamic instability are not modeled.
- Propellant burn is modeled as linear — mass decreases uniformly over burn time. The real burn profile depends on grain geometry.
- Flight simulation does not include OpenRocket — it uses a simplified custom simulator. Integration with OpenRocket is planned for future versions.

---

## 10. Next Steps

- Implement dependent variables in the search space (e.g., core as a fraction of outer) to reduce invalid designs and accelerate convergence
- Eliminate pass 1 using cached pressures from previous simulations as estimates
- Parallelize Random Search with multiprocessing to reduce phase 1 runtime
- Integrate OpenRocket for more accurate trajectory simulation
- Add stability model (static margin) as an additional constraint
- Implement automatic calibration of MAX_PRES from initial samples
