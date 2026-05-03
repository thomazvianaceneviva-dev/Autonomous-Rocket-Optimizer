\*\*Rocket Motor Optimizer\*\*  
\*\*Technical Documentation\*\*  
Geometric optimization pipeline for rocket motors

\---

\#\#\# 1\. Overview

Rocket Motor Optimizer is an automatic optimization pipeline for the geometry of experimental solid-propellant rocket motors. The objective is to find the geometric design that brings the rocket to the target apogee with maximum Isp, within defined physical constraints.

The system combines two simulators in sequence: OpenMotor (internal ballistics) and a custom trajectory simulator based on numerical integration of equations of motion. Optimization is conducted in two phases — Random Search followed by CMA-ES.

\#\#\# 2\. Project Architecture

\#\#\#\# 2.1 File structure

\*\*File / Folder\*\* | \*\*Function\*\*  
optimizer.py | Entry point — controls the full pipeline  
core/models.py | Definition of the MotorDesign and EvalResult dataclasses  
motor\_sim/openmotor\_sim.py | Internal ballistics simulation via motorlib (OpenMotor)  
flight/flight\_simulator.py | Trajectory simulation via numerical ODE integration  
motorlib/ | Python library of OpenMotor (motor, nozzle, grains, etc.)  
optimizer\_workspace/results/ | Automatically generated result CSVs

\#\#\#\# 2.2 Data flow

The pipeline follows this sequence for each evaluated design:

\* The optimizer generates a MotorDesign with 5 geometric parameters  
\* simulate\_openmotor() receives the design and runs 2 motorlib simulations  
\* Pass 1: provisional expansion (4.0) to obtain real chamber pressure  
\* Pass 2: nozzle with optimal expansion calculated from real pressure  
\* simulate\_flight() receives the result and integrates the equation of motion  
\* The score is computed and returned to the optimizer (Random Search or CMA-ES)

\#\#\# 3\. Search Space

\#\#\#\# 3.1 Optimized variables

The optimizer varies 5 geometric motor parameters, all internally normalized to \[0, 1\]:

\*\*Variable\*\* | \*\*Minimum\*\* | \*\*Maximum\*\* | \*\*Unit\*\*  
Grain outer diameter | 50 | 60 | mm  
Grain length | 60 | 150 | mm  
Core diameter | 15 | 40 | mm  
Throat diameter | 10 | 20 | mm  
Number of grains | 1 | 4 | dimensionless

\#\#\#\# 3.2 Fixed parameters

The following parameters are not optimized and must be manually adjusted in the code according to the project:

\*\*Parameter\*\* | \*\*Current value\*\* | \*\*Where to change\*\*  
Propellant | KNSB 65/35 | openmotor\_sim.py → \_prop()  
Propellant density | 1841 kg/m³ | openmotor\_sim.py → \_prop()  
Maximum chamber pressure | 40 bar | openmotor\_sim.py → MAX\_PRES  
Nozzle efficiency | 0.95 | openmotor\_sim.py → \_nozzle()  
Total rocket mass | 4.079 kg | optimizer.py → TOTAL\_MASS  
Drag coefficient (Cd) | 0.85 | flight\_simulator.py → CD  
Target apogee | 1000 m | optimizer.py → TARGET\_APOGEE

\#\#\# 4\. Motor Simulation (OpenMotor)

\#\#\#\# 4.1 Two-pass method

The motor simulation uses two passes to accurately compute optimal nozzle expansion without hardcoding chamber pressure:

\* Pass 1: simulates with fixed expansion ratio (4.0) only to obtain real average chamber pressure  
\* Optimal expansion calculation: uses pressure from pass 1 and the motorlib function eRatioFromPRatio to compute the expansion ratio that equalizes exit pressure with ambient pressure (101325 Pa)  
\* Pass 2: simulates again with the optimal nozzle — this result is the one used

\#\#\#\# 4.2 Filters

Invalid designs are discarded in three layers, from cheapest to most expensive:

\* Geometric: core\_diam ≥ outer\_diam discarded before any simulation  
\* Pressure: peak\_pressure \> MAX\_PRES (40 bar) discarded after pass 1  
\* Alerts: any ERROR-level alert from motorlib discarded after pass 2

Discarded designs receive score \= 1e9, signaling CMA-ES to avoid that region of the space.

\#\#\# 5\. Flight Simulation (Trajectory)

\#\#\#\# 5.1 Equation of motion

The trajectory is computed by numerically integrating the 1D (vertical axis) equation of motion:

m(t) · dv/dt \= F\_thrust(t) − F\_drag(v, h) − m(t) · g

Where each term is calculated as:

\* F\_thrust(t): interpolated from the motorlib thrust curve, or constant thrust as fallback  
\* F\_drag(v, h) \= 0.5 · ρ(h) · Cd · A\_ref · v · |v| — keeps the sign so drag always opposes motion  
\* m(t): decreases linearly from TOTAL\_MASS to dry\_mass during burn\_time, constant afterward

\#\#\#\# 5.2 Atmospheric model

Air density ρ(h) uses the standard ISA atmospheric model (troposphere up to 11 km), accounting for temperature and pressure drop with altitude. This is important because drag decreases as the rocket ascends.

\#\#\#\# 5.3 Variable mass

The total rocket mass is always 4.079 kg (taken from a hypothetical mass table of each subsystem of the “millennium”). Propellant mass comes directly from the motorlib result (getPropellantMass()) and varies per design. Therefore:

\* dry\_mass \= 4.079 − prop\_mass (varies per design)  
\* The rocket starts at 4.079 kg and ends the burn at dry\_mass kg

\#\#\#\# 5.4 Numerical integration

scipy.integrate.solve\_ivp with RK45 method is used with maximum step of 50 ms and tolerances rtol=1e-6, atol=1e-8. Integration stops when vertical velocity reaches zero — this is the apogee.

\#\#\# 6\. Optimization Pipeline

\#\#\#\# 6.1 Phase 1 — Random Search

Generates N\_RANDOM random designs within the limits defined in LOW and HIGH. All are evaluated and saved in phase1\_all.csv. The TOP\_K best (lowest score) are passed to phase 2\. The goal is to map the search space and prevent CMA-ES from starting in a poor region.

\#\#\#\# 6.2 Phase 2 — CMA-ES

CMA-ES (Covariance Matrix Adaptation Evolution Strategy) is initialized with the mean of the TOP\_K designs from phase 1\. At each generation it samples CMA\_POPSIZE candidates from a multivariate Gaussian distribution, evaluates all, and adapts the distribution toward the best ones. Over generations, the distribution learns the geometry of the search space — including correlations between variables — and converges to the optimum.

\#\#\#\# 6.3 Objective function (score)

The score is computed as:

score \= (apogee − target)² / target² − 0.005 · Isp

Minimizing this value means: approaching the target apogee (1000 m) and preferring designs with higher Isp. The weight 0.005 can be adjusted — higher values give more importance to Isp relative to apogee.

\#\#\# 7\. Requirements and Installation

\#\#\#\# 7.1 Required software

\*\*Software\*\* | \*\*Version\*\* | \*\*Notes\*\*  
Python | 3.11.x | Do not use 3.12+ due to incompatibility with scikit-fmm  
Java | 17.x | Required to run OpenRocket (future)  
Microsoft C++ Build Tools | 2019+ | Required to compile motorlib Cython extension

\#\#\#\# 7.2 Python dependencies

Install with pip inside a virtual environment (.venv):

pip install numpy scipy scikit-fmm scikit-image PyYAML cython setuptools cma pandas

\#\#\#\# 7.3 Compile Cython extension

Required once after installing dependencies. Run in the project folder:

python setup.py build\_ext \--inplace

\#\#\#\# 7.4 Configure virtual environment in VS Code

Ctrl+Shift+P → Python: Select Interpreter → select the interpreter with (.venv) in the path.

\#\#\# 8\. How to Run

\#\#\#\# 8.1 First execution

Open the VS Code integrated terminal (Ctrl+\`) and activate the virtual environment:

.venv\\Scripts\\Activate.ps1

Then run the optimizer:

python optimizer.py

The program will request two inputs:

\* Phase 1 population (random search): number of random designs to evaluate — recommended 200 for testing, 1000+ for real optimization  
\* Top K for CMA-ES: how many designs to pass from phase 1 to phase 2 — recommended 20 to 40 (relatively time-consuming due to simulations)

\#\#\#\# 8.2 Results

Results are automatically saved in optimizer\_workspace/results/:

\* phase1\_all.csv — all designs evaluated in random search with score  
\* phase1\_topk.csv — TOP\_K best selected for CMA-ES  
\* phase2\_cmaes.csv — best design of each CMA-ES generation  
\* best\_designs.csv — final top 10 designs

\#\#\#\# 8.3 Parameters to adjust before running

The following values in optimizer.py should be reviewed before each optimization run:

\* TARGET\_APOGEE: target apogee in meters  
\* TOTAL\_MASS: total rocket mass in kg  
\* LOW and HIGH: search space bounds in mm — must reflect construction constraints  
\* CMA\_MAX\_ITER: maximum number of CMA-ES generations — more generations \= more accurate but slower  
\* CMA\_POPSIZE: candidates per generation — default 24, reduce for quick tests

\#\#\# 9\. Known Limitations

\* Cd is fixed at 0.85 — it does not vary with Mach number or rocket geometry. For designs very different from the reference rocket (Lyly.ork), this value should be updated. This assumption is only acceptable due to low Mach numbers.  
\* Dry mass is computed as TOTAL\_MASS − prop\_mass. This assumes total mass is always 4.079 kg regardless of motor size. Designs with much larger or smaller motors than the reference will have inaccurate dry mass.  
\* Trajectory integration is 1D (vertical axis). Wind, rotation, and aerodynamic instability are not modeled.  
\* Propellant burn is modeled as linear — mass decreases uniformly over burn time. The real burn profile depends on grain geometry.  
\* Flight simulation does not include OpenRocket — it uses a simplified custom simulator. Integration with OpenRocket is planned for future versions.

\#\#\# 10\. Next Steps

\* Implement dependent variables in the search space (e.g., core as a fraction of outer) to reduce invalid designs and accelerate convergence  
\* Eliminate pass 1 using cached pressures from previous simulations as estimates  
\* Parallelize Random Search with multiprocessing to reduce phase 1 runtime  
\* Integrate OpenRocket for more accurate trajectory simulation  
\* Add stability model (static margin) as an additional constraint  
\* Implement automatic calibration of MAX\_PRES from initial samples

\*\*Rocket Motor Optimizer — Technical Documentation | Page\*\*
