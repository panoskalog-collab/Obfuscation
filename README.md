# Obfuscation
This repository provides a comprehensive Simulation of Urban MObility (SUMO) and TraCI-based framework to evaluate the fundamental trade-off between **location privacy (position obfuscation)** and **vehicular safety (forward collision warning degradation)** in Vehicular Ad-Hoc Networks (VANETs).


# VANET Privacy vs. Safety: Position Obfuscation Simulation Framework

This repository provides a comprehensive Simulation of Urban MObility (SUMO) and TraCI-based framework to evaluate the fundamental trade-off between **location privacy (position obfuscation)** and **vehicular safety (forward collision warning degradation)** in Vehicular Ad-Hoc Networks (VANETs).

By injecting spatial noise into the perception layer (V2V safety beacons) while keeping the physical mobility layer completely pristine, this framework accurately quantifies how privacy-enhancing mechanisms delay or suppress Time-to-Collision (TTC) safety alerts.

## 🔬 Core Features & Noise Models
The simulation evaluates vehicle interactions across multiple traffic densities and noise levels using two distinct obfuscation models:
1. **Bounded Uniform Obfuscation (Average-Case):** Simulates standard privacy-preserving random noise, displacing the reported position uniformly within the bounds of the vehicle's current lane.
2. **Bounded Adversarial Obfuscation (Worst-Case):** Simulates a theoretical geometric attack where the obfuscated position is intentionally placed at the exact spot within the noise radius that maximizes the perceived distance from the following vehicle, representing the absolute ceiling of safety degradation.

## 📊 Macroscopic Risk Metrics
Moving beyond discrete warning failures (False Negatives), this framework calculates continuous, probability-driven risk metrics on the fly:
* **TIT (Time Integrated TTC):** Quantifies the severity-weighted duration of hazard exposure caused by a missed warning.
* **TAR (Total Added Risk):** Aggregates the absolute mathematical probability of a rear-end crash induced specifically by the privacy obfuscation.
* **ARR (Added Risk Ratio):** Normalizes the TAR against the baseline risk of an ideal, un-obfuscated network to allow for fair comparisons across different urban topologies.

The underlying exponential crash probability model uses the following default calibration parameters: 
Maximum baseline risk ($\alpha = 1.0$), environmental decay ($\lambda = 0.5$), warning effectiveness ($\eta = 0.8$), and behavioral penalty for missed warnings ($\delta = 0.2$).

---

## ⚙️ Prerequisites and Installation

### 1. Install SUMO
You must have **SUMO (Simulation of Urban MObility)** installed on your machine.
* [Download and Install SUMO](https://eclipse.dev/sumo/)
* Ensure the `SUMO_HOME` environment variable is configured correctly on your system.

### 2. Python Dependencies
This framework requires Python 3.8+ and the SUMO Python APIs. Install the required packages via `pip`:
```bash
pip install -r requirements.txt



To run cross-map simulations, the repository expects subdirectories for each urban topology (e.g., AMSTERDAM, BELGIUM, etc.). Each city folder must contain its respective SUMO configuration files and the pre-generated route pool.

├── full_matrix_runner.py      # The main simulation execution script
├── requirements.txt           # Python dependencies
├── .gitignore                 
├── README.md                  
└── <CITY_NAME>/               # City Directory
    ├── network.net.xml        # SUMO road network
    ├── simulation.sumocfg     # SUMO configuration file
    └── route_pool.json        # Pre-calculated shortest paths



Running Multiple Cities in Parallel
If you are running on a multi-core machine or server, you can use tmux to launch all city simulations simultaneously in detached terminal sessions:

for city in AMSTERDAM BELGIUM LISAVONA MUNICH PIRAEUS PORTO ROME WARSAW; do
  tmux new-session -d -s "$city" "python3 full_matrix_runner.py $city"
done
