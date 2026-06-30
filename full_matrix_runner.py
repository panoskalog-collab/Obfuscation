#!/usr/bin/env python3
import os
import sys
import csv
import math
import random
import json
from typing import Dict, Tuple, Optional, List, Set

import traci
import sumolib

# ============================================================
# DYNAMIC PATH CONFIGURATION
# ============================================================
if len(sys.argv) < 2:
    print("❌ ERROR: You must specify a city folder!")
    print("Usage: python3 full_matrix_runner.py <CITY_FOLDER_NAME>")
    print("Example: python3 full_matrix_runner.py AMSTERDAM")
    sys.exit(1)

# Set BASE_DIR dynamically based on the command line argument
CITY_ARG = sys.argv[1]
BASE_DIR = os.path.abspath(CITY_ARG)

if not os.path.isdir(BASE_DIR):
    print(f"❌ ERROR: Directory '{BASE_DIR}' not found.")
    sys.exit(1)

NET_PATH = os.path.join(BASE_DIR, "network.net.xml")
CONFIG_FILE = os.path.join(BASE_DIR, "simulation.sumocfg")
ROUTE_POOL_FILE = os.path.join(BASE_DIR, "route_pool.json")
ROUTE_FILE = os.path.join(BASE_DIR, "generated_traffic.rou.xml")
RESULTS_FILE = os.path.join(BASE_DIR, "full_matrix_results.csv")

# ============================================================
# FULL MATRIX RUNNER (JSON Route Pool + Two-Strike Policy + Risk Math)
# ============================================================

# ---- CRASH PROBABILITY PARAMETERS ----
RISK_ALPHA = 1.0   # Maximum baseline risk (100% crash inevitability at TTC=0)
RISK_LAMBDA = 0.5  # Decay rate based on environment (Standard urban/highway decay)
RISK_ETA = 0.8     # Risk reduction from a correctly issued warning (80%)
RISK_DELTA = 0.2   # Penalty for missed warning due to driver over-reliance (+20%)

# ---- MATRIX DEFINITIONS ----
CAR_COUNTS = [500, 1500, 2500, 3500]
NOISE_LEVELS = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]

# ---- SIM ----
DURATION = 3600                 # steps (dt=0.5 => 1800 seconds)
SUMO_STEP_LENGTH_S = 0.5
SEEDS = [1001, 2002, 3003, 4004, 5005, 6006, 7007, 8008, 9009, 10010]

# ---- TTC thresholds (paper) ----
TTC_THRESHOLD = 3.0   
TTC_CRIT = 1.5
TTC_SEVERE = 1.0
LEADER_LOOKAHEAD = 100

# ---- SUMO ----
SUMO_BINARY = os.environ.get("SUMO_BINARY", "sumo") 
IGNORE_ROUTE_ERRORS = True
NO_STEP_LOG = True
TIME_TO_TELEPORT_S: Optional[int] = None
STUCK_REROUTE_THRESHOLD_S = 30.0 
STUCK_REMOVE_THRESHOLD_S = 90.0  

DEFAULT_LANE_WIDTH_M = 3.2
ADV_S_SAMPLES = 11   
EPS = 1e-12

EXPECTED_HEADER = [
    "Num_Cars", "Seed", "Noise_Delta_m", "StepLength_s",
    "Interactions", "SafeInteractions", "True_Warn_Events", "True_Crit_Events", "True_Severe_Events",
    "TET_true", "TET_safe", "TIT_true", "TIT_safe",
    "Uniform_MW_tot", "Uniform_CMW_tot", "Uniform_SCMW_tot", "Uniform_FP",
    "Uniform_MW_rate", "Uniform_CMW_rate", "Uniform_SCMW_rate", "Uniform_FP_rate", "Uniform_FPR_count",
    "Uniform_MW_perTrueWarn", "Uniform_CMW_perTrueCrit", "Uniform_SCMW_perTrueSevere",
    "Uniform_TET_rep", "Uniform_TET_miss", "Uniform_TET_fp",
    "Uniform_R_miss_TET", "Uniform_R_fp_TET",
    "Uniform_TIT_rep", "Uniform_TIT_miss", "Uniform_TIT_fp",
    "Uniform_R_miss_TIT", "Uniform_R_fp_TIT",
    "Uniform_TAR", "Uniform_ARR",
    "Adv_MW_tot", "Adv_CMW_tot", "Adv_SCMW_tot", "Adv_FP",
    "Adv_MW_rate", "Adv_CMW_rate", "Adv_SCMW_rate", "Adv_FP_rate", "Adv_FPR_count",
    "Adv_MW_perTrueWarn", "Adv_CMW_perTrueCrit", "Adv_SCMW_perTrueSevere",
    "Adv_TET_rep", "Adv_TET_miss", "Adv_TET_fp",
    "Adv_R_miss_TET", "Adv_R_fp_TET",
    "Adv_TIT_rep", "Adv_TIT_miss", "Adv_TIT_fp",
    "Adv_R_miss_TIT", "Adv_R_fp_TIT",
    "Adv_TAR", "Adv_ARR"
]

VTYPE_LINE = (
    '  <vType id="bg_car" vClass="passenger" '
    'accel="2.6" decel="4.5" '
    'tau="0.7" minGap="0.5" sigma="0.8"/>\n'
)

# ============================================================
# UTILITIES AND NOISE MODELS
# ============================================================
MAP_ABS = os.path.abspath(NET_PATH)
try:
    NET = sumolib.net.readNet("file://" + MAP_ABS)
except Exception as e:
    print(f"Warning: Could not preload sumolib network. Error: {e}")
    NET = None

_lane_shape_cache: Dict[str, Optional[List[Tuple[float, float]]]] = {}
_lane_len_cache: Dict[str, Optional[float]] = {}

def rate(n: float, d: float) -> float: return 0.0 if d <= 0 else float(n) / float(d)
def get_dist(p1: Tuple[float, float], p2: Tuple[float, float]) -> float: return math.hypot(p1[0] - p2[0], p1[1] - p2[1])
def safe_ttc(distance: float, rel_speed: float) -> float:
    if rel_speed <= 0: return 99.0
    if distance <= 0: return 0.0
    return distance / rel_speed

def _get_lane_width_m(veh_id: str) -> float:
    try:
        lane_id = traci.vehicle.getLaneID(veh_id)
        if lane_id:
            w = traci.lane.getWidth(lane_id)
            if w and w > 0: return float(w)
    except traci.TraCIException: pass
    return float(DEFAULT_LANE_WIDTH_M)

def _point_dir_at_s(shape: List[Tuple[float, float]], s: float) -> Tuple[float, float, float, float]:
    if not shape: return (0.0, 0.0, 1.0, 0.0)
    if len(shape) == 1: return (shape[0][0], shape[0][1], 1.0, 0.0)
    s = max(0.0, s)
    for k in range(len(shape) - 1):
        x1, y1 = shape[k]
        x2, y2 = shape[k + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        if seg_len < 1e-9: continue
        if s <= seg_len:
            t = s / seg_len
            return (x1 + t * (x2 - x1), y1 + t * (y2 - y1), (x2 - x1) / seg_len, (y2 - y1) / seg_len)
        s -= seg_len
    x1, y1 = shape[-2]; x2, y2 = shape[-1]
    seg_len = math.hypot(x2 - x1, y2 - y1)
    if seg_len < 1e-9: return (x2, y2, 1.0, 0.0)
    return (x2, y2, (x2 - x1) / seg_len, (y2 - y1) / seg_len)

def _get_lane_shape(lane_id: str) -> Optional[List[Tuple[float, float]]]:
    if not lane_id or lane_id.startswith(":") or NET is None: return None
    if lane_id in _lane_shape_cache: return _lane_shape_cache[lane_id]
    try:
        shape = [(float(x), float(y)) for (x, y) in NET.getLane(lane_id).getShape()]
        _lane_shape_cache[lane_id] = shape; return shape
    except Exception:
        _lane_shape_cache[lane_id] = None; return None

def _get_lane_length_cached(lane_id: str) -> Optional[float]:
    if not lane_id or lane_id.startswith(":") or NET is None: return None
    if lane_id in _lane_len_cache: return _lane_len_cache[lane_id]
    try:
        L = float(NET.getLane(lane_id).getLength())
        _lane_len_cache[lane_id] = L; return L
    except Exception:
        _lane_len_cache[lane_id] = None; return None

def _fallback_heading_uniform_noise(veh_id: str, pos: Tuple[float, float], delta: float) -> Tuple[float, float]:
    try: ang_deg = traci.vehicle.getAngle(veh_id)
    except traci.TraCIException: ang_deg = 0.0
    theta = math.radians(float(ang_deg))
    ux, uy = math.cos(theta), math.sin(theta)
    d_long, d_lat = random.uniform(-delta, delta), random.uniform(-min(delta, 0.5 * _get_lane_width_m(veh_id)), min(delta, 0.5 * _get_lane_width_m(veh_id)))
    return (pos[0] + d_long * ux + d_lat * -uy, pos[1] + d_long * uy + d_lat * ux)

def _fallback_heading_adversarial_noise(veh_id: str, pos: Tuple[float, float], follower_pos: Tuple[float, float], delta: float) -> Tuple[float, float]:
    try: ang_deg = traci.vehicle.getAngle(veh_id)
    except traci.TraCIException: ang_deg = 0.0
    theta = math.radians(float(ang_deg))
    ux, uy = math.cos(theta), math.sin(theta)
    lat_bound = min(delta, 0.5 * _get_lane_width_m(veh_id))
    best = pos; best_dist = get_dist(pos, follower_pos)
    for d_long in (-delta, -0.5 * delta, 0.0, 0.5 * delta, delta):
        for d_lat in (-lat_bound, 0.0, lat_bound):
            cand = (pos[0] + d_long * ux + d_lat * -uy, pos[1] + d_long * uy + d_lat * ux)
            if (dist := get_dist(cand, follower_pos)) > best_dist:
                best_dist, best = dist, cand
    return best

def add_uniform_polyline_lane_noise(veh_id: str, delta: float) -> Tuple[float, float]:
    pos, lane_id = traci.vehicle.getPosition(veh_id), traci.vehicle.getLaneID(veh_id) or ""
    shape, lane_len = _get_lane_shape(lane_id), _get_lane_length_cached(lane_id)
    if not shape or not lane_len or len(shape) < 2: return _fallback_heading_uniform_noise(veh_id, pos, delta)
    try: s0 = float(traci.vehicle.getLanePosition(veh_id))
    except traci.TraCIException: s0 = 0.0
    lat_bound = min(delta, 0.5 * _get_lane_width_m(veh_id))
    x_c, y_c, tx, ty = _point_dir_at_s(shape, max(0.0, min(s0 + random.uniform(-delta, delta), lane_len)))
    d_lat = random.uniform(-lat_bound, lat_bound)
    return (x_c + d_lat * -ty, y_c + d_lat * tx)

def add_adversarial_polyline_lane_noise(veh_id: str, follower_pos: Tuple[float, float], delta: float) -> Tuple[float, float]:
    pos, lane_id = traci.vehicle.getPosition(veh_id), traci.vehicle.getLaneID(veh_id) or ""
    shape, lane_len = _get_lane_shape(lane_id), _get_lane_length_cached(lane_id)
    if not shape or not lane_len or len(shape) < 2: return _fallback_heading_adversarial_noise(veh_id, pos, follower_pos, delta)
    try: s0 = float(traci.vehicle.getLanePosition(veh_id))
    except traci.TraCIException: s0 = 0.0
    lat_bound = min(delta, 0.5 * _get_lane_width_m(veh_id))
    s_lo, s_hi = sorted([max(0.0, min(s0 - delta, lane_len)), max(0.0, min(s0 + delta, lane_len))])
    s_cands = [s_lo] if ADV_S_SAMPLES <= 1 or abs(s_hi - s_lo) < 1e-12 else [s_lo + k * (s_hi - s_lo)/(ADV_S_SAMPLES-1) for k in range(ADV_S_SAMPLES)]
    best, best_dist = pos, get_dist(pos, follower_pos)
    for s in s_cands:
        x_c, y_c, tx, ty = _point_dir_at_s(shape, s)
        for d_lat in (-lat_bound, 0.0, lat_bound):
            if (dist := get_dist((cand := (x_c + d_lat * -ty, y_c + d_lat * tx)), follower_pos)) > best_dist:
                best_dist, best = dist, cand
    return best

def generate_demand_from_pool(num_cars: int) -> None:
    with open(ROUTE_POOL_FILE, 'r') as f: route_pool = json.load(f)
    route_ids = list(route_pool.keys())
    vehicles = [(f"veh_{i}", random.randint(0, max(0, DURATION - 50)), " ".join(route_pool[random.choice(route_ids)])) for i in range(num_cars)]
    vehicles.sort(key=lambda x: x[1])
    with open(ROUTE_FILE, "w", newline="") as f:
        f.write("<routes>\n" + VTYPE_LINE)
        for vid, depart, edges in vehicles:
            f.write(f'  <vehicle id="{vid}" type="bg_car" depart="{depart}" departPos="random_free" departLane="best">\n    <route edges="{edges}"/>\n  </vehicle>\n')
        f.write("</routes>\n")

def ensure_results_header(path: str) -> None:
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        with open(path, "w", newline="") as f: csv.writer(f).writerow(EXPECTED_HEADER)
        return
    with open(path, "r", newline="") as f: rows = list(csv.reader(f))
    if rows and rows[0] == EXPECTED_HEADER: return
    try: os.replace(path, path + ".bak_noheader")
    except OSError: pass
    with open(path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(EXPECTED_HEADER)
        for row in rows: 
            if row: w.writerow(row)

def load_completed_runs(path: str) -> Set[Tuple[int, float, int]]:
    done = set()
    if not os.path.exists(path): return done
    try:
        with open(path, "r", newline="") as f:
            r = csv.DictReader(f)
            if not r.fieldnames or not {"Num_Cars", "Noise_Delta_m", "Seed"}.issubset(set(r.fieldnames)): return done
            for row in r:
                try: done.add((int(row["Num_Cars"]), float(row["Noise_Delta_m"]), int(row["Seed"])))
                except Exception: continue
    except Exception: pass
    return done

def init_mode_metrics() -> Dict[str, float]:
    return {"MW_tot": 0, "CMW_tot": 0, "SCMW_tot": 0, "FP": 0, "TET_rep": 0.0, "TET_miss": 0.0, "TET_fp": 0.0, "TIT_rep": 0.0, "TIT_miss": 0.0, "TIT_fp": 0.0, "TAR": 0.0, "Total_Ideal_Risk": 0.0}

def update_mode_metrics(acc: Dict[str, float], ttc_true: float, ttc_rep: float, dt: float) -> None:
    w_true, w_rep = (ttc_true < TTC_THRESHOLD), (ttc_rep < TTC_THRESHOLD)
    mw_ij, fp_ij = (w_true and not w_rep), (not w_true and w_rep)
    p_0 = RISK_ALPHA * math.exp(-RISK_LAMBDA * ttc_true) if ttc_true < 99.0 else 0.0
    p_ideal = p_0 * (1.0 - RISK_ETA * float(w_true))
    p_v = min(1.0, p_0 * (1.0 - RISK_ETA * float(w_true and w_rep)) + RISK_DELTA * float(mw_ij))
    
    acc["TAR"] += (p_v - p_ideal)
    acc["Total_Ideal_Risk"] += p_ideal

    if mw_ij:
        acc["MW_tot"] += 1
        if ttc_true <= TTC_CRIT: acc["CMW_tot"] += 1
        if ttc_true <= TTC_SEVERE: acc["SCMW_tot"] += 1
        acc["TET_miss"] += dt; acc["TIT_miss"] += (TTC_THRESHOLD - ttc_true) * dt
    if fp_ij:
        acc["FP"] += 1; acc["TET_fp"] += dt; acc["TIT_fp"] += (TTC_THRESHOLD - ttc_rep) * dt
    if w_rep:
        acc["TET_rep"] += dt; acc["TIT_rep"] += (TTC_THRESHOLD - ttc_rep) * dt

def finalize_mode_metrics(acc: Dict[str, float], ints: int, s_ints: int, tw: int, tc: int, ts: int, tet_t: float, tet_s: float, tit_t: float, tit_s: float) -> Dict[str, float]:
    d_int = float(ints) if ints > 0 else 1.0
    return {
        "MW_tot": acc["MW_tot"], "CMW_tot": acc["CMW_tot"], "SCMW_tot": acc["SCMW_tot"], "FP": acc["FP"],
        "MW_rate": rate(acc["MW_tot"], d_int), "CMW_rate": rate(acc["CMW_tot"], d_int), "SCMW_rate": rate(acc["SCMW_tot"], d_int), "FP_rate": rate(acc["FP"], d_int),
        "FPR_count": rate(acc["FP"], float(s_ints) + EPS), "MW_perTrueWarn": rate(acc["MW_tot"], float(tw) + EPS),
        "CMW_perTrueCrit": rate(acc["CMW_tot"], float(tc) + EPS), "SCMW_perTrueSevere": rate(acc["SCMW_tot"], float(ts) + EPS),
        "TET_rep": acc["TET_rep"], "TET_miss": acc["TET_miss"], "TET_fp": acc["TET_fp"], "R_miss_TET": acc["TET_miss"] / (tet_t + EPS), "R_fp_TET": acc["TET_fp"] / (tet_s + EPS),
        "TIT_rep": acc["TIT_rep"], "TIT_miss": acc["TIT_miss"], "TIT_fp": acc["TIT_fp"], "R_miss_TIT": acc["TIT_miss"] / (tit_t + EPS), "R_fp_TIT": acc["TIT_fp"] / (tit_s + EPS),
        "TAR": acc["TAR"], "ARR": acc["TAR"] / acc["Total_Ideal_Risk"] if acc["Total_Ideal_Risk"] > 0 else 0.0
    }

def run_simulation(delta: float, seed: int) -> Dict[str, float]:
    sumo_cmd = [SUMO_BINARY, "-c", CONFIG_FILE, "--step-length", str(SUMO_STEP_LENGTH_S), "--route-files", os.path.abspath(ROUTE_FILE), "--seed", str(seed)]
    if IGNORE_ROUTE_ERRORS: sumo_cmd += ["--ignore-route-errors", "true"]
    if TIME_TO_TELEPORT_S is not None: sumo_cmd += ["--time-to-teleport", str(TIME_TO_TELEPORT_S)]
    if NO_STEP_LOG: sumo_cmd += ["--no-step-log", "true"]

    traci.start(sumo_cmd)
    interactions = safe_interactions = true_warn = true_crit = true_severe = 0
    tet_true = tet_safe = tit_true = tit_safe = 0.0
    u_acc, a_acc = init_mode_metrics(), init_mode_metrics()
    rerouted_vehicles, stuck_rerouted_count, stuck_removed_count, step, dt = set(), 0, 0, 0, SUMO_STEP_LENGTH_S

    try:
        while step < DURATION and traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            veh_ids = list(traci.vehicle.getIDList())
            for veh_id in veh_ids:
                try:
                    leader_info = traci.vehicle.getLeader(veh_id, dist=LEADER_LOOKAHEAD)
                    if not leader_info: continue
                    leader_id = leader_info[0]
                    f_pos, f_spd = traci.vehicle.getPosition(veh_id), traci.vehicle.getSpeed(veh_id)
                    l_pos, l_spd = traci.vehicle.getPosition(leader_id), traci.vehicle.getSpeed(leader_id)
                    dist_true, rel_spd = get_dist(f_pos, l_pos), f_spd - l_spd
                    ttc_true = safe_ttc(dist_true, rel_spd)

                    interactions += 1
                    if ttc_true < TTC_THRESHOLD:
                        true_warn += 1
                        if ttc_true <= TTC_CRIT: true_crit += 1
                        if ttc_true <= TTC_SEVERE: true_severe += 1
                        tet_true += dt; tit_true += (TTC_THRESHOLD - ttc_true) * dt
                    else:
                        safe_interactions += 1
                        tet_safe += dt; tit_safe += (ttc_true - TTC_THRESHOLD) * dt

                    u_rep = get_dist(f_pos, add_uniform_polyline_lane_noise(leader_id, delta))
                    update_mode_metrics(u_acc, ttc_true, safe_ttc(u_rep, rel_spd), dt)
                    a_rep = get_dist(f_pos, add_adversarial_polyline_lane_noise(leader_id, f_pos, delta))
                    update_mode_metrics(a_acc, ttc_true, safe_ttc(max(dist_true, a_rep), rel_spd), dt)
                except traci.TraCIException: continue

            for v_id in veh_ids:
                try:
                    wt = traci.vehicle.getWaitingTime(v_id)
                    if wt > STUCK_REMOVE_THRESHOLD_S:
                        traci.vehicle.remove(v_id); stuck_removed_count += 1; rerouted_vehicles.discard(v_id)
                    elif wt > STUCK_REROUTE_THRESHOLD_S and v_id not in rerouted_vehicles:
                        traci.vehicle.rerouteTraveltime(v_id); stuck_rerouted_count += 1; rerouted_vehicles.add(v_id)
                    elif wt == 0: rerouted_vehicles.discard(v_id)
                except traci.TraCIException: pass
            step += 1
    finally:
        traci.close()

    return {
        "Interactions": interactions, "SafeInteractions": safe_interactions, "True_Warn_Events": true_warn, "True_Crit_Events": true_crit, "True_Severe_Events": true_severe,
        "TET_true": tet_true, "TET_safe": tet_safe, "TIT_true": tit_true, "TIT_safe": tit_safe,
        "uniform": finalize_mode_metrics(u_acc, interactions, safe_interactions, true_warn, true_crit, true_severe, tet_true, tet_safe, tit_true, tit_safe),
        "adv": finalize_mode_metrics(a_acc, interactions, safe_interactions, true_warn, true_crit, true_severe, tet_true, tet_safe, tit_true, tit_safe)
    }

def main():
    if not os.path.exists(NET_PATH) or not os.path.exists(CONFIG_FILE) or not os.path.exists(ROUTE_POOL_FILE):
        print(f"❌ ERROR: Missing required configuration files in {BASE_DIR}")
        sys.exit(1)

    ensure_results_header(RESULTS_FILE)
    completed = load_completed_runs(RESULTS_FILE)
    
    print(f"=== RUNNING V2V OBFUSCATION MATRIX FOR: {os.path.basename(BASE_DIR)} ===")
    total_runs = len(CAR_COUNTS) * len(NOISE_LEVELS) * len(SEEDS)
    run_index = 0

    for cars in CAR_COUNTS:
        for seed in SEEDS:
            random.seed(seed)
            generate_demand_from_pool(cars)
            for delta in NOISE_LEVELS:
                run_index += 1
                if (cars, float(delta), seed) in completed:
                    print(f"[{run_index}/{total_runs}] SKIP: Cars={cars}, Δ={delta}, Seed={seed}")
                    continue
                
                print(f"[{run_index}/{total_runs}] RUN: Cars={cars}, Δ={delta}, Seed={seed}")
                random.seed(seed)
                res = run_simulation(delta, seed)
                u, a = res["uniform"], res["adv"]

                with open(RESULTS_FILE, "a", newline="") as f:
                    w = csv.writer(f)
                    w.writerow([
                        cars, seed, f"{delta:.6f}", f"{SUMO_STEP_LENGTH_S:.3f}",
                        res["Interactions"], res["SafeInteractions"], res["True_Warn_Events"], res["True_Crit_Events"], res["True_Severe_Events"],
                        f"{res['TET_true']:.6f}", f"{res['TET_safe']:.6f}", f"{res['TIT_true']:.6f}", f"{res['TIT_safe']:.6f}",
                        u["MW_tot"], u["CMW_tot"], u["SCMW_tot"], u["FP"], f"{u['MW_rate']:.12f}", f"{u['CMW_rate']:.12f}", f"{u['SCMW_rate']:.12f}", f"{u['FP_rate']:.12f}", f"{u['FPR_count']:.12f}", f"{u['MW_perTrueWarn']:.12f}", f"{u['CMW_perTrueCrit']:.12f}", f"{u['SCMW_perTrueSevere']:.12f}", f"{u['TET_rep']:.6f}", f"{u['TET_miss']:.6f}", f"{u['TET_fp']:.6f}", f"{u['R_miss_TET']:.12f}", f"{u['R_fp_TET']:.12f}", f"{u['TIT_rep']:.6f}", f"{u['TIT_miss']:.6f}", f"{u['TIT_fp']:.6f}", f"{u['R_miss_TIT']:.12f}", f"{u['R_fp_TIT']:.12f}", f"{u['TAR']:.6f}", f"{u['ARR']:.6f}",
                        a["MW_tot"], a["CMW_tot"], a["SCMW_tot"], a["FP"], f"{a['MW_rate']:.12f}", f"{a['CMW_rate']:.12f}", f"{a['SCMW_rate']:.12f}", f"{a['FP_rate']:.12f}", f"{a['FPR_count']:.12f}", f"{a['MW_perTrueWarn']:.12f}", f"{a['CMW_perTrueCrit']:.12f}", f"{a['SCMW_perTrueSevere']:.12f}", f"{a['TET_rep']:.6f}", f"{a['TET_miss']:.6f}", f"{a['TET_fp']:.6f}", f"{a['R_miss_TET']:.12f}", f"{a['R_fp_TET']:.12f}", f"{a['TIT_rep']:.6f}", f"{a['TIT_miss']:.6f}", f"{a['TIT_fp']:.6f}", f"{a['R_miss_TIT']:.12f}", f"{a['R_fp_TIT']:.12f}", f"{a['TAR']:.6f}", f"{a['ARR']:.6f}",
                    ])
                    f.flush()

if __name__ == "__main__":
    main()
