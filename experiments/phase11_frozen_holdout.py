"""Frozen Phase 11: new-city native MuJoCo validation of staged/slew candidate.

No threshold tuning is allowed after seeing these holdout results. A failed
preflight counts as a failed mission, not a missing denominator.
"""
from __future__ import annotations
import argparse,csv,json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.urban_scene import city
from mars_lora.transit_route import staged_route
from mujoco_sim.validate_mission import run

# Fixed independently from earlier V1-V5 and Phase 8-10 development cities.
FROZEN_SEEDS=(31489,56003,87643,102671)
ROUTES=("shortest","rf_aware")
WINDS=(.25,.50)
ALTITUDE_M=10.
SLEW_M_S=.65
SIGMA_M=.10

def evaluate(output="results/phase11_heldout_frozen"):
    target=Path(output);target.mkdir(parents=True,exist_ok=True)
    rows=[];failures=[]
    for seed in FROZEN_SEEDS:
        obstacles=INDUSTRIAL+tuple(city(seed))
        for method in ROUTES:
            try:
                route=staged_route(method,obstacles,transit_z=ALTITUDE_M,
                    localization_sigma_m=SIGMA_M)
            except (ValueError,RuntimeError,AssertionError) as error:
                for wind in WINDS:
                    failures.append({"city_seed":seed,"route":method,"wind":wind,
                                     "type":"planning_failure","error":str(error)})
                print("NO PLAN",seed,method,str(error),flush=True)
                continue
            for wind in WINDS:
                trace=target / f"flight_{seed}_{method}_{str(wind).replace('.','p')}.csv"
                try:
                    r=run("heavy",duration=240,wind=wind,out=str(trace),
                        route=route,wind_phase=(seed%19)*.12,
                        city_seed=seed,controller_mode="legacy",
                        localization_mode="lidar_imu3d_hybrid",
                        dynamic_replanning=False,altitude_slew_m_s=SLEW_M_S,
                        localization_drift_m_sqrt_s=.03,
                        localization_noise_sigma_m=.02,
                        localization_seed=seed+71,
                        lidar_blackout_window=(45.,75.))
                except (RuntimeError,ValueError,AssertionError) as error:
                    failures.append({"city_seed":seed,"route":method,"wind":wind,
                         "type":"simulation_failure","error":str(error)})
                    print("FAILED RUN",seed,method,wind,str(error),flush=True)
                    continue
                r.update(route=method,wind_n=wind,
                    safe=bool(r["completed"] and r["contact_samples"]==0
                              and r["min_clearance_m"]>=.25))
                rows.append(r)
                print("HOLDOUT",seed,method,wind,"safe",r["safe"],
                      "contacts",r["contact_samples"],"min_clearance",r["min_clearance_m"],
                      flush=True)
    if rows:
        with (target/"trials.csv").open("w",newline="") as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]))
            writer.writeheader();writer.writerows(rows)
    expected=len(FROZEN_SEEDS)*len(ROUTES)*len(WINDS)
    safe=sum(r["safe"] for r in rows)
    summary={
        "phase":"Frozen new-city independent simulation evaluation, NOT physical hardware validation",
        "frozen_spec":{"city_seeds":list(FROZEN_SEEDS),"routes":list(ROUTES),
            "wind_forces_n":list(WINDS),"vehicle":"heavy",
            "transit_altitude_m":ALTITUDE_M,"altitude_slew_m_s":SLEW_M_S,
            "planner_sigma_m":SIGMA_M,"lidar_blackout_s":[45,75],
            "dynamic_replanning":False,"controller":"legacy"},
        "expected":expected,"executed":len(rows),
        "failed_before_or_during_simulation":failures,
        "safe":safe,"failed":expected-safe,
        "contact_failures":sum(r["contact_samples"]>0 for r in rows),
        "clearance_failures":sum(r["min_clearance_m"]<.25 for r in rows),
        "incomplete_flights":sum(not r["completed"] for r in rows),
        "full_matrix_complete":len(rows)==expected,
        "all_mission_safety_gate_passed":len(rows)==expected and safe==expected,
        "first_failed_events":[{"seed":r["city_seed"],"method":r["route"],
            "wind":r["wind_n"],"first_contact":r["first_contact"],
            "first_clearance_violation":r["first_clearance_violation"]}
            for r in rows if not r["safe"]],
        "scope_warnings":[
            "Heldout is procedural MuJoCo city geometry, not independent flight hardware or real sensors",
            "No unknown-map 3D SLAM, consistent covariance, adaptive LoRa or full battery energy test",
            "Pass criteria include mission completion, zero contact samples and >=.25m city clearance",
            "All plan and simulation failures stay in the expected denominator",
            "No hyperparameter adjustment based on results is permitted for this heldout batch"
        ]}
    (target/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary,indent=2),flush=True)
    return summary

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",default="results/phase11_heldout_frozen")
    args=parser.parse_args()
    evaluate(args.output)
