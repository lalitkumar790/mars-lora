"""Evaluation of actual 3D LiDAR/inertial feedback with uncertainty replanning.

Hard safety: a flight passes only when completed, no contacts and minimum
city-wide clearance >=0.25 m. This is NOT unknown-map SLAM or measured IMU.
Two seeds not used in the prior research training/first holdouts.
The map, sensor noises and planner settings are locked in source before tests.
"""
from __future__ import annotations
import csv,json
from pathlib import Path
from mars_lora.research_gate import score_safety
from mars_lora.rf_astar import INDUSTRIAL,plan
from mars_lora.urban_scene import city
from mujoco_sim.validate_mission import run

CITIES=(91311,20483)
MODES=(("lidar_match",False),("lidar_imu3d",True))
ROUTES=("shortest","rf_aware")
DRIFT=.03
def evaluate(output="results/imu3d_replanning",hybrid=False,city_seeds=CITIES):
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    rows=[];planner_errors=[]
    modes=(("lidar_match",False),("lidar_imu3d_hybrid",False),("lidar_imu3d_hybrid",True)) if hybrid else MODES
    for seed in city_seeds:
        solids=INDUSTRIAL+tuple(city(seed))
        try:
            routes={name:plan(name,solids=solids,localization_sigma_m=.10).points
                    for name in ROUTES}
        except (RuntimeError,ValueError,AssertionError) as error:
            planner_errors.append(dict(city_seed=seed,error=str(error)))
            print("NO PLAN",seed,str(error),flush=True);continue
        for route,points in routes.items():
            for mode,replanning in modes:
                result=run("heavy",duration=240,wind=.25,
                    wind_phase=(seed%19)*.12,city_seed=seed,
                    controller_mode="legacy",route=points,
                    localization_mode=mode,dynamic_replanning=replanning,
                    localization_drift_m_sqrt_s=DRIFT,
                    localization_noise_sigma_m=.02,
                    localization_seed=seed+71,
                    lidar_blackout_window=((45.,75.) if hybrid else None))
                result.update(route=route,
                    evaluation_policy=(mode+('_replan' if replanning else '_no_replan')),
                    safe=bool(result["completed"] and result["contact_samples"]==0
                              and result["min_clearance_m"]>=.25))
                rows.append(result)
                print("3D ESTIMATOR",seed,route,mode,
                      "ATE",result["navigation_ate_rmse_m"],"safe",result["safe"],
                      "replans",result["map_replans"],"contacts",result["contact_samples"],
                      flush=True)
    (out/"planning_failures.json").write_text(json.dumps(planner_errors,indent=2)+"\n")
    if rows:
        with (out/"trials.csv").open("w",newline="") as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    # Intent-to-treat research scoring: every predeclared city/route/policy
    # remains in the denominator, even when A* cannot produce a safe plan.
    summary,gate=score_safety(rows,planner_errors,city_seeds,ROUTES,modes)
    (out/"summary.json").write_text(json.dumps({
        "phase":"Experimental 3D range plus inertial state estimation, uncertainty-based map replanning",
        "city_seeds":list(city_seeds),"physical_runs":len(rows),
        "expected_runs":gate["expected_flights"],
        "end_to_end_safety":gate,
        "hybrid_fusion_and_shared_blackout":hybrid,
        "planner_errors":planner_errors,"results":summary,
        "caution":[
          "No true simultaneous mapping: exact static prior geometry available",
          "IMU measurements simulated from native velocity finite differences, with Gaussian noise and drift",
          "Initial launch location known and yaw/rotor-attitude loop still use ideal native physics",
          "3D scan matching uses 48 synthetic rays with AABB buildings, no moving pedestrians/vehicles",
          "Only heavy UAV, procedural cities and mild disturbance; not real-world validation",
          "A* replanning uses estimated XYZ and heuristic covariance proxy, not proven consistent Gaussian uncertainty",
          "Unplanned, unsafe and incomplete missions are included as failures in the full experimental denominator",
          "Position ATE is measured on executed flights only and cannot validate mission safety",
          "No LoRa credit is assigned to physically failed flights"
        ]},indent=2)+"\n")
    print(json.dumps(summary,indent=2),flush=True)
    return summary
if __name__=="__main__":
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",default="results/imu3d_replanning_v5")
    parser.add_argument("--hybrid",action="store_true")
    parser.add_argument("--city-seeds",type=int,nargs="+",default=list(CITIES))
    args=parser.parse_args()
    if len(set(args.city_seeds))!=len(args.city_seeds):
        parser.error("City seeds must be distinct")
    evaluate(output=args.output,hybrid=args.hybrid,city_seeds=tuple(args.city_seeds))
