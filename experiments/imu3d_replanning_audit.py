"""Evaluation of actual 3D LiDAR/inertial feedback with uncertainty replanning.

Hard safety: a flight passes only when completed, no contacts and minimum
city-wide clearance >=0.25 m. This is NOT unknown-map SLAM or measured IMU.
Two seeds not used in the prior research training/first holdouts.
The map, sensor noises and planner settings are locked in source before tests.
"""
from __future__ import annotations
import csv,json
from pathlib import Path
from statistics import mean
from mars_lora.rf_astar import INDUSTRIAL,plan
from mars_lora.urban_scene import city
from mujoco_sim.validate_mission import run

CITIES=(91311,20483)
MODES=(("lidar_match",False),("lidar_imu3d",True))
ROUTES=("shortest","rf_aware")
DRIFT=.03
def evaluate(output="results/imu3d_replanning"):
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    rows=[];planner_errors=[]
    for seed in CITIES:
        solids=INDUSTRIAL+tuple(city(seed))
        try:
            routes={name:plan(name,solids=solids,localization_sigma_m=.10).points
                    for name in ROUTES}
        except (RuntimeError,ValueError,AssertionError) as error:
            planner_errors.append(dict(city_seed=seed,error=str(error)))
            print("NO PLAN",seed,str(error),flush=True);continue
        for route,points in routes.items():
            for mode,replanning in MODES:
                result=run("heavy",duration=240,wind=.25,
                    wind_phase=(seed%19)*.12,city_seed=seed,
                    controller_mode="legacy",route=points,
                    localization_mode=mode,dynamic_replanning=replanning,
                    localization_drift_m_sqrt_s=DRIFT,
                    localization_noise_sigma_m=.02,
                    localization_seed=seed+71)
                result.update(route=route,
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
    summary=[]
    for mode,replanning in MODES:
        xs=[x for x in rows if x["localization_mode"]==mode]
        if not xs:continue
        summary.append(dict(mode=mode,scenarios=len(xs),
           safety_pass=sum(x["safe"] for x in xs),
           mean_ate_m=mean(x["navigation_ate_rmse_m"] for x in xs),
           total_map_replans=sum(x["map_replans"] for x in xs),
           total_replan_failures=sum(x["map_replan_failures"] for x in xs),
           total_lidar_corrections=sum(x["lidar_matches_accepted"] for x in xs)))
    (out/"summary.json").write_text(json.dumps({
        "phase":"Experimental 3D range plus inertial state estimation, uncertainty-based map replanning",
        "city_seeds":list(CITIES),"physical_runs":len(rows),
        "expected_runs":len(CITIES)*len(ROUTES)*len(MODES),
        "planner_errors":planner_errors,"results":summary,
        "caution":[
          "No true simultaneous mapping: exact static prior geometry available",
          "IMU measurements simulated from native velocity finite differences, with Gaussian noise and drift",
          "Initial launch location known and yaw/rotor-attitude loop still use ideal native physics",
          "3D scan matching uses 48 synthetic rays with AABB buildings, no moving pedestrians/vehicles",
          "Only heavy UAV, two new cities and mild disturbance; not real-world validation",
          "A* replanning uses estimated XYZ and heuristic covariance proxy, not proven consistent Gaussian uncertainty",
          "Unsafe/incomplete flights remain explicit in outcome; no optimistic packet replay for them"
        ]},indent=2)+"\n")
    print(json.dumps(summary,indent=2),flush=True)
    return summary
if __name__=="__main__":evaluate()
