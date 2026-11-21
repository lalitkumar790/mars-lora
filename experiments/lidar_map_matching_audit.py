"""GPS-free map-aided 2D LiDAR correction versus dead-reckoned pose drift.

Not SLAM: ground-truth attitude/velocity proxies and an exact prior city map
remain available; lidar ranges and synthetic drift are the only XY correction
inputs to the scan matcher. Tests actual closed-loop MuJoCo paths and safety.
"""
from __future__ import annotations
import csv,json
from pathlib import Path
from statistics import mean
from mars_lora.rf_astar import INDUSTRIAL,plan
from mars_lora.urban_scene import city
from mujoco_sim.validate_mission import run

CITIES=(15193,77099)
DRIFTS=(.03,.06)
MODES=("drift","lidar_match")
def evaluate(output="results/lidar_map_matching"):
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    rows=[]
    for seed in CITIES:
        solids=INDUSTRIAL+tuple(city(seed))
        routes={name:plan(name,solids=solids,localization_sigma_m=.1).points
                for name in ("shortest","rf_aware")}
        for airframe in ("heavy","crazyflie"):
            for route,points in routes.items():
                for drift in DRIFTS:
                    for mode in MODES:
                        result=run(airframe,duration=340,wind=.25,
                            city_seed=seed,wind_phase=(seed%17)*.13,
                            controller_mode="legacy",route=points,
                            localization_drift_m_sqrt_s=drift,
                            localization_noise_sigma_m=.02,
                            localization_seed=seed+100,localization_mode=mode)
                        result.update(route=route,drift_m_sqrt_s=drift,
                            safety_pass=bool(result["completed"] and
                                result["contact_samples"]==0 and result["min_clearance_m"]>=.25))
                        rows.append(result)
                        print("SCANMATCH",seed,airframe,route,drift,mode,
                              "ATE",result["navigation_ate_rmse_m"],
                              "scan",result["lidar_matches_accepted"],
                              "contacts",result["contact_samples"],
                              "safe",result["safety_pass"],flush=True)
    with (out/"trials.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    summary=[]
    for drift in DRIFTS:
        for mode in MODES:
            subset=[r for r in rows if r["drift_m_sqrt_s"]==drift
                    and r["localization_mode"]==mode]
            summary.append(dict(drift_m_sqrt_s=drift,mode=mode,
                cases=len(subset),safe=sum(r["safety_pass"] for r in subset),
                mean_ate_m=mean(r["navigation_ate_rmse_m"] for r in subset),
                accepted_scan_matches=sum(r["lidar_matches_accepted"] for r in subset),
                rejected_scan_matches=sum(r["lidar_matches_rejected"] for r in subset)))
    with (out/"summary.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
    (out/"manifest.json").write_text(json.dumps({
        "purpose":"Compare open-loop synthetic XY drift against known-map LiDAR scan correction inside real MuJoCo controller",
        "flight_count":len(rows),"test_city_seeds":list(CITIES),
        "drift_levels_m_sqrt_s":list(DRIFTS),
        "lidar_beams":16,"scan_period_s":2.0,"max_range_m":25.0,
        "success_gate":"mission finished, zero native MuJoCo contacts, >=0.25m measured full-city clearance",
        "limitations":[
            "known perfect static building map; NOT simultaneous localization and mapping",
            "simulated ideal horizontal LiDAR rays and yaw from ground truth, not real sensor",
            "ground-truth body velocity is used by controller, so GPS-free autonomy is not proven",
            "single noisy Brownian drift realization per city, no sensor data validation",
            "two cities previously tested, not independent held-out mapping study",
            "very high altitude routes may lack horizontal structural returns"
        ]},indent=2)+"\n")
    print(json.dumps(summary,indent=2),flush=True)
    return summary
if __name__=="__main__":evaluate()
