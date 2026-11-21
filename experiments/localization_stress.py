"""Native MuJoCo pose-drift sensitivity, NOT SLAM or GPS-less autonomy.

Commanded XY uses a biased, noisy simulated position estimate. Attitude and
velocity still use MuJoCo truth proxies; ATE is scored against simulator truth.
"""
import csv,json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL,plan
from mars_lora.urban_scene import city
from mujoco_sim.validate_mission import run
SEEDS=(15193,77099)
DRIFTS=(0.,.01,.03,.06)
def evaluate(output="results/localization_stress"):
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    rows=[]
    for seed in SEEDS:
        solids=INDUSTRIAL+tuple(city(seed))
        routes={name:plan(name,solids=solids,localization_sigma_m=.10).points
                for name in ("shortest","rf_aware")}
        for vehicle in ("heavy","crazyflie"):
            for route,points in routes.items():
                for drift in DRIFTS:
                    result=run(vehicle,duration=340,wind=.25,
                        wind_phase=(seed%17)*.13,city_seed=seed,route=points,
                        controller_mode="legacy",
                        localization_drift_m_sqrt_s=drift,
                        localization_noise_sigma_m=.02,
                        localization_seed=seed+100)
                    result.update(route=route,drift_m_sqrt_s=drift,
                        safe=bool(result["completed"] and result["contact_samples"]==0
                                  and result["min_clearance_m"]>=.25))
                    rows.append(result)
                    print("POSE",seed,vehicle,route,drift,
                          result["navigation_ate_rmse_m"],result["safe"],flush=True)
    with (out/"trials.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]))
        w.writeheader();w.writerows(rows)
    summary={
        "cases":len(rows),"safe":sum(r["safe"] for r in rows),
        "by_drift":[{"drift_m_sqrt_s":d,
                     "cases":sum(r["drift_m_sqrt_s"]==d for r in rows),
                     "safe":sum(r["safe"] for r in rows if r["drift_m_sqrt_s"]==d),
                     "mean_ate_m":sum(r["navigation_ate_rmse_m"] for r in rows
                                      if r["drift_m_sqrt_s"]==d)/8} for d in DRIFTS],
        "limitations":["Brownian drift is synthetic, not sensor-based SLAM",
                       "attitude and velocity remain simulator truth proxies",
                       "static 3-sigma planning margin assumes 0.10 m localization error",
                       "two previously examined cities, mild deterministic wind",
                       "fixed-seed results do not quantify general field safety"]
    }
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    return summary
if __name__=="__main__":evaluate()
