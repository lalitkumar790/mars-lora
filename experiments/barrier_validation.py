"""First-stage barrier-controller test on a compact disturbance envelope.

Keep every failure. This stage is exploratory; it does not certify safety.
"""
from __future__ import annotations
import json,csv
from pathlib import Path
from mujoco_sim.validate_mission import run

def execute(routes="results/rf_navigation/routes.json",output="results/barrier_guard",
            winds=(0.,.25,.5,1.,1.5),controller_mode="barrier_guard"):
    paths=json.loads(Path(routes).read_text())
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    rows=[]
    for vehicle in ("heavy","crazyflie"):
        for route in ("shortest","rf_aware"):
            for wind in winds:
                r=run(vehicle,duration=440,wind=wind,
                      route=paths[route]["points"],controller_mode=controller_mode)
                r.update(route=route,wind_scale=wind,passed=bool(
                    r["completed"] and r["contact_samples"]==0 and r["min_clearance_m"]>=.25))
                rows.append(r)
                print(vehicle,route,wind,"completed",r["completed"],
                      "contact",r["contact_samples"],"clearance",r["min_clearance_m"],
                      "PASS",r["passed"],flush=True)
    with (out/"trials.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]))
        w.writeheader();w.writerows(rows)
    summary={"scenarios":len(rows),
             "passed":sum(x["passed"] for x in rows),
             "failed":[x for x in rows if not x["passed"]],
             "method":"Native MuJoCo, full-city signed clearance, finite difference acceleration barrier "+controller_mode,
             "limitations":["Tests are deterministic and share the legacy wind time series",
                            "Controller has only static map ground truth, not noisy lidar",
                            "A CBF-style acceleration projection is not formal safety proof under motor saturation",
                            "No hardware validation or independent city seeds"]}
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    return summary
if __name__=="__main__":
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument("--controller",choices=("barrier_guard","barrier_ttc"),default="barrier_guard")
    parser.add_argument("--output",default="results/barrier_guard")
    args=parser.parse_args()
    execute(output=args.output,controller_mode=args.controller)
