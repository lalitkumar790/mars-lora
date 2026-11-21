"""Wind robustness sweep for previously planned 3D urban UAV routes.

Fixed routes and aircraft. Varies deterministic forcing amplitude, NOT random
wind seeds; outputs are a scenario sweep, not statistical robustness estimates.
"""
from __future__ import annotations
import argparse,csv,json
from pathlib import Path
from mujoco_sim.validate_mission import run

def evaluate(routes_file,out_dir,wind_levels=(0.,.25,.5,1.,1.5)):
    import json
    paths=json.loads(Path(routes_file).read_text())
    directory=Path(out_dir);directory.mkdir(parents=True,exist_ok=True)
    results=[]
    for mode in ("legacy","high_authority"):
        for airframe in ("heavy","crazyflie"):
            for route in ("shortest","rf_aware"):
                for wind in wind_levels:
                    outcome=run(airframe,duration=440,wind=float(wind),
                                route=paths[route]["points"],require_completion=False,
                                controller_mode=mode)
                    outcome["route"]=route
                    outcome["wind_scale"]=float(wind)
                    # Contact and clearance are measured in native MuJoCo.
                    outcome["passed"]=bool(outcome["completed"] and
                                         outcome["contact_samples"]==0 and
                                         outcome["min_clearance_m"]>=.25)
                    results.append(outcome)
                    print(f"{mode} {airframe} {route} wind={wind}: "
                          f"completed={outcome['completed']} contacts={outcome['contact_samples']} "
                          f"clearance={outcome['min_clearance_m']:.2f}m "
                          f"tracking={outcome['xy_tracking_rms_m']:.2f}m",flush=True)
    with (directory/"wind_sweep.csv").open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(results[0]))
        writer.writeheader();writer.writerows(results)
    summary={
        "scenarios":len(results),
        "passed":sum(x["passed"] for x in results),
        "passed_by_controller":{mode:sum(x["passed"] for x in results if x["controller_mode"]==mode)
                                for mode in ("legacy","high_authority")},
        "failed":[x for x in results if not x["passed"]],
        "wind_levels":list(wind_levels),
        "scope":"Deterministic sinusoidal disturbance force amplitudes, not real wind speeds",
        "limitations":[
            "No randomized aerodynamic turbulence or different obstacle layouts",
            "Wind amplitude is a force-proxy parameter, not m/s",
            "The higher-authority gains are exploratory, not tuned on held-out wind distributions",
            "One trial per airframe/route/wind; no uncertainty intervals",
            "Planner precomputes geometry before wind; controller is not retuned"
        ]
    }
    (directory/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps({k:v for k,v in summary.items() if k!="failed"}))
    return results

if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--routes",default="results/rf_navigation/routes.json")
    parser.add_argument("--output",default="results/wind_robustness")
    args=parser.parse_args()
    evaluate(args.routes,args.output)
