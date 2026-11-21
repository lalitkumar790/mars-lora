"""Phase 10: isolate the terminal descent-contact failure.

Development-only physical simulation, same staged flight path and seeded
sensor/wind across the two rate-control variants. Dynamic replanning disabled.
"""
from __future__ import annotations
import csv,json,argparse
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.transit_route import staged_route
from mars_lora.urban_scene import city
from mujoco_sim.validate_mission import run


def evaluate(output="results/phase10_descent_slew_dev",seeds=(91311,20483)):
    dest=Path(output);dest.mkdir(parents=True,exist_ok=True)
    rows=[];planner_errors=[]
    for seed in seeds:
        solids=INDUSTRIAL+tuple(city(seed))
        for method in ("shortest","rf_aware"):
            try:
                points=staged_route(method,solids,transit_z=10.,
                                    localization_sigma_m=.10)
            except (ValueError,RuntimeError,AssertionError) as error:
                planner_errors.append({"seed":seed,"route":method,"error":str(error)})
                continue
            for slew in (None,.65):
                label="unlimited" if slew is None else "slew_0p65m_s"
                trace=dest / f"flight_{seed}_{method}_{label}.csv"
                result=run("heavy",duration=240,wind=.25,out=str(trace),
                    route=points,wind_phase=(seed%19)*.12,city_seed=seed,
                    controller_mode="legacy",localization_mode="lidar_imu3d_hybrid",
                    dynamic_replanning=False,altitude_slew_m_s=slew,
                    localization_drift_m_sqrt_s=.03,
                    localization_noise_sigma_m=.02,
                    localization_seed=seed+71,lidar_blackout_window=(45.,75.))
                result.update(route=method,descent_policy=label,
                    safe=bool(result["completed"] and result["contact_samples"]==0
                              and result["min_clearance_m"]>=.25))
                rows.append(result)
                print(seed,method,label,"safe",result["safe"],
                      "contacts",result["contact_samples"],
                      "clearance",result["min_clearance_m"],
                      "complete",result["completed"],flush=True)
    if rows:
        with (dest/"trials.csv").open("w",newline="") as file:
            writer=csv.DictWriter(file,fieldnames=list(rows[0]))
            writer.writeheader();writer.writerows(rows)
    summaries=[]
    for label in ("unlimited","slew_0p65m_s"):
        r=[x for x in rows if x["descent_policy"]==label]
        summaries.append({"policy":label,"expected":len(seeds)*2,
            "executed":len(r),"safe":sum(x["safe"] for x in r),
            "contact_failures":sum(x["contact_samples"]>0 for x in r),
            "clearance_failures":sum(x["min_clearance_m"]<.25 for x in r),
            "incomplete":sum(not x["completed"] for x in r),
            "first_contacts":[{"seed":x["city_seed"],"route":x["route"],
                "contact":x["first_contact"]} for x in r]})
    summary={"scope":"Post-hoc MuJoCo development, not flight certification",
             "expected_missions":len(seeds)*2*2,"executed":len(rows),
             "planning_failures":planner_errors,"results":summaries}
    (dest/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary,indent=2),flush=True)
    return summary

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--output",default="results/phase10_descent_slew_dev")
    a=p.parse_args()
    evaluate(a.output)
