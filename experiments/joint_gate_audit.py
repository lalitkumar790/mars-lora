"""Joint candidate (route + fixed SF) falsification on held-out-looking scenarios.

IMPORTANT: The five propagation profiles have already been inspected by the
researchers. This is post-hoc *development* evidence, not independent held-out
validation. Flight trajectories are still pre-executed native MuJoCo traces.
"""
from __future__ import annotations
import argparse,csv,json
from pathlib import Path
from statistics import mean
from mars_lora.deadline_gate import choose_deadline_route
from mars_lora.route_gate import simulated_pilot
from experiments.replay_mujoco_radio import load_trajectory,trial
from experiments.heldout_validation import PROFILES,STATIONS

METRICS=("pdr","critical_deadline_ratio","aoi_p95_s",
         "outage_fraction","tx_energy_j","mission_duration_s")

def evaluate(paths,heldout,output,seeds=20):
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    routes=json.loads(Path(paths,"routes.json").read_text())
    alternatives={name:record["points"] for name,record in routes.items()}
    traces={(airframe,route):load_trajectory(Path(paths,f"{airframe}_{route}_flight.csv"))
            for airframe in ("heavy","crazyflie") for route in ("shortest","rf_aware")}
    with open(Path(heldout,"trials.csv"),newline="") as f:
        published=list(csv.DictReader(f))
    matched={(r["scenario"],r["airframe"],int(r["seed"]),r["route"]):r for r in published}
    all_rows=[]
    for profile_name,profile in PROFILES.items():
        station=STATIONS[profile_name]
        for airframe in ("heavy","crazyflie"):
            for seed in range(seeds):
                pilot=simulated_pilot(profile,station,seed)
                chosen,candidates=choose_deadline_route(
                    alternatives,pilot["pilot_rssi_dbm"],
                    pilot["observed_channel_busy"],receiver=station)
                # Actually replay the jointly chosen fixed SF + priority on
                # the matching *executed MuJoCo trajectory*. Do not infer
                # performance from the planner's own predicted score.
                observed=trial(traces[(airframe,chosen.route)],
                              "priority-adaptive",seed,station,shadow_sigma=5.,
                              profile=profile,spatial_fading=True,
                              sf_override=chosen.spreading_factor)
                baseline=matched[(profile_name,airframe,seed,"shortest")]
                rf_base=matched[(profile_name,airframe,seed,"rf_aware")]
                row=dict(scenario=profile_name,airframe=airframe,seed=seed,
                         choice=chosen.route,chosen_sf=chosen.spreading_factor,
                         observed_pilot_dbm=pilot["pilot_rssi_dbm"],
                         observed_busy_fraction=pilot["observed_channel_busy"],
                         predicted_alert_success=chosen.estimated_alert_success,
                         predicted_utility=chosen.score,
                         predicted_short_score=candidates[0].score,
                         predicted_rf_score=candidates[1].score)
                for metric in METRICS:
                    row["joint_"+metric]=observed[metric]
                    row["short_adaptive_"+metric]=baseline[metric]
                    row["rf_adaptive_"+metric]=rf_base[metric]
                all_rows.append(row)
    with (out/"trials.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(all_rows[0]))
        w.writeheader();w.writerows(all_rows)
    summary=[]
    for scenario in PROFILES:
        for airframe in ("heavy","crazyflie"):
            records=[r for r in all_rows if r["scenario"]==scenario and r["airframe"]==airframe]
            row=dict(scenario=scenario,airframe=airframe,n=len(records),
                     selected_shortest=sum(r["choice"]=="shortest" for r in records),
                     selected_rf=sum(r["choice"]=="rf_aware" for r in records),
                     average_sf=mean(r["chosen_sf"] for r in records))
            for m in METRICS:
                row[m+"_joint"]=mean(float(r["joint_"+m]) for r in records)
                row[m+"_short_adaptive"]=mean(float(r["short_adaptive_"+m]) for r in records)
                row[m+"_rf_adaptive"]=mean(float(r["rf_adaptive_"+m]) for r in records)
                row[m+"_gain_over_short"]=row[m+"_joint"]-row[m+"_short_adaptive"]
            summary.append(row)
            print(scenario,airframe,
                  f"joint PDR {row['pdr_joint']:.3f}; deadline {row['critical_deadline_ratio_joint']:.3f}; "
                  f"short baseline {row['critical_deadline_ratio_short_adaptive']:.3f}; "
                  f"SF {row['average_sf']:.2f}",flush=True)
    with (out/"summary.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(summary[0]))
        w.writeheader();w.writerows(summary)
    (out/"manifest.json").write_text(json.dumps({
        "purpose":"Falsify a joint ex-ante map-known route and fixed-SF gate",
        "scientific_status":"post-hoc development; not independent validation",
        "completed_trials":len(all_rows),
        "policies":"joint selected route with fixed selected SF and priority scheduling versus both adaptive-SF route baselines",
        "caveats":[
            "Threshold weights chosen after inspecting synthetic channel scenario outcomes",
            "A pilot at the initial position is a poor estimator of full route quality",
            "Selected static SF is coordinated with receiver at zero setup cost, optimistic",
            "Same synthetic propagation family used for both planning and evaluation",
            "Trajectories and safety results are reused, not re-flown in response to radio",
            "No sensor uncertainty, no GPS-denied SLAM, no real LoRa hardware"
        ]
    },indent=2)+"\n")
    return summary

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--flight",default="results/rf_navigation")
    p.add_argument("--heldout",default="results/heldout_validation")
    p.add_argument("--output",default="results/joint_gate")
    p.add_argument("--seeds",type=int,default=20)
    a=p.parse_args()
    evaluate(a.flight,a.heldout,a.output,a.seeds)
