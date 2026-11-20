"""Execute map-known 3-D A* routes through *native MuJoCo*, then evaluate LoRa.

Methods:
 S0: shortest geometric route, SF9
 S1: same shortest route, adaptive+priority
 R2: RF-weighted 3D route, same adaptive+priority

Thus S1 vs R2 isolates *planning changes*. This is a map-known benchmark:
perfect planning geometry is not GPS-denied SLAM. Results remain synthetic
radio evidence and cannot be treated as hardware measurements.
"""
from __future__ import annotations
import argparse,csv,json,math
from pathlib import Path
from statistics import mean,stdev
from mars_lora.rf_astar import plan,GCS
from mujoco_sim.validate_mission import run
from experiments.replay_mujoco_radio import load_trajectory,trial

def execute(out,airframes=("heavy","crazyflie"),seeds=20):
    root=Path(out);root.mkdir(parents=True,exist_ok=True)
    routes={key:plan(key) for key in ("shortest","rf_aware")}
    route_details={
        name:dict(points=route.points,length_m=route.geometric_length_m,
                  predicted_mean_success=route.predicted_mean_success,
                  minimum_planned_clearance_m=route.minimum_clearance_m,
                  astar_expanded=route.expanded_nodes,objective=route.objective)
        for name,route in routes.items()
    }
    (root/"routes.json").write_text(json.dumps(route_details,indent=2)+"\n")
    flights=[]
    trials=[]
    for airframe in airframes:
        for name,route in routes.items():
            trajectory=root/(f"{airframe}_{name}_flight.csv")
            # Same airframe and wind across planner comparisons. Safety and
            # completion gate requires zero MuJoCo contact events.
            info=run(airframe,duration=440,wind=.25,out=str(trajectory),
                     route=route.points,require_completion=True)
            info.update(route=name,planned_distance_m=route.geometric_length_m,
                        predicted_mean_success=route.predicted_mean_success)
            flights.append(info)
            trace=load_trajectory(trajectory)
            for sigma in (4.,8.):
                for seed in range(seeds):
                    for method in (("fixed",) if name=="shortest" else ()):
                        pass
                    policies=("fixed","priority-adaptive") if name=="shortest" else ("priority-adaptive",)
                    for policy in policies:
                        x=trial(trace,policy,seed,GCS,sigma)
                        x.update(airframe=airframe,route=name,
                            condition=("S0_shortest_fixed" if policy=="fixed"
                                else "S1_shortest_adaptive" if name=="shortest"
                                else "R2_RF_route_adaptive"),
                            planned_length_m=route.geometric_length_m,
                            min_clearance_m=info["min_clearance_m"],
                            contacts=info["contact_samples"],
                            xy_tracking_rms_m=info["xy_tracking_rms_m"])
                        trials.append(x)
    with (root/"flights.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(flights[0]));w.writeheader();w.writerows(flights)
    with (root/"trials.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(trials[0]));w.writeheader();w.writerows(trials)
    grouped={}
    for x in trials:
        key=(x["airframe"],x["shadow_sigma_db"],x["condition"])
        grouped.setdefault(key,[]).append(x)
    metrics=("pdr","critical_deadline_ratio","aoi_p95_s","outage_fraction",
             "nlos_fraction","tx_energy_j","mission_duration_s","planned_length_m",
             "min_clearance_m","xy_tracking_rms_m")
    summary=[]
    for (airframe,sigma,condition),xs in sorted(grouped.items()):
        record=dict(airframe=airframe,shadow_sigma_db=sigma,condition=condition,n=len(xs))
        for m in metrics:
            v=[float(x[m]) for x in xs]
            record[m+"_mean"]=mean(v)
            record[m+"_sd"]=stdev(v)
        summary.append(record)
    with (root/"summary.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
    (root/"manifest.json").write_text(json.dumps({
        "experiment":"Map-known 3D RF-aware A* executed in MuJoCo, then matched-seed LoRa replay",
        "n_trials":len(trials),"seeds_per_condition":seeds,
        "goal_xyz_m":(-24,24,4),"start_xyz_m":(0,0,4),
        "receiver_xyz_m":GCS,
        "conditions":["S0_shortest_fixed","S1_shortest_adaptive","R2_RF_route_adaptive"],
        "limitations":[
            "Planner assumes known perfect static city map; does not use estimated SLAM pose",
            "RF path loss, obstruction and reception probabilities remain uncalibrated",
            "Controller tracks planned waypoints with errors, measured by native MuJoCo",
            "Routes may differ in duration and therefore generated alert counts; compare normalized ratios cautiously",
            "Random fading is time-indexed, not shared as a continuous spatial shadow field",
            "Different navigation trajectories are not executed under independently randomized wind"
        ]
    },indent=2)+"\n")
    print(json.dumps({"routes":route_details,"flights":flights,
                      "trials":len(trials),"summary":summary},indent=2))
    return summary

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",default="results/rf_navigation")
    p.add_argument("--seeds",type=int,default=20)
    args=p.parse_args()
    if args.seeds<2:p.error("Need at least two seeds")
    execute(args.output,seeds=args.seeds)
if __name__=="__main__":main()
