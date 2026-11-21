"""Phase 15 passive LiDAR observability audit on two development native flights."""
from __future__ import annotations
import json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.urban_scene import city
from mars_lora.transit_route import staged_route
from mars_lora.kinodynamic_reference import quintic_reference
from mujoco_sim.validate_mission import run

CASES=((91311,"shortest",.25),(48221,"rf_aware",.35))

def evaluate(out="results/phase15_observability_dev"):
    output=Path(out);output.mkdir(parents=True,exist_ok=True)
    results=[]
    for seed,method,wind in CASES:
        solids=INDUSTRIAL+tuple(city(seed))
        points=staged_route(method,solids,transit_z=10.,localization_sigma_m=.1)
        reference=quintic_reference(points,solids,radius_m=.32,sigma_m=.1,
             clearance_margin_m=.55,max_speed_m_s=1.,
             max_accel_m_s2=.8,max_jerk_m_s3=1.5,max_duration_s=210.)
        f=run("heavy",duration=240,wind=wind,city_seed=seed,
           wind_phase=(seed%19)*.12,route=points,trajectory_reference=reference,
           out=str(output/f"flight_{seed}_{method}.csv"),eskf15_shadow=True,
           controller_mode="legacy",localization_mode="lidar_imu3d_hybrid",
           dynamic_replanning=False,localization_drift_m_sqrt_s=.03,
           localization_noise_sigma_m=.02,localization_seed=seed+71,
           lidar_blackout_window=(45.,75.))
        first_contact_time=(f["first_contact"]["t_s"]
                            if f["first_contact"] is not None else None)
        scans=f["lidar_observability_timeline"]
        before=[x for x in scans if first_contact_time is None or
                x["t_s"]<first_contact_time]
        first_bad=next((x for x in before if not x["informative"]),None)
        first_good=next((x for x in before if x["informative"]),None)
        max_precontact_gap=0.
        last_good=None
        for frame in before:
            if frame["informative"]:last_good=frame["t_s"]
            if last_good is not None:
                max_precontact_gap=max(max_precontact_gap,
                                       frame["t_s"]-last_good)
        item={"precontact_scan_count":len(before),
             "precontact_informative":sum(x["informative"] for x in before),
             "precontact_rank0":sum(x["rank"]==0 for x in before),
             "precontact_rank2":sum(x["rank"]==2 for x in before),
             "precontact_rank3":sum(x["rank"]==3 for x in before),
             "precontact_max_observation_gap_s":round(max_precontact_gap,3),
             "first_precontact_noninformative_scan_s":(
                 first_bad["t_s"] if first_bad else None),
             "first_precontact_informative_scan_s":(
                 first_good["t_s"] if first_good else None),
             "seed":seed,"method":method,"safe":bool(
                 f["completed"] and f["contact_samples"]==0 and f["min_clearance_m"]>=.25),
            "legacy_precontact_rmse_m":f["legacy_precontact_rmse_m"],
            "eskf15_precontact_rmse_m":f["eskf15_shadow_precontact_rmse_m"],
            "eskf15_first_error_1m":f["eskf15_error_1m_first"],
            "eskf15_first_error_5m":f["eskf15_error_5m_first"],
            "first_contact":f["first_contact"],
            "observable_scans":f["lidar_observable_scans"],
            "unobservable_scans":f["lidar_unobservable_scans"],
            "rank_histogram":f["lidar_observability_rank_histogram"],
            "longest_observation_gap_s":f["lidar_longest_observation_gap_s"],
            "first_unobservable_scan_t":f["lidar_first_unobservable_scan_t"],
            "first_high_residual_scan_t":f["lidar_first_high_residual_scan_t"],
            "scan_3d_accepted":f["eskf15_lidar3d_accepted"],
            "scan_3d_rejected":f["eskf15_lidar3d_rejected"]}
        (output/f"scan_timeline_{seed}_{method}.json").write_text(
           json.dumps(f["lidar_observability_timeline"],indent=2)+"\n")
        print("OBSERVABILITY",json.dumps(item),flush=True)
        results.append(item)
    summary={"scope":"Known-map synthetic 3D LiDAR local translational observability, passive, 2 DEVELOPMENT flights",
             "expected":2,"executed":len(results),"results":results}
    (output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print("SUMMARY",json.dumps(summary,indent=2),flush=True)
    return summary

if __name__=="__main__":evaluate()
