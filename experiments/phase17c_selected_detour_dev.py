"""Phase 17c native matched verification of a *selected development* viewpoint.

The candidate (seed 48221 shortest) was selected AFTER seeing the Phase17b
preflight results. This is a development-only, post-selection flight experiment
and can never be relabeled as an independent heldout confirmation.
"""
from __future__ import annotations
import json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.urban_scene import city
from mars_lora.transit_route import staged_route
from mars_lora.active_viewpoint import choose_observable_detour
from mars_lora.kinodynamic_reference import quintic_reference
from mujoco_sim.validate_mission import run

def evaluate():
    seed=48221;method="shortest";wind=.35
    root=Path("results/phase17c_selected_detour_dev")
    root.mkdir(parents=True,exist_ok=True)
    solids=INDUSTRIAL+tuple(city(seed))
    route=staged_route(method,solids,transit_z=10.,localization_sigma_m=.10)
    proposal=choose_observable_detour(route,solids,max_reference_s=210.,
                                     minimum_gain=.05)
    rows=[]
    for policy,points in (("baseline",route),("map_oracle_detour",proposal.points)):
        reference=quintic_reference(points,solids,radius_m=.32,sigma_m=.1,
                   clearance_margin_m=.55,max_speed_m_s=1.,
                   max_accel_m_s2=.8,max_jerk_m_s3=1.5,max_duration_s=210.)
        flight=run("heavy",duration=240,wind=wind,city_seed=seed,
             wind_phase=(seed%19)*.12,route=points,
             trajectory_reference=reference,
             out=str(root/f"flight_{policy}.csv"),
             controller_mode="legacy",localization_mode="lidar_imu3d_hybrid",
             dynamic_replanning=False,
             localization_drift_m_sqrt_s=.03,
             localization_noise_sigma_m=.02,localization_seed=seed+71,
             lidar_blackout_window=(45.,75.))
        row={"policy":policy,
             "safe":bool(flight["completed"] and flight["contact_samples"]==0
                           and flight["min_clearance_m"]>=.25),
             "completed":flight["completed"],
             "contacts":flight["contact_samples"],
             "min_clearance_m":flight["min_clearance_m"],
             "elapsed_s":flight["elapsed_s"],
             "localization_rmse_m":flight["navigation_ate_rmse_m"],
             "precontact_rmse_m":flight["legacy_precontact_rmse_m"],
             "first_contact":flight["first_contact"],
             "route_length_m":proposal.baseline_length_m if policy=="baseline"
                            else proposal.candidate_length_m,
             "reference_duration_s":reference.duration_s,
             "nominal_info_fraction":proposal.baseline_informative_fraction
                            if policy=="baseline" else proposal.candidate_informative_fraction,
             "inserted_waypoint":proposal.inserted_waypoint}
        print("DETOUR",json.dumps(row),flush=True)
        rows.append(row)
    summary={"scope":"Post-selected single development city; known-map oracle score, no online body-LiDAR fusion",
            "seed":seed,"route":method,"proposal_status":proposal.outcome,
            "expected":2,"executed":len(rows),"records":rows}
    (root/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print("SUMMARY",json.dumps(summary,indent=2),flush=True)
    return summary
if __name__=="__main__":evaluate()
