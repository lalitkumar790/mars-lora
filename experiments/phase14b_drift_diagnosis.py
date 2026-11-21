"""Phase 14b divergence timeline: known failed DEVELOPMENT flight only.

No controller changes. No tuning here. This is a diagnosis of the previously
observed large 15-state pre-contact drift in a feature-poor high-rise route.
"""
from __future__ import annotations
import json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.urban_scene import city
from mars_lora.transit_route import staged_route
from mars_lora.kinodynamic_reference import quintic_reference
from mujoco_sim.validate_mission import run

def evaluate():
    seed=48221
    solids=INDUSTRIAL+tuple(city(seed))
    route=staged_route("rf_aware",solids,transit_z=10.,localization_sigma_m=.1)
    reference=quintic_reference(route,solids,radius_m=.32,sigma_m=.1,
         clearance_margin_m=.55,max_speed_m_s=1.,
         max_accel_m_s2=.8,max_jerk_m_s3=1.5,max_duration_s=210.)
    output=Path("results/phase14b_drift_diagnosis")
    output.mkdir(parents=True,exist_ok=True)
    f=run("heavy",duration=240,wind=.35,city_seed=seed,
          wind_phase=(seed%19)*.12,route=route,trajectory_reference=reference,
          out=str(output/"flight.csv"),controller_mode="legacy",
          localization_mode="lidar_imu3d_hybrid",dynamic_replanning=False,
          eskf15_shadow=True,localization_drift_m_sqrt_s=.03,
          localization_noise_sigma_m=.02,localization_seed=seed+71,
          lidar_blackout_window=(45.,75.))
    summary={
        "scope":"posthoc known-failed native development flight, no controller action",
        "first_error_1m":f["eskf15_error_1m_first"],
        "first_error_5m":f["eskf15_error_5m_first"],
        "precontact_attitude_rmse_deg":f["eskf15_precontact_attitude_rmse_deg"],
        "precontact_position_rmse_m":f["eskf15_shadow_precontact_rmse_m"],
        "first_contact":f["first_contact"],
        "scan_3d_accepted":f["eskf15_lidar3d_accepted"],
        "scan_3d_rejected":f["eskf15_lidar3d_rejected"],
        "scan_planar_accepted":f["eskf15_planar_accepted"],
        "scan_planar_rejected":f["eskf15_planar_rejected"],
        "completed":f["completed"],"contact_samples":f["contact_samples"],
        "warning":"No independence claim. Sensor model, lack of features and filter model mismatch still unresolved."}
    (output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary,indent=2),flush=True)
if __name__=="__main__":evaluate()
