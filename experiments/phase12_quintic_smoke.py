"""Single development-route diagnostic: physically track a quintic reference.

This is deliberately not a holdout or 3D autonomy success claim.
"""
from __future__ import annotations
import json
from pathlib import Path
from mars_lora.rf_astar import INDUSTRIAL
from mars_lora.urban_scene import city
from mars_lora.transit_route import staged_route
from mars_lora.kinodynamic_reference import quintic_reference
from mujoco_sim.validate_mission import run

def main():
    seed=91311
    solids=INDUSTRIAL+tuple(city(seed))
    points=staged_route("shortest",solids,transit_z=10.,localization_sigma_m=.1)
    ref=quintic_reference(points,solids,radius_m=.32,sigma_m=.1,
        clearance_margin_m=.55,max_speed_m_s=1.0,
        max_accel_m_s2=.8,max_jerk_m_s3=1.5,max_duration_s=210.)
    dest=Path("results/phase12_quintic_tracking_smoke")
    dest.mkdir(parents=True,exist_ok=True)
    r=run("heavy",duration=240,wind=.25,wind_phase=(seed%19)*.12,
          city_seed=seed,route=points,out=str(dest/"flight.csv"),
          controller_mode="legacy",localization_mode="lidar_imu3d_hybrid",
          dynamic_replanning=False,trajectory_reference=ref,
          localization_drift_m_sqrt_s=.03,localization_noise_sigma_m=.02,
          localization_seed=seed+71,lidar_blackout_window=(45.,75.))
    summary={"context":"single development-city native MuJoCo timed reference smoke, NOT independent validation",
             "reference_duration_s":ref.duration_s,"sample_count":len(ref.samples),
             "reference_speed_peak_m_s":ref.measured_max_speed_m_s,
             "reference_accel_peak_m_s2":ref.measured_max_accel_m_s2,
             "reference_jerk_peak_m_s3":ref.measured_max_jerk_m_s3,
             "reference_min_clearance_m":ref.minimum_clearance_m,
             "native":r,
             "safe":bool(r["completed"] and r["contact_samples"]==0 and r["min_clearance_m"]>=.25)}
    (dest/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary,indent=2),flush=True)

if __name__=="__main__":
    main()
