"""First-contact localization causality diagnostic on FAILED development case.

Averages over all 240s after an impact can hide whether the estimator
was inaccurate before the impact. Compare the first-contact state instead.
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
    seed=48221;method="rf_aware";wind=.35
    solids=INDUSTRIAL+tuple(city(seed))
    points=staged_route(method,solids,transit_z=10.,localization_sigma_m=.10)
    ref=quintic_reference(points,solids,radius_m=.32,sigma_m=.10,
        clearance_margin_m=.55,max_speed_m_s=1.,
        max_accel_m_s2=.8,max_jerk_m_s3=1.5,max_duration_s=210.)
    root=Path("results/phase12b_contact_causality")
    root.mkdir(parents=True,exist_ok=True)
    result={}
    for policy in ("staged","quintic"):
        kwargs={"trajectory_reference":ref} if policy=="quintic" else {"altitude_slew_m_s":.65}
        f=run("heavy",duration=240,wind=wind,city_seed=seed,
             wind_phase=(seed%19)*.12,route=points,
             out=str(root/f"{policy}.csv"),
             controller_mode="legacy",localization_mode="lidar_imu3d_hybrid",
             dynamic_replanning=False,localization_drift_m_sqrt_s=.03,
             localization_noise_sigma_m=.02,localization_seed=seed+71,
             lidar_blackout_window=(45.,75.),online_risk_monitor=True,**kwargs)
        result[policy]={
           "safe":bool(f["completed"] and f["contact_samples"]==0 and f["min_clearance_m"]>=.25),
           "completed":f["completed"],"min_clearance_m":f["min_clearance_m"],
           "contacts":f["contact_samples"],"whole_flight_ate_rmse_m":f["navigation_ate_rmse_m"],
           "first_contact":f["first_contact"],
           "first_clearance_violation":f["first_clearance_violation"],
           "reference_started":f["reference_started"],
           "risk_alarm_first":f["first_risk_alarm"],
           "risk_alarm_count":f["risk_alarm_count"],
           "confidence_warning_first":f["loss_of_confidence_first"],
           "confidence_warning_samples":f["loss_of_confidence_samples"],
           "confidence_lead_time_s":(
               round(f["first_contact"]["t_s"]-f["loss_of_confidence_first"]["t_s"],3)
               if f["first_contact"] is not None and f["loss_of_confidence_first"] is not None
               else None),
           "risk_monitor_checks":f["risk_monitor_checks"],
           "lead_time_s":(round(f["first_contact"]["t_s"]-f["first_risk_alarm"]["t_s"],3)
               if f["first_contact"] is not None and
                  f["first_risk_alarm"] is not None else None)}
        print(policy,json.dumps(result[policy]),flush=True)
    (root/"summary.json").write_text(json.dumps(result,indent=2)+"\n")
if __name__=="__main__":
    main()
