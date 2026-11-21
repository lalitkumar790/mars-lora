"""Reproducible paired estimator study on identical simulated sensor packets.

24 seeded development trajectories, synthetic world-frame gravity-compensated
acceleration / position surrogate / barometer / flow. This isolates filter
calibration and outlier handling; *not* actual scan registration or MuJoCo
rotor dynamics and not an independent frozen UAV-flight holdout.
"""
from __future__ import annotations
import argparse,json,math,random,statistics
from pathlib import Path
from mars_lora.lidar_imu3d import InertialMapEstimator
from mars_lora.lidar_imu_eskf import TranslationalESKF

DEVELOPMENT_SEEDS=tuple(range(81101,81125))
DT=.1
DURATION_S=50
OUTAGE=(15.,30.)
POSITION_OBS_PERIOD=2.
OUTLIER_TIMES=(8.,36.)


def one_seed(seed):
    rng=random.Random(seed)
    truth=[0.,0.,4.]
    vel=[0.,0.,0.]
    accel_bias=[rng.gauss(0.,.004) for _ in range(3)]
    legacy=InertialMapEstimator.initial(tuple(truth))
    eskf=TranslationalESKF.initial(tuple(truth))
    legacy_errors=[];eskf_errors=[];coverage=[]
    rejected_outliers=0;accepted_outliers=0
    next_scan=2.
    for i in range(int(DURATION_S/DT)):
        t=i*DT
        accel=(.10*math.sin(.19*t),.08*math.cos(.27*t),0.)
        old=tuple(vel)
        for j in range(3):
            truth[j]+=old[j]*DT+.5*accel[j]*DT*DT
            vel[j]+=accel[j]*DT
        measured=tuple(accel[j]+accel_bias[j]+rng.gauss(0.,.025) for j in range(3))
        legacy.predict(measured,0.,DT)
        eskf.predict(measured,DT)
        if i%2==0:
            z=truth[2]+rng.gauss(0.,.06)
            flowx=vel[0]+rng.gauss(0.,.045)
            flowy=vel[1]+rng.gauss(0.,.045)
            legacy.correct_altitude(z)
            legacy.correct_horizontal_flow(flowx,flowy)
            eskf.update_altitude(z,std_m=.06)
            eskf.update_horizontal_flow(flowx,flowy,std_m=.045)
        if t+1.e-8>=next_scan:
            blackout=OUTAGE[0]<=t<=OUTAGE[1]
            if not blackout:
                xyz=[truth[j]+rng.gauss(0.,.12) for j in range(3)]
                outlier=any(abs(t-o)<.06 for o in OUTLIER_TIMES)
                if outlier:
                    xyz[0]+=6.
                    xyz[1]-=5.
                # Legacy accepted scan registration is represented here by
                # the same externally supplied position fix; no use of truth
                # except simulated observations and offline error scoring.
                for j in range(3):
                    legacy.position[j]+=.82*(xyz[j]-legacy.position[j])
                    legacy.velocity[j]*=.92
                legacy.sigma_m=max(.055,min(.45,.68*legacy.sigma_m+.025))
                outcome=eskf.update_position(xyz,std_m=.12)
                if outlier:
                    rejected_outliers+=int(not outcome.accepted)
                    accepted_outliers+=int(outcome.accepted)
            next_scan+=POSITION_OBS_PERIOD
        eold=math.dist(legacy.position,truth)
        enew=math.dist(eskf.position,truth)
        legacy_errors.append(eold)
        eskf_errors.append(enew)
        # 99% 3D chi-squared confidence ellipsoid, not '3 sigma'.
        coverage.append(eskf.position_nees(truth)<=11.345)
    return {"seed":seed,"legacy_rmse_m":math.sqrt(sum(v*v for v in legacy_errors)/len(legacy_errors)),
            "eskf_rmse_m":math.sqrt(sum(v*v for v in eskf_errors)/len(eskf_errors)),
            "legacy_max_error_m":max(legacy_errors),
            "eskf_max_error_m":max(eskf_errors),
            "eskf_99pct_ellipsoid_coverage":sum(coverage)/len(coverage),
            "eskf_rejected_position_outliers":rejected_outliers,
            "eskf_accepted_position_outliers":accepted_outliers,
            "eskf_bias_estimate_m_s2":eskf.bias.tolist(),
            "injected_bias_m_s2":accel_bias,
            "eskf_position_rms_sigma_m":eskf.position_rms_sigma_m}


def evaluate(out="results/phase13_eskf_development"):
    output=Path(out);output.mkdir(parents=True,exist_ok=True)
    trials=[one_seed(seed) for seed in DEVELOPMENT_SEEDS]
    summary={"scope":"synthetic sensor-fusion development, not native MuJoCo flight or actual scan matching",
             "seed_count":len(trials),"seeds":list(DEVELOPMENT_SEEDS),
             "sensor_assumptions":{"imu":"world-frame gravity-compensated acceleration",
                 "accel_noise_std_m_s2":.025,"bias_std_m_s2":.004,
                 "position_fix_std_m":.12,"fix_period_s":2.,
                 "blackout_s":list(OUTAGE),"injected_position_outlier_times_s":list(OUTLIER_TIMES),
                 "altitude_std_m":.06,"flow_std_m_s":.045},
             "legacy_mean_rmse_m":statistics.mean(x["legacy_rmse_m"] for x in trials),
             "eskf_mean_rmse_m":statistics.mean(x["eskf_rmse_m"] for x in trials),
             "legacy_mean_max_error_m":statistics.mean(x["legacy_max_error_m"] for x in trials),
             "eskf_mean_max_error_m":statistics.mean(x["eskf_max_error_m"] for x in trials),
             "eskf_99pct_ellipsoid_empirical_coverage":statistics.mean(x["eskf_99pct_ellipsoid_coverage"] for x in trials),
             "rejected_injected_outliers":sum(x["eskf_rejected_position_outliers"] for x in trials),
             "accepted_injected_outliers":sum(x["eskf_accepted_position_outliers"] for x in trials),
             "trials":trials}
    (output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps({k:v for k,v in summary.items() if k!="trials"},indent=2),flush=True)
    return summary


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--output",default="results/phase13_eskf_development")
    args=p.parse_args();evaluate(args.output)
