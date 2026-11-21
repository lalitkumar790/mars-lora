"""Fixed, previously untested native-city evaluation of safety/deadline/energy routing.

This is an out-of-sample *city layout* evaluation of a frozen, exploratory,
model-based policy, not hardware or GPS-denied autonomy validation.

Plan (map known, sigma=0.10m) -> execute full MuJoCo dynamics once per route
and airframe -> require successful flight with no collisions and clearance
>=0.25 m -> compare selected route/fixed-SF against shortest+adaptive and
RF-route+adaptive with identical location-fixed fading seed per paired case.

All city seeds, profiles and policy weights are declared here BEFORE looking
at this workflow's results. Never silently drop a failed city/flight.
"""
from __future__ import annotations
import csv,json,math
from pathlib import Path
from statistics import mean,stdev
from mars_lora.rf_astar import INDUSTRIAL,GCS,plan
from mars_lora.urban_scene import city
from mars_lora.deadline_gate import choose_deadline_route
from mars_lora.route_gate import simulated_pilot
from experiments.replay_mujoco_radio import trial,load_trajectory
from analysis.flight_energy_proxy import ASSUMPTIONS,hover_proxy

# 37021 is the original training/visualization city; never use it here.
CITY_SEEDS=(94103,58241)
RADIO_SEEDS=tuple(range(701,709))
SIGMA_M=.10
WIND_SCALE=.25
HOLDOUT_PROFILES={
    "shifted_relay":{
        "name":"shifted_relay","station":(210.,185.,85.),
        "path_loss_exponent":3.6,"obstruction_db":13.,
        "rx_bias_db":-1.5,"interference_fraction":.12
    },
    "high_shadow":{
        "name":"high_shadow","station":(160.,240.,105.),
        "path_loss_exponent":4.2,"obstruction_db":21.,
        "rx_bias_db":-3.,"interference_fraction":.24
    },
}
CONTROLLERS=("legacy",)
POLICIES=("shortest_adaptive","rf_adaptive","joint_guard")
METRICS=("pdr","critical_deadline_ratio","aoi_p95_s",
         "outage_fraction","tx_energy_j","mission_duration_s")
# Values fixed *before* seeing new layouts. These are assumed watt budgets.
ENERGY_WEIGHT=.003
BUSY_GUARD=.20
AIRTIME_BUDGET=.20

def write_csv(path,rows,keys):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)

def evaluate(output="results/unseen_city",city_seeds=CITY_SEEDS):
    # Import native MuJoCo only when executing the physical experiment;
    # pure planner/unit tests must run without its optional dependency.
    from mujoco_sim.validate_mission import run
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    manifest={
        "city_seeds":list(city_seeds),
        "radio_seeds":list(RADIO_SEEDS),
        "city_training_seed_excluded":37021,
        "localization_sigma_m_supplied_not_estimated":SIGMA_M,
        "planning_clearance_margin_m":.55+3*SIGMA_M,
        "physics_wind_force_scale":WIND_SCALE,
        "controller":"legacy",
        "profiles":HOLDOUT_PROFILES,
        "policy_hyperparameters":{
            "airtime_budget_fraction":AIRTIME_BUDGET,
            "channel_busy_guard":BUSY_GUARD,
            "propulsion_energy_weight_score_per_j":ENERGY_WEIGHT,
            "flight_speed_assumption_mps":1.2
        },
        "limitations":[
            "native MuJoCo control uses true state, no GPS-denied SLAM",
            "map-known planner receives perfect static city geometry",
            "0.10m sigma is assumed, not obtained from an estimator",
            "LoRa obstruction and pilot are simulated using the same broad model class",
            "only two frozen unseen city layouts, two synthetic radio profiles",
            "flight-energy cost is an uncalibrated hover-power proxy",
            "policy thresholds developed on prior five radio scenarios",
            "packet replay reuses each executed route and does not close online radio/flight loop"
        ]
    }
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    flights=[];trials=[];scenes=[]
    for city_seed in city_seeds:
        solids=INDUSTRIAL+tuple(city(city_seed))
        record={"city_seed":city_seed,"buildings":len(solids)-len(INDUSTRIAL)}
        try:
            alternatives={
                name:plan(name,solids=solids,localization_sigma_m=SIGMA_M)
                for name in ("shortest","rf_aware")
            }
            record.update(planner_success=True,
                          lengths_m={name:route.geometric_length_m for name,route in alternatives.items()},
                          clearances_m={name:route.minimum_clearance_m for name,route in alternatives.items()})
        except (ValueError,RuntimeError,AssertionError) as err:
            record.update(planner_success=False,planning_failure=str(err))
            scenes.append(record);print("PLANNER FAIL",record,flush=True);continue
        scenes.append(record)
        path_points={name:route.points for name,route in alternatives.items()}
        for vehicle in ("heavy","crazyflie"):
            valid_routes={}
            radius=.08 if vehicle=="crazyflie" else .32
            for name in ("shortest","rf_aware"):
                outtrace=out/f"flight_city{city_seed}_{vehicle}_{name}.csv"
                outcome=run(vehicle,duration=340,wind=WIND_SCALE,
                            wind_phase=(city_seed%17)*.13,
                            controller_mode="legacy",city_seed=city_seed,
                            route=path_points[name],out=outtrace,require_completion=False)
                status=bool(outcome["completed"] and outcome["contact_samples"]==0
                            and outcome["min_clearance_m"]>=.25)
                outcome.update(route=name,city_seed=city_seed,
                               airframe=vehicle,safety_pass=status)
                flights.append(outcome)
                print("FLIGHT",city_seed,vehicle,name,"safe",status,
                      "clearance",outcome["min_clearance_m"],
                      "contacts",outcome["contact_samples"],flush=True)
                if status:valid_routes[name]=load_trajectory(outtrace)
            if len(valid_routes)!=2:
                print("SKIP RADIO: one or both actual physics routes failed",city_seed,vehicle,flush=True)
                continue
            rotor=ASSUMPTIONS[vehicle]
            hover_w=hover_proxy(rotor["mass_kg"],rotor["radius_m"])
            for profile_name,config in HOLDOUT_PROFILES.items():
                station=tuple(config["station"])
                for radio_seed in RADIO_SEEDS:
                    pilot=simulated_pilot(config,station,radio_seed,scene=solids)
                    choice,candidates=choose_deadline_route(
                        path_points,pilot["pilot_rssi_dbm"],pilot["observed_channel_busy"],
                        receiver=station,solids=solids,
                        airtime_budget_fraction=AIRTIME_BUDGET,
                        flight_hover_power_w=hover_w,
                        propulsion_energy_weight=ENERGY_WEIGHT)
                    guard=pilot["observed_channel_busy"]>=BUSY_GUARD
                    selected="shortest" if guard else choice.route
                    sf=None if guard else choice.spreading_factor
                    # A successful plan/flight is a hard gate. These replay
                    # comparisons NEVER claim success for unsafe flights.
                    for policy in POLICIES:
                        route=({"shortest_adaptive":"shortest","rf_adaptive":"rf_aware",
                                "joint_guard":selected})[policy]
                        override=sf if policy=="joint_guard" else None
                        result=trial(valid_routes[route],"priority-adaptive",radio_seed,
                                     station,shadow_sigma=5.,step=.25,
                                     profile=config,spatial_fading=True,
                                     sf_override=override,scene=solids)
                        result.update(city_seed=city_seed,airframe=vehicle,
                                      profile_name=profile_name,method=policy,route=route,
                                      selected_sf=("adaptive" if override is None else override),
                                      guarded_fallback=bool(guard and policy=="joint_guard"),
                                      observed_pilot_rssi_dbm=pilot["pilot_rssi_dbm"],
                                      observed_busy=pilot["observed_channel_busy"],
                                      model_predicted_alert_probability=choice.estimated_alert_success)
                        result["full_mission_energy_proxy_j"]=(
                            result["tx_energy_j"]+hover_w*result["mission_duration_s"])
                        trials.append(result)
        print("SCENE DONE",city_seed,flush=True)
    (out/"scenes.json").write_text(json.dumps(scenes,indent=2)+"\n")
    if flights:
        write_csv(out/"flights.csv",flights,list(flights[0]))
    if trials:
        write_csv(out/"trials.csv",trials,list(trials[0]))
    summary=[]
    for city_seed in city_seeds:
        for airframe in ("heavy","crazyflie"):
            for profile in HOLDOUT_PROFILES:
                subset=[r for r in trials if r["city_seed"]==city_seed and
                        r["airframe"]==airframe and r["profile_name"]==profile]
                if not subset:continue
                groups={}
                for r in subset:groups.setdefault(int(r["seed"]),{})[r["method"]]=r
                if any(set(x)!=set(POLICIES) for x in groups.values()):
                    raise RuntimeError("Missing matched seed policy triplet")
                row=dict(city_seed=city_seed,airframe=airframe,
                         profile=profile,matched_n=len(groups))
                for policy in POLICIES:
                    for metric in (*METRICS,"full_mission_energy_proxy_j"):
                        row[f"{policy}_{metric}"]=mean(float(p[policy][metric]) for p in groups.values())
                for metric in ("critical_deadline_ratio","full_mission_energy_proxy_j"):
                    changes=[float(p["joint_guard"][metric])-float(p["shortest_adaptive"][metric])
                             for p in groups.values()]
                    row[f"joint_minus_shortest_{metric}"]=mean(changes)
                    row[f"joint_minus_shortest_{metric}_ci95_halfwidth"]=(
                        1.96*stdev(changes)/math.sqrt(len(changes))
                        if len(changes)>1 else float("nan"))
                summary.append(row)
    if summary:write_csv(out/"paired_summary.csv",summary,list(summary[0]))
    (out/"summary.json").write_text(json.dumps({
       "cities_requested":len(city_seeds),
       "cities_planned":sum(x["planner_success"] for x in scenes),
       "flight_cases":len(flights),
       "safe_flights":sum(x["safety_pass"] for x in flights),
       "radio_trials":len(trials),
       "paired_groups":len(summary),
       "scientific_note":"No unsafe or incomplete MuJoCo route can contribute radio scores; failures preserved separately."
    },indent=2)+"\n")
    print((out/"summary.json").read_text(),flush=True)
    return summary

if __name__=="__main__":
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",default="results/unseen_city")
    parser.add_argument("--city-seeds",type=int,nargs="+",default=list(CITY_SEEDS))
    args=parser.parse_args()
    if len(args.city_seeds)!=len(set(args.city_seeds)) or 37021 in args.city_seeds:
        parser.error("Holdout city seeds must be distinct and exclude training seed 37021")
    evaluate(output=args.output,city_seeds=tuple(args.city_seeds))
