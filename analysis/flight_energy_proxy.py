"""Sensitivity analysis: physical flight-time energy can dwarf LoRa TX energy.

This is NOT electrical power measured from MuJoCo or actual rotors.
Momentum-theory *hover* induced-power baseline:
  P_induced = (m*g)^(3/2) / sqrt(2*rho*A_total)
  P_electrical_proxy = P_induced/eta
Ignoring translational flight, climb, rotor profile power, electronics,
aerodynamic interactions, wind and controller transients.

References:
 - Numerical and Experimental Estimation of the Efficiency of a
   Quadcopter Rotor Operating at Hover, Energies 2019, 12(2):261.
   https://doi.org/10.3390/en12020261
"""
from __future__ import annotations
import csv,json,math
from pathlib import Path

RHO=1.225
G=9.81
ASSUMPTIONS={
    # Rotor radii are illustrative and not vendor-verified.
    "heavy":{"mass_kg":1.325,"radius_m":.11},
    "crazyflie":{"mass_kg":.027,"radius_m":.023}
}
def hover_proxy(mass_kg,radius_m,eta=.55):
    if mass_kg<=0 or radius_m<=0 or not 0<eta<=1:raise ValueError("Invalid assumptions")
    area=4*math.pi*radius_m**2
    return (mass_kg*G)**1.5/math.sqrt(2*RHO*area)/eta

def evaluate(flights="results/rf_navigation/flights.csv",
             summary="results/rf_navigation/summary.csv",
             output="results/flight_energy_proxy"):
    with open(flights,newline="") as f:flights_rows=list(csv.DictReader(f))
    with open(summary,newline="") as f:radio_rows=list(csv.DictReader(f))
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    results=[]
    for airframe in ASSUMPTIONS:
        record=ASSUMPTIONS[airframe]
        plans={r["route"]:r for r in flights_rows if r["airframe"]==airframe}
        if set(plans)!={"shortest","rf_aware"}:raise ValueError("Missing flight baseline")
        heavy=hover_proxy(record["mass_kg"],record["radius_m"])
        delta_t=float(plans["rf_aware"]["elapsed_s"])-float(plans["shortest"]["elapsed_s"])
        for sigma in (4.,8.):
            shortest=next(x for x in radio_rows if x["airframe"]==airframe and
                float(x["shadow_sigma_db"])==sigma and x["condition"]=="S1_shortest_adaptive")
            rf=next(x for x in radio_rows if x["airframe"]==airframe and
                float(x["shadow_sigma_db"])==sigma and x["condition"]=="R2_RF_route_adaptive")
            radio_diff=float(rf["tx_energy_j_mean"])-float(shortest["tx_energy_j_mean"])
            row=dict(airframe=airframe,shadow_sigma_db=sigma,
                     mass_kg=record["mass_kg"],rotor_radius_m=record["radius_m"],
                     assumed_hover_efficiency=.55,
                     ideal_induced_hover_w=hover_proxy(record["mass_kg"],record["radius_m"],eta=1.),
                     electrical_hover_proxy_w=heavy,extra_flight_s=delta_t,
                     propulsion_proxy_delta_j=heavy*delta_t,radio_tx_proxy_delta_j=radio_diff,
                     summed_proxy_delta_j=heavy*delta_t+radio_diff,
                     radio_only_claim_reverses=(radio_diff<0 and heavy*delta_t+radio_diff>0))
            results.append(row)
    with (out/"energy_sensitivity.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(results[0]))
        w.writeheader();w.writerows(results)
    (out/"manifest.json").write_text(json.dumps({
        "status":"Illustrative momentum-theory hover proxy; not propulsion measurement",
        "assumptions":{"rho_kg_m3":RHO,"gravity_m_s2":G,
                       "number_rotors":4,"vehicles":ASSUMPTIONS,
                       "assumed_efficiency":.55},
        "limitations":[
           "Rotor radii assumed; not OEM CAD-certified diameters",
           "Constant hover induced-power baseline, not integrated motor physics",
           "Does not include profile/parasite drag, climbs, descents, motor losses and payload",
           "Flight duration represents native MuJoCo motion at fixed wind 0.25",
           "LoRa TX proxy excludes MCU/RX and is not hardware measured",
           "Summing proxies is not a calibrated physical energy result"
        ],
        "reference":"https://doi.org/10.3390/en12020261"
    },indent=2)+"\n")
    for record in results:
        print(record)
    return results
if __name__=="__main__":evaluate()
