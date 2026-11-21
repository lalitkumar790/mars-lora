"""Exploratory preflight LoRa route-detour gate.

Decision inputs ONLY: an observable RSSI pilot and sampled channel busy rate.
Not future packet outcomes, current route PDR, hidden fading or scenario labels.
This gate is a candidate mechanism, not a validated radio estimator.
"""
from __future__ import annotations
import math,random
from mars_lora.channel import SENSITIVITY_DBM
from experiments.replay_mujoco_radio import obstructed,spatial_shadow_field

PILOT_POSITION=(0.,0.,4.)
def choose_route(pilot_rssi_dbm:float,busy_fraction:float,
                 margin_db:float=10.,max_busy:float=.25)->str:
    """Choose shortest if signal is adequate or congestion dominates.

    The first part avoids spending flight energy to improve an already good
    SF9 link; the second avoids detours when congestion, not propagation,
    is the dominant issue. This is an *experimental* policy heuristic.
    """
    if not math.isfinite(pilot_rssi_dbm) or not 0<=busy_fraction<=1:
        raise ValueError("Invalid observable pilot metrics")
    if busy_fraction>=max_busy or pilot_rssi_dbm>=SENSITIVITY_DBM[9]+margin_db:
        return "shortest"
    return "rf_aware"

def simulated_pilot(profile:dict,station:tuple,seed:int,position=PILOT_POSITION,
                    shadow_sigma_db=5.,listening_samples=32,scene=None):
    """Synthetic preflight beacons and CCA observations, NOT hidden true channel.

    The policy gets only the two returned noisy estimates. The simulator
    uses profile parameters to generate those readings, as a test harness.
    """
    d=max(1.,math.dist(position,station))
    blocked=(obstructed(position,station) if scene is None
             else obstructed(position,station,scene))
    los_loss=(float(profile.get("obstruction_db",17.))+
              float(profile.get("extra_nlos_loss_db",0.))) if blocked else 0.
    power=(14-31.2-10*float(profile.get("path_loss_exponent",4.))*math.log10(d)
           -los_loss+float(profile.get("rx_bias_db",0.))
           +spatial_shadow_field(position,seed,shadow_sigma_db))
    noise=random.Random(seed*617+104729)
    noisy_rssi=power+noise.gauss(0.,1.2)
    occupancy=float(profile.get("interference_fraction",.1))
    busy=sum(noise.random()<occupancy for _ in range(listening_samples))/listening_samples
    return {"pilot_rssi_dbm":noisy_rssi,"observed_channel_busy":busy}
