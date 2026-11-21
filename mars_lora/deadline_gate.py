"""Risk-aware ex-ante route & spreading-factor selector (EXPERIMENTAL).

Uses ONLY a preflight radio pilot, an estimated busy fraction, fixed known 3D
geometry and deterministic radio assumptions, not future packet observations.
This is a model-based heuristic, not a calibrated predictive controller.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import dist,exp,isfinite,log10
from mars_lora.rf_astar import GCS,START,SOLIDS,obstructed
from mars_lora.channel import SENSITIVITY_DBM,airtime_s

@dataclass(frozen=True)
class Candidate:
    route: str
    spreading_factor: int
    estimated_alert_success: float
    estimated_missed_alert_fraction: float
    expected_airtime_s: float
    estimated_flight_s: float
    score: float
    estimated_propulsion_energy_j: float=0.

def _expected_rssi(point,receiver,solids=SOLIDS):
    d=max(1.,dist(point,receiver))
    return 14.-31.2-40.*log10(d)-(17. if obstructed(point,receiver,solids) else 0.)

def _samples(points,spacing=1.5):
    from math import ceil
    locations=[]
    for a,b in zip(points,points[1:]):
        n=max(1,ceil(dist(a,b)/spacing))
        for i in range(n):
            t=(i+.5)/n
            locations.append(tuple(a[j]*(1-t)+b[j]*t for j in range(3)))
    return locations

def predict_candidate(name,points,pilot_rssi_dbm,busy_fraction,receiver=GCS,
                      flight_speed_mps=1.2,deadline_s=3.,
                      missed_alert_weight=100.,time_weight=.15,
                      airtime_budget_fraction=None,solids=SOLIDS,
                      flight_hover_power_w=0.,propulsion_energy_weight=0.):
    if not points or len(points)<2:raise ValueError("Need existing planned route")
    if not isfinite(pilot_rssi_dbm) or not 0<=busy_fraction<=1:
        raise ValueError("Invalid radio pilot")
    if flight_speed_mps<=0 or deadline_s<=0 or flight_hover_power_w<0 or propulsion_energy_weight<0:
        raise ValueError("Invalid flight/energy parameters")
    # Pilot-correct the *known* path-loss model. The correction is capped:
    # single-site RSSI does not identify spatial shadow distribution.
    bias=max(-20.,min(20.,pilot_rssi_dbm-_expected_rssi(points[0],receiver,solids)))
    length=sum(dist(a,b) for a,b in zip(points,points[1:]))
    locations=_samples(points)
    if not locations:raise ValueError("Empty route")
    estimates=[_expected_rssi(p,receiver,solids)+bias for p in locations]
    flight_time=length/flight_speed_mps
    estimated_propulsion_energy_j=flight_hover_power_w*flight_time
    best=None
    # The same SF is used for the entire route in this *offline* gate.
    # This is NOT the adaptive-SF replay policy used to score the outcome.
    for sf in sorted(SENSITIVITY_DBM):
        toa=airtime_s(sf,12)
        if toa>deadline_s:continue
        if airtime_budget_fraction is not None:
            if not 0<airtime_budget_fraction<=1:raise ValueError('Invalid deadline airtime budget')
            # Reserve time for queue delay, channel occupancy and possible
            # asynchronous receiver coordination, not just PHY airtime.
            if toa>deadline_s*airtime_budget_fraction*(1-busy_fraction):continue
        probability=sum(
           (1.-busy_fraction)/
           (1.+exp(-max(-55.,min(55.,rssi-SENSITIVITY_DBM[sf]))/1.8))
           for rssi in estimates)/len(estimates)
        # No inferred retry opportunities: one attempt per alert.
        if airtime_budget_fraction is None:
            timely_proxy=probability
            airtime_penalty=.12*toa*10.
        else:
            # Conservative queue-risk proxy: even a received packet is
            # not useful to the critical control loop if it arrives late.
            remaining=max(0.,1.-toa/(deadline_s*max(.05,1-busy_fraction)))
            timely_proxy=probability*remaining
            airtime_penalty=10.*toa
        score=(missed_alert_weight*(1-timely_proxy) + time_weight*flight_time
               + airtime_penalty
               + propulsion_energy_weight*estimated_propulsion_energy_j)
        candidate=Candidate(name,sf,timely_proxy,1-timely_proxy,toa,flight_time,score,
                            estimated_propulsion_energy_j)
        if best is None or candidate.score<best.score:best=candidate
    if best is None:raise RuntimeError("No SF meets the alert airtime limit")
    return best

def choose_deadline_route(routes,pilot_rssi_dbm,busy_fraction,receiver=GCS,**kwargs):
    """Evaluate pre-computed route alternatives and pick lowest risk/cost.

    Both routes must have exactly the same start/end; future radio outcomes
    may NOT be passed in the arguments.
    """
    if set(routes)!={"shortest","rf_aware"}:
        raise ValueError("Expected both known static routes")
    a,b=(routes[k] for k in ("shortest","rf_aware"))
    if tuple(a[0])!=tuple(b[0]) or tuple(a[-1])!=tuple(b[-1]):
        raise ValueError("Cannot compare different start/goal missions")
    candidates=[predict_candidate(name,routes[name],pilot_rssi_dbm,
                                  busy_fraction,receiver,**kwargs)
                for name in ("shortest","rf_aware")]
    return min(candidates,key=lambda c:(c.score,c.estimated_flight_s)),candidates
