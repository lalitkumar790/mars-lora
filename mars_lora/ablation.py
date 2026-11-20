"""Controlled 2x2x2 LoRa policy ablation, separate from legacy experiment.

All eight combinations use identical message-generation rates and wall-clock
duration. The radio model is synthetic, not hardware-validated. Reception RNG
and spatial fading are indexed by time, not policy-dependent draw count.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import random
from mars_lora.channel import airtime_s, SENSITIVITY_DBM, pose, percentile

@dataclass(frozen=True)
class Variant:
    adaptive: bool
    priority: bool
    slow: bool

    @property
    def name(self) -> str:
        return f"A{int(self.adaptive)}P{int(self.priority)}S{int(self.slow)}"

@dataclass
class Trial:
    policy: str
    seed: int
    speed: float
    shadow: float
    generated: int
    received: int
    alert_total: int
    alert_timely: int
    late_alerts: int
    pdr: float
    alert_satisfaction: float
    mean_aoi: float
    p95_aoi: float
    outage_fraction: float
    mission_progress_m: float
    tx_airtime_s: float
    tx_energy_j: float
    sf_switches: int

def run_ablation(variant: Variant, seed: int, speed: float = 5.0,
                 shadow: float = 4.0, duration: int = 360) -> Trial:
    if duration <= 0 or speed <= 0 or shadow < 0:
        raise ValueError("Invalid simulation parameters")
    rng = random.Random(seed)
    # Common-random-number trace; every method sees the same time-indexed
    # fading, receiver draw and noise sequence at each nominal observation.
    dt = .5
    steps = int(duration / dt)
    fade = []
    x = 0.
    for _ in range(steps):
        x = .96*x + math.sqrt(1-.96**2)*rng.gauss(0, shadow)
        fade.append(x)
    rx_draw = [rng.random() for _ in range(steps)]
    estimate_noise = [rng.gauss(0, 1.5) for _ in range(steps)]
    speed_actual = speed*(.8 if variant.slow else 1.)
    sf = 9
    switches = 0
    events = []
    sequence = 0
    received = 0
    alert_total = 0
    alert_timely = 0
    late_alerts = 0
    last_update = None
    aoi_samples = []
    outage_steps = 0
    last_rx = 0.
    tx_airtime = 0.
    busy_until = 0.
    feedback = []
    consecutive_failures = 0
    for k in range(steps):
        t = k*dt
        px,py,pz = pose(t, speed_actual)
        distance = max(1., math.sqrt(px*px+py*py+pz*pz))
        obstructed = px > 190 and 40 < py < 230
        rssi = 14 - 31.2 - 40*math.log10(distance) - (17 if obstructed else 0) + fade[k]
        if k % 4 == 0:
            events.append((t, t+8., False, sequence))
            sequence += 1
        if k > 0 and k % 90 == 0:
            events.append((t, t+3., True, sequence))
            alert_total += 1
            sequence += 1
        # Predictive SF uses RSSI information only from delivered ACKs.
        # 5-second decision clock, bounded SF change.
        if variant.adaptive and k % 10 == 0:
            if feedback:
                tlast, last = feedback[-1]
                slope = 0.
                if len(feedback)>1:
                    tprev, prev = feedback[-2]
                    slope = max(-2.,min(2.,(last-prev)/max(.5,tlast-tprev)))
                predicted = last + min(slope,0)*5
                target = next((s for s in range(7,13)
                               if predicted >= SENSITIVITY_DBM[s] + 8),12)
                if consecutive_failures >= 3:
                    target = max(target,min(12,sf+1))
                new_sf = max(sf-1,min(sf+1,target))
                if new_sf != sf:
                    switches += 1
                sf = new_sf
        if last_update is None:
            aoi_samples.append(t)
        else:
            aoi_samples.append(t-last_update)
        if t-last_rx>15:
            outage_steps += 1
        if t < busy_until or not events:
            continue
        if variant.priority:
            # Critical first; preserve FIFO order within class.
            events.sort(key=lambda e:(not e[2],e[3]))
        created,deadline,is_alert,seq = events.pop(0)
        if t > deadline:
            if is_alert:
                late_alerts += 1
            continue
        toa = airtime_s(sf,12 if is_alert else 20)
        tx_airtime += toa
        busy_until = t+toa
        margin = rssi-SENSITIVITY_DBM[sf]
        probability = .9/(1+math.exp(-max(-60,min(60,margin))/1.8))
        if rx_draw[k] < probability:
            arrival = t+toa
            last_rx = arrival
            # Only fresh routine telemetry counts as information freshness.
            if not is_alert:
                last_update = created
            if is_alert:
                if arrival <= deadline:
                    alert_timely += 1
                else:
                    late_alerts += 1
            received += 1
            feedback.append((arrival,rssi+estimate_noise[k]))
            feedback = feedback[-2:]
            consecutive_failures = 0
        else:
            consecutive_failures += 1
    return Trial(variant.name, seed,speed,shadow,sequence,received,
                 alert_total,alert_timely,late_alerts,
                 received/sequence,
                 alert_timely/alert_total if alert_total else float('nan'),
                 sum(aoi_samples)/len(aoi_samples),
                 percentile(aoi_samples,.95),
                 outage_steps/steps,
                 duration*speed_actual,tx_airtime,tx_airtime*.12,switches)
