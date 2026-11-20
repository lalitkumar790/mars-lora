"""Deterministic raw-LoRa event abstraction for mobility experiments.

Not a validated RF simulator. Receiver SF synchronization is assumed ideal.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import random

SENSITIVITY_DBM = {7: -123, 8: -126, 9: -129, 10: -132, 11: -134.5, 12: -137}

def airtime_s(sf: int, payload_bytes: int = 16, bandwidth: int = 125000,
              coding_rate: int = 1, preamble: int = 8) -> float:
    """Semtech-style explicit-header LoRa airtime; CR=1 means coding rate 4/5."""
    if sf not in SENSITIVITY_DBM or payload_bytes < 0:
        raise ValueError("Unsupported spreading factor or negative payload")
    ts = (2 ** sf) / bandwidth
    de = int(sf >= 11 and bandwidth == 125000)
    numerator = 8 * payload_bytes - 4 * sf + 28 + 16
    denominator = 4 * (sf - 2 * de)
    payload_symbols = 8 + max(math.ceil(numerator / denominator) * (coding_rate + 4), 0)
    return (preamble + 4.25 + payload_symbols) * ts

@dataclass
class Packet:
    created: float
    deadline: float
    critical: bool
    seq: int

@dataclass
class Outcome:
    policy: str
    speed: float
    shadow: float
    seed: int
    generated: int
    received: int
    critical_generated: int
    critical_on_time: int
    critical_received: int
    mean_aoi: float
    p95_aoi: float
    outage_fraction: float
    tx_energy_j: float
    mission_duration_s: float
    total_airtime_s: float

def percentile(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    s = sorted(values)
    i = (len(s)-1)*p
    lo = math.floor(i)
    return s[lo] * (1-(i-lo)) + s[math.ceil(i)] * (i-lo)

def pose(t: float, speed: float) -> tuple[float, float, float]:
    """Repeatable rectangular 3D inspection route; distance in metres."""
    d = (t * speed) % 900
    if d < 250:
        return 40 + d, 20, 35
    if d < 450:
        return 290, 20 + d - 250, 35
    if d < 700:
        return 290 - (d - 450), 220, 35
    return 40, 220 - (d - 700), 35

def link_rssi_dbm(position: tuple[float,float,float], shadow_offset: float,
                  rng: random.Random, previous_shadow: float,
                  correlation: float = 0.96) -> tuple[float,float]:
    x, y, z = position
    distance = max(1, math.sqrt(x*x + y*y + z*z))
    # 868 MHz free-space path loss at 1m is about 31.2 dB; exponent 4 represents severe urban attenuation.
    path_loss = 31.2 + 10 * 4.0 * math.log10(distance)
    # Synthetic obstruction rectangle along inspection corridor.
    obstruction = 17.0 if (x > 190 and 40 < y < 230) else 0.0
    shadow = correlation * previous_shadow + math.sqrt(1-correlation**2) * rng.gauss(0, shadow_offset)
    return 14 - path_loss - obstruction + shadow, shadow

def select_sf(policy: str, estimated_rssi: float, previous_sf: int) -> int:
    if policy == "fixed":
        return 9
    # 6 dB margin; restrict transitions to one SF per decision.
    target = next((sf for sf in range(7,13)
                   if estimated_rssi >= SENSITIVITY_DBM[sf] + 6), 12)
    if policy in ("reactive", "priority"):
        return target
    return min(previous_sf + 1, max(previous_sf - 1, target))

def run_trial(policy: str, speed: float, shadow: float, seed: int,
              duration: float = 360.0, dt: float = 0.5) -> Outcome:
    """Policy comparison with matched per-seed fading draws.

    Single-server queue with airtime blocking; success at end-of-airtime.
    Alerts at deterministic timestamps. 10% random external-channel occupancy
    is a synthetic contention approximation, *not* a MAC collision model.
    """
    if policy not in {"fixed", "reactive", "priority", "mars"}:
        raise ValueError("Unknown policy")
    if speed <= 0 or shadow < 0 or duration <= 0 or dt <= 0:
        raise ValueError("Invalid scenario parameter")
    rng = random.Random(seed)
    sf = 9
    last_rssi = -110.0
    correlated = 0.0
    queue: list[Packet] = []
    generated = received = crit_gen = crit_ok = crit_received = 0
    busy_until = 0.0
    tx_energy = airtime_total = 0.0
    latest_created: float | None = None
    aois: list[float] = []
    outages = 0
    seq = 0
    next_regular = 0.0
    next_critical = 45.0
    last_received_at = 0.0
    actual_speed = speed * (0.8 if policy == "mars" else 1.0)
    mission_duration = duration * speed / actual_speed
    steps = math.ceil(mission_duration / dt)
    for k in range(steps):
        t = k * dt
        position = pose(t, actual_speed)
        rssi, correlated = link_rssi_dbm(position, shadow, rng, correlated)
        # Receiver update based on last ACK-observed strength; ideal SF sync.
        if k % max(1, round(5/dt)) == 0:
            sf = select_sf(policy, last_rssi, sf)
        interval = 2 if policy in {"fixed","reactive"} else (3 if policy == "priority" else 4)
        if t + 1e-9 >= next_regular:
            queue.append(Packet(t, t+8, False, seq))
            generated += 1
            seq += 1
            next_regular += interval
        if t + 1e-9 >= next_critical:
            queue.append(Packet(t, t+3, True, seq))
            generated += 1
            crit_gen += 1
            seq += 1
            next_critical += 45
        if policy in {"priority","mars"}:
            queue.sort(key=lambda p:(not p.critical, p.deadline, p.seq))
        # Count freshness continuously, including outages.
        if latest_created is not None:
            aois.append(t - latest_created)
        else:
            aois.append(t)
        if t - last_received_at > 15:
            outages += 1
        if t + 1e-9 < busy_until or not queue:
            continue
        packet = queue.pop(0)
        # Drop stale packet prior to airtime allocation.
        if packet.deadline < t:
            continue
        toa = airtime_s(sf, 12 if packet.critical else 20)
        busy_until = t + toa
        tx_energy += 0.12 * toa  # assumed 120 mW TX power; proxy only
        airtime_total += toa
        # Sigmoid approximation of receiver threshold transition.
        sensitivity_margin = rssi - SENSITIVITY_DBM[sf]
        reception_prob = 1 / (1 + math.exp(-max(-60,min(60,sensitivity_margin))/1.8))
        # Explicitly synthetic co-channel blocking probability.
        reception_prob *= 0.90
        if rng.random() < reception_prob:
            arrival = t + toa
            received += 1
            latest_created = packet.created
            last_received_at = arrival
            last_rssi = rssi
            if packet.critical:
                crit_received += 1
                if arrival <= packet.deadline:
                    crit_ok += 1
        # ACK SNR is not available after a failed transmission; previous estimate remains.
    return Outcome(policy, speed, shadow, seed, generated, received, crit_gen,
                   crit_ok, crit_received, sum(aois)/len(aois), percentile(aois, 0.95),
                   outages/steps, tx_energy, mission_duration, airtime_total)
