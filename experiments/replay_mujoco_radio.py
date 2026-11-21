"""Time-aligned MuJoCo pose replay into a synthetic raw-LoRa simulator.

This is the FIRST integrated physics -> radio experiment, not hardware LoRa.
All radio policies share the same validated MuJoCo trajectory, traffic creation
schedule, time-indexed fading and RX random draws per seed/scenario.
"""
from __future__ import annotations
import argparse,csv,json,math,random
from bisect import bisect_left
from collections import deque
from pathlib import Path
from statistics import mean,stdev
from functools import lru_cache
from mars_lora.channel import airtime_s,SENSITIVITY_DBM,percentile
from mars_lora.urban_scene import city

SCENE=[
 (5,4,1.4,1.1,1.4,1.4),(-5,5,1.9,1.25,1.1,1.9),
 (5,-4,1.6,1.4,1,1.6),(-3,-4,.6,2,.7,.6),
 (1,8,2.5,.24,.24,2.5),(-8,0,2.7,.32,.32,2.7),
 (-5,0,5.45,3.3,.24,.23)
]+city()

def intersect_segment(a,b,box):
    cx,cy,cz,hx,hy,hz=box
    lo=0.;hi=1.
    for s,e,c,h in zip(a,b,(cx,cy,cz),(hx,hy,hz)):
        diff=e-s
        if abs(diff)<1e-12:
            if abs(s-c)>h:return False
        else:
            t0=(c-h-s)/diff;t1=(c+h-s)/diff
            if t0>t1:t0,t1=t1,t0
            lo=max(lo,t0);hi=min(hi,t1)
            if hi<lo:return False
    return hi>.02 and lo<.98

def obstructed(uav,gcs):
    return any(intersect_segment(uav,gcs,b) for b in SCENE)

def load_trajectory(path):
    with open(path,newline='') as f:
        data=[r for r in csv.DictReader(f)]
    records=[(float(r['t_s']),float(r['x_m']),float(r['y_m']),float(r['z_m'])) for r in data]
    if len(records)<20:raise ValueError("Need actual MuJoCo flight CSV, not a guessed mission.")
    return records

def at_time(trace,t):
    j=bisect_left([r[0] for r in trace],t)
    if j<=0:return trace[0][1:]
    if j>=len(trace):return trace[-1][1:]
    a,b=trace[j-1],trace[j]
    mix=max(0.,min(1.,(t-a[0])/max(1e-9,b[0]-a[0])))
    return tuple(a[k]*(1-mix)+b[k]*mix for k in (1,2,3))

@lru_cache(maxsize=120000)
def _spatial_corner(seed,ix,iy,iz):
    code=(seed*0x9e3779b1+ix*0x85ebca6b+
          iy*0xc2b2ae35+iz*0x27d4eb2f)&0xffffffff
    return random.Random(code).gauss(0.,1.)


def spatial_shadow_field(position, seed, sigma, correlation_m=14.):
    """Fixed-seed C1-continuous 3-D shadow field, common across both routes.

    Location-based fades replace non-comparable time-indexed fades for
    route-comparison stress tests. Not calibrated measured RF propagation.
    """
    if sigma==0:return 0.
    x,y,z=(v/correlation_m for v in position)
    ix,iy,iz=(math.floor(v) for v in (x,y,z))
    fx,fy,fz=(v-math.floor(v) for v in (x,y,z))
    def smooth(t):return t*t*(3-2*t)
    fx,fy,fz=map(smooth,(fx,fy,fz))
    total=0.
    for a in (0,1):
        for b in (0,1):
            for c in (0,1):
                # Deterministic cell-specific independent normals.
                weight=(fx if a else 1-fx)*(fy if b else 1-fy)*(fz if c else 1-fz)
                total+=weight*_spatial_corner(seed,ix+a,iy+b,iz+c)
    # Finite grid interpolation smooths the field. sigma is a nominal
    # corner SD; interpolated point SD varies across a cell.
    return sigma*total


def trial(trace,policy,seed,gcs,shadow_sigma=4.,step=.25,
          profile=None,spatial_fading=False,sf_override=None):

    horizon=trace[-1][0]
    N=math.ceil(horizon/step)
    settings=profile or {}
    exponent=float(settings.get('path_loss_exponent',4.0))
    obstruction_db=float(settings.get('obstruction_db',17.0))
    rx_bias=float(settings.get('rx_bias_db',0.0))
    extra_nlos=float(settings.get('extra_nlos_loss_db',0.0))
    interference=float(settings.get('interference_fraction',.1))
    if exponent<=0 or obstruction_db<0 or not 0<=interference<1:
        raise ValueError('Invalid propagation stress profile')
    if sf_override is not None and sf_override not in SENSITIVITY_DBM:
        raise ValueError('SF override must be between 7 and 12')
    rng=random.Random(seed)
    fades=[];fade=0
    for _ in range(N):
        fade=.96*fade+math.sqrt(1-.96**2)*rng.gauss(0,shadow_sigma)
        fades.append(fade)
    draws=[rng.random() for _ in range(N)]
    queue=deque()
    generated=delivered=alert_n=alert_ok=expired=0
    busy_until=0.;sf=(sf_override if sf_override is not None else 9);next_alert=25.;last_ack=-float('inf')
    last_sample=0.;aoi=[];outage=[];energy=airtime=0
    nlos=0;changes=0;last_rssi=None;pending=[]
    trace_times=[row[0] for row in trace]
    for k in range(N):
        t=k*step
        j=bisect_left(trace_times,t)
        if j<=0:p=trace[0][1:]
        elif j>=len(trace):p=trace[-1][1:]
        else:
            a,b=trace[j-1],trace[j]
            w=(t-a[0])/max(1e-9,b[0]-a[0])
            p=tuple(a[i]*(1-w)+b[i]*w for i in (1,2,3))
        blocked=obstructed(p,gcs)
        nlos+=int(blocked)
        d=max(1.,math.dist(p,gcs))
        shadow=spatial_shadow_field(p,seed,shadow_sigma) if spatial_fading else fades[k]
        rssi=14-31.2-10*exponent*math.log10(d)-((obstruction_db+extra_nlos) if blocked else 0)+rx_bias+shadow
        # ACKs and telemetry are delivered only after LoRa time-on-air
        # completes, never at the beginning of transmission.
        arrived=[entry for entry in pending if entry[0]<=t+1e-9]
        pending=[entry for entry in pending if entry[0]>t+1e-9]
        for arrival,critical,made,deadline,observed_rssi in arrived:
            delivered+=1;last_ack=arrival;last_rssi=observed_rssi
            if not critical:last_sample=max(last_sample,made)
            if critical and arrival<=deadline:alert_ok+=1
        if k%8==0: # 2-second telemetry, fixed across policies
            queue.append((t,t+8,False,20));generated+=1
        if t+.001>=next_alert:
            queue.append((t,t+3,True,12));generated+=1;alert_n+=1;next_alert+=25
        aoi.append(max(0,t-last_sample))
        outage.append(int(t-last_ack>15))
        if sf_override is None and policy in ('adaptive','priority-adaptive') and k%20==0 and last_rssi is not None:
            sfnew=next((s for s in range(7,13) if last_rssi>=SENSITIVITY_DBM[s]+8),12)
            sfnew=max(sf-1,min(sf+1,sfnew))
            if sfnew!=sf:
                changes+=1;sf=sfnew;busy_until=max(busy_until,t+.5) # handshake cost
        if t<busy_until or not queue:continue
        if policy=='priority-adaptive':
            q=list(queue)
            q.sort(key=lambda item:(not item[2],item[0]))
            queue=deque(q)
        made,deadline,critical,payload=queue.popleft()
        if t>deadline:expired+=1;continue
        toa=airtime_s(sf,payload)
        airtime+=toa;energy+=.12*toa
        busy_until=t+toa
        margin=rssi-SENSITIVITY_DBM[sf]
        probability=(1-interference)/(1+math.exp(-max(-55,min(55,margin))/1.8))
        if draws[k]<probability:
            pending.append((busy_until,critical,made,deadline,rssi))
    # Final arrivals still in flight beyond the mission horizon are NOT
    # credited to on-mission delivery. Avoids overstating deadline success.
    return {
        'propagation_profile':settings.get('name','nominal'),
        'spatial_fading':spatial_fading,'sf_override':sf_override,
        'policy':policy,'seed':seed,'gcs_x':gcs[0],'gcs_y':gcs[1],'gcs_z':gcs[2],
        'shadow_sigma_db':shadow_sigma,'pdr':delivered/max(1,generated),
        'critical_deadline_ratio':alert_ok/max(1,alert_n),
        'aoi_p95_s':percentile(aoi,.95),'outage_fraction':mean(outage),
        'nlos_fraction':nlos/max(1,N),
        'tx_energy_j':energy,'airtime_s':airtime,'expired':expired,
        'sf_switches':changes,'mission_duration_s':horizon,
        'packets_generated':generated,'alerts_generated':alert_n
    }

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--flight',default='results/flight-validation')
    p.add_argument('--output',default='results/crosslayer')
    p.add_argument('--seeds',type=int,default=20)
    args=p.parse_args()
    if args.seeds<2:p.error('seeds must be >= 2')
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    rows=[]
    for vehicle in ('heavy','crazyflie'):
        trace=load_trajectory(Path(args.flight)/(vehicle+'.csv'))
        for gcs in ((230.,220.,5.),(230.,220.,120.)):
            for sigma in (4.,8.):
                for seed in range(args.seeds):
                    for policy in ('fixed','adaptive','priority-adaptive'):
                        r=trial(trace,policy,seed,gcs,sigma)
                        r['airframe']=vehicle;rows.append(r)
    with (out/'trials.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    summary=[]
    for vehicle in ('heavy','crazyflie'):
        for gcs_z in (5.,120.):
            for sigma in (4.,8.):
                for policy in ('fixed','adaptive','priority-adaptive'):
                    xs=[r for r in rows if r['airframe']==vehicle and r['gcs_z']==gcs_z
                        and r['shadow_sigma_db']==sigma and r['policy']==policy]
                    result={'airframe':vehicle,'gcs_z':gcs_z,'shadow_sigma_db':sigma,'policy':policy,'n':len(xs)}
                    for metric in ('pdr','critical_deadline_ratio','aoi_p95_s','outage_fraction','nlos_fraction','tx_energy_j','mission_duration_s'):
                        vals=[x[metric] for x in xs]
                        result[metric+'_mean']=mean(vals)
                        result[metric+'_sd']=stdev(vals)
                    summary.append(result)
    with (out/'summary.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
    (out/'manifest.json').write_text(json.dumps({
        'method':'Actual MuJoCo CSV replay -> 3D ray/AABB obstruction -> seeded raw-LoRa events',
        'trial_count':len(rows),'seed_count':args.seeds,
        'gcs_positions_m':[(230,220,5),(230,220,120)],'shadow_sigma_db':[4,8],
        'policies':['fixed','adaptive','priority-adaptive'],
        'limitations':['synthetic path-loss and shadow','3D AABB blockage not measured RF',
          'no physical radio','receiver SF handshakes idealized as 0.5 s',
          'all policies have same flight trajectory: no RF-aware path optimization yet']
    },indent=2)+'\n')
    print('Physics-replayed cross-layer trials:',len(rows))
    for r in summary[:3]:print(r)
if __name__=='__main__':main()
