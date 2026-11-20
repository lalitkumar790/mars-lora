"""Risk-constrained 3D A* routes for the MARS-LoRa urban benchmark.

Assumptions: this planner receives a prior *perfect* static occupancy map.
It is a map-known oracle baseline, NOT GPS-denied perception-based mapping.
The only RF input is the deterministic expected link budget; seeded fading and
packet outcomes are never given to the planner.

We generate two physically distinct route policies between identical endpoints:
(1) collision-safe geometric shortest cost; (2) clearance-constrained
radio-aware cost using the same 3D city occupancy. They are subsequently
executed by MuJoCo, and the real executed physics traces enter packet replay.
"""
from __future__ import annotations
from dataclasses import dataclass
import heapq, math
from mars_lora.urban_scene import city

INDUSTRIAL=(
    (5,4,1.4,1.1,1.4,1.4),(-5,5,1.9,1.25,1.1,1.9),
    (5,-4,1.6,1.4,1,1.6),(-3,-4,.6,2,.7,.6),
    (1,8,2.5,.24,.24,2.5),(-8,0,2.7,.32,.32,2.7),
    (-5,0,5.45,3.3,.24,.23),
)
SOLIDS=INDUSTRIAL+tuple(city())
START=(0.,0.,4.)
GOAL=(-24.,24.,4.)
GCS=(35.,28.,8.)
SPACING=3.
ALTITUDES=(4.,7.,10.,13.,17.,21.,26.)

def signed_clearance(point,solids=SOLIDS):
    x,y,z=point
    closest=float("inf")
    for cx,cy,cz,sx,sy,sz in solids:
        dx=max(abs(x-cx)-sx,0.)
        dy=max(abs(y-cy)-sy,0.)
        dz=max(abs(z-cz)-sz,0.)
        outside=math.sqrt(dx*dx+dy*dy+dz*dz)
        if not(dx or dy or dz):
            outside=-min(sx-abs(x-cx),sy-abs(y-cy),sz-abs(z-cz))
        closest=min(closest,outside)
    return closest

def line_intersects(a,b,box):
    x,y,z,sx,sy,sz=box
    lo,hi=0.,1.
    for p,q,c,h in zip(a,b,(x,y,z),(sx,sy,sz)):
        d=q-p
        if abs(d)<1.e-12:
            if abs(p-c)>h:return False
        else:
            t0,t1=(c-h-p)/d,(c+h-p)/d
            if t0>t1:t0,t1=t1,t0
            lo=max(lo,t0);hi=min(hi,t1)
            if lo>hi:return False
    return lo<.999 and hi>.001

def obstructed(point,gcs=GCS):
    return any(line_intersects(point,gcs,box) for box in SOLIDS)

def path_clearance(points,radius=.32,margin=.55):
    lowest=float("inf")
    for a,b in zip(points,points[1:]):
        length=math.dist(a,b)
        for k in range(max(2,math.ceil(length/.25)+1)):
            t=k/(max(2,math.ceil(length/.25)+1)-1)
            p=tuple(a[j]*(1-t)+b[j]*t for j in range(3))
            lowest=min(lowest,signed_clearance(p)-radius)
    return lowest

def predict_success(p,gcs=GCS,sf=9):
    """Uncalibrated SF9 expected-reception heuristic, *not* an empirical radio model."""
    from mars_lora.channel import SENSITIVITY_DBM
    d=max(1.,math.dist(p,gcs))
    rssi=14-31.2-40*math.log10(d)-(17 if obstructed(p,gcs) else 0)
    margin=rssi-SENSITIVITY_DBM[sf]
    return .9/(1+math.exp(-max(-45.,min(45.,margin))/1.8))

def idx_to_pos(idx):
    i,j,k=idx
    return (float(i)*SPACING,float(j)*SPACING,ALTITUDES[k])

def nearest_k(z):
    return min(range(len(ALTITUDES)),key=lambda k:abs(ALTITUDES[k]-z))

@dataclass
class Route:
    name: str
    points: list[tuple[float,float,float]]
    geometric_length_m: float
    predicted_mean_success: float
    minimum_clearance_m: float
    expanded_nodes: int
    objective: float

def plan(name="shortest",gcs=GCS,radius=.32,margin=.55,
         max_nodes=12000,max_lateral=33):
    if name not in ("shortest","rf_aware"):raise ValueError("Unknown method")
    if radius<=0 or margin<0:raise ValueError("Invalid clearance")
    start=(0,0,nearest_k(START[2]))
    goal=(-8,8,nearest_k(GOAL[2]))
    def safe_node(n):
        i,j,k=n
        if abs(i*SPACING)>max_lateral or abs(j*SPACING)>max_lateral:return False
        if k<0 or k>=len(ALTITUDES):return False
        return signed_clearance(idx_to_pos(n))>=radius+margin
    if not safe_node(start) or not safe_node(goal):raise ValueError("Unsafe start or goal")
    def segment_safe(a,b):
        a,b=idx_to_pos(a),idx_to_pos(b)
        steps=max(2,math.ceil(math.dist(a,b)/.35))
        for t in range(steps+1):
            r=t/steps
            p=tuple(a[j]*(1-r)+b[j]*r for j in range(3))
            if signed_clearance(p)<radius+margin:return False
        return True
    def heuristic(n):
        return math.dist(idx_to_pos(n),GOAL)
    counter=0
    todo=[(heuristic(start),0.,counter,start)]
    parent={start:None};best={start:0.}
    expanded=0
    while todo:
        _,cost,_,node=heapq.heappop(todo)
        if cost>best[node]+1e-9:continue
        expanded+=1
        if node==goal:break
        if expanded>max_nodes:raise RuntimeError("A* exceeded node budget")
        i,j,k=node
        for step in ((1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)):
            neighbor=(i+step[0],j+step[1],k+step[2])
            if not safe_node(neighbor) or not segment_safe(node,neighbor):continue
            a,b=idx_to_pos(node),idx_to_pos(neighbor)
            distance=math.dist(a,b)
            mid=tuple((a[t]+b[t])/2 for t in range(3))
            # Keep safety mandatory. Vertical moves cost extra flight time,
            # but RF penalties can make a longer/vertical route competitive.
            travel=distance*(1.4 if step[2] else 1.0)
            rf_penalty=0.
            if name=="rf_aware":
                p_success=predict_success(mid,gcs)
                # The penalty is an *expected link-quality proxy*,
                # not an actual AoI/deadline measured during planning.
                rf_penalty=distance*4.5*(1-p_success)
            new=cost+travel+rf_penalty
            if new<best.get(neighbor,float("inf"))-1e-9:
                best[neighbor]=new;parent[neighbor]=node;counter+=1
                heapq.heappush(todo,(new+heuristic(neighbor),new,counter,neighbor))
    else:raise RuntimeError(f"No collision-safe {name} path")
    reverse=[goal]
    while reverse[-1]!=start:reverse.append(parent[reverse[-1]])
    positions=[idx_to_pos(n) for n in reversed(reverse)]
    # Compress collinear runs only if the resulting segment stays safe.
    compact=[positions[0]]
    for i in range(1,len(positions)-1):
        a,b,c=compact[-1],positions[i],positions[i+1]
        ab=tuple(b[t]-a[t] for t in range(3))
        bc=tuple(c[t]-b[t] for t in range(3))
        same_direction=(sum(ab[t]*bc[t] for t in range(3))>0 and
          math.dist(a,c) <= math.dist(a,b)+math.dist(b,c)+1e-6 and
          math.sqrt(sum((ab[(t+1)%3]*bc[(t+2)%3]-ab[(t+2)%3]*bc[(t+1)%3])**2 for t in range(3)))<1e-6)
        if not same_direction:compact.append(b)
    compact.append(positions[-1])
    total=sum(math.dist(a,b) for a,b in zip(compact,compact[1:]))
    probabilities=[]
    for a,b in zip(compact,compact[1:]):
        n=max(2,math.ceil(math.dist(a,b)/.75))
        for k in range(n):
            t=(k+.5)/n
            p=tuple(a[i]*(1-t)+b[i]*t for i in range(3))
            probabilities.append(predict_success(p,gcs))
    clearance=path_clearance(compact,radius,margin)
    if clearance<margin-1e-6:raise AssertionError("Planner produced clearance violation")
    return Route(name,compact,total,sum(probabilities)/len(probabilities),clearance,expanded,best[goal])
