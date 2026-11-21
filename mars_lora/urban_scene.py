"""Exact Python mirror of public/src/city.js seeded urban building geometry."""
from __future__ import annotations

CITY_SEED=37021

def city(seed=CITY_SEED):
    state=seed
    def rng():
        nonlocal state
        state=(1664525*state+1013904223)&0xffffffff
        return state/4294967296
    buildings=[]
    for ix in range(-4,5):
        for iy in range(-4,5):
            if (ix==0 and iy==0) or (abs(ix)<=1 and abs(iy)<=1):
                continue
            x=ix*12.2+(rng()-.5)*1.9
            y=iy*12.2+(rng()-.5)*1.9
            if abs(x)<13 and abs(y)<13:
                continue
            sx=3+rng()*1.4
            sy=3+rng()*1.6
            height=6+rng()*17
            rng() # matching city.js facade tint draw
            buildings.append((x,y,height/2,sx,sy,height/2))
    return buildings

def add_city_mjcf(xml,seed=CITY_SEED):
    def geom(i,b):
        x,y,z,hx,hy,hz=b
        return ('<geom name="urban_%d" type="box" pos="%.8f %.8f %.8f" '
                'size="%.8f %.8f %.8f" friction=".9 .03 .003" '
                'rgba=".28 .34 .42 1"/>')%(i,x,y,z,hx,hy,hz)
    if '</worldbody>' not in xml:raise ValueError('Missing MuJoCo worldbody')
    return xml.replace('</worldbody>','\n'.join(geom(i,b) for i,b in enumerate(city(seed)))+'</worldbody>')
