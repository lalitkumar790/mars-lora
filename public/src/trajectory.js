/** Continuous-path lookahead, cross-track and braking logic.
 * Unit tested without MuJoCo. No perfect state is read here:
 * callers provide GPS-less estimated XY (or reference truth mode).
 */
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
export function followSegment(start,end,current,{lookahead=.95,speed=1.5,braking=.9}={}){
 const [ax,ay]=start,[bx,by]=end,[x,y]=current;
 const dx=bx-ax,dy=by-ay,L=Math.hypot(dx,dy);
 if(L<1e-8)return {target:[bx,by],velocity:[0,0],remaining:0,crossTrack:Math.hypot(x-bx,y-by),progress:1};
 const ux=dx/L,uy=dy/L;
 const projected=clamp((x-ax)*ux+(y-ay)*uy,0,L);
 const cross=Math.abs((x-ax)*uy-(y-ay)*ux);
 const advance=clamp(projected+lookahead,0,L),remaining=Math.max(0,L-projected);
 const goalDistance=Math.hypot(bx-x,by-y);
 const v=speed*Math.min(1,Math.sqrt(Math.max(.001,2*braking*goalDistance))/Math.max(.01,speed));
 const brakingGain=clamp(remaining/Math.max(1,lookahead*1.3),0,1);
 return {target:[ax+ux*advance,ay+uy*advance],velocity:[ux*v*brakingGain,uy*v*brakingGain],
    remaining:goalDistance,crossTrack:cross,progress:projected/L};
}
export class WaypointTracker{
 constructor(){this.segmentStart=null;this.lastIndex=-1;}
 reset(){this.segmentStart=null;this.lastIndex=-1;}
 update(index,points,current,speed){
   const endpoint=points[index];
   if(!endpoint)return null;
   const begin=index===0?[current[0],current[1]]:[points[index-1][0],points[index-1][1]];
   return followSegment(begin,endpoint,current,{speed,lookahead:Math.max(.65,Math.min(1.8,speed*.9))});
 }
}
