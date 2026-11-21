/** Online, map-known urban 3-D A* planner for interactive MuJoCo missions.
 * Geometry is the same deterministic scene used for actual MuJoCo contacts.
 * RF cost is only a fixed model-predicted SF9 link, never observed packet data.
 * NOT a GPS-denied planner: positions and building occupancy are known here.
 */
import {CITY_BUILDINGS} from './city.js';
import {obstacles} from './mission.js';
const getScene=()=>[...obstacles,...CITY_BUILDINGS];
export const AIRSPACE_GCS=[230,220,120];
export const AIRSPACE_GOAL=[-30,24,4];
const levels=[4,7,10,13,17,21,26],spacing=3;
const pOf=([i,j,k])=>[i*spacing,j*spacing,levels[k]];
const key=n=>n.join(',');
const signedClearance=p=>{
 let best=Infinity;
 for(const b of getScene()){
  const dx=Math.max(Math.abs(p[0]-b.x)-b.hx,0),dy=Math.max(Math.abs(p[1]-b.y)-b.hy,0),dz=Math.max(Math.abs(p[2]-b.z)-b.hz,0);
  const distance=(dx||dy||dz)?Math.hypot(dx,dy,dz):
    -Math.min(b.hx-Math.abs(p[0]-b.x),b.hy-Math.abs(p[1]-b.y),b.hz-Math.abs(p[2]-b.z));
  best=Math.min(best,distance);
 }
 return best;
};
function rayBox(a,b,box){
 let lo=0,hi=1;
 for(let d=0;d<3;d++){
  const center=[box.x,box.y,box.z][d],half=[box.hx,box.hy,box.hz][d],v=b[d]-a[d];
  if(Math.abs(v)<1e-12){if(Math.abs(a[d]-center)>half)return false;}
  else{
   const t0=(center-half-a[d])/v,t1=(center+half-a[d])/v;
   lo=Math.max(lo,Math.min(t0,t1));hi=Math.min(hi,Math.max(t0,t1));
   if(lo>hi)return false;
  }
 }
 return lo<.999&&hi>.001;
}
export function predictCoverage(p,station=AIRSPACE_GCS){
 const blockage=getScene().some(b=>rayBox(p,station,b));
 const d=Math.max(1,Math.hypot(p[0]-station[0],p[1]-station[1],p[2]-station[2]));
 const rssi=14-31.2-40*Math.log10(d)-(blockage?17:0);
 const sensitivity=-129;
 const margin=rssi-sensitivity;
 return {p:.9/(1+Math.exp(-Math.max(-45,Math.min(45,margin))/1.8)),rssi,blocked:blockage,margin};
}
export function checkRoute(route,radius=.32,buffer=.55){
 let min=Infinity;
 for(let i=1;i<route.length;i++){
  const a=route[i-1],b=route[i],length=Math.hypot(...a.map((v,k)=>b[k]-v));
  const n=Math.max(2,Math.ceil(length/.25));
  for(let j=0;j<=n;j++){
   const t=j/n,p=a.map((v,k)=>v+(b[k]-v)*t);
   min=Math.min(min,signedClearance(p)-radius);
  }
 }
 return {safe:min>=buffer-.001,clearance:min};
}
export function planInteractiveRoute(method='shortest',radius=.32,options={}){
 if(!['shortest','rf_aware'].includes(method))throw Error('Unsupported planner');
 const start=[0,0,0],margin=.65;
 const goalPosition=options.goal??AIRSPACE_GOAL;
 const goal=[Math.round(goalPosition[0]/spacing),Math.round(goalPosition[1]/spacing),0];
 const nodeSafe=n=>{
  if(Math.abs(n[0])>22||Math.abs(n[1])>22||n[2]<0||n[2]>=levels.length)return false;
  return signedClearance(pOf(n))>=radius+margin;
 };
 const safeEdge=(a,b)=>{
  const p=pOf(a),q=pOf(b),length=Math.hypot(...p.map((v,i)=>q[i]-v));
  const steps=Math.ceil(length/.35);
  for(let i=0;i<=steps;i++){
   const t=i/steps;
   if(signedClearance(p.map((v,j)=>v+(q[j]-v)*t))<radius+margin)return false;
  }
  return true;
 };
 if(!nodeSafe(start)||!nodeSafe(goal))throw Error('Start or goal inside exclusion zone');
 const nodes=new Map([[key(start),{n:start,g:0,parent:null}]]);
 const frontier=[{n:start,g:0,f:Math.hypot(...pOf(start).map((x,i)=>x-pOf(goal)[i]))}];
 let expanded=0;
 while(frontier.length){
  frontier.sort((a,b)=>b.f-a.f);
  const cur=frontier.pop();
  if(cur.g>nodes.get(key(cur.n)).g+1e-8)continue;
  expanded++;if(expanded>16000)throw Error('Planner budget exceeded');
  if(key(cur.n)===key(goal))break;
  for(const [dx,dy,dz] of [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]]){
   const n=[cur.n[0]+dx,cur.n[1]+dy,cur.n[2]+dz];
   if(!nodeSafe(n)||!safeEdge(cur.n,n))continue;
   const a=pOf(cur.n),b=pOf(n),dist=Math.hypot(...a.map((v,j)=>b[j]-v));
   const mid=a.map((v,j)=>(v+b[j])/2);
   const rfPenalty=method==='rf_aware'?4.5*dist*(1-predictCoverage(mid).p):0;
   const g=cur.g+dist*(dz?1.4:1)+rfPenalty;
   const id=key(n),old=nodes.get(id);
   if(old&&g>=old.g-1e-9)continue;
   nodes.set(id,{n,g,parent:key(cur.n)});
   frontier.push({n,g,f:g+Math.hypot(...b.map((v,j)=>pOf(goal)[j]-v))});
  }
 }
 if(!nodes.has(key(goal)))throw Error('No collision-safe route found');
 const rev=[];let k=key(goal);
 while(k!==null){const item=nodes.get(k);rev.push(pOf(item.n));k=item.parent;}
 rev.reverse();
 const route=[rev[0]];
 for(let i=1;i<rev.length-1;i++){
  const a=route[route.length-1],b=rev[i],c=rev[i+1];
  const u=a.map((v,j)=>b[j]-v),v=b.map((x,j)=>c[j]-x);
  const cross=Math.hypot(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]);
  if(cross>1e-6||u.reduce((s,x,j)=>s+x*v[j],0)<=0)route.push(b);
 }
 route.push(rev[rev.length-1]);
 const check=checkRoute(route,radius,margin);
 if(!check.safe)throw Error('Path violates obstacle clearance');
 const length=route.slice(1).reduce((s,p,i)=>s+Math.hypot(...p.map((v,j)=>v-route[i][j])),0);
 return {route,method,expanded,clearance:check.clearance,length_m:length,goal:pOf(goal),
   coverage:route.map(p=>predictCoverage(p).p)};
}

/** Choose a safe goal with a non-repeating direction and varying distance.
 * Uses the live collision geometry, so all random goals are physically valid.
 * The random generation is not used by published fixed-seed comparisons.
 */
export function randomUrbanGoal(previous=null,random=Math.random){
 const choices=[];
 for(let i=-18;i<=18;i+=2)for(let j=-18;j<=18;j+=2){
   const radius=Math.hypot(i*spacing,j*spacing);
   if(radius<24||radius>67)continue;
   const p=[i*spacing,j*spacing,4];
   if(previous&&Math.hypot(p[0]-previous[0],p[1]-previous[1])<12)continue;
   if(signedClearance(p)<1.8)continue;
   choices.push(p);
 }
 if(!choices.length)throw Error('No open airspace destinations in this city');
 return choices[Math.floor(random()*choices.length)];
}
