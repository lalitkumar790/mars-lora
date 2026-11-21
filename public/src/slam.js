/** GPS-denied mapping demonstrator.
 * The sensor simulator reads true MuJoCo pose exclusively in simulateScan().
 * The estimator receives only range beams + biased noisy velocity/yaw-gyro
 * proxies; ground-truth XY is never passed to SLAM.update().
 * Planar scan matching + occupancy grid, NOT robust 6-DoF or loop-closure SLAM.
 */
export const SCENE_SOLIDS = [
 {name:'east',x:5,y:4,z:1.4,hx:1.1,hy:1.4,hz:1.4},
 {name:'west',x:-5,y:5,z:1.9,hx:1.25,hy:1.1,hz:1.9},
 {name:'south',x:5,y:-4,z:1.6,hx:1.4,hy:1,hz:1.6},
 {name:'container',x:-3,y:-4,z:.6,hx:2,hy:.7,hz:.6},
 {name:'tower',x:1,y:8,z:2.5,hx:.24,hy:.24,hz:2.5},
 {name:'pylon',x:-8,y:0,z:2.7,hx:.32,hy:.32,hz:2.7},
 {name:'beam',x:-5,y:0,z:5.45,hx:3.3,hy:.24,hz:.23}
];
export const wrapAngle = x => Math.atan2(Math.sin(x),Math.cos(x));
export function yawOfQuaternion(q) {
 const [w,x,y,z]=q;
 return Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));
}
function slabRay(x,y,dx,dy,box,maxRange) {
 let low=0,high=maxRange;
 for(const [p,v,c,r] of [[x,dx,box.x,box.hx],[y,dy,box.y,box.hy]]) {
   const l=c-r,u=c+r;
   if(Math.abs(v)<1e-9) {if(p<l||p>u)return null;}
   else {
     let a=(l-p)/v,b=(u-p)/v;
     if(a>b)[a,b]=[b,a];
     low=Math.max(low,a);high=Math.min(high,b);
     if(low>high)return null;
   }
 }
 return low>0.05?low:null;
}
/** Emulates a 2D planar range scan at current MuJoCo altitude.
 * Geometry is accessed only here: the synthetic raycast sensor front end.
 */
export function simulateScan(truth, {beams=48,maxRange=17,seed=0}={}) {
 const [x,y,z,yaw]=truth;
 const ranges=[],angles=[];
 const next=()=>{seed=(1664525*seed+1013904223)>>>0;return seed/4294967296;};
 for(let i=0;i<beams;i++){
   const angle=-Math.PI+2*Math.PI*i/beams;
   const phi=yaw+angle,dx=Math.cos(phi),dy=Math.sin(phi);
   let best=maxRange;
   for(const box of SCENE_SOLIDS) {
     // Includes surfaces at the sensor's height only, no impossible ground hit.
     if(z<box.z-box.hz||z>box.z+box.hz)continue;
     const d=slabRay(x,y,dx,dy,box,maxRange);
     if(d!==null)best=Math.min(best,d);
   }
   const valid=best<maxRange;
   ranges.push(valid?Math.max(.08,Math.min(maxRange,best+(next()-.5)*.06)):maxRange);
   angles.push(angle);
 }
 return {ranges,angles,maxRange,valid:ranges.filter(r=>r<maxRange-.01).length};
}
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
export class PlanarSLAM {
 constructor({size=100,resolution=.35,seed=314159}={}) {
   this.size=size;this.resolution=resolution;this.seed=seed;
   this.map=new Float32Array(size*size);
   this.originX=-size*resolution/2;this.originY=-size*resolution/2;
   this.maxSize=1024;this.expansions=0;this.mapLimitReached=false;
   this.pose={x:0,y:0,yaw:0};this.odomPose={x:0,y:0,yaw:0};
   this.lastT=null;this.scanCount=0;this.matches=0;
   this.lastCorrection=0;this.lastFitness=0;this.lastHits=0;this.path=[];
   this.initialized=false;this.lastScan=null;
 }
 reset(){this.size=100;this.map=new Float32Array(this.size*this.size);this.originX=-this.size*this.resolution/2;this.originY=this.originX;this.expansions=0;this.mapLimitReached=false;this.pose={x:0,y:0,yaw:0};this.odomPose={x:0,y:0,yaw:0};this.lastT=null;this.scanCount=0;this.matches=0;this.lastCorrection=0;this.lastFitness=0;this.lastHits=0;this.path=[];this.initialized=false;this.lastScan=null;}
 index(x,y) {
   const gx=Math.floor((x-this.originX)/this.resolution),gy=Math.floor((y-this.originY)/this.resolution);
   return gx>=0&&gy>=0&&gx<this.size&&gy<this.size?gy*this.size+gx:-1;
 }
 /** Double the mapped area and preserve every previously observed cell.
  * A hard 1024-cell axis limit prevents runaway localization from exhausting RAM.
  * This expands the visible map; it is NOT true SLAM/loop closure.
  */
 ensureBounds(x,y,margin=0){
   if(!Number.isFinite(x)||!Number.isFinite(y)||!Number.isFinite(margin)||margin<0)return false;
   while(x-margin<this.originX||y-margin<this.originY||
         x+margin>=this.originX+this.size*this.resolution||
         y+margin>=this.originY+this.size*this.resolution){
     if(this.size*2>this.maxSize){this.mapLimitReached=true;return false;}
     const old=this.size,next=old*2,shift=old/2;
     const expanded=new Float32Array(next*next);
     for(let row=0;row<old;row++){
       expanded.set(this.map.subarray(row*old,(row+1)*old),(row+shift)*next+shift);
     }
     this.map=expanded;this.size=next;
     this.originX-=shift*this.resolution;this.originY-=shift*this.resolution;
     this.expansions++;
   }
   return true;
 }
 bounds(){return {minX:this.originX,minY:this.originY,
   maxX:this.originX+this.size*this.resolution,
   maxY:this.originY+this.size*this.resolution};}
 integrateOdom(dt,vx,vy,yawRate) {
   // Input represents body-frame VIO/IMU-velocity pseudo-measurements, not GPS.
   for(const p of [this.pose,this.odomPose]){
     p.yaw=wrapAngle(p.yaw+yawRate*dt);
     const c=Math.cos(p.yaw),s=Math.sin(p.yaw);
     p.x+=(c*vx-s*vy)*dt;p.y+=(s*vx+c*vy)*dt;
   }
 }
 score(scan,pose) {
   let used=0,score=0;
   const skip=2;
   for(let i=0;i<scan.ranges.length;i+=skip){
     const d=scan.ranges[i];
     if(d>=scan.maxRange-.05)continue;
     const a=pose.yaw+scan.angles[i],ix=this.index(pose.x+d*Math.cos(a),pose.y+d*Math.sin(a));
     if(ix<0)continue;
     used++;
     const occupancy=this.map[ix];
     // Nearby hits contribute to correlation even when quantized into neighbor cell.
     const gx=ix%this.size,gy=Math.floor(ix/this.size);
     let best=occupancy;
     for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){
       const xx=gx+dx,yy=gy+dy;
       if(xx>=0&&yy>=0&&xx<this.size&&yy<this.size)best=Math.max(best,this.map[yy*this.size+xx]*.75);
     }
     score+=best;
   }
   return {score:used?score/used:0,used};
 }
 match(scan) {
   if(this.scanCount<5||scan.valid<5)return;
   const base={...this.pose},baseline=this.score(scan,base);
   let best={...base},result=baseline;
   // Discrete scan-to-map local correlation, deliberately bounded
   // to reduce map corruption at weakly observable altitudes.
   for(const yaw of [-.09,0,.09]){
     for(const dx of [-.35,0,.35])for(const dy of [-.35,0,.35]){
       const candidate={x:base.x+dx,y:base.y+dy,yaw:wrapAngle(base.yaw+yaw)};
       const r=this.score(scan,candidate);
       if(r.used>=5&&r.score>result.score+.015){best=candidate;result=r;}
     }
   }
   if(result.score> .20 && result.score>baseline.score+.025){
     this.lastCorrection=Math.hypot(best.x-base.x,best.y-base.y);
     this.pose=best;this.matches++;
   }else this.lastCorrection=0;
   this.lastFitness=result.score;
 }
 mark(x,y,update) {
   const id=this.index(x,y);
   if(id>=0)this.map[id]=clamp(this.map[id]+update,-3.5,3.5);
 }
 update(t,{bodyVx,bodyVy,yawRate},scan) {
   if(this.lastT!==null){
     const dt=clamp(t-this.lastT,0,1);
     this.integrateOdom(dt,bodyVx,bodyVy,yawRate);
   }
   this.lastT=t;
   // Preserve all explored cells when the UAV crosses the initial 35 m map.
   // Expand prior to score() so registration can also use older scans.
   this.ensureBounds(this.pose.x,this.pose.y,scan.maxRange);
   this.match(scan);
   for(let i=0;i<scan.ranges.length;i++){
     const d=scan.ranges[i],angle=this.pose.yaw+scan.angles[i],c=Math.cos(angle),s=Math.sin(angle);
     const last=Math.min(d,scan.maxRange);
     // Inverse sensor model: free cells along beam, occupied endpoint only on hit.
     for(let step=.35;step<last-.25;step+=this.resolution*1.8){
       this.mark(this.pose.x+step*c,this.pose.y+step*s,-.045);
     }
     if(d<scan.maxRange-.05)this.mark(this.pose.x+d*c,this.pose.y+d*s,.47);
   }
   this.lastScan=scan;this.lastHits=scan.valid;
   this.scanCount++;this.initialized=true;
   this.path.push({...this.pose,t});
   if(this.path.length>10000)this.path.shift();
   return this.pose;
 }
 occupancyPercentage(){
   let n=0;for(const v of this.map)if(Math.abs(v)>.25)n++;
   return 100*n/this.map.length;
 }
 evaluate(truth) {
   // Ground truth is consumed exclusively for UI/evaluation, not filtering.
   return Math.hypot(this.pose.x-truth[0],this.pose.y-truth[1]);
 }
}
