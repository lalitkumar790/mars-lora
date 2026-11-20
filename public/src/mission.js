/** Collision-aware mission envelope. This is a conservative, deterministic
 * high-altitude transit planner, not a general 3-D path-planner.
 * All solids below have collision counterparts in both browser MJCFs.
 */
export const obstacles = Object.freeze([
 {name:'East inspection block',x:5,y:4,z:1.4,hx:1.1,hy:1.4,hz:1.4},
 {name:'West inspection block',x:-5,y:5,z:1.9,hx:1.25,hy:1.1,hz:1.9},
 {name:'South inspection block',x:5,y:-4,z:1.6,hx:1.4,hy:1,hz:1.6},
 {name:'Storage container',x:-3,y:-4,z:.6,hx:2,hy:.7,hz:.6},
 {name:'Antenna tower',x:1,y:8,z:2.5,hx:.24,hy:.24,hz:2.5},
 {name:'Power pylon',x:-8,y:0,z:2.7,hx:.32,hy:.32,hz:2.7},
 {name:'Overhead beam',x:-5,y:0,z:5.45,hx:3.3,hy:.24,hz:.23}
]);
export const maxObstacleHeight = Math.max(...obstacles.map(o=>o.z+o.hz));
export const transitAltitude = Math.ceil((maxObstacleHeight+.9)*2)/2; // 7 m
export const missionWaypoints=[
 [0,0,transitAltitude],
 [2,0,transitAltitude],
 [2,8,transitAltitude],
 [-7,8,transitAltitude],
 [-7,-7,transitAltitude],
 [0,0,transitAltitude],
 [0,0,2.5]
];
/** Axis aligned *surface* distance, with simple cylinder-as-AABB conservatism.
 * Input vehicleRadius expands collision solids; negative -> overlap.
 */
export function obstacleClearance(x,y,z,vehicleRadius=.25) {
 let best=Infinity,nearest='none';
 for(const o of obstacles) {
  const dx=Math.max(Math.abs(x-o.x)-o.hx,0);
  const dy=Math.max(Math.abs(y-o.y)-o.hy,0);
  const dz=Math.max(Math.abs(z-o.z)-o.hz,0);
  const outside=Math.hypot(dx,dy,dz);
  const inside=Math.min(o.hx-Math.abs(x-o.x),o.hy-Math.abs(y-o.y),o.hz-Math.abs(z-o.z));
  const signed=(dx===0&&dy===0&&dz===0?-inside:outside)-vehicleRadius;
  if(signed<best){best=signed;nearest=o.name;}
 }
 return {distance:best,nearest,warning:best<.65,overlap:best<=0};
}
export function plannedPathClearance(points=missionWaypoints,radius=.25) {
 let min=Infinity;
 for(let i=1;i<points.length;i++){
  const a=points[i-1],b=points[i];
  const n=Math.ceil(Math.hypot(...b.map((v,j)=>v-a[j]))/.1);
  for(let k=0;k<=n;k++){
   const t=k/Math.max(1,n),p=a.map((v,j)=>v+t*(b[j]-v));
   min=Math.min(min,obstacleClearance(...p,radius).distance);
  }
 }
 return min;
}

/** GPS-denied planar-LiDAR mapping exercise at 3.2m.
 * Deliberately avoids overhanging western roof and antenna pylon while
 * passing close enough to see vertical features in the horizontal scan.
 */
export const mappingWaypoints=[
 [0,0,3.2],[0,-7,3.2],[-9.5,-7,3.2],
 [-9.5,7,3.2],[-7,7,3.2],[-9.5,7,3.2],
 [-9.5,-7,3.2],[0,-7,3.2],[0,0,3.2]
];
