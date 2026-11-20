/** Laser-scan obstruction guard. Inputs are MuJoCo rangefinder beams,
 * never world geometry. Stops forward motion when the heading sector is
 * blocked. This is not global re-planning and cannot guarantee safety.
 */
export function scanHazard(scan,heading=0,trigger=2.1){
  if(!scan?.ranges?.length)return {blocked:false,distance:Infinity,sector:'none',valid:0};
  let best=Infinity,angle=0,valid=0;
  for(let i=0;i<scan.ranges.length;i++){
    const d=scan.ranges[i];
    if(!Number.isFinite(d)||d>=scan.maxRange-.01)continue;
    valid++;
    const a=scan.angles[i];
    const diff=Math.atan2(Math.sin(a-heading),Math.cos(a-heading));
    if(Math.abs(diff)<=Math.PI/4 && d<best){best=d;angle=diff;}
  }
  return {blocked:best<trigger,distance:best,sector:Math.abs(angle)<.23?'forward':angle>0?'left':'right',valid};
}
