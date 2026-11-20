/** Dual-plane 3D obstacle perception, generated as native MuJoCo rangefinders.
 * Horizontal 32 x 360-degree LiDAR is used for planar SLAM. A second,
 * downward-canted 32-ray ring detects building roofs below the UAV.
 */
export const RAY_COUNT=32;
export const MAX_RANGE=25;
export const DOWNWARD_DEG=-32;
export function withNativeLidar(xml,n=RAY_COUNT){
 let sites='',sensors='';
 for(const [ring,elevation] of [['lidar',0],['depth',DOWNWARD_DEG*Math.PI/180]]){
  for(let i=0;i<n;i++){
   const az=-Math.PI+2*Math.PI*i/n;
   const dx=Math.cos(elevation)*Math.cos(az),dy=Math.cos(elevation)*Math.sin(az),dz=Math.sin(elevation);
   // Unit quaternion takes site +Z into the desired beam direction.
   const w=Math.sqrt((1+dz)/2);
   const quat=[w,-dy/(2*w),dx/(2*w),0].map(v=>v.toFixed(9)).join(' ');
   const id=String(i).padStart(2,'0');
   sites+='<site name="'+ring+'_'+id+'" pos="0 0 0.01" quat="'+quat+'" size=".002" rgba="0 .9 1 .2"/>\n';
   sensors+='<rangefinder name="'+ring+'_range_'+id+'" site="'+ring+'_'+id+'"/>\n';
  }
 }
 if(!xml.includes('</body>'))throw Error('Missing UAV body in MJCF');
 xml=xml.replace('</body>',sites+'</body>');
 if(xml.includes('<sensor>'))xml=xml.replace('</sensor>',sensors+'</sensor>');
 else xml=xml.replace('</mujoco>','<sensor>'+sensors+'</sensor></mujoco>');
 return xml;
}
function rayBatch(data,offset,count,maxRange=MAX_RANGE){
 if(!data?.sensordata||data.sensordata.length<2*count)throw Error('MuJoCo LiDAR sensor array unavailable');
 const start=data.sensordata.length-2*count+offset*count;
 const ranges=[],angles=[];
 for(let i=0;i<count;i++){
  let d=Number(data.sensordata[start+i]);
  if(!Number.isFinite(d)||d<=0||d>maxRange)d=maxRange;
  ranges.push(d);angles.push(-Math.PI+2*Math.PI*i/count);
 }
 return {ranges,angles,maxRange,valid:ranges.filter(r=>r<maxRange-.05).length};
}
export function nativeScan(data,{beams=RAY_COUNT,maxRange=MAX_RANGE}={}){
 return rayBatch(data,0,beams,maxRange);
}
export function obstacleScan(data,{beams=RAY_COUNT,maxRange=MAX_RANGE}={}){
 const ring=rayBatch(data,1,beams,maxRange);
 let forward=Infinity,nearest=Infinity,angle=0;
 for(let i=0;i<beams;i++){
  const d=ring.ranges[i];
  if(d>=maxRange-.01)continue;
  if(d<nearest){nearest=d;angle=ring.angles[i];}
  if(Math.abs(ring.angles[i])<=Math.PI/3)forward=Math.min(forward,d);
 }
 return {...ring,tiltDeg:DOWNWARD_DEG,nearest,nearestAngle:angle,forward};
}
