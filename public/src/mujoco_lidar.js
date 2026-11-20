/** Native MuJoCo LiDAR rangefinder site generator.
 * Each range ray is a real MuJoCo sensor attached to the drone body.
 * Sampling from data.sensordata (not world pose) is the localization input.
 */
export const RAY_COUNT=32;
export const MAX_RANGE=18;
export function withNativeLidar(xml,n=RAY_COUNT){
 let sites='',sensors='';
 for(let i=0;i<n;i++){
   const a=-Math.PI+2*Math.PI*i/n, sin=Math.sin(a),cos=Math.cos(a),q=Math.SQRT1_2;
   // Rotate site local +Z onto horizontal bearing (cos a,sin a,0).
   const quat=[q,-sin*q,cos*q,0].map(v=>v.toFixed(9)).join(' ');
   const id=String(i).padStart(2,'0');
   sites+='<site name="lidar_'+id+'" pos="0 0 0.01" quat="'+quat+'" size=".002" rgba="0 .9 1 .2"/>\n';
   sensors+='<rangefinder name="range_'+id+'" site="lidar_'+id+'"/>\n';
 }
 if(!xml.includes('</body>'))throw Error('Missing UAV body in MJCF');
 xml=xml.replace('</body>',sites+'</body>');
 if(xml.includes('<sensor>'))xml=xml.replace('</sensor>',sensors+'</sensor>');
 else xml=xml.replace('</mujoco>','<sensor>'+sensors+'</sensor></mujoco>');
 return xml;
}
export function nativeScan(data, {beams=RAY_COUNT,maxRange=MAX_RANGE}={}){
 if(!data?.sensordata||data.sensordata.length<beams)throw Error('MuJoCo lidar sensordata unavailable');
 const start=data.sensordata.length-beams;
 const ranges=[],angles=[];
 for(let i=0;i<beams;i++){
   let d=Number(data.sensordata[start+i]);
   if(!Number.isFinite(d)||d<=0||d>maxRange)d=maxRange;
   ranges.push(d);angles.push(-Math.PI+2*Math.PI*i/beams);
 }
 return {ranges,angles,maxRange,valid:ranges.filter(r=>r<maxRange-.05).length};
}
