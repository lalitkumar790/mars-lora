import quadXML from '../quad.xml?raw';
import crazyflieXML from '../crazyflie.xml?raw';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import loadMujoco from '@mujoco/mujoco';
import wasmUrl from '@mujoco/mujoco/mujoco.wasm?url';

const $ = id => document.getElementById(id);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a1725);
scene.fog = new THREE.FogExp2(0x0a1725, 0.012);
const camera = new THREE.PerspectiveCamera(52, 1, .03, 150);
camera.up.set(0, 0, 1);
camera.position.set(8, -12, 9);
const renderer = new THREE.WebGLRenderer({ antialias:true, powerPreference:'high-performance' });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.outputColorSpace = THREE.SRGBColorSpace;
$('viewport').appendChild(renderer.domElement);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;
orbit.target.set(0, 0, 1);
scene.add(new THREE.HemisphereLight(0xa7d6fc, 0x162a36, 2));
const sun = new THREE.DirectionalLight(0xffecd2, 2.6);
sun.position.set(-12, 8, 24);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera,{ left:-22,right:22,top:22,bottom:-22,near:1,far:70 });
scene.add(sun);

const ground = new THREE.Mesh(new THREE.PlaneGeometry(100, 100), new THREE.MeshStandardMaterial({color:0x142b39, roughness:.95}));
ground.receiveShadow=true;
scene.add(ground);
const grid = new THREE.GridHelper(90, 45, 0x25475b, 0x213a4b);
grid.rotation.x=Math.PI/2;
grid.position.z=.006;
scene.add(grid);

function box(x,y,z,sx,sy,sz,color) {
 const m=new THREE.Mesh(new THREE.BoxGeometry(sx,sy,sz),new THREE.MeshStandardMaterial({color,metalness:.25,roughness:.65}));
 m.position.set(x,y,z);m.castShadow=true;m.receiveShadow=true;scene.add(m);return m;
}
for (const v of [[5,4,1.4,2.2,2.8,2.8,0x556e83],[-5,5,1.9,2.5,2.2,3.8,0x657084],[5,-4,1.6,2.8,2,3.2,0x567689],[-3,-4,.6,4,1.4,1.2,0xa65a3e],[-8,0,2.7,.64,.64,5.4,0x687891],[-5,0,5.45,6.6,.48,.46,0x788696]])box(...v);
const tower=new THREE.Mesh(new THREE.CylinderGeometry(.24,.24,5,14),new THREE.MeshStandardMaterial({color:0x697e89,metalness:.65,roughness:.45}));
tower.position.set(1,8,2.5);tower.rotation.x=Math.PI/2;tower.castShadow=true;scene.add(tower);
for(let i=0;i<3;i++){const tank=new THREE.Mesh(new THREE.CylinderGeometry(.5,.5,1.5,24),new THREE.MeshStandardMaterial({color:0x536f80,metalness:.3}));tank.position.set(-10+i*1.3,-8,.75);tank.rotation.x=Math.PI/2;scene.add(tank);}
const station=box(0,-2,.28,.38,.4,.56,0x47d9b1);
const beacon=new THREE.PointLight(0x5cebbd,3,7);
beacon.position.copy(station.position).add(new THREE.Vector3(0,0,.8));scene.add(beacon);
const pole=new THREE.Mesh(new THREE.CylinderGeometry(.025,.025,1.5,10),new THREE.MeshBasicMaterial({color:0x83ffe0}));
pole.rotation.x=Math.PI/2;pole.position.set(0,-2,1.25);scene.add(pole);
const towerTop=new THREE.Mesh(new THREE.SphereGeometry(.1,10,10),new THREE.MeshBasicMaterial({color:0x6afcd0}));
towerTop.position.set(0,-2,2);scene.add(towerTop);

const drone=new THREE.Group();
const body=new THREE.Mesh(new THREE.BoxGeometry(.22,.16,.07),new THREE.MeshStandardMaterial({color:0x26b9f2,metalness:.7,roughness:.25}));
body.castShadow=true;drone.add(body);
function cylinderBetween(a,b,r,color){
 const va=new THREE.Vector3(...a),vb=new THREE.Vector3(...b),diff=vb.clone().sub(va);
 const mesh=new THREE.Mesh(new THREE.CylinderGeometry(r,r,diff.length(),12),new THREE.MeshStandardMaterial({color,metalness:.6,roughness:.35}));
 mesh.position.copy(va).add(vb).multiplyScalar(.5);
 mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),diff.normalize());
 mesh.castShadow=true;drone.add(mesh);
}
cylinderBetween([-.21,-.21,0],[.21,.21,0],.013,0xb5cedd);
cylinderBetween([-.21,.21,0],[.21,-.21,0],.013,0xb5cedd);
const rotors=[];
for(const [x,y] of [[.21,.21],[.21,-.21],[-.21,-.21],[-.21,.21]]){
 const hub=new THREE.Mesh(new THREE.CylinderGeometry(.045,.045,.035,16),new THREE.MeshStandardMaterial({color:0x152837,metalness:.6}));
 hub.rotation.x=Math.PI/2;hub.position.set(x,y,.02);drone.add(hub);
 const rotor=new THREE.Group();rotor.position.set(x,y,.045);
 for(const theta of [0,Math.PI/2]){
 const blade=new THREE.Mesh(new THREE.BoxGeometry(.31,.029,.007),new THREE.MeshStandardMaterial({color:0xb8e7f8,transparent:true,opacity:.82}));
 blade.rotation.z=theta;rotor.add(blade);
 }
 rotors.push(rotor);drone.add(rotor);
}
const navlight=new THREE.PointLight(0x2bd1ff,1.8,2);
navlight.position.set(.13,0,.05);drone.add(navlight);scene.add(drone);
const tracePoints=[];
let traceLine=null;
const traceMaterial=new THREE.LineBasicMaterial({color:0x55dcf2,transparent:true,opacity:.7});
const waypointObjects=[];
const waypointPositions=[[2,0,3.5],[2,8,3.5],[-7,8,4],[-7,-7,4],[0,0,2.5]];
for(let i=0;i<waypointPositions.length;i++){
 const [x,y,z]=waypointPositions[i];
 const marker=new THREE.Mesh(new THREE.TorusGeometry(.35,.035,8,24),new THREE.MeshBasicMaterial({color:0xc6a974,transparent:true,opacity:.85}));
 marker.position.set(x,y,z);scene.add(marker);waypointObjects.push(marker);
}
let airframe='heavy', mjMass=1.2, arm=.21, momentScale=1, modelThrustMax=15;
let mujoco,model,data,ready=false,paused=false,follow=true,mission=false,waypointIndex=0,flightMode='Standby',lastUi=0,lastLog=0,lastTrace=0,desired=[0,0,.5],lastFrame=performance.now(),accumulator=0;
let wind=.25,missionSpeed=1.5,flightLog=[];
const simStep=.004;
const snap=(v,min,max)=>Math.max(min,Math.min(max,v));
function report(message) {
 const at=data?Number(data.time).toFixed(1):'0.0';
 const pane=$('events');pane.textContent='['+at+'s] '+message+'\n'+pane.textContent.slice(0,3200);
}
function status(label,color='#58e3a1'){
 $('status').textContent=label;$('dot').style.background=color;
}
function quatToRPY(q) {
 const [w,x,y,z]=q;
 return [
 Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
 Math.asin(snap(2*(w*y-z*x),-1,1)),
 Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
 ];
}
function setMode(next){flightMode=next;$('flightmode').textContent=next.toUpperCase();report(next);}
function goTo(index){waypointIndex=index;report('Navigating to waypoint '+(index+1));}
function setTarget(axis,direction){
 if(!ready || paused)return;
 mission=false;desired[axis]=snap(desired[axis]+direction,-14,14);
 if(axis===2)desired[2]=snap(desired[2],.32,8);
 setMode('Manual command');
}
function controller(){
 if(!ready)return;
 const pos=data.qpos,vel=data.qvel;
 const [roll,pitch,yaw]=quatToRPY([pos[3],pos[4],pos[5],pos[6]]);
 if(mission){
 const wp=waypointPositions[waypointIndex],delta=[wp[0]-pos[0],wp[1]-pos[1],wp[2]-pos[2]];
 const dist=Math.hypot(...delta);
 if(dist<.55){
  if(waypointIndex===waypointPositions.length-1){mission=false;setMode('Mission complete');}
  else goTo(waypointIndex+1);
 }
 const target=waypointPositions[waypointIndex];
 const planar=Math.hypot(target[0]-pos[0],target[1]-pos[1]);
 // Slow the target convergence enough to respect requested mission speed.
 const scale=Math.min(1,missionSpeed/Math.max(.1,planar));
 desired[0]=pos[0]+(target[0]-pos[0])*scale;
 desired[1]=pos[1]+(target[1]-pos[1])*scale;
 desired[2]=target[2];
 }
 const ax=snap(1.1*(desired[0]-pos[0])-1.4*vel[0],-1.5,1.5);
 const ay=snap(1.1*(desired[1]-pos[1])-1.4*vel[1],-1.5,1.5);
 const rollTarget=snap(-ay/9.81,-.2,.2);
 const pitchTarget=snap(ax/9.81,-.2,.2);
 const t=mjMass*snap(9.81+3.0*(desired[2]-pos[2])-2.2*vel[2],1,18);
 const mx=momentScale*snap(1.1*(rollTarget-roll)-.22*vel[3],-.6,.6);
 const my=momentScale*snap(1.1*(pitchTarget-pitch)-.22*vel[4],-.6,.6);
 const a=arm, xy=[[a,a],[a,-a],[-a,-a],[-a,a]];
 for(let i=0;i<4;i++){
  const [x,y]=xy[i];data.ctrl[i]=snap(t/4+mx*y/(4*a*a)-my*x/(4*a*a),0,modelThrustMax);
 }
 // The free body has body id 1. External force applied at its COM.
 if(data.xfrc_applied?.length>=12){
  data.xfrc_applied[6]=wind*(mjMass/1.2)*Math.sin(data.time*.6);
  data.xfrc_applied[7]=wind*(mjMass/1.2)*Math.cos(data.time*.43);
 }
}
function appendLog(){
 const p=data.qpos,v=data.qvel;
 flightLog.push([data.time,...Array.from(p.slice(0,7)),...Array.from(v.slice(0,3)),...Array.from(data.ctrl.slice(0,4))]);
 if(flightLog.length>18000)flightLog.shift();
}
function updateUI(){
 const p=data.qpos,v=data.qvel,x=p[0],y=p[1],z=p[2],t=data.time;
 const dist=Math.hypot(x,y+2,z-2);
 const sf=Number($('sf').value.slice(2));
 const rssi=14-31.2-40*Math.log10(Math.max(1,dist));
 const sens={7:-123,8:-126,9:-129,10:-132,11:-134.5,12:-137}[sf];
 $('time').textContent=t.toFixed(1)+' s';
 $('altitude').textContent=z.toFixed(2)+' m';
 $('velocity').textContent=Math.hypot(...Array.from(v.slice(0,3))).toFixed(2)+' m/s';
 $('thrust').textContent=Array.from(data.ctrl).reduce((s,n)=>s+n,0).toFixed(1)+' N';
 $('distance').textContent=dist.toFixed(1)+' m';
 $('rssi').textContent=rssi.toFixed(1)+' dBm';
 $('linklabel').textContent='Estimated margin '+(rssi-sens).toFixed(1)+' dB at SF'+sf+' (path-loss-only illustration; no packet traffic).';
 $('waypoints').textContent=mission?'Active waypoint '+(waypointIndex+1)+' / '+waypointPositions.length:'Mission inactive • '+waypointPositions.length+' waypoints available';
 $('xpos').textContent=x.toFixed(2);$('ypos').textContent=y.toFixed(2);$('zpos').textContent=z.toFixed(2);
}
function resize(){
 const root=$('viewport');
 const w=root.clientWidth,h=root.clientHeight;
 if(w<1||h<1)return;
 camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);
}
new ResizeObserver(resize).observe($('viewport'));
function frame(now){
 requestAnimationFrame(frame);
 const elapsed=Math.min(.05,(now-lastFrame)/1000);lastFrame=now;
 if(ready&&!paused){
  accumulator=Math.min(.08,accumulator+elapsed);
  let steps=0;
  while(accumulator>=simStep&&steps<24){
   if(Math.floor(data.time/.02)!==Math.floor((data.time+simStep)/.02))controller();
   mujoco.mj_step(model,data);
   accumulator-=simStep;steps++;
  }
  if(data.time-lastLog>.1){lastLog=data.time;appendLog();}
  if(data.time-lastTrace>.15){
   lastTrace=data.time;
   tracePoints.push(new THREE.Vector3(data.qpos[0],data.qpos[1],Math.max(.025,data.qpos[2])));
   if(tracePoints.length>450)tracePoints.shift();
   if(traceLine){scene.remove(traceLine);traceLine.geometry.dispose();}
   if(tracePoints.length>1){traceLine=new THREE.Line(new THREE.BufferGeometry().setFromPoints(tracePoints),traceMaterial);scene.add(traceLine);}
  }
  if(data.time-lastUi>.15){lastUi=data.time;updateUI();}
  if(data.qpos[2]<-1 || !Number.isFinite(data.qpos[2])){
   paused=true;status('SIMULATION HALTED', '#ff7777');report('Nonfinite or invalid altitude. Reset simulation.');
  }
  for(const [i,r] of rotors.entries())r.rotation.z+=Math.max(.08,data.ctrl[i]/9)*.15;
 }
 if(ready){
  const p=data.qpos;
  drone.position.set(p[0],p[1],p[2]);
  drone.quaternion.set(p[4],p[5],p[6],p[3]);
  if(follow){
   const target=new THREE.Vector3(p[0],p[1],p[2]);
   const d=target.clone().sub(orbit.target);camera.position.addScaledVector(d,.08);orbit.target.lerp(target,.08);
  }
  for(let i=0;i<waypointObjects.length;i++){
   waypointObjects[i].material.color.set((mission&&waypointIndex===i)?0x65e2ff:0xc6a974);
  }
 }
 orbit.update();renderer.render(scene,camera);
}
requestAnimationFrame(frame);
function on(id,fn){$(id).addEventListener('click',fn);}
on('takeoff',()=>{if(!ready)return;mission=false;desired=[data.qpos[0],data.qpos[1],2.5];setMode('Takeoff / hover');});
on('land',()=>{if(!ready)return;mission=false;desired=[data.qpos[0],data.qpos[1],.32];setMode('Landing');});
on('hover',()=>{if(!ready)return;mission=false;desired=[data.qpos[0],data.qpos[1],Math.max(.35,data.qpos[2])];setMode('Hold position');});
on('mission',()=>{if(!ready)return;mission=true;goTo(0);setMode('Waypoint mission');});
on('pause',()=>{if(!ready)return;paused=!paused;$('pause').textContent=paused?'▶ Resume':'Ⅱ Pause';status(paused?'PAUSED':'PHYSICS ONLINE',paused?'#f0b764':'#63eda1');});
on('camera',()=>{follow=!follow;report(follow?'Camera following UAV':'Free orbit camera');});
on('reset',()=>{if(!ready)return;mujoco.mj_resetData(model,data);mujoco.mj_forward(model,data);paused=false;accumulator=0;mission=false;desired=[0,0,.5];waypointIndex=0;flightLog=[];tracePoints.length=0;if(traceLine){scene.remove(traceLine);traceLine.geometry.dispose();traceLine=null;}lastLog=lastUi=lastTrace=0;$('pause').textContent='Ⅱ Pause';setMode('Reset / standby');status('PHYSICS ONLINE');updateUI();});
document.querySelectorAll('[data-move]').forEach(button=>button.addEventListener('click',()=>{
 const map={'x-':[0,-1],'x+':[0,1],'y-':[1,-1],'y+':[1,1],'z-':[2,-.4],'z+':[2,.4]};
 setTarget(...map[button.dataset.move]);
}));
const keys={w:[1,1],s:[1,-1],a:[0,-1],d:[0,1],q:[2,-.4],e:[2,.4]};
window.addEventListener('keydown',event=>{
 if(event.repeat||event.target.matches('input, select, textarea'))return;
 const action=keys[event.key.toLowerCase()];
 if(action){event.preventDefault();setTarget(...action);}
 if(event.code==='Space'){event.preventDefault();$('pause').click();}
});
for(const [id,fn] of [['wind',v=>wind=v],['speed',v=>missionSpeed=v]]){
 $(id).addEventListener('input',e=>{fn(Number(e.target.value));$(id+'val').textContent=Number(e.target.value).toFixed(2)+(id==='wind'?' N':' m/s');});
}
on('export',()=>{
 if(!flightLog.length){report('No flight trace to export');return;}
 const header='time_s,x_m,y_m,z_m,qw,qx,qy,qz,vx_mps,vy_mps,vz_mps,rotor1_N,rotor2_N,rotor3_N,rotor4_N';
 const blob=new Blob([header+'\n'+flightLog.map(r=>r.join(',')).join('\n')],{type:'text/csv'});
 const link=document.createElement('a');link.href=URL.createObjectURL(blob);link.download='mars-lora-mujoco-flight.csv';link.click();setTimeout(()=>URL.revokeObjectURL(link.href),1000);
 report('Exported '+flightLog.length+' physics samples');
});
async function initialize(){
 try{
  status('LOADING MUJOCO WASM','#f0b764');
  mujoco=await loadMujoco({locateFile:file=>file.endsWith('.wasm')?wasmUrl:file});
  const xml=airframe==='crazyflie'?crazyflieXML:quadXML;
  model=mujoco.MjModel.from_xml_string(xml);
  if(!model)throw Error('MuJoCo MJCF compilation returned no model');
  data=new mujoco.MjData(model);
  mjMass=airframe==='crazyflie'?.027:1.2;arm=airframe==='crazyflie'?.046:.21;momentScale=airframe==='crazyflie'?(2.3951e-5/.019):1;modelThrustMax=airframe==='crazyflie'?.15:15;
  drone.scale.setScalar(airframe==='crazyflie'?.23:1);
  mujoco.mj_forward(model,data);
  ready=true;desired=[0,0,.5];controller();updateUI();
  status('PHYSICS ONLINE');
  $('help').hidden=true;
  report('MuJoCo WebAssembly engine initialized, model compiled ('+model.nbody+' bodies).');
  report('Use Take off, waypoint mission, or manual controls.');
 }catch(error){
  console.error(error);status('PHYSICS FAILED','#ff6f81');
  $('help').hidden=false;$('help').textContent='Unable to load MuJoCo WebAssembly: '+error.message+'. Check browser WebGL/WebAssembly support and refresh.';
  report('Initialization error: '+error.message);
 }
}
$('airframe').addEventListener('change',async e=>{
 if(!mujoco)return;
 paused=true;ready=false;airframe=e.target.value;
 mission=false;flightLog=[];tracePoints.length=0;if(traceLine){scene.remove(traceLine);traceLine.geometry.dispose();traceLine=null;}
 model=null;data=null;lastLog=lastUi=lastTrace=0;accumulator=0;
 status('RECOMPILING AIRFRAME','#f0b764');
 try{
  model=mujoco.MjModel.from_xml_string(airframe==='crazyflie'?crazyflieXML:quadXML);
  data=new mujoco.MjData(model);
  mjMass=airframe==='crazyflie'?.027:1.2;arm=airframe==='crazyflie'?.046:.21;
  momentScale=airframe==='crazyflie'?(2.3951e-5/.019):1;modelThrustMax=airframe==='crazyflie'?.15:15;
  drone.scale.setScalar(airframe==='crazyflie'?.23:1);
  mujoco.mj_forward(model,data);ready=true;paused=false;desired=[0,0,.5];controller();updateUI();
  setMode('Standby');status('PHYSICS ONLINE');report('Loaded '+(airframe==='crazyflie'?'Crazyflie 2-derived 27 g':'Inspection 1.2 kg')+' airframe.');
 }catch(err){console.error(err);status('MODEL LOAD FAILED','#f87171');$('help').hidden=false;$('help').textContent=err.message;}
});
initialize();
