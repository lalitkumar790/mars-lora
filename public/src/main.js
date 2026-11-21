import quadXML from '../quad.xml?raw';
import crazyflieXML from '../crazyflie.xml?raw';
import { LiveLink } from './link.js';
import { PlanarSLAM,yawOfQuaternion } from './slam.js';
import {withNativeLidar,nativeScan,obstacleScan} from './mujoco_lidar.js';
import {loadAirframeCAD,PHYSICAL_AIRFRAMES} from './airframes.js';
import {withCity,addCityToScene,regenerateCity,CITY_BUILDINGS,ACTIVE_CITY_SEED} from './city.js';
import {WaypointTracker} from './trajectory.js';
import {scanHazard} from './avoidance.js';
import {planInteractiveRoute,predictCoverage,randomUrbanGoal} from './rf_route_planner.js';
import { missionWaypoints, mappingWaypoints, obstacles, obstacleClearance, plannedPathClearance, transitAltitude } from './mission.js';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import loadMujoco from '@mujoco/mujoco';
import wasmUrl from '@mujoco/mujoco/mujoco.wasm?url';

const $ = id => document.getElementById(id);
const scene = new THREE.Scene();
let cityGroup=addCityToScene(scene);
scene.background = new THREE.Color(0x0a1725);
scene.fog = new THREE.FogExp2(0x0a1725, 0.006);
const camera = new THREE.PerspectiveCamera(52, 1, .03, 260);
camera.up.set(0, 0, 1);
camera.position.set(8, -12, 9);
const renderer = new THREE.WebGLRenderer({ antialias:true, powerPreference:'high-performance' });
renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
renderer.shadowMap.enabled = !navigator.webdriver;
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

const ground = new THREE.Mesh(new THREE.PlaneGeometry(200, 200), new THREE.MeshStandardMaterial({color:0x142b39, roughness:.95}));
ground.receiveShadow=true;
scene.add(ground);
const grid = new THREE.GridHelper(185, 37, 0x25475b, 0x213a4b);
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
const plannedLine=new THREE.Line(
 new THREE.BufferGeometry().setFromPoints(missionWaypoints.map(w=>new THREE.Vector3(...w))),
 new THREE.LineDashedMaterial({color:0xeca85f,dashSize:.6,gapSize:.35,transparent:true,opacity:.75}));
plannedLine.computeLineDistances();scene.add(plannedLine);
const targetMarker=new THREE.Mesh(new THREE.SphereGeometry(.14,14,10),new THREE.MeshBasicMaterial({color:0xf5c26b}));
scene.add(targetMarker);
const velocityArrow=new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(),1,0x43efd9,.28,.12);
const thrustArrow=new THREE.ArrowHelper(new THREE.Vector3(0,0,1),new THREE.Vector3(),1,0xffbd69,.3,.15);
scene.add(velocityArrow,thrustArrow);
const waypointObjects=[];
function useRoute(route){
 waypointPositions=route;plannedLine.geometry.dispose();plannedLine.geometry=new THREE.BufferGeometry().setFromPoints(route.map(p=>new THREE.Vector3(...p)));plannedLine.computeLineDistances();
 for(const obj of waypointObjects){scene.remove(obj);obj.geometry.dispose();obj.material.dispose();}waypointObjects.length=0;
 for(const [x,y,z] of route){const m=new THREE.Mesh(new THREE.TorusGeometry(.35,.035,8,24),new THREE.MeshBasicMaterial({color:0xc6a974,transparent:true,opacity:.85}));m.position.set(x,y,z);scene.add(m);waypointObjects.push(m);}
}
let waypointPositions=missionWaypoints;
for(let i=0;i<waypointPositions.length;i++){
 const [x,y,z]=waypointPositions[i];
 const marker=new THREE.Mesh(new THREE.TorusGeometry(.35,.035,8,24),new THREE.MeshBasicMaterial({color:0xc6a974,transparent:true,opacity:.85}));
 marker.position.set(x,y,z);scene.add(marker);waypointObjects.push(marker);
}
let airframe='heavy', mjMass=1.325, armX=.14,armY=.18, momentScale=1, modelThrustMax=13;
let visualInitialized=false;
const pathTracker=new WaypointTracker();let pathError=0,speedReference=0;
let currentPlan=null,previousUrbanGoal=null,urbanMissionCounter=0,missionCitySeed=ACTIVE_CITY_SEED;
let lastHorizontalScan=null;
let sensedObstacles={nearest:Infinity,forward:Infinity,valid:0};let hazard={blocked:false,distance:Infinity};let obstacleHoldTime=0;
let mujoco,model,data,ready=false,paused=false,follow=true,mission=false,waypointIndex=0,flightMode='Standby',lastUi=0,lastLog=0,lastTrace=0,desired=[0,0,.5],lastFrame=performance.now(),accumulator=0;
let wind=.25,missionSpeed=1.5,flightLog=[];
let rotorState=[0,0,0,0];let lastDebug={target:[0,0,.5],positionError:0,altError:0,thrust:0,tilt:0,windForce:0,clearance:Infinity,nearest:'none',contacts:0,controlHz:50,collisionCount:0};
const debugSamples=[];let lastDebugPlot=-1,lastWarning=-50;
const radio=new LiveLink();let lastRadio=-1;
const slam=new PlanarSLAM();const slamLog=[];let lastScan=-1,lastObservableTime=0,slamSeed=23,odomBiasX=.015,odomBiasY=-.012,odomYawBias=.0015,previousYaw=null,slamTruthError=0;
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
function regenerateScenario(seed){
 if(!mujoco)throw Error('Physics engine is not initialized');
 ready=false;paused=true;mission=false;
 const newBuildings=regenerateCity(seed);
 if(cityGroup){
   scene.remove(cityGroup);
   cityGroup.traverse(obj=>{if(obj.isMesh||obj.isInstancedMesh){obj.geometry?.dispose();obj.material?.dispose();}});
 }
 cityGroup=addCityToScene(scene,newBuildings);
 const xml=withCity(withNativeLidar(airframe==='crazyflie'?crazyflieXML:quadXML),newBuildings);
 const nextModel=mujoco.MjModel.from_xml_string(xml);
 if(!nextModel)throw Error('MuJoCo failed to build randomized city');
 model=nextModel;data=new mujoco.MjData(model);
 mujoco.mj_forward(model,data);
 rotorState=Array(4).fill(mjMass*9.81/4);
 desired=[0,0,.5];accumulator=0;
 pathTracker.reset();lastHorizontalScan=null;hazard={blocked:false,distance:Infinity};sensedObstacles={nearest:Infinity,forward:Infinity,valid:0};
 radio.reset();lastRadio=-1;flightLog=[];lastLog=lastUi=lastTrace=0;
 tracePoints.length=0;if(traceLine){scene.remove(traceLine);traceLine.geometry.dispose();traceLine=null;}
 slam.reset();slamLog.length=0;lastScan=-1;previousYaw=null;lastObservableTime=0;
 missionCitySeed=seed;ready=true;paused=false;status('PHYSICS ONLINE');controller();updateUI();
 $('cityStats').textContent=newBuildings.length+' collidable buildings · seed '+seed+' · wide streets and open parks';
 report('Fresh city and MuJoCo collision geometry built from seed '+seed);
}
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
 // GPS-denied guidance uses only the estimated SLAM XY pose, never MuJoCo XY.
 // Roll, pitch and yaw feedback remain simulated IMU/attitude measurements.
 const navSlam=$('navSource').value==='slam';
 const nx=navSlam?slam.pose.x:pos[0],ny=navSlam?slam.pose.y:pos[1];
 if(mission){
 const wp=waypointPositions[waypointIndex],delta=[wp[0]-nx,wp[1]-ny,wp[2]-pos[2]];
 const dist=Math.hypot(...delta);
 const speed3=Math.hypot(vel[0],vel[1],vel[2]);
 if(dist<.5 && speed3<.65){
  if(waypointIndex===waypointPositions.length-1){mission=false;setMode('Mission complete');}
  else goTo(waypointIndex+1);
 }
 const target=waypointPositions[waypointIndex];
 const guidance=pathTracker.update(waypointIndex,waypointPositions,[nx,ny],missionSpeed);
 // Scan a sector centered on the actual travel bearing, not blindly on
 // the drone body X axis: the city route can face any compass direction.
 const bearing=Math.atan2(target[1]-ny,target[0]-nx)-yaw;
 hazard=lastHorizontalScan?scanHazard(lastHorizontalScan,bearing,1.25):{blocked:false,distance:Infinity};
 const rising=target[2]>=3.8 && pos[2]<target[2]-.35;
 // Continuous path targeting: project onto path segment, look ahead,
 // and slow down as the segment end approaches (rather than chase points).
 desired[0]=rising?nx:guidance.target[0];
 desired[1]=rising?ny:guidance.target[1];
 desired[2]=target[2];
 pathError=guidance.crossTrack;
 speedReference=Math.hypot(...guidance.velocity);
 // Rangefinder-only collision supervisor: forward sector blocks movement
 // in the direction of the mission. Safety check on MuJoCo truth remains
 // a separate research oracle and is not fed to guidance.
 if(hazard.blocked && pos[2]>1.6 && !rising && Math.hypot(target[0]-nx,target[1]-ny)>.7){
   mission=false;
   desired=[nx,ny,pos[2]];
   obstacleHoldTime=data.time;
   setMode('LiDAR obstacle brake — hover');
   report('Sensor-based obstacle hold: forward range '+hazard.distance.toFixed(2)+' m. Manual re-plan required.');
 }

 }
 const ax=snap(1.12*(desired[0]-nx)-1.85*vel[0],-1.4,1.4);
 const ay=snap(1.12*(desired[1]-ny)-1.85*vel[1],-1.4,1.4);
 // Rotate world-XY acceleration demand into yaw-relative body axes.
 const bx=Math.cos(yaw)*ax+Math.sin(yaw)*ay;
 const by=-Math.sin(yaw)*ax+Math.cos(yaw)*ay;
 const rollTarget=snap(-by/9.81,-.16,.16);
 const pitchTarget=snap(bx/9.81,-.16,.16);
 // Gravity compensation accounts for the tilted thrust vector.
 const tiltFactor=Math.max(.8,Math.cos(roll)*Math.cos(pitch));
 const t=mjMass*snap((9.81+3.0*(desired[2]-pos[2])-2.9*vel[2])/tiltFactor,1,18);
 const mx=momentScale*snap(.95*(rollTarget-roll)-.32*vel[3],-.55,.55);
 const my=momentScale*snap(.95*(pitchTarget-pitch)-.32*vel[4],-.55,.55);
 const xy=airframe==='crazyflie'?[[armX,armY],[armX,-armY],[-armX,-armY],[-armX,armY]]:
   [[-armX,-armY],[-armX,armY],[armX,armY],[armX,-armY]];
 for(let i=0;i<4;i++){
  const [x,y]=xy[i];
  // Roll/pitch moments allocated from the measured rotor arm distances.
  // This mixer also includes yaw reaction torque for Skydio X2.
  const yawMoment=airframe==='crazyflie'?0:snap(-.01*yaw-.008*vel[5],-.02,.02);
  const yawSign=(i%2===0?-1:1),kYaw=airframe==='crazyflie'?0:.0201;
  const yawTrim=kYaw?yawMoment*yawSign/(4*kYaw):0;
  const demand=snap(t/4+mx*y/(4*armY*armY)-my*x/(4*armX*armX)+yawTrim,0,modelThrustMax);
  // Approximate first-order electronic motor response: 60 ms time constant,
  // updated at 50 Hz. Rotor RPM is NOT explicitly simulated.
  rotorState[i]+=(1-Math.exp(-.02/(airframe==='crazyflie'?.045:.08)))*(demand-rotorState[i]);
  data.ctrl[i]=snap(rotorState[i],0,modelThrustMax);
 }
 // The free body has body id 1. External force applied at its COM.
 if(data.xfrc_applied?.length>=12){
  // Force applied in world axes: wind disturbance + linear aero drag.
  // Drag coefficient is an uncalibrated approximation, not CFD.
  data.xfrc_applied[6]=wind*(airframe==='crazyflie'?.07:1)*Math.sin(data.time*.6)-.18*mjMass*vel[0];
  data.xfrc_applied[7]=wind*(airframe==='crazyflie'?.07:1)*Math.cos(data.time*.43)-.18*mjMass*vel[1];
  data.xfrc_applied[8]=-.12*mjMass*vel[2];
 }
 const clearance=obstacleClearance(pos[0],pos[1],pos[2],airframe==='crazyflie'?.08:.32);
 lastDebug={target:[...desired],positionError:Math.hypot(desired[0]-nx,desired[1]-ny),altError:desired[2]-pos[2],thrust:t,tilt:Math.hypot(roll,pitch)*180/Math.PI,
  windForce:wind*(airframe==='crazyflie'?.07:1),clearance:clearance.distance,nearest:clearance.nearest,contacts:data.ncon,controlHz:50,collisionCount:lastDebug.collisionCount};
 if(mission && navSlam && data.time-lastObservableTime>12){
   mission=false;desired=[nx,ny,Math.max(pos[2],3.2)];
   setMode('Localization lost — hold');
   report('GPS-denied navigation stopped: no usable LiDAR landmarks for over 12 simulated seconds.');
 }else if(mission && (clearance.distance<.25 || (data.ncon>0 && pos[2]>1.0))){
   mission=false;
   desired=[pos[0],pos[1],Math.max(pos[2],transitAltitude)];
   setMode('Safety hold — clearance breach');
   report('Mission aborted: '+clearance.nearest+' clearance '+clearance.distance.toFixed(2)+' m; physical contacts '+data.ncon);
 }else if(mission && clearance.warning && data.time-lastWarning>2){
   lastWarning=data.time;report('Close approach: '+clearance.nearest+' clearance '+clearance.distance.toFixed(2)+' m');
 }
}
function appendLog(){
 const p=data.qpos,v=data.qvel;
 flightLog.push([data.time,...Array.from(p.slice(0,7)),...Array.from(v.slice(0,3)),...Array.from(data.ctrl.slice(0,4)),
   ...lastDebug.target,lastDebug.positionError,lastDebug.altError,lastDebug.tilt,lastDebug.clearance,data.ncon,
   radio.summary(data.time).rssi??'',radio.sf,radio.generated,radio.received,radio.summary(data.time).aoi]);
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
 const rs=radio.summary(t);
 $('radioPdr').textContent=rs.pdr.toFixed(1)+'%';
 $('radioAoi').textContent=rs.aoi.toFixed(1)+' s';
 $('radioAlerts').textContent=rs.alerts;
 $('radioQueue').textContent=String(rs.queue);
 $('rfAirtime').textContent=rs.txAirtime.toFixed(1)+' s';
 $('rfEnergy').textContent=rs.txEnergy.toFixed(3)+' J';
 $('rfDrops').textContent=rs.lost+' / '+rs.expired;
 $('rfDeadline').textContent=rs.alertDeadlineRatio===null?'—':rs.alertDeadlineRatio.toFixed(0)+'%';
 $('rfShadow').textContent='Simulated '+(rs.obstructed?'NLOS obstacle blockage':'LOS')+' · range '+rs.distance.toFixed(1)+' m. RSSI shadowing stochastic; derived PDR and AoI are model outputs, not physical measurements.';
 drawMissionMap([x,y,z],rs.obstructed);
 $('linklabel').textContent='Live estimated RSSI '+(rs.rssi?.toFixed(1)??'—')+' dBm · SF'+rs.sf+' · margin '+(Number.isFinite(rs.margin)?rs.margin.toFixed(1):'—')+' dB. Browser channel abstraction, not radio hardware.';
 $('pathError').textContent=pathError.toFixed(2)+' m';
 $('planRF').textContent=(predictCoverage([x,y,z]).p*100).toFixed(0)+'%';
 $('waypoints').textContent=mission?'Active waypoint '+(waypointIndex+1)+' / '+waypointPositions.length:'Mission inactive • '+waypointPositions.length+' waypoints available';
 $('scanBeams').textContent=slam.lastScan?slam.lastScan.valid+'/'+slam.lastScan.ranges.length:'—';
 $('slamScans').textContent=String(slam.scanCount);
 $('slamError').textContent=slamTruthError.toFixed(2)+' m';
 $('slamCorrections').textContent=String(slam.matches);
 $('obstacleStatus').textContent='Horizontal LiDAR: '+(Number.isFinite(hazard.distance)?hazard.distance.toFixed(2)+'m':'clear')+' forward · down-tilted roof scan '+(Number.isFinite(sensedObstacles.nearest)?sensedObstacles.nearest.toFixed(2)+'m':'clear')+' · '+sensedObstacles.valid+' returns. '+(hazard.blocked?'BRAKE REQUEST':'No frontal hazard');
 $('cityStats').textContent=CITY_BUILDINGS.length+' collidable city buildings · seed '+missionCitySeed+' · wide avenues and parks';
 $('slamStatus').textContent=slam.lastHits<4?'LOW OBSERVABILITY: horizontal 2D scan sees too few nearby surfaces at current altitude. Odometry drift can accumulate.':'Mapping returns '+slam.lastHits+' / '+(slam.lastScan?.ranges.length??0)+', local scan match score '+slam.lastFitness.toFixed(2)+', map coverage '+slam.occupancyPercentage().toFixed(1)+'%.';
 drawSLAMMap();
 $('xpos').textContent=x.toFixed(2);$('ypos').textContent=y.toFixed(2);$('zpos').textContent=z.toFixed(2);
 $('desiredAlt').textContent=lastDebug.target[2].toFixed(2)+' m';
 $('altError').textContent=lastDebug.altError.toFixed(2)+' m';
 $('tiltDeg').textContent=lastDebug.tilt.toFixed(1)+'°';
 $('clearance').textContent=lastDebug.clearance.toFixed(2)+' m';
 $('contacts').textContent=String(data.ncon);
 $('controllerState').textContent='50 Hz position / velocity → attitude PD → 4 rotor thrusts. Desired XYZ: '+lastDebug.target.map(n=>n.toFixed(2)).join(', ')+' m; position error '+lastDebug.positionError.toFixed(2)+' m. Wind force '+lastDebug.windForce.toFixed(3)+' N.';
 const rp=rs.packet;
 $('packetState').textContent=rp?(rp.expired?'EXPIRED':rp.ok?'RECEIVED':'LOST')+' · '+(rp.alert?'critical alert':'telemetry')+' · SF'+(rp.sf??rs.sf)+' · airtime '+((rp.toa??0)*1000).toFixed(1)+' ms':'No packets processed yet';
 $('rfParams').textContent='Tx 14 dBm · BW 125 kHz · coding rate 4/5 · SF '+rs.sf+' · log-distance exponent 4 · correlated shadow σ 4 dB · obstruction loss 17 dB · synthetic competing traffic 10% · routine deadline 8s / alert 3s.';
 if(t-lastDebugPlot>.3){lastDebugPlot=t;debugSamples.push({t,z,ref:lastDebug.target[2],rssi:rs.rssi,aoi:rs.aoi});if(debugSamples.length>120)debugSamples.shift();drawDebugPlot();drawRFPlot();}

}
function drawDebugPlot(){
 const cv=$('debugChart');if(!cv)return;const ctx=cv.getContext('2d');if(!ctx)return;
 const w=cv.width,h=cv.height;ctx.clearRect(0,0,w,h);ctx.fillStyle='#091927';ctx.fillRect(0,0,w,h);
 ctx.strokeStyle='#35556b';ctx.lineWidth=1;for(let i=1;i<4;i++){const y=i*h/4;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();}
 if(debugSamples.length<2)return;
 const xmax=debugSamples.length-1;
 const path=(key,stroke,min,max)=>{ctx.beginPath();ctx.lineWidth=2;ctx.strokeStyle=stroke;debugSamples.forEach((p,i)=>{const n=Number(p[key]);const x=i/xmax*w,y=h-8-(Math.max(min,Math.min(max,n))-min)/(max-min)*(h-16);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.stroke();};
 path('ref','#f1bb78',0,9);path('z','#58d6eb',0,9);
 ctx.fillStyle='#b7cce0';ctx.font='12px sans-serif';ctx.fillText('Altitude: actual blue / target amber',9,17);
}
function drawRFPlot(){
 const cv=$('rfChart');if(!cv)return;const ctx=cv.getContext('2d');if(!ctx)return;
 const w=cv.width,h=cv.height;ctx.fillStyle='#091927';ctx.fillRect(0,0,w,h);
 for(let j=1;j<4;j++){ctx.strokeStyle='#254253';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(0,h*j/4);ctx.lineTo(w,h*j/4);ctx.stroke();}
 if(debugSamples.length<2)return;
 const curve=(key,min,max,color)=>{ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();debugSamples.forEach((p,i)=>{const n=p[key]??min,x=i/(debugSamples.length-1)*w,y=h-8-(Math.max(min,Math.min(max,n))-min)/(max-min)*(h-16);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.stroke();};
 curve('rssi',-150,-60,'#61d6f0');curve('aoi',0,20,'#f7bb67');
 ctx.font='12px sans-serif';ctx.fillStyle='#b7cce0';ctx.fillText('RSSI blue (-150..-60 dBm) · AoI amber (0..20s)',9,17);
}
function drawSLAMMap(){
 const cv=$('slamMap');if(!cv)return;
 const ctx=cv.getContext('2d');if(!ctx)return;const w=cv.width,h=cv.height;
 ctx.fillStyle='#0d2231';ctx.fillRect(0,0,w,h);
 const s=slam.size,r=slam.resolution,scale=Math.min(w,h)/(s*r);
 const tx=x=>w/2+x*scale,ty=y=>h/2-y*scale;
 for(let gy=0;gy<s;gy++)for(let gx=0;gx<s;gx++){
  const value=slam.map[gy*s+gx];if(Math.abs(value)<.22)continue;
  ctx.fillStyle=value>0?'rgba(239,178,103,'+(Math.min(.95,.2+value*.25))+')':'rgba(72,121,145,'+(Math.min(.55,.07-value*.1))+')';
  const x=(gx-s/2)*r,y=(gy-s/2)*r;
  ctx.fillRect(tx(x),ty(y+r),Math.max(1.2,scale*r),Math.max(1.2,scale*r));
 }
 ctx.strokeStyle='#5ed7ea';ctx.lineWidth=1.5;ctx.beginPath();
 for(let i=0;i<slam.path.length;i++){
  const p=slam.path[i];if(i===0)ctx.moveTo(tx(p.x),ty(p.y));else ctx.lineTo(tx(p.x),ty(p.y));
 }ctx.stroke();
 ctx.fillStyle='#e6f2ff';ctx.beginPath();ctx.arc(tx(slam.pose.x),ty(slam.pose.y),5,0,2*Math.PI);ctx.fill();
 ctx.font='12px sans-serif';ctx.fillStyle='#c0d9ed';ctx.fillText('Occupancy grid: blue free · amber occupied · white pose',8,17);
}
function drawMissionMap(pos,obstructed){
 const cv=$('missionMap');if(!cv)return;const ctx=cv.getContext('2d');if(!ctx)return;
 const w=cv.width,h=cv.height,pad=24;
 const way=waypointPositions;
 const minx=Math.min(-12,pos[0],...way.map(p=>p[0]))-4;
 const maxx=Math.max(12,pos[0],...way.map(p=>p[0]))+4;
 const miny=Math.min(-12,pos[1],...way.map(p=>p[1]))-4;
 const maxy=Math.max(12,pos[1],...way.map(p=>p[1]))+4;
 const scale=Math.min((w-2*pad)/(maxx-minx),(h-2*pad)/(maxy-miny));
 const px=x=>pad+(x-minx)*scale,py=y=>h-pad-(y-miny)*scale;
 ctx.fillStyle='#0b2031';ctx.fillRect(0,0,w,h);
 ctx.strokeStyle='#223f52';ctx.lineWidth=1;
 for(let x=Math.ceil(minx/5)*5;x<maxx;x+=5){ctx.beginPath();ctx.moveTo(px(x),0);ctx.lineTo(px(x),h);ctx.stroke();}
 for(let y=Math.ceil(miny/5)*5;y<maxy;y+=5){ctx.beginPath();ctx.moveTo(0,py(y));ctx.lineTo(w,py(y));ctx.stroke();}
 for(const o of [...obstacles,...CITY_BUILDINGS]){
  if(o.x+o.hx<minx||o.x-o.hx>maxx||o.y+o.hy<miny||o.y-o.hy>maxy)continue;
  ctx.fillStyle='#8b4854';ctx.globalAlpha=.8;
  ctx.fillRect(px(o.x-o.hx),py(o.y+o.hy),o.hx*2*scale,o.hy*2*scale);
 }
 ctx.globalAlpha=1;ctx.strokeStyle=currentPlan?.method==='rf_aware'?'#79e1ab':'#d8a759';ctx.lineWidth=2;
 ctx.setLineDash([5,4]);ctx.beginPath();
 way.forEach((p,i)=>i===0?ctx.moveTo(px(p[0]),py(p[1])):ctx.lineTo(px(p[0]),py(p[1])));ctx.stroke();ctx.setLineDash([]);
 ctx.strokeStyle=obstructed?'#ffac77':'#61ddeb';ctx.lineWidth=1;ctx.beginPath();
 ctx.moveTo(px(pos[0]),py(pos[1]));ctx.lineTo(px(Math.max(minx,Math.min(maxx,230))),py(Math.max(miny,Math.min(maxy,220))));ctx.stroke();
 ctx.fillStyle='#6adfff';ctx.beginPath();ctx.arc(px(pos[0]),py(pos[1]),5,0,Math.PI*2);ctx.fill();
 ctx.strokeStyle='#fff';ctx.beginPath();ctx.arc(px(pos[0]),py(pos[1]),8,0,Math.PI*2);ctx.stroke();
 ctx.fillStyle='#b9d2e5';ctx.font='12px sans-serif';ctx.fillText('UAV z '+pos[2].toFixed(1)+' m · WP '+(waypointIndex+1)+'/'+way.length,8,17);
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
 const elapsed=Math.min(.2,(now-lastFrame)/1000);lastFrame=now;
 if(ready&&!paused){
  accumulator=Math.min(.2,accumulator+elapsed);
  let steps=0;
  while(accumulator>=simStep&&steps<50){
   if(Math.floor(data.time/.02)!==Math.floor((data.time+simStep)/.02))controller();
   mujoco.mj_step(model,data);
   accumulator-=simStep;steps++;
  }
  if(data.time-lastScan>=.5){
    lastScan=data.time;
    const p=data.qpos,v=data.qvel;
    const trueYaw=yawOfQuaternion(Array.from(p.slice(3,7)));
    const scan=nativeScan(data);
    lastHorizontalScan=scan;
    sensedObstacles=obstacleScan(data);
    if(scan.valid>=3)lastObservableTime=data.time;
    // A noisy body-frame velocity/gyro sensor proxy. This intentionally
    // injects bias and does not expose ground-truth position to the estimator.
    const vx=Math.cos(trueYaw)*v[0]+Math.sin(trueYaw)*v[1];
    const vy=-Math.sin(trueYaw)*v[0]+Math.cos(trueYaw)*v[1];
    const dt=previousYaw===null?0:Math.max(.001,data.time-(slam.lastT??data.time));
    const gyro=previousYaw===null?0:Math.atan2(Math.sin(trueYaw-previousYaw),Math.cos(trueYaw-previousYaw))/Math.max(.5,dt);
    previousYaw=trueYaw;
    slam.update(data.time,{bodyVx:vx+odomBiasX+.025*Math.sin(data.time*7.2),bodyVy:vy+odomBiasY+.023*Math.cos(data.time*5.8),yawRate:gyro+odomYawBias},scan);
    slamTruthError=slam.evaluate([p[0],p[1]]);
    slamLog.push({t:Number(data.time.toFixed(3)),range_min:.08,range_max:scan.maxRange,
      angle_min:-Math.PI,angle_increment:2*Math.PI/scan.ranges.length,
      ranges:scan.ranges.map(v=>Number(v.toFixed(3))),
      odom:{x:Number(slam.odomPose.x.toFixed(4)),y:Number(slam.odomPose.y.toFixed(4)),yaw:Number(slam.odomPose.yaw.toFixed(5))},
      estimated:{x:Number(slam.pose.x.toFixed(4)),y:Number(slam.pose.y.toFixed(4)),yaw:Number(slam.pose.yaw.toFixed(5))}});
    if(slamLog.length>12000)slamLog.shift();
  }
  if(data.time-lastRadio>=.5){
    lastRadio=data.time;
    radio.tick(data.time,[data.qpos[0],data.qpos[1],data.qpos[2]],$('radioMode').value,2,Number($('sf').value.slice(2)));
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
  targetMarker.position.set(...lastDebug.target);
  const velVec=new THREE.Vector3(data.qvel[0],data.qvel[1],data.qvel[2]);
  const norm=Math.max(.001,velVec.length());
  velocityArrow.position.set(p[0],p[1],p[2]);
  velocityArrow.setDirection(velVec.multiplyScalar(1/norm));
  velocityArrow.setLength(Math.max(.05,Math.min(2,norm*.6)),.2,.1);
  thrustArrow.position.set(p[0],p[1],p[2]);
  thrustArrow.setDirection(new THREE.Vector3(0,0,1).applyQuaternion(drone.quaternion).normalize());
  thrustArrow.setLength(Math.max(.05,Math.min(2.2,lastDebug.thrust/(mjMass*9.81))),.24,.12);
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
on('mission',()=>{
 if(!ready)return;
 const kind=$('missionType').value;
 if($('navSource').value==='slam' && kind!=='mapping'){
   $('navSource').value='truth';
   report('A* requires map-known reference navigation: automatically switched away from experimental 2D GPS-denied SLAM. This is not GPS-free flight.');
 }
 const isAStar=kind==='astar_shortest'||kind==='astar_rf';
 let route=kind==='mapping'?mappingWaypoints:missionWaypoints;
 if(isAStar){
   try{
      // New urban blocks and new destination for EVERY click. First reset
      // MuJoCo with the same collision geometry the A* planner observes.
      const seed=Math.floor(Math.random()*0x100000000)>>>0;
      regenerateScenario(seed);
      const goal=randomUrbanGoal(previousUrbanGoal);
      currentPlan=planInteractiveRoute(kind==='astar_rf'?'rf_aware':'shortest',airframe==='crazyflie'?.08:.32,{goal});
      previousUrbanGoal=goal;urbanMissionCounter++;
      route=currentPlan.route;
      $('missionIdentity').textContent='Flight #'+urbanMissionCounter+' · city seed '+seed+' · goal '+goal.map(v=>v.toFixed(0)).join(', ')+' m';
      $('plannerStats').textContent='NEW randomized '+currentPlan.method+' mission #'+urbanMissionCounter+'; goal '+currentPlan.goal.join(', ')+'m · '+currentPlan.expanded+' A* expansions. City and physics colliders regenerated together.';
      $('planDistance').textContent=currentPlan.length_m.toFixed(1)+' m';
      $('planClearance').textContent=currentPlan.clearance.toFixed(2)+' m';
      $('planRF').textContent=(currentPlan.coverage.reduce((a,b)=>a+b,0)/currentPlan.coverage.length*100).toFixed(0)+'%';
      $('plannerStatus').textContent='ROUTE READY: automatically climbing to 4 m, then following a new collision-checked path.';
      paused=false;status('PHYSICS ONLINE');
      

   }catch(error){
      $('plannerStatus').textContent='A* failed: '+error.message;report('A* planner refused mission: '+error.message);return;
   }
 }else{
   currentPlan=null;$('planDistance').textContent='—';$('planClearance').textContent='—';
 }
 useRoute(route);
 if(!isAStar && plannedPathClearance(waypointPositions,airframe==='crazyflie'?.08:.32)<.5){
   report('Mission refused: clearance envelope violates 0.5 m threshold.');return;
 }
 hazard={blocked:false,distance:Infinity};mission=true;pathTracker.reset();goTo(0);
 if(isAStar){
   setMode(currentPlan.method==='rf_aware'?'RF-aware A* city mission':'Shortest-safe A* city mission');
   report('Running randomized map-known 3D route '+currentPlan.length_m.toFixed(1)+' m. City seed '+missionCitySeed+'; ground-truth position is used for this A* demonstration.');
 }else if($('missionType').value==='mapping'){
   setMode('GPS-denied landmark survey');
   report('3.2 m mapping survey: LiDAR returns from vertical landmarks support occupancy mapping. Navigation source: '+$('navSource').value+'.');
 }else{
   setMode('Collision-aware elevated mission');
   report('Climb to '+transitAltitude.toFixed(1)+' m above obstacle roofs before cross-site travel.');
 }
});
on('pause',()=>{if(!ready)return;paused=!paused;$('pause').textContent=paused?'▶ Resume':'Ⅱ Pause';status(paused?'PAUSED':'PHYSICS ONLINE',paused?'#f0b764':'#63eda1');});
on('camera',()=>{follow=!follow;report(follow?'Camera following UAV':'Free orbit camera');});
on('cityCamera',()=>{
 follow=false;camera.position.set(52,-65,57);orbit.target.set(0,0,8);orbit.update();
 report('City overview: inspecting the physically collidable urban canyon. Use OrbitControls to zoom into streets.');
});
on('reset',()=>{if(!ready)return;pathTracker.reset();lastHorizontalScan=null;hazard={blocked:false,distance:Infinity};mujoco.mj_resetData(model,data);mujoco.mj_forward(model,data);rotorState=Array(4).fill(mjMass*9.81/4);paused=false;accumulator=0;mission=false;currentPlan=null;desired=[0,0,.5];waypointIndex=0;flightLog=[];slamLog.length=0;slam.reset();previousYaw=null;lastScan=-1;lastObservableTime=0;debugSamples.length=0;lastDebugPlot=-1;radio.reset();lastRadio=-1;tracePoints.length=0;if(traceLine){scene.remove(traceLine);traceLine.geometry.dispose();traceLine=null;}lastLog=lastUi=lastTrace=0;$('pause').textContent='Ⅱ Pause';setMode('Reset / standby');status('PHYSICS ONLINE');updateUI();});
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
on('resetSlam',()=>{slam.reset();slamLog.length=0;previousYaw=null;lastScan=-1;lastObservableTime=0;report('GPS-denied mapping map and pose estimate reset.');});
$('navSource').addEventListener('change',()=>{report('Horizontal navigation input: '+($('navSource').value==='slam'?'GPS-denied scan-matched odometry':'simulator ground truth'));});
on('alert',()=>{if(!ready)return;radio.sendAlert(data.time);report('Injected 3-second critical radio alert');});
$('radioMode').addEventListener('change',()=>{if(!ready)return;radio.reset();lastRadio=-1;report('Reset radio emulator: '+$('radioMode').selectedOptions[0].textContent);});
on('exportSlam',()=>{
 if(!slamLog.length){report('No LiDAR/odometry scan frames to export');return;}
 const blob=new Blob([slamLog.map(s=>JSON.stringify(s)).join('\n')+'\n'],{type:'application/x-ndjson'});
 const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='mars-lora-lidar-odom.jsonl';a.click();
 setTimeout(()=>URL.revokeObjectURL(a.href),1000);
 report('Exported '+slamLog.length+' native MuJoCo LiDAR scans with drifty odometry for ROS 2 replay');
});
on('export',()=>{
 if(!flightLog.length){report('No flight trace to export');return;}
 const header='time_s,x_m,y_m,z_m,qw,qx,qy,qz,vx_mps,vy_mps,vz_mps,rotor1_N,rotor2_N,rotor3_N,rotor4_N,target_x,target_y,target_z,xy_error_m,z_error_m,tilt_deg,obstacle_clearance_m,contact_count,rssi_dbm,sf,generated,received,aoi_s';
 const blob=new Blob([header+'\n'+flightLog.map(r=>r.join(',')).join('\n')],{type:'text/csv'});
 const link=document.createElement('a');link.href=URL.createObjectURL(blob);link.download='mars-lora-mujoco-flight.csv';link.click();setTimeout(()=>URL.revokeObjectURL(link.href),1000);
 report('Exported '+flightLog.length+' physics samples');
});
async function initialize(){
 try{
  status('LOADING MUJOCO WASM','#f0b764');
  mujoco=await loadMujoco({locateFile:file=>file.endsWith('.wasm')?wasmUrl:file});
  const xml=withCity(withNativeLidar(airframe==='crazyflie'?crazyflieXML:quadXML));
  model=mujoco.MjModel.from_xml_string(xml);
  if(!model)throw Error('MuJoCo MJCF compilation returned no model');
  data=new mujoco.MjData(model);
  ({mass_kg:mjMass,rotor_x_m:armX,rotor_y_m:armY,max_rotor_thrust_N:modelThrustMax}=PHYSICAL_AIRFRAMES[airframe]);
  momentScale=airframe==='crazyflie'?(2.3951e-5/.019):1;
  drone.scale.setScalar(1);
  mujoco.mj_forward(model,data);
  rotorState=[mjMass*9.81/4,mjMass*9.81/4,mjMass*9.81/4,mjMass*9.81/4];ready=true;desired=[0,0,.5];controller();updateUI();
  status('PHYSICS ONLINE');
  $('help').hidden=true;
  report('MuJoCo WebAssembly engine initialized, model compiled ('+model.nbody+' bodies).');
  report('Use Take off, waypoint mission, or manual controls.');
  $('cadState').textContent='Loading original CAD mesh…';
  loadAirframeCAD(drone,airframe,rotors,report)
    .then(()=>$('cadState').textContent='CAD READY: original '+(airframe==='crazyflie'?'Crazyflie 2':'Skydio X2')+' visual mesh')
    .catch(e=>{$('cadState').textContent='CAD LOAD FAILED: '+e.message;report('CAD load fallback: '+e.message);});
 }catch(error){
  console.error(error);status('PHYSICS FAILED','#ff6f81');
  $('help').hidden=false;$('help').textContent='Unable to load MuJoCo WebAssembly: '+error.message+'. Check browser WebGL/WebAssembly support and refresh.';
  report('Initialization error: '+error.message);
 }
}
$('airframe').addEventListener('change',async e=>{
 if(!mujoco)return;
 paused=true;ready=false;airframe=e.target.value;pathTracker.reset();lastHorizontalScan=null;hazard={blocked:false,distance:Infinity};
 mission=false;currentPlan=null;flightLog=[];rotorState=[0,0,0,0];slamLog.length=0;slam.reset();previousYaw=null;lastScan=-1;debugSamples.length=0;lastDebugPlot=-1;radio.reset();lastRadio=-1;tracePoints.length=0;if(traceLine){scene.remove(traceLine);traceLine.geometry.dispose();traceLine=null;}
 model=null;data=null;lastLog=lastUi=lastTrace=0;accumulator=0;
 status('RECOMPILING AIRFRAME','#f0b764');
 try{
  model=mujoco.MjModel.from_xml_string(withCity(withNativeLidar(airframe==='crazyflie'?crazyflieXML:quadXML)));
  loadAirframeCAD(drone,airframe,rotors,report).catch(e=>report('CAD load fallback: '+e.message));
  data=new mujoco.MjData(model);
  ({mass_kg:mjMass,rotor_x_m:armX,rotor_y_m:armY,max_rotor_thrust_N:modelThrustMax}=PHYSICAL_AIRFRAMES[airframe]);
  momentScale=airframe==='crazyflie'?(2.3951e-5/.019):1;
  drone.scale.setScalar(1);
  mujoco.mj_forward(model,data);rotorState=Array(4).fill(mjMass*9.81/4);ready=true;paused=false;desired=[0,0,.5];controller();updateUI();
  setMode('Standby');status('PHYSICS ONLINE');report('Loaded '+(airframe==='crazyflie'?'Crazyflie 2 27 g':'Skydio X2 1.325 kg')+' airframe.');
 }catch(err){console.error(err);status('MODEL LOAD FAILED','#f87171');$('help').hidden=false;$('help').textContent=err.message;}
});
initialize();
