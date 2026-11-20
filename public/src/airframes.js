import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import * as THREE from 'three';
import skydioOBJ from '../assets/skydio/X2_lowpoly.obj?url';
import cf0 from '../assets/crazyflie/cf2_0.obj?url';
import cf1 from '../assets/crazyflie/cf2_1.obj?url';
import cf2 from '../assets/crazyflie/cf2_2.obj?url';
import cf3 from '../assets/crazyflie/cf2_3.obj?url';
import cf4 from '../assets/crazyflie/cf2_4.obj?url';
import cf5 from '../assets/crazyflie/cf2_5.obj?url';
import cf6 from '../assets/crazyflie/cf2_6.obj?url';

const loader=new OBJLoader();
const crazyflieAssets=[cf0,cf1,cf2,cf3,cf4,cf5,cf6];
const cfColors=[0x191e24,0x1c3940,0xcdb564,0x95a4b2,0x909ba8,0x10171d,0xe2eef0];
let loadNumber=0;
function bladeGroup(x,y,z,r){
  const group=new THREE.Group();group.position.set(x,y,z);
  const hub=new THREE.Mesh(new THREE.CylinderGeometry(r*.13,r*.13,r*.16,16),new THREE.MeshStandardMaterial({color:0x232629,metalness:.55,roughness:.35}));
  hub.rotation.x=Math.PI/2;group.add(hub);
  const material=new THREE.MeshStandardMaterial({color:0x10141a,metalness:.15,roughness:.46,side:THREE.DoubleSide});
  for(let i=0;i<2;i++){
    const blade=new THREE.Mesh(new THREE.BoxGeometry(r*1.85,r*.20,r*.032),material);
    blade.rotation.z=i*Math.PI/2;blade.castShadow=true;group.add(blade);
  }
  return group;
}
export const PHYSICAL_AIRFRAMES={
  crazyflie:{mass_kg:.027,moment_kgm2:[2.3951e-5,2.3951e-5,3.2347e-5],rotor_x_m:.046,rotor_y_m:.046,
    max_rotor_thrust_N:.15,description:'Crazyflie 2 Menagerie: original CAD and mass/inertia (27g); simplified independently-driven motor forces'},
  heavy:{mass_kg:1.325,moment_kgm2:[.019,.019,.035],rotor_x_m:.14,rotor_y_m:.18,
    max_rotor_thrust_N:13,description:'Skydio X2 Menagerie: original CAD and 1.325kg component mass; inertias are approximate'}
};
export async function loadAirframeCAD(drone,airframe,rotors,report=()=>{}){
  const ticket=++loadNumber;
  const group=new THREE.Group();
  if(airframe==='crazyflie'){
    const meshes=await Promise.all(crazyflieAssets.map(url=>loader.loadAsync(url)));
    for(let i=0;i<meshes.length;i++){
      const child=meshes[i];
      child.traverse(obj=>{
        if(!obj.isMesh)return;
        obj.material=new THREE.MeshStandardMaterial({color:cfColors[i],metalness:i===2||i===4?.55:.23,roughness:.48,side:THREE.DoubleSide});
        obj.castShadow=true;obj.receiveShadow=true;
      });
      group.add(child);
    }
    for(const [x,y] of [[.046,.046],[.046,-.046],[-.046,-.046],[-.046,.046]]){
      const rotor=bladeGroup(x,y,.018,.032);group.add(rotor);rotors.push(rotor);
    }
  }else{
    const root=await loader.loadAsync(skydioOBJ);
    root.scale.setScalar(.01); // Original MJCF mesh is centimeters.
    root.quaternion.set(0,1,1,0).normalize(); // Upstream x2.xml visual quaternion
    root.traverse(obj=>{if(!obj.isMesh)return;obj.material=new THREE.MeshStandardMaterial({
      color:0x374351,metalness:.35,roughness:.46,side:THREE.DoubleSide
    });obj.castShadow=true;obj.receiveShadow=true;});
    group.add(root);
    for(const [x,y,z] of [[-.14,-.18,.05],[-.14,.18,.05],[.14,.18,.08],[.14,-.18,.08]]){
      const rotor=bladeGroup(x,y,z,.11);group.add(rotor);rotors.push(rotor);
    }
  }
  if(ticket!==loadNumber){group.traverse(obj=>{if(obj.geometry)obj.geometry.dispose()});return;}
  drone.clear();
  drone.add(group);
  drone.scale.setScalar(1);
  report('Loaded original '+(airframe==='crazyflie'?'Crazyflie 2':'Skydio X2')+' CAD mesh. Physics uses matching mass/rotor layout; motor model is simplified.');
}
