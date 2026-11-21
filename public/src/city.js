/** City-scale urban canyon benchmark. This is synthetic, not a GIS import.
 * One deterministic building list drives MuJoCo colliders and Three.js render.
 * Central industrial mission zone (-12..12) stays free of added city solids.
 */
import * as THREE from 'three';
export const CITY_SEED=37021;
export let CITY_BUILDINGS=[];
export let ACTIVE_CITY_SEED=CITY_SEED;
export const CITY_SPACING=20;

export function cityBuildings(seed=CITY_SEED){
 let state=seed>>>0;
 const rand=()=>((state=(1664525*state+1013904223)>>>0)/4294967296);
 const buildings=[];
 for(let ix=-4;ix<=4;ix++)for(let iy=-4;iy<=4;iy++){
   // Preserve a generous open airport/launch zone around the central campus.
   if(Math.abs(ix)<=1&&Math.abs(iy)<=1)continue;
   // Every layout has parks, open lots and staggered structures.
   const district=(ix+iy+20)%7;
   if(district===1 || rand()<.17)continue;
   const x=ix*CITY_SPACING+(rand()-.5)*4;
   const y=iy*CITY_SPACING+(rand()-.5)*4;
   const hx=3.6+rand()*2.3,hy=3.4+rand()*2.6;
   const height=6+rand()*14;
   buildings.push({name:'city_'+ix+'_'+iy,x,y,z:height/2,hx,hy,hz:height/2,kind:rand()});
 }
 return buildings;
}
export function regenerateCity(seed=CITY_SEED){
 ACTIVE_CITY_SEED=seed>>>0;
 CITY_BUILDINGS=cityBuildings(ACTIVE_CITY_SEED);
 return CITY_BUILDINGS;
}
regenerateCity(CITY_SEED);
export function withCity(xml,buildings=CITY_BUILDINGS){
 const mjcf=buildings.map(b=>
  '<geom name="'+b.name+'" type="box" pos="'+[b.x,b.y,b.z].join(' ')+'" size="'+[b.hx,b.hy,b.hz].join(' ')+'" rgba="'+(.2+b.kind*.2)+' '+(.27+b.kind*.1)+' '+(.38+b.kind*.15)+' 1" friction=".9 .03 .003"/>'
 ).join('\n');
 return xml.replace('</worldbody>',mjcf+'\n</worldbody>');
}
export function addCityToScene(scene,buildings=CITY_BUILDINGS){
 const group=new THREE.Group(),matrix=new THREE.Object3D();
 const buildingMaterial=new THREE.MeshStandardMaterial({color:0xffffff,roughness:.8});
 const buildings=new THREE.InstancedMesh(new THREE.BoxGeometry(1,1,1),buildingMaterial,buildings.length);
 const roofs=new THREE.InstancedMesh(new THREE.BoxGeometry(1,1,1),new THREE.MeshStandardMaterial({color:0x263949}),buildings.length);
 const windowMatrices=[];
 for(let i=0;i<buildings.length;i++){
  const b=buildings[i];
  matrix.position.set(b.x,b.y,b.z);matrix.scale.set(2*b.hx,2*b.hy,2*b.hz);matrix.rotation.set(0,0,0);matrix.updateMatrix();
  buildings.setMatrixAt(i,matrix.matrix);
  buildings.setColorAt(i,new THREE.Color().setHSL(.57+b.kind*.05,.13,.3+b.kind*.16));
  matrix.position.set(b.x,b.y,b.z+b.hz+.07);matrix.scale.set(2*b.hx+.2,2*b.hy+.2,.15);matrix.updateMatrix();roofs.setMatrixAt(i,matrix.matrix);
  const floors=Math.min(12,Math.floor((2*b.hz)/2.5));
  for(let level=1;level<=floors;level++){
    for(const side of [-1,1])for(let slot=-1;slot<=1;slot++)
      windowMatrices.push([b.x+slot*b.hx*.68,b.y+side*(b.hy+.006),level*2.4,side]);
  }
 }
 buildings.instanceMatrix.needsUpdate=true;
 if(buildings.instanceColor)buildings.instanceColor.needsUpdate=true;
 buildings.castShadow=false;buildings.receiveShadow=true;
 roofs.instanceMatrix.needsUpdate=true;group.add(buildings,roofs);
 const win=new THREE.InstancedMesh(new THREE.PlaneGeometry(.86,.94),
   new THREE.MeshBasicMaterial({color:0x93b9c6,transparent:true,opacity:.35,side:THREE.DoubleSide}),windowMatrices.length);
 windowMatrices.forEach(([x,y,z,side],i)=>{
  matrix.position.set(x,y,z);matrix.rotation.set(side===1?Math.PI/2:-Math.PI/2,0,0);matrix.scale.set(1,1,1);matrix.updateMatrix();win.setMatrixAt(i,matrix.matrix);
 });
 win.instanceMatrix.needsUpdate=true;group.add(win);
 const roads=new THREE.InstancedMesh(new THREE.PlaneGeometry(1,1),
   new THREE.MeshStandardMaterial({color:0x1c3039,roughness:.98}),18);
 const stripesData=[];
 for(let coord=-4;coord<=4;coord++)for(const axis of ['x','y']){
   const i=(coord+4)*2+(axis==='x'?0:1);
   matrix.position.set(axis==='x'?0:coord*CITY_SPACING+CITY_SPACING/2,axis==='x'?coord*12.2+6.1:0,.012);
   matrix.rotation.set(0,0,0);matrix.scale.set(axis==='x'?185:8,axis==='x'?8:185,1);matrix.updateMatrix();
   roads.setMatrixAt(i,matrix.matrix);
   for(let d=-85;d<=85;d+=7stripesData.push([axis,d,coord*12.2+6.1]);
 }
 roads.instanceMatrix.needsUpdate=true;group.add(roads);
 // Landscaped parcels: parks, plazas, walkways and lightweight vegetation.
 // These ornamental meshes intentionally have no MuJoCo contact shape;
 // the geometry used for safety planning is the city buildings above.
 const plazas=new THREE.Group();
 const parkMat=new THREE.MeshStandardMaterial({color:0x2c684b,roughness:1});
 const plazaMat=new THREE.MeshStandardMaterial({color:0x667681,roughness:1});
 const trunkMat=new THREE.MeshStandardMaterial({color:0x5a4939,roughness:1});
 const crownMat=new THREE.MeshStandardMaterial({color:0x3c8761,roughness:1});
 const trunkGeo=new THREE.CylinderGeometry(.11,.16,1.2,7);
 const crownGeo=new THREE.ConeGeometry(.95,2.2,8);
 for(let ix=-3;ix<=3;ix++)for(let iy=-3;iy<=3;iy++){
   if(Math.abs(ix)<=1&&Math.abs(iy)<=1)continue;
   const px=ix*CITY_SPACING,py=iy*CITY_SPACING;
   if(buildings.some(b=>Math.abs(b.x-px)<10&&Math.abs(b.y-py)<10))continue;
   const isPark=(ix+iy+20)%2===0;
   const parcel=new THREE.Mesh(new THREE.PlaneGeometry(12,12),isPark?parkMat:plazaMat);
   parcel.position.set(px,py,.028);plazas.add(parcel);
   if(isPark)for(let t=0;t<4;t++){
      const dx=(t%2?3.3:-3.3),dy=t<2?-3.2:3.2;
      const trunk=new THREE.Mesh(trunkGeo,trunkMat);
      trunk.rotation.x=Math.PI/2;trunk.position.set(px+dx,py+dy,.6);
      const canopy=new THREE.Mesh(crownGeo,crownMat);
      canopy.rotation.x=Math.PI/2;canopy.position.set(px+dx,py+dy,2.1);
      plazas.add(trunk,canopy);
   }
 }
 group.add(plazas);
 const stripes=new THREE.InstancedMesh(new THREE.PlaneGeometry(1,1),new THREE.MeshBasicMaterial({color:0xa6aa91}),stripesData.length);
 stripesData.forEach(([axis,d,road],i)=>{
  matrix.position.set(axis==='x'?d:road,axis==='x'?road:d,.016);
  matrix.rotation.set(0,0,0);matrix.scale.set(axis==='x'?2.1:.1,axis==='x'?.1:2.1,1);matrix.updateMatrix();stripes.setMatrixAt(i,matrix.matrix);
 });
 stripes.instanceMatrix.needsUpdate=true;group.add(stripes);
 scene.add(group);return group;
}
