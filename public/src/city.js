/** City-scale urban canyon benchmark. This is synthetic, not a GIS import.
 * One deterministic building list drives MuJoCo colliders and Three.js render.
 * Central industrial mission zone (-12..12) stays free of added city solids.
 */
import * as THREE from 'three';
export const CITY_SEED=37021;
export function cityBuildings(seed=CITY_SEED){
 let state=seed>>>0;
 const rand=()=>((state=(1664525*state+1013904223)>>>0)/4294967296);
 const buildings=[];
 for(let ix=-4;ix<=4;ix++){
  for(let iy=-4;iy<=4;iy++){
   if(ix===0&&iy===0)continue;
   if(Math.abs(ix)<=1 && Math.abs(iy)<=1)continue;
   const x=ix*12.2+(rand()-.5)*1.9;
   const y=iy*12.2+(rand()-.5)*1.9;
   // Main roads between blocks and the two primary central avenues.
   if(Math.abs(x)<13&&Math.abs(y)<13)continue;
   const sx=3+rand()*1.4,sy=3.0+rand()*1.6;
   const height=6+rand()*17;
   buildings.push({name:'city_'+ix+'_'+iy,x,y,z:height/2,hx:sx,hy:sy,hz:height/2,kind:rand()});
  }
 }
 return buildings;
}
export const CITY_BUILDINGS=cityBuildings();
export function withCity(xml,enabled=true){
 if(!enabled)return xml;
 const mjcf=CITY_BUILDINGS.map(b=>
  '<geom name="'+b.name+'" type="box" pos="'+[b.x,b.y,b.z].join(' ')+'" size="'+[b.hx,b.hy,b.hz].join(' ')+'" rgba="'+(.2+b.kind*.2)+' '+(.27+b.kind*.1)+' '+(.38+b.kind*.15)+' 1" friction=".9 .03 .003"/>'
 ).join('\n');
 return xml.replace('</worldbody>',mjcf+'\n</worldbody>');
}
export function addCityToScene(scene){
 const group=new THREE.Group(),matrix=new THREE.Object3D();
 const buildingMaterial=new THREE.MeshStandardMaterial({color:0xffffff,roughness:.8});
 const buildings=new THREE.InstancedMesh(new THREE.BoxGeometry(1,1,1),buildingMaterial,CITY_BUILDINGS.length);
 const roofs=new THREE.InstancedMesh(new THREE.BoxGeometry(1,1,1),new THREE.MeshStandardMaterial({color:0x263949}),CITY_BUILDINGS.length);
 const windowMatrices=[];
 for(let i=0;i<CITY_BUILDINGS.length;i++){
  const b=CITY_BUILDINGS[i];
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
   matrix.position.set(axis==='x'?0:coord*12.2+6.1,axis==='x'?coord*12.2+6.1:0,.012);
   matrix.rotation.set(0,0,0);matrix.scale.set(axis==='x'?116:5,axis==='x'?5:116,1);matrix.updateMatrix();
   roads.setMatrixAt(i,matrix.matrix);
   for(let d=-52;d<=52;d+=5)stripesData.push([axis,d,coord*12.2+6.1]);
 }
 roads.instanceMatrix.needsUpdate=true;group.add(roads);
 const stripes=new THREE.InstancedMesh(new THREE.PlaneGeometry(1,1),new THREE.MeshBasicMaterial({color:0xa6aa91}),stripesData.length);
 stripesData.forEach(([axis,d,road],i)=>{
  matrix.position.set(axis==='x'?d:road,axis==='x'?road:d,.016);
  matrix.rotation.set(0,0,0);matrix.scale.set(axis==='x'?2.1:.1,axis==='x'?.1:2.1,1);matrix.updateMatrix();stripes.setMatrixAt(i,matrix.matrix);
 });
 stripes.instanceMatrix.needsUpdate=true;group.add(stripes);
 scene.add(group);return group;
}
