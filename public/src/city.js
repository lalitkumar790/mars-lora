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
 const group=new THREE.Group();
 const windowTransforms=[];
 const windowGeometry=new THREE.PlaneGeometry(.86,.94);
 const materials=[
  new THREE.MeshStandardMaterial({color:0x46576a,roughness:.82}),
  new THREE.MeshStandardMaterial({color:0x536a78,roughness:.75}),
  new THREE.MeshStandardMaterial({color:0x7b726e,roughness:.8}),
  new THREE.MeshStandardMaterial({color:0x6b8991,roughness:.74})
 ];
 for(const b of CITY_BUILDINGS){
  const index=Math.floor(b.kind*materials.length);
  const mesh=new THREE.Mesh(new THREE.BoxGeometry(2*b.hx,2*b.hy,2*b.hz),materials[Math.min(index,3)]);
  mesh.position.set(b.x,b.y,b.z);mesh.castShadow=true;mesh.receiveShadow=true;group.add(mesh);
  const roof=new THREE.Mesh(new THREE.BoxGeometry(2*b.hx+.2,2*b.hy+.2,.15),new THREE.MeshStandardMaterial({color:0x263949,roughness:.85}));
  roof.position.set(b.x,b.y,b.z+b.hz+.07);group.add(roof);
  const floors=Math.min(12,Math.floor((2*b.hz)/2.5));
  for(let level=1;level<=floors;level++){
   const z=level*2.4;
   for(const side of [-1,1])for(let slot=-1;slot<=1;slot++){
    windowTransforms.push([b.x+slot*(b.hx*.68),b.y+side*(b.hy+.006),z,side]);
   }
  }
 }
 const winMat=new THREE.MeshBasicMaterial({color:0x93b9c6,transparent:true,opacity:.28,side:THREE.DoubleSide});
 const windows=new THREE.InstancedMesh(windowGeometry,winMat,windowTransforms.length);
 const obj=new THREE.Object3D();
 for(let i=0;i<windowTransforms.length;i++){
   const [x,y,z,side]=windowTransforms[i];
   obj.position.set(x,y,z);obj.rotation.set(side===1?Math.PI/2:-Math.PI/2,0,0);
   obj.updateMatrix();windows.setMatrixAt(i,obj.matrix);
 }
 windows.instanceMatrix.needsUpdate=true;group.add(windows);
 // Streets are visual surfaces only; MuJoCo has a continuous ground plane.
 const roadMat=new THREE.MeshStandardMaterial({color:0x1c3039,roughness:.98});
 const stripeMat=new THREE.MeshBasicMaterial({color:0xa6aa91});
 for(let coord=-4;coord<=4;coord++){
  for(const axis of ['x','y']){
   const road=new THREE.Mesh(new THREE.PlaneGeometry(axis==='x'?116:5,axis==='x'?5:116),roadMat);
   road.position[axis==='x'?'y':'x']=coord*12.2+6.1;road.position.z=.012;
   group.add(road);
   for(let d=-52;d<=52;d+=5){
    const stripe=new THREE.Mesh(new THREE.PlaneGeometry(axis==='x'?2.1:.1,axis==='x'?.1:2.1),stripeMat);
    if(axis==='x'){stripe.position.set(d,coord*12.2+6.1,.016);}
    else stripe.position.set(coord*12.2+6.1,d,.016);
    group.add(stripe);
   }
  }
 }
 scene.add(group);
 return group;
}
