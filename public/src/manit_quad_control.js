// MARS-LoRa heavy (Skydio X2) rotor mixer, adapted for metre-scale GIS missions.
// Original quad.xml: 1.325 kg, 0.14/0.18 m arms, 13 N/rotor, 0.0201 yaw reaction.
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
export function quadControl(data,target,alt,rotorState){
 const p=data.qpos,v=data.qvel,[w,x,y,z]=[p[3],p[4],p[5],p[6]];
 const roll=Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),pitch=Math.asin(clamp(2*(w*y-z*x),-1,1)),yaw=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));
 const dx=target[0]-p[0],dy=target[1]-p[1],d=Math.hypot(dx,dy),speed=Math.min(7,Math.sqrt(2*1.3*d),d*1.1),vx=d>1e-6?dx/d*speed:0,vy=d>1e-6?dy/d*speed:0;
 const ax=clamp(1.6*(vx-v[0]),-2,2),ay=clamp(1.6*(vy-v[1]),-2,2);
 const bx=Math.cos(yaw)*ax+Math.sin(yaw)*ay,by=-Math.sin(yaw)*ax+Math.cos(yaw)*ay;
 const rollTarget=clamp(-by/9.81,-.24,.24),pitchTarget=clamp(bx/9.81,-.24,.24);
 const mass=1.325,t=mass*clamp((9.81+3*(alt-p[2])-2.9*v[2])/Math.max(.8,Math.cos(roll)*Math.cos(pitch)),1,18);
 const mx=clamp(.95*(rollTarget-roll)-.32*v[3],-.55,.55),my=clamp(.95*(pitchTarget-pitch)-.32*v[4],-.55,.55),mz=clamp(-.01*yaw-.008*v[5],-.02,.02);
 const arms=[[-.14,-.18],[-.14,.18],[.14,.18],[.14,-.18]];
 for(let i=0;i<4;i++){const [rx,ry]=arms[i],demand=clamp(t/4+mx*ry/(4*.18**2)-my*rx/(4*.14**2)+mz*(i%2===0?-1:1)/(4*.0201),0,13);rotorState[i]+=(1-Math.exp(-.02/.08))*(demand-rotorState[i]);data.ctrl[i]=clamp(rotorState[i],0,13)}
 if(data.xfrc_applied?.length>=12){data.xfrc_applied[6]=-.18*mass*v[0];data.xfrc_applied[7]=-.18*mass*v[1];data.xfrc_applied[8]=-.12*mass*v[2]}
}
