/** Interactive RAW LoRa abstraction linked to MuJoCo poses.
 * This is a demonstration model, NOT validated field LoRa and NOT LoRaWAN.
 * SF coordination assumed ideal; no MAC regulatory duty-cycle enforcement.
 */
export class LiveLink {
  constructor(seed=2026) { this.seed=seed;this.reset(); }
  reset() {
    this.sf=9;this.next=0;this.busy=0;this.queue=[];this.seq=0;
    this.generated=0;this.received=0;this.onTime=0;this.alerts=0;this.alertOnTime=0;
    this.latestStatus=0;this.lastRSSI=null;this.lastSNR=null;
    this.lastPacket=null;this.lastAck=-10;this.lastAdapt=0;this.failures=0;
    this.shadow=0;this.nextShadow=-1;
  }
  random() { this.seed=(1664525*this.seed+1013904223)>>>0;return this.seed/4294967296; }
  gaussian() {return Math.sqrt(-2*Math.log(Math.max(1e-10,this.random())))*Math.cos(2*Math.PI*this.random());}
  airtime(sf,bytes=20){
    const bw=125000,de=sf>=11?1:0,ts=Math.pow(2,sf)/bw;
    const n=Math.ceil((8*bytes-4*sf+44)/(4*(sf-2*de)));
    return (8+4.25+8+Math.max(n*5,0))*ts;
  }
  blocked(pos) {
    // Simple horizontal ray/AABB crossing, not site-survey RF ray tracing.
    const buildings=[[5,4,1.1,1.4],[-5,5,1.25,1.1],[5,-4,1.4,1.0],[-3,-4,2,.7]];
    const [x,y]=pos, [ox,oy]=[0,-2];
    for(const [bx,by,hx,hy] of buildings) {
      for(let i=1;i<=15;i++) {
        const t=i/16,px=ox+(x-ox)*t,py=oy+(y-oy)*t;
        if(Math.abs(px-bx)<hx && Math.abs(py-by)<hy)return true;
      }
    }
    return false;
  }
  sendAlert(t) {
    this.generated++;this.alerts++;
    this.queue.unshift({created:t,deadline:t+3,alert:true,seq:++this.seq,bytes:12});
  }
  tick(t,xyz,mode='fixed',interval=2) {
    if(t+1e-8>=this.next) {
      this.generated++;
      this.queue.push({created:t,deadline:t+8,alert:false,seq:++this.seq,bytes:20});
      this.next=t+interval;
    }
    const d=Math.max(1,Math.hypot(xyz[0],xyz[1]+2,xyz[2]-2));
    if(t>this.nextShadow) {
      this.shadow=.96*this.shadow+Math.sqrt(1-.96*.96)*this.gaussian()*4;
      this.nextShadow=t+.5;
    }
    const rssi=14-31.2-40*Math.log10(d)-(this.blocked(xyz)?17:0)+this.shadow;
    this.lastRSSI=rssi;
    if(mode==='adaptive' && t-this.lastAdapt>=5){
      this.lastAdapt=t;
      let target=this.sf;
      if(this.lastPacket?.ok){
        target=12;
        for(let s=7;s<=12;s++){if(this.lastPacket.rssi>this.sensitivity(s)+8){target=s;break;}}
      } else if(this.failures>=2)target=Math.min(12,this.sf+1);
      this.sf=Math.max(this.sf-1,Math.min(this.sf+1,target));
    }
    if(t<this.busy||!this.queue.length)return;
    if(mode==='priority')this.queue.sort((a,b)=>Number(b.alert)-Number(a.alert)||a.seq-b.seq);
    const packet=this.queue.shift();
    if(packet.deadline<t){
      this.lastPacket={ok:false,expired:true,alert:packet.alert};
      return;
    }
    const toa=this.airtime(this.sf,packet.bytes), arrival=t+toa;
    this.busy=arrival;
    const margin=rssi-this.sensitivity(this.sf);
    const prob=.9/(1+Math.exp(-Math.max(-50,Math.min(50,margin))/1.8));
    const ok=this.random()<prob;
    this.lastPacket={ok,alert:packet.alert,rssi,sf:this.sf,toa,time:arrival,expired:false};
    if(ok) {
      this.received++;this.lastAck=arrival;this.failures=0;
      if(!packet.alert)this.latestStatus=packet.created;
      if(arrival<=packet.deadline){this.onTime++;if(packet.alert)this.alertOnTime++;}
    } else this.failures++;
  }
  sensitivity(sf) {return {7:-123,8:-126,9:-129,10:-132,11:-134.5,12:-137}[sf];}
  summary(t) {
    return {sf:this.sf,pdr:this.generated?100*this.received/this.generated:100,
      aoi:Math.max(0,t-this.latestStatus),alerts:this.alertOnTime+'/'+this.alerts,
      queue:this.queue.length,rssi:this.lastRSSI,margin:this.lastRSSI-this.sensitivity(this.sf),
      packet:this.lastPacket};
  }
}
