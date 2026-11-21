import assert from 'node:assert/strict';
import {planInteractiveRoute,planBuildingSiteMission,checkRoute,predictCoverage,randomUrbanGoal} from '../src/rf_route_planner.js';
const short=planInteractiveRoute('shortest',.32);
const rf=planInteractiveRoute('rf_aware',.32);
for(const plan of [short,rf]){
 assert.deepEqual(plan.route[0],[0,0,4]);
 assert.deepEqual(plan.route.at(-1),[-30,24,4]);
 assert.ok(plan.expanded>2);
 assert.ok(plan.length_m>=54);
 assert.ok(plan.route.length>=3);
 assert.ok(checkRoute(plan.route,.32).safe);
}
assert.ok(short.length_m<=rf.length_m+1e-6);
// Different random cities may have a safe LOS street corridor requiring no climb.
const a=randomUrbanGoal(null,()=>0),b=randomUrbanGoal(a,()=>.9);
assert.notDeepEqual(a,b,'Repeated mission destinations should vary');
const varied=planInteractiveRoute('rf_aware',.32,{goal:b});
assert.ok(checkRoute(varied.route,.32).safe);
assert.ok(predictCoverage([0,0,17]).p>=predictCoverage([0,0,4]).p);
console.log('PASS 3D route comparison; shortest='+short.length_m.toFixed(1)+'m; RF='+rf.length_m.toFixed(1)+'m; clearance='+rf.clearance.toFixed(2)+'m');

const multistop=planBuildingSiteMission('rf_aware',.32,{count:2,random:()=>.73});
assert.equal(multistop.sites.length,2);
assert.equal(multistop.route[0][0],0);
assert.deepEqual(multistop.route.at(-1),[0,0,4]);
assert.ok(multistop.length_m>60);
assert.ok(checkRoute(multistop.route,.32,1.3).safe);
assert.notEqual(multistop.sites[0].building,multistop.sites[1].building);
for(const site of multistop.sites){
 assert.deepEqual(multistop.route[site.waypointIndex],site.position);
 assert.ok(site.waypointIndex>0&&site.waypointIndex<multistop.route.length-1);
}
console.log('PASS multi-building inspection with '+multistop.sites.length+' distinct physical building sites and return to base');
