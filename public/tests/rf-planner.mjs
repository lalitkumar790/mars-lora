import assert from 'node:assert/strict';
import {planInteractiveRoute,checkRoute,predictCoverage,randomUrbanGoal} from '../src/rf_route_planner.js';
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
