import assert from 'node:assert/strict';
import {planInteractiveRoute,checkRoute,predictCoverage} from '../src/rf_route_planner.js';
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
assert.ok(rf.route.some(p=>p[2]>4),'RF aware route should be able to escape radio-shadowed urban canyon');
assert.ok(predictCoverage([0,0,17]).p>=predictCoverage([0,0,4]).p);
console.log('PASS 3D route comparison; shortest='+short.length_m.toFixed(1)+'m; RF='+rf.length_m.toFixed(1)+'m; clearance='+rf.clearance.toFixed(2)+'m');
