import test from 'node:test';
import assert from 'node:assert/strict';
import { PlanarSLAM } from '../src/slam.js';

test('occupancy measurements remain at their world coordinates after expansion',()=>{
 const slam=new PlanarSLAM({size:20,resolution:.5});
 slam.mark(0,0,2);
 const before=slam.index(0,0);
 assert.equal(slam.map[before],2);
 assert.equal(slam.ensureBounds(24,0,8),true);
 assert.ok(slam.size>20);
 assert.equal(slam.map[slam.index(0,0)],2);
 slam.mark(24,0,1);
 assert.equal(slam.map[slam.index(24,0)],1);
 assert.notEqual(slam.index(24,0),slam.index(0,0));
 assert.ok(slam.bounds().maxX>=32);
});

test('scan integration expands the map while preserving past occupancy',()=>{
 const slam=new PlanarSLAM({size:20,resolution:.5});
 const empty={ranges:[4],angles:[0],maxRange:4,valid:0};
 slam.update(0,{bodyVx:0,bodyVy:0,yawRate:0},empty);
 slam.mark(1,1,2);
 slam.update(10,{bodyVx:50,bodyVy:0,yawRate:0},empty);
 assert.ok(slam.size>20);
 assert.ok(slam.index(slam.pose.x,slam.pose.y)>=0);
 assert.equal(slam.map[slam.index(1,1)],2);
});

test('reset restores initial size, frame, path and unknown map',()=>{
 const slam=new PlanarSLAM({size:20,resolution:.5});
 slam.mark(0,0,1);
 slam.ensureBounds(35,35,1);
 slam.reset();
 assert.equal(slam.size,20);
 assert.equal(slam.originX,-5);
 assert.equal(slam.map.length,400);
 assert.equal(slam.map[slam.index(0,0)],0);
 assert.equal(slam.expansions,0);
 assert.equal(slam.mapLimitReached,false);
 assert.equal(slam.path.length,0);
});

test('unbounded drift cannot grow map indefinitely',()=>{
 const slam=new PlanarSLAM();
 const result=slam.ensureBounds(1000000,1000000,1);
 assert.equal(result,false);
 assert.ok(slam.size<=slam.maxSize);
 assert.equal(slam.mapLimitReached,true);
});
