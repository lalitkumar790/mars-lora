import {writeFile,readFile} from 'node:fs/promises';
import {query} from './src/manit_geo_data.js';
const output=new URL('./manit-default.json',import.meta.url);
const valid=x=>Array.isArray(x?.elements)&&x.elements.length>100&&!x.remark&&x.elements.some(e=>e.tags?.highway&&e.geometry)&&x.elements.some(e=>e.tags?.building&&e.geometry);
try{const x=JSON.parse(await readFile(output,'utf8'));if(valid(x)){console.log('MANIT: using bundled data',x.elements.length);process.exit(0)}}catch{}
for(const endpoint of ['https://overpass-api.de/api/interpreter','https://overpass.kumi.systems/api/interpreter']){try{console.log('MANIT: fetching public OSM data from',endpoint);const r=await fetch(endpoint,{method:'POST',body:new URLSearchParams({data:query(2000)}),signal:AbortSignal.timeout(50000)});if(!r.ok)throw Error('HTTP '+r.status);const x=await r.json();if(!valid(x))throw Error('incomplete geometry');await writeFile(output,JSON.stringify(x));console.log('MANIT: bundled',x.elements.length,'OSM elements',x.osm3s?.timestamp_osm_base);process.exit(0)}catch(e){console.warn('MANIT: '+e.message)}}
await writeFile(output,'null');console.warn('MANIT: build-time data unavailable; viewer will attempt automatic download on startup.');
