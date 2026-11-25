const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync(__dirname+'/sensor_page_check.js','utf8').replace('}loop();','}');
const elements={},events={},docEvents={},timers=[],sent=[];
function element(id=''){return elements[id]??= {style:{},tagName:'DIV',value:id==='drivePower'?'25':'',hidden:false,disabled:false,append(){},replaceChildren(){},setAttribute(){},removeAttribute(){},closest(){return {append(){}};}};}
const snapshot=(state)=>({state,ready:state==='disarmed',message:'test',blocked_reason:'',duty:[0,0],pulses:[1500,1500],settings:{motor1:1,motor2:-1,front:1,rear:-1,travel:25}});
const context={console,URL,URLSearchParams,location:{hash:'#token=test'},crypto:{randomUUID:()=> 'test-client-123'},localStorage:{getItem:()=>null,setItem(){}},AbortSignal:{timeout:()=>null},setTimeout(){},setInterval:f=>timers.push(f),document:{hidden:false,getElementById:element,createElement:()=>({style:{},append(){},setAttribute(){}}),createTextNode:x=>x,addEventListener:(e,f)=>docEvents[e]=f},window:{addEventListener:(e,f)=>events[e]=f},fetch:async(path,opts)=>{const command=JSON.parse(opts.body);sent.push(command);return {ok:true,json:async()=>snapshot(command.action==='stop'?'disarmed':'armed')};}};
vm.createContext(context);vm.runInContext(source,context);
const render=state=>{context.sample=snapshot(state);vm.runInContext('renderDrive(sample)',context);};
const key=(type,k,target={tagName:'BODY'})=>events[type]({key:k,target,repeat:false,preventDefault(){}});
const settle=()=>new Promise(resolve=>setImmediate(resolve));
(async()=>{
 assert.equal(sent.length,0,'page load must not send arm');
 render('disarmed');await element('arm').onclick();assert.equal(sent.at(-1).action,'arm');
 render('armed');key('keydown','w');key('keydown','a');await timers[0]();assert.deepEqual(sent.at(-1).keys,['w','a']);assert.equal(sent.at(-1).power,25);
 key('keyup','w');await timers[0]();assert.deepEqual(sent.at(-1).keys,['a']);
 key('keyup','a');await settle();assert.equal(sent.at(-1).action,'keys');assert.deepEqual(sent.at(-1).keys,[],'last key release must command zero without disarming');
 render('armed');key('keydown','w');events.blur();await settle();assert.equal(sent.at(-1).action,'stop','blur must disarm');
 render('armed');key('keydown','w');context.document.hidden=true;docEvents.visibilitychange();await settle();assert.equal(sent.at(-1).action,'stop','hidden page must disarm');context.document.hidden=false;
 render('armed');key('keydown','w');key('keydown','Escape');await settle();assert.equal(sent.at(-1).action,'stop','Escape must disarm');
 render('armed');key('keydown','w',{tagName:'INPUT'});await timers[0]();assert.equal(sent.at(-1).action,'keys');assert.deepEqual(sent.at(-1).keys,[],'editing must not drive');
 render('armed');key('keydown','w');key('keydown',' ');await settle();assert.equal(sent.at(-1).action,'stop','Space must disarm');
 assert.equal(sent.filter(c=>c.action==='arm').length,1,'only explicit arm button may arm');
 console.log('PASS: integrated page startup, explicit arm, combined keys, release zero while armed, focus/visibility loss, Escape, Space and editing guard.');
})().catch(e=>{console.error(e);process.exitCode=1;});
