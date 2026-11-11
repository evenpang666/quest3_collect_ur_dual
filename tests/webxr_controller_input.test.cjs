// Run with: node --test tests/webxr_controller_input.test.cjs
const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, '../quest3_robot_data/web/index.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
const elements = new Map();
const context = vm.createContext({
  document: {getElementById(id) {
    if (!elements.has(id)) elements.set(id, {});
    return elements.get(id);
  }},
  location: {hostname: 'localhost', protocol: 'http:'},
});
vm.runInContext(script.replace(/\}\)\(\);\s*$/, `
  globalThis.inputs={isHandInput,isPhysicalController,controller,trackedController,controllerHint};
})();`), context);
const inputs = context.inputs;
const touch = (handedness='left') => ({
  handedness, targetRayMode:'tracked-pointer', gripSpace:{},
  profiles:['oculus-touch-v3','generic-trigger-squeeze-thumbstick'],
  gamepad:{buttons:[{value:.2},{value:.8},{value:0},{value:0},{value:0},{value:0}]},
});
const fixedHand = () => ({
  ...touch(), profiles:['meta-fixed-hand','generic-fixed-hand','generic-hand-select','generic-trigger'],
});
const pose = {transform:{position:{x:1,y:2,z:3},orientation:{x:0,y:0,z:0,w:1}}};

test('fixed-hand gamepads are rejected even without the hand-tracking API', () => {
  const hand = fixedHand();
  assert.equal(hand.hand, undefined);
  assert.equal(inputs.isHandInput(hand), true);
  assert.equal(inputs.isPhysicalController(hand), false);
  assert.equal(inputs.controller(hand, {getPose(){throw Error('must not use bare-hand pose')}}), null);
});

test('articulated hands and non-controller pointers are rejected', () => {
  assert.equal(inputs.isPhysicalController({...touch(),hand:{}}), false);
  assert.equal(inputs.isPhysicalController({...touch(),targetRayMode:'gaze'}), false);
  assert.equal(inputs.isPhysicalController({...touch(),gamepad:null}), false);
  assert.equal(inputs.isPhysicalController({...touch(),gripSpace:null}), false);
});

test('Touch controller poses and button mappings are preserved', () => {
  assert.equal(inputs.isPhysicalController(touch()), true);
  const result = inputs.controller(touch(), {getPose:()=>pose});
  assert.deepEqual(Array.from(result.position), [1,2,3]);
  assert.deepEqual(Array.from(result.orientation), [0,0,0,1]);
  assert.equal(result.trigger, .2);
  assert.equal(result.squeeze, .8);
});

test('a hand listed before the physical controller does not hide the controller', () => {
  const tracked = touch();
  const stale = {...touch(),gripSpace:{stale:true}};
  const result = inputs.trackedController([fixedHand(),stale,touch('right'),tracked], 'left', {
    getPose:space=>space===tracked.gripSpace?pose:null,
  });
  assert.ok(result);
  assert.equal(result.squeeze,.8);
  assert.equal(inputs.trackedController([fixedHand(),stale], 'left', {getPose:()=>null}),null);
});

test('tracking messages distinguish hands, missing controllers, and lost focus', () => {
  assert.match(inputs.controllerHint([fixedHand()],null,null,'visible'),/裸手/);
  assert.match(inputs.controllerHint([touch()],{},null,'visible'),/右实体手柄/);
  assert.match(inputs.controllerHint([],null,null,'visible-blurred'),/输入焦点/);
  assert.match(inputs.controllerHint([touch(),touch('right')],{},{},'visible'),/均已追踪/);
});
