'use strict';
const {test} = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');
const fs = require('node:fs');
const {spawnSync} = require('node:child_process');
const root = path.resolve(__dirname, '../../..');
const binary = path.join(root, 'build/bin/rm_cube');
const turnedR = 'UUFUUFUUFRRRRRRRRRFFDFFDFFDDDBDDBDDBLLLLLLLLLUBBUBBUBB';
function rpc(commands) {
  const run = spawnSync(binary, ['--root', root, '--rpc', '--dual', '--wrist-speed', '8', '--jaw-speed', '32'], {
    cwd: root, input: commands.map(([command,args]) => JSON.stringify({command,args})).join('\n')+'\n',
    encoding: 'utf8', timeout: 30000, maxBuffer: 16*1024*1024,
  });
  assert.ifError(run.error); assert.equal(run.status,0,run.stderr);
  const replies = run.stdout.trim().split('\n').map(JSON.parse);
  assert.equal(replies.length,commands.length); return replies;
}
test('限时规划不执行物理动作，超时和非法成本有明确结果', () => {
  const commands = [
    ['plan',{facelets:turnedR,max_search_ms:0}],
    ['plan',{max_search_ms:0}],
    ['plan',{facelets:turnedR,max_search_ms:300,threads:2}],
    ['plan',{cost_profile:{duration_s:{A_P90:-1}}}],
    ['plan',{overlap:{A_P90:0.1}}],
    ['plan',{facelets:'UUUUURUUURURRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB',max_search_ms:0}],
  ];
  const [expired,empty,plan,bad,overlap,impossible] = rpc(commands);
  assert.equal(expired.ok,true);assert.equal(expired.result.found,false);
  assert.equal(expired.result.stop_reason,'deadline');
  assert.equal(empty.result.found,true);assert.deepEqual(empty.result.actions,[]);
  assert.equal(plan.ok,true);assert.equal(plan.result.found,true);
  assert.equal(plan.result.search_time_in_objective,false);
  assert.equal(plan.result.global_optimal,false);
  assert.equal(plan.result.threads_requested,2);
  assert.ok(plan.result.threads_used>=1&&plan.result.threads_used<=2);
  assert.ok(plan.result.worker_memory_limit_mb*plan.result.threads_used<=64);
  assert.ok(plan.result.search_ms<500,'取消检查不能严重超过预算');
  assert.equal(plan.result.state.robot_ready,false);assert.equal(plan.result.state.solved,true);
  assert.equal(bad.ok,false);assert.equal(overlap.ok,false);assert.equal(impossible.ok,false);
});
test('相对元动作可以连续执行，非法完整序列在第一步前被拒绝', () => {
  const [init,invalid,before,first,second] = rpc([
    ['gripper',{action:'initialize'}],
    ['gripper',{action:'execute_primitives',actions:['A_OPEN','B_OPEN']}],
    ['state',{}],
    ['gripper',{action:'execute_primitives',actions:['A_P90']}],
    ['gripper',{action:'execute_primitives',actions:['A_N90']}],
  ]);
  assert.equal(init.ok,true);assert.equal(invalid.ok,false);
  assert.equal(before.result.simulation_time_s,init.result.simulation_time_s);
  assert.equal(first.ok,true,first.error);assert.equal(first.result.solved,false);
  assert.equal(second.ok,true,second.error);assert.equal(second.result.solved,true);
  assert.equal(second.result.cube_motor_force_max,0);
});
test('CLI 只规划输出十二元动作及时间配置', t => {
  const dir = fs.mkdtempSync(path.join(root,'output/search-cli-test-'));
  t.after(() => fs.rmSync(dir,{recursive:true,force:true}));
  const costs = path.join(dir,'costs.json');
  fs.writeFileSync(costs,JSON.stringify({duration_s:{A_P90:0.001}}));
  const run=spawnSync(binary,['--root',root,'--headless','--dual','--scramble','R','--plan-only',
    '--search-ms','300','--search-threads','2','--cost-profile',costs,'--wrist-speed','8','--jaw-speed','32','--output',dir],
    {cwd:root,encoding:'utf8',timeout:30000,maxBuffer:16*1024*1024});
  assert.ifError(run.error);assert.equal(run.status,0,run.stderr);
  const plan=JSON.parse(fs.readFileSync(path.join(dir,'plan.json')));
  assert.equal(plan.found,true);assert.equal(plan.objective,'execution_time');
  assert.equal(plan.threads_requested,2);
  assert.ok(plan.threads_used>=1&&plan.threads_used<=2);
  assert.equal(plan.action_count,1);assert.equal(plan.actions[0].action,'A_P90');
  assert.equal(plan.estimated_execution_s,0.001);
  const state=JSON.parse(fs.readFileSync(path.join(dir,'verification.json')));
  assert.equal(state.robot_ready,false);assert.equal(state.solved,false);
});
