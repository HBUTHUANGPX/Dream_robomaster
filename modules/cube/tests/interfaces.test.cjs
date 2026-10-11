'use strict';
const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {spawnSync} = require('node:child_process');
const repository = path.resolve(__dirname, '../../..');
const native = path.join(repository, 'build/bin/rm_cube');
const launcher = path.join(repository, 'target/release/robomaster');
const solved = 'UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB';
const identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

function run(program, args, input) {
  const result = spawnSync(program, args, {
    cwd: repository, input, encoding: 'utf8', timeout: 120000,
    maxBuffer: 32 * 1024 * 1024, env: {...process.env, DISPLAY: ''},
  });
  assert.ifError(result.error);
  assert.equal(result.status, 0, result.stderr);
  return result;
}
function requests(commands) {
  return commands.map(([command, args = {}]) => JSON.stringify({command, args})).join('\n') + '\n';
}
function replies(output, count) {
  const lines = output.trim().split('\n');
  assert.equal(lines.length, count, '每个请求必须只返回一行 JSON');
  return lines.map(line => JSON.parse(line));
}

test('机械计划采用夹爪实际时间，零等待不计入成本', () => {
  const parent = path.join(repository, 'output/cube-mechanical');
  fs.mkdirSync(parent, {recursive: true});
  const directory = fs.mkdtempSync(path.join(parent, 'timing-test-'));
  try {
    for (const [speed, settle] of [[0, 0], [0, 0.15], [10, 0]]) {
      const config = path.join(directory, 'parameters.json');
      fs.writeFileSync(config, JSON.stringify({rx_jaw_duration_s: 0.1203,
        rx_jaw_settle_s: 0, rx_wrist_settle_s: settle, rx_wrist_speed_rad_s: speed}));
      run(native, ['--root', repository, '--mechanical', '--plan-only',
        '--scramble', 'R U', '--mechanical-config', config, '--output', directory]);
      const plan = JSON.parse(fs.readFileSync(path.join(directory, 'plan.json')));
      const actions = plan.optimization.candidate;
      const jaws = actions.filter(action => action.mode === 'jaw');
      assert.ok(jaws.length > 0);
      assert.ok(jaws.every(action => Math.abs(action.duration_s - 0.1205) < 1e-10));
      if (speed === 0) assert.ok(actions.filter(action => action.mode !== 'jaw')
        .every(action => Math.abs(action.duration_s - 1.5 - settle) < 1e-10));
      assert.ok(Math.abs(actions.reduce((sum, action) => sum + action.duration_s, 0) -
        plan.optimization.chosen.estimated_execution_s) < 1e-10);
    }
  } finally { fs.rmSync(directory, {recursive: true, force: true}); }
});

test('连续魔方请求在非单位朝向下执行 move、gripper 和 scramble', () => {
  const commands = [
    ['gripper', {action: 'initialize'}],
    ['move', {move: 'U'}], ['move', {move: 'R'}],
    ['gripper', {action: 'execute', moves: 'U'}],
    ['gripper', {action: 'execute', moves: 'R'}],
    ['scramble', {moves: 'U'}], ['scramble', {moves: 'R'}],
  ];
  const output = run(native, ['--root', repository, '--rpc', '--dual', '--wrist-speed', '8', '--jaw-speed', '32'], requests(commands));
  const responses = replies(output.stdout, commands.length);
  for (let i = 0; i < responses.length; i++) {
    const response = responses[i];
    assert.equal(response.ok, true, `${JSON.stringify(commands[i])}: ${response.error}`);
    assert.deepEqual(response.result.moves, ['U', 'R', 'U', 'R', 'U', 'R'].slice(0, i));
    assert.ok(response.result.warnings.every(number => number === 0));
    assert.equal(response.result.cube_motor_force_max, 0);
    assert.equal(response.result.rotation_clearance.forbidden_contacts, 0);
  }
  for (const index of [1, 3, 5]) assert.notDeepEqual(responses[index].result.orientation, identity);
  // 独立的面电机路径提供同一转动序列的实测色块结果。
  const reference = run(native, ['--root', repository, '--rpc'], requests([['move', {moves: 'U R U R U R'}]]));
  const expected = replies(reference.stdout, 1)[0];
  assert.equal(expected.ok, true, expected.error);
  assert.equal(responses.at(-1).result.facelets, expected.result.facelets);
  assert.notEqual(expected.result.facelets, solved);
});

test('实际统一入口的 RPC 标准输出只含响应，诊断写入标准错误', () => {
  const commands = [['state'], ['move', {move: 'invalid'}]];
  const output = run(path.join(repository, 'rm'), ['cube', '--rpc'], requests(commands));
  const responses = replies(output.stdout, commands.length);
  assert.equal(responses[0].ok, true);
  assert.equal(responses[0].result.facelets, solved);
  assert.equal(responses[1].ok, false);
  assert.equal(typeof responses[1].error, 'string');
  assert.match(output.stderr, /正在检查 C\+\+ 构建配置/);
  assert.match(output.stderr, /正在增量构建 Rust 启动器/);
});

test('Rust 帮助中的魔方无窗口示例可以执行并还原实测色块', t => {
  const help = run(launcher, ['--help']).stdout;
  const example = help.match(/^魔方无窗口：(robomaster .+)。$/m);
  assert.ok(example, '帮助中缺少可执行的无窗口示例');
  fs.mkdirSync(path.join(repository, 'output'), {recursive: true});
  const directory = fs.mkdtempSync(path.join(repository, 'output/cube-help-test-'));
  t.after(() => fs.rmSync(directory, {recursive: true, force: true}));
  const args = example[1].split(/\s+/).slice(1);
  const output = run(launcher, [...args, '--output', directory]);
  const report = JSON.parse(output.stdout);
  assert.equal(report.solved, true);
  assert.equal(report.facelets, solved);
  assert.ok(report.moves.length >= 2, '示例必须执行打乱和求解');
  assert.ok(report.warnings.every(number => number === 0));
  assert.equal(JSON.parse(fs.readFileSync(path.join(directory, 'verification.json'), 'utf8')).facelets, solved);
});
