'use strict';
const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const {spawnSync} = require('node:child_process');
const repository = path.resolve(__dirname, '..');

function executable(file, content) {
  fs.mkdirSync(path.dirname(file), {recursive: true});
  fs.writeFileSync(file, content, {mode: 0o755});
}
function fixture(t) {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'rm 入口测试 '));
  t.after(() => fs.rmSync(root, {recursive: true, force: true}));
  for (const file of ['rm', 'tools/doctor.sh', 'tools/setup.sh', 'tools/bootstrap-native.sh']) {
    const destination = path.join(root, file);
    fs.mkdirSync(path.dirname(destination), {recursive: true});
    fs.copyFileSync(path.join(repository, file), destination);
    fs.chmodSync(destination, 0o755);
  }
  fs.copyFileSync(path.join(repository, 'Makefile'), path.join(root, 'Makefile'));
  const files = [
    'assets/navigation.xml', 'assets/robot.xml', 'assets/arena/rmuc2023.stl',
    'assets/meshes/armor_am02.obj', 'assets/meshes/armor_frame_a.obj',
    'assets/armor_labels/3.png', 'assets/rm_auto_aim/mlp.onnx', 'assets/rm_auto_aim/label.txt',
    'assets/robotiq_2f85/2f85.xml', 'web/navigation/index.html', 'web/duel/index.html', 'web/duel/app.js',
    '.deps/mujoco/lib/libmujoco.so',
    '.deps/opencv/lib/cmake/opencv4/OpenCVConfig.cmake',
  ];
  for (const component of ['core', 'imgproc', 'imgcodecs', 'calib3d', 'dnn'])
    files.push(`.deps/opencv/lib/libopencv_${component}.so`);
  for (const mesh of ['base', 'base_mount', 'driver', 'coupler', 'follower', 'spring_link', 'pad', 'silicone_pad'])
    files.push(`assets/robotiq_2f85/assets/${mesh}.stl`);
  for (const file of files) {
    fs.mkdirSync(path.dirname(path.join(root, file)), {recursive: true});
    fs.writeFileSync(path.join(root, file), 'fixture');
  }
  const header = path.join(root, '.deps/mujoco/include/mujoco/mujoco.h');
  fs.mkdirSync(path.dirname(header), {recursive: true});
  fs.writeFileSync(header, '#define mjVERSION_HEADER 3015000\n');
  fs.writeFileSync(path.join(root, '.deps/opencv/lib/cmake/opencv4/OpenCVConfig-version.cmake'), 'set(OpenCV_VERSION 4.12.0)\n');
  const fake = path.join(root, 'fake tools');
  const dispatcher = `#!/bin/bash
name="\${0##*/}"
case "$name:$\{1:-}" in
  cmake:--version) echo 'cmake version 3.25.1'; exit 0;;
  cargo:--version) if [[ -f "$FIXTURE_ROOT/rust-installed" ]]; then echo 'cargo 1.85.0'; else echo "cargo \${FAKE_CARGO_VERSION:-1.85.0}"; fi; exit 0;;
  rustc:--version) if [[ -f "$FIXTURE_ROOT/rust-installed" ]]; then echo 'rustc 1.85.0'; elif [[ "\${FAKE_RUST_BROKEN:-}" == 1 ]]; then exit 1; else echo "rustc \${FAKE_RUST_VERSION:-1.85.0}"; fi; exit 0;;
  node:--version) echo "v\${FAKE_NODE_VERSION:-20.0.0}"; exit 0;;
  pkg-config:*) [[ -z "\${MISSING_PACKAGE:-}" ]] || exit 1; echo '3.4.0'; exit 0;;
  dpkg-query:*) echo 'install ok installed'; exit 0;;
  df:*) echo 'Filesystem 1024-blocks Used Available Capacity Mounted'; echo '/dev/test 20971520 0 20971520 0% /'; exit 0;;
  c++:*) cat >/dev/null; exit 0;;
esac
printf '%s\\n' "$name" "$@" >> "$TRACE"
printf '\\n' >> "$TRACE"
if [[ "$name" == curl ]]; then
  output=''; installer=false
  while [[ $# -gt 0 ]]; do
    if [[ "$1" == https://sh.rustup.rs ]]; then installer=true; fi
    if [[ "$1" == -o ]]; then shift; output=$1; fi
    shift
  done
  if [[ "$installer" != true ]]; then exit 99; fi
  printf '#!/bin/sh\\ntouch "$FIXTURE_ROOT/rust-installed"\\n' > "$output"
  exit 0
fi
if [[ "$name" == cmake && "\${1:-}" == --build ]]; then exit "\${FAIL_BUILD:-0}"; fi
exit 0
`;
  for (const tool of ['cmake', 'ninja', 'cargo', 'rustc', 'node', 'pkg-config', 'c++', 'dpkg-query', 'df', 'apt-get', 'curl'])
    executable(path.join(fake, tool), dispatcher);
  for (const tool of ['cargo', 'rustc']) executable(path.join(root, '.deps/cargo/bin', tool), dispatcher);
  executable(path.join(root, '.deps/node/bin/node'), dispatcher);
  const worker = `#!/bin/bash
if [[ "\${1:-}" == --help ]]; then
  if [[ "\${FAKE_OLD_LAUNCHER:-}" != 1 ]]; then printf '  robomaster stop [--root DIR]\\n'; fi
  exit 0
fi
printf '%s\\n' "\${0##*/}" "$@" >> "$TRACE"
printf '\\n' >> "$TRACE"
for arg in "$@"; do if [[ "$arg" == --unknown ]]; then echo '未知参数：--unknown' >&2; exit 2; fi; done
exit "\${FAIL_WORKER:-0}"
`;
  executable(path.join(root, 'target/release/robomaster'), worker);
  executable(path.join(root, 'build/bin/rm_robot'), worker);
  const trace = path.join(root, 'trace');
  fs.writeFileSync(trace, '');
  const env = {...process.env, PATH: `${fake}:/usr/bin:/bin`, TRACE: trace, FIXTURE_ROOT: root, DISPLAY: ':99'};
  delete env.MUJOCO_ROOT; delete env.OpenCV_DIR; delete env.ROBOMASTER_ROOT;
  return {
    root,
    env,
    run(args, extra = {}) {
      fs.writeFileSync(trace, '');
      const result = spawnSync(path.join(root, 'rm'), args, {cwd: '/', env: {...env, ...extra}, encoding: 'utf8'});
      return {...result, calls: fs.readFileSync(trace, 'utf8').trim().split('\n\n').filter(Boolean).map(x => x.split('\n'))};
    },
  };
}

test('入口脚本存在且可执行', () => {
  fs.accessSync(path.join(repository, 'rm'), fs.constants.X_OK);
});
test('准备命令复用已有工具和 SDK，不安装系统包且不改写共享库', t => {
  const f = fixture(t);
  const library = path.join(f.root, '.deps/mujoco/lib/libmujoco.so');
  const before = fs.statSync(library);
  const r = f.run(['setup', '--with-tests']);
  assert.equal(r.status, 0, r.stderr);
  assert.ok(!r.calls.some(c => ['apt-get', 'curl'].includes(c[0])));
  const after = fs.statSync(library);
  assert.equal(after.ino, before.ino); assert.equal(after.mtimeMs, before.mtimeMs);
});
test('压缩包只给根入口执行权限时，内部脚本仍可运行', t => {
  const f = fixture(t);
  for (const file of ['tools/doctor.sh', 'tools/setup.sh', 'tools/bootstrap-native.sh'])
    fs.chmodSync(path.join(f.root, file), 0o644);
  for (const args of [['doctor'], ['setup']]) {
    const r = f.run(args); assert.equal(r.status, 0, r.stderr);
  }
});
test('准备命令修复旧 Cargo 和未完成安装的 Rust 工具链', t => {
  for (const extra of [{FAKE_CARGO_VERSION: '1.84.0'}, {FAKE_RUST_BROKEN: '1'}]) {
    const f = fixture(t); const r = f.run(['setup'], extra);
    assert.equal(r.status, 0, r.stderr);
    assert.ok(fs.existsSync(path.join(f.root, 'rust-installed')));
    assert.ok(r.calls.some(c => c[0] === 'curl' && c.includes('https://sh.rustup.rs')));
  }
});
test('底盘帮助不需要 DISPLAY，也不执行构建', t => {
  const f = fixture(t); const r = f.run(['robot', '--help'], {DISPLAY: ''});
  assert.equal(r.status, 0, r.stderr); assert.match(r.stdout, /底盘/); assert.deepEqual(r.calls, []);
});
test('停止命令不检查开发工具或资产，也不执行构建', t => {
  const f = fixture(t);
  fs.rmSync(path.join(f.root, 'assets'), {recursive: true});
  const r = f.run(['stop'], {MISSING_PACKAGE: '1', FAKE_RUST_BROKEN: '1', DISPLAY: ''});
  assert.equal(r.status, 0, r.stderr);
  assert.deepEqual(r.calls, [['robomaster', 'stop', '--root', f.root]]);
});
test('停止命令拒绝多余参数，缺少启动器时给出结束方式', t => {
  const f = fixture(t);
  let r = f.run(['stop', '--unknown']);
  assert.notEqual(r.status, 0); assert.deepEqual(r.calls, []);
  fs.unlinkSync(path.join(f.root, 'target/release/robomaster'));
  r = f.run(['stop']);
  assert.notEqual(r.status, 0); assert.match(r.stderr, /Ctrl\+C/);
  assert.deepEqual(r.calls, []);
});
test('更新代码后遇到旧启动器时，停止命令给出中文处理方式', t => {
  const f = fixture(t);
  const r = f.run(['stop'], {FAKE_OLD_LAUNCHER: '1'});
  assert.notEqual(r.status, 0); assert.match(r.stderr, /旧|更新/);
  assert.match(r.stderr, /Ctrl\+C/); assert.deepEqual(r.calls, []);
});
test('缺少系统开发库时给出准备指令', t => {
  const f = fixture(t); const r = f.run(['build'], {MISSING_PACKAGE: '1'});
  assert.notEqual(r.status, 0); assert.match(r.stderr, /缺少开发库/);
  assert.match(r.stderr, /\.\/rm setup/); assert.deepEqual(r.calls, []);
});
test('任意工作目录和含空格路径：默认启动两项服务', t => {
  const f = fixture(t); const r = f.run([]);
  assert.equal(r.status, 0, r.stderr);
  assert.ok(r.calls.some(c => c[0] === 'cmake' && c[1] === '--build'));
  assert.deepEqual(r.calls.at(-1), ['robomaster', 'up', '--root', f.root]);
});
test('make build 兼容任意工作目录和含空格的仓库路径', t => {
  const f = fixture(t);
  const r = spawnSync('make', ['-f', path.join(f.root, 'Makefile'), 'build'], {
    cwd: '/', env: f.env, encoding: 'utf8',
  });
  assert.equal(r.status, 0, r.stderr);
  assert.match(r.stdout, /构建完成/);
});
test('未知命令和入口多余参数不执行构建', t => {
  const f = fixture(t);
  for (const args of [['not-a-command'], ['build', '--unknown'], ['doctor', '--unknown']]) {
    const r = f.run(args); assert.notEqual(r.status, 0); assert.deepEqual(r.calls, []);
    assert.match(r.stderr, /未知|不接受|参数/);
  }
});
test('服务参数保持独立参数并传给 Rust', t => {
  const f = fixture(t);
  for (const module of ['navigation', 'duel']) {
    const r = f.run([module, '--port', '19001']); assert.equal(r.status, 0, r.stderr);
    assert.deepEqual(r.calls.at(-1), ['robomaster', 'serve', module, '--root', f.root, '--port', '19001']);
  }
});
test('魔方仅在无参数时启用窗口，显式参数保持原样', t => {
  const f = fixture(t);
  let r = f.run(['cube']); assert.equal(r.status, 0, r.stderr);
  assert.deepEqual(r.calls.at(-1), ['robomaster', 'run', 'cube', '--root', f.root, '--', '--viewer']);
  r = f.run(['cube', '--headless', '--scramble', "R U R'"] , {DISPLAY: ''});
  assert.equal(r.status, 0, r.stderr);
  assert.deepEqual(r.calls.at(-1), ['robomaster', 'run', 'cube', '--root', f.root, '--', '--headless', '--scramble', "R U R'"]);
});
test('窗口命令缺少 DISPLAY 时先报错，底盘支持显式无窗口模式', t => {
  const f = fixture(t);
  for (const args of [['cube'], ['cube', '--viewer'], ['robot']]) {
    const r = f.run(args, {DISPLAY: ''}); assert.notEqual(r.status, 0);
    assert.match(r.stderr, /DISPLAY/); assert.deepEqual(r.calls, []);
  }
  const r = f.run(['robot', '--headless', '--duration', '.2'], {DISPLAY: ''});
  assert.equal(r.status, 0, r.stderr);
  assert.deepEqual(r.calls.at(-1), ['rm_robot', '--root', f.root, '--headless', '--duration', '.2']);
});
test('缺失资产给出路径并停止，不调用构建或服务', t => {
  const f = fixture(t); fs.unlinkSync(path.join(f.root, 'assets/arena/rmuc2023.stl'));
  const r = f.run(['navigation']); assert.notEqual(r.status, 0);
  assert.match(r.stderr, /assets\/arena\/rmuc2023.stl/); assert.deepEqual(r.calls, []);
});
test('缺少自瞄对战页面脚本时停止，避免启动不可交互的页面', t => {
  const f = fixture(t); fs.unlinkSync(path.join(f.root, 'web/duel/app.js'));
  const r = f.run(['duel']); assert.notEqual(r.status, 0);
  assert.match(r.stderr, /web\/duel\/app.js/); assert.deepEqual(r.calls, []);
});
test('构建失败及下游参数错误保留失败状态', t => {
  const f = fixture(t);
  let r = f.run(['navigation'], {FAIL_BUILD: '7'}); assert.equal(r.status, 7);
  assert.ok(!r.calls.some(c => c[0] === 'robomaster'));
  r = f.run(['duel', '--unknown']); assert.equal(r.status, 2); assert.match(r.stderr, /未知参数/);
  r = f.run(['cube', '--headless'], {FAIL_WORKER: '13'}); assert.equal(r.status, 13);
});
test('Node 仅是测试依赖，旧 Rust 阻止构建且给出准备命令', t => {
  const f = fixture(t);
  let r = f.run(['build'], {FAKE_NODE_VERSION: '16.0.0'}); assert.equal(r.status, 0, r.stderr);
  r = f.run(['test'], {FAKE_NODE_VERSION: '16.0.0'}); assert.notEqual(r.status, 0);
  assert.match(r.stderr, /Node|node/); assert.match(r.stderr, /\.\/rm setup/);
  r = f.run(['build'], {FAKE_RUST_VERSION: '1.84.0'}); assert.notEqual(r.status, 0);
  assert.match(r.stderr, /1\.85|Rust/); assert.match(r.stderr, /\.\/rm setup/);
  assert.deepEqual(r.calls, []);
  r = f.run(['build'], {FAKE_RUST_BROKEN: '1'}); assert.notEqual(r.status, 0);
  assert.match(r.stderr, /\.\/rm setup/); assert.deepEqual(r.calls, []);
});
