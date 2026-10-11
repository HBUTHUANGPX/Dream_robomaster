// 在仓库根目录运行。每个试验独立启动仿真，完成后才读取报告。
import fs from 'node:fs';
import path from 'node:path';
import { spawn } from 'node:child_process';

const [casesPath, outputPath] = process.argv.slice(2);
if (!casesPath || !outputPath) throw new Error('用法：node modules/cube/tools/benchmark-mechanical-motion.mjs 试验JSON 输出目录');
const root = process.cwd();
const cases = JSON.parse(fs.readFileSync(casesPath, 'utf8'));
const base = JSON.parse(fs.readFileSync('modules/cube/configs/mechanical-rx-controlled.json', 'utf8'));
const output = path.resolve(outputPath);
fs.mkdirSync(output, { recursive: true });
const runs = [];
for (const test of cases) {
  if (!/^[a-zA-Z0-9_-]+$/.test(test.name) || typeof test.scramble !== 'string') throw new Error('试验名称或打乱无效');
  const directory = path.join(output, test.name);
  if (fs.existsSync(directory)) throw new Error(`输出已存在：${directory}`);
  fs.mkdirSync(directory);
  const parameters = { ...base, rx_wrist_settle_s: 0, rx_jaw_settle_s: 0,
    alignment_timeout_s: 0, alignment_regrasp_attempts: 0, ...test.parameters };
  const config = path.join(directory, 'parameters.json');
  fs.writeFileSync(config, JSON.stringify(parameters, null, 2) + '\n');
  const log = fs.openSync(path.join(directory, 'run.log'), 'w');
  let status;
  try {
    status = await new Promise((resolve, reject) => {
      const child = spawn(path.join(root, 'build/bin/rm_cube'), ['--root', root, '--mechanical',
        '--rx-bundle', 'assets/rx_gripper/mujoco_linkage_v5', '--mechanical-config', config,
        '--scramble', test.scramble, '--solve', '--output', directory], { stdio: ['ignore', log, log] });
      child.once('error', reject);
      child.once('close', (code, signal) => resolve({ code, signal }));
    });
  } finally { fs.closeSync(log); }
  const reportPath = path.join(directory, 'report.json');
  const report = fs.existsSync(reportPath) ? JSON.parse(fs.readFileSync(reportPath, 'utf8')) : {};
  const run = { name: test.name, scramble: test.scramble, parameters, status,
    passed: status.code === 0 && report.passed === true && report.solved === true,
    error: report.error, action: report.current_action, wall_time_s: report.wall_time_s,
    execution_time_s: report.execution_time_s,
    final_pose: report.final_pose, last_pose: report.steps?.at(-1),
    last_wrist: report.wrist_motions?.at(-1), last_jaw: report.jaw_checks?.at(-1),
    wrist_motions: report.wrist_motions, checkpoints: report.checkpoints,
    alignment_waits: report.alignment_waits, alignment_regrasps: report.alignment_regrasps,
    wrist_tn: report.wrist_tn };
  runs.push(run);
  fs.writeFileSync(path.join(output, 'results.json'), JSON.stringify({ runs }, null, 2) + '\n');
  console.log(JSON.stringify({ name: run.name, passed: run.passed, error: run.error, wall_time_s: run.wall_time_s }));
}
