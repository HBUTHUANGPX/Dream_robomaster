import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';

const root = fileURLToPath(new URL('../assets/rx_gripper/', import.meta.url));
const manifest = JSON.parse(fs.readFileSync(path.join(root, 'manifest.json'), 'utf8'));
for (const [name, expected] of Object.entries(manifest.files)) {
  const actual = createHash('sha256').update(fs.readFileSync(path.join(root, name))).digest('hex');
  if (actual !== expected) throw new Error(`RX资产校验失败：${name}`);
}
const bundle = 'mujoco_linkage_v5/';
const xml = fs.readFileSync(path.join(root, bundle, 'free_sweep.xml'), 'utf8');
for (const [, name] of xml.matchAll(/\bfile="([^"]+)"/g)) {
  if (!Object.hasOwn(manifest.files, bundle + name))
    throw new Error(`RX场景引用未纳入清单：${name}`);
}
console.log(`RX资产校验通过：${Object.keys(manifest.files).length}个文件，场景网格引用完整。`);
