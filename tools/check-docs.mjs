#!/usr/bin/env node
// 检查维护文档的中文标题、明显英文段落和本地链接。
import { existsSync, readFileSync, readdirSync, statSync } from 'node:fs';
import { dirname, join, relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const ignored = new Set(['.git', '.deps', '.cache', '.venv', '.venv-cube',
  'build', 'target', 'output', 'artifacts', 'node_modules', '__pycache__', '.pytest_cache']);
const documents = [];
function collect(directory) {
  for (const entry of readdirSync(directory, { withFileTypes: true })) {
    if (entry.isSymbolicLink()) continue;
    const path = join(directory, entry.name);
    if (entry.isDirectory()) {
      if (!ignored.has(entry.name) && !entry.name.startsWith('build-')) collect(path);
    } else if (/\.(md|markdown|rst|adoc)$/i.test(entry.name)) documents.push(path);
  }
}
collect(root);
const failures = [];
const chinese = /\p{Script=Han}/u;
function report(path, line, problem) {
  failures.push(`${relative(root, path)}:${line}: ${problem}`);
}
function headingIds(path) {
  const ids = new Set();
  for (const match of readFileSync(path, 'utf8').matchAll(/^#{1,6}\s+(.+?)\s*#*$/gm)) {
    ids.add(match[1].replace(/`/g, '').toLowerCase()
      .replace(/[^\p{L}\p{N}_\-\s]/gu, '').replace(/\s/g, '-'));
  }
  return ids;
}
for (const path of documents) {
  const content = readFileSync(path, 'utf8');
  if (!chinese.test(content)) report(path, 1, '文档没有中文说明。');
  let fence = null;
  const lines = content.split('\n');
  for (let index = 0; index < lines.length; index++) {
    const line = lines[index];
    const marker = line.match(/^\s*(`{3,}|~{3,})/);
    if (marker) {
      if (!fence) fence = marker[1][0];
      else if (fence === marker[1][0]) fence = null;
      continue;
    }
    if (fence) continue;
    if (/^\s*#{1,6}\s/.test(line) && !chinese.test(line)) {
      report(path, index + 1, '标题必须包含中文。');
    }
    const prose = line.replace(/`[^`]*`/g, '')
      .replace(/!?\[([^\]]*)\]\([^)]*\)/g, '$1')
      .replace(/https?:\/\/\S+/g, '').replace(/<[^>]*>/g, '');
    const words = prose.match(/\b[A-Za-z]{2,}\b/g) ?? [];
    if (!chinese.test(prose) && words.length >= 3) {
      report(path, index + 1, '存在英文说明，请改为中文；程序标识使用行内代码。');
    }
    for (const match of line.matchAll(/!?\[[^\]]*\]\((<?[^\s)]+>?)(?:\s+"[^"]*")?\)/g)) {
      const target = match[1].replace(/^<|>$/g, '');
      if (/^[a-z][a-z\d+.-]*:/i.test(target)) continue;
      const [filename, anchor] = target.split('#', 2);
      const destination = filename ? resolve(dirname(path), decodeURIComponent(filename)) : path;
      if (!existsSync(destination)) {
        report(path, index + 1, `本地链接不存在：${target}`);
      } else if (anchor && destination.endsWith('.md') && statSync(destination).isFile()
          && !headingIds(destination).has(decodeURIComponent(anchor))) {
        report(path, index + 1, `文档锚点不存在：${target}`);
      }
    }
  }
  if (fence) report(path, lines.length, '代码块没有关闭。');
}
if (failures.length) {
  console.error(failures.join('\n'));
  console.error(`文档检查失败：${failures.length} 项，已检查 ${documents.length} 个文件。`);
  process.exitCode = 1;
} else {
  console.log(`文档检查通过：${documents.length} 个文件；中文标题、明显英文段落和本地链接均通过检查。`);
}
