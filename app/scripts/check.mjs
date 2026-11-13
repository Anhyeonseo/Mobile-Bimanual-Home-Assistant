import { readdirSync } from 'node:fs';
import { join, extname } from 'node:path';
import { spawnSync } from 'node:child_process';

function sourceFiles(directory) {
  return readdirSync(directory, { withFileTypes: true }).flatMap((entry) => {
    const path = join(directory, entry.name);
    return entry.isDirectory()
      ? sourceFiles(path)
      : ['.js', '.mjs'].includes(extname(path))
        ? [path]
        : [];
  });
}

const files = [
  ...sourceFiles('src'),
  ...sourceFiles('scripts'),
  ...sourceFiles('tests'),
  'server.mjs',
];
for (const path of files) {
  const result = spawnSync(process.execPath, ['--check', path], { stdio: 'inherit' });
  if (result.status !== 0) process.exit(result.status || 1);
}
console.log(`Syntax checked ${files.length} JavaScript modules.`);
