import { cp, rm, writeFile } from 'node:fs/promises';
import { basename, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const appRoot = resolve(fileURLToPath(new URL('..', import.meta.url)));
const source = resolve(appRoot, 'src');
const output = resolve(appRoot, 'dist');

// Limit cleanup to this app's generated output, including on Windows.
if (dirname(output) !== appRoot || basename(output) !== 'dist') {
  throw new Error(`Unexpected build directory: ${output}`);
}
await rm(output, { recursive: true, force: true });
await cp(source, output, { recursive: true, errorOnExist: true });
await writeFile(resolve(output, '.nojekyll'), '');
console.log('Built static app in app/dist.');
