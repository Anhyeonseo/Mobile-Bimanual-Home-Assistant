import assert from 'node:assert/strict';
import { readdir, readFile, stat } from 'node:fs/promises';
import { extname, relative, resolve, sep } from 'node:path';
import { fileURLToPath } from 'node:url';

const output = fileURLToPath(new URL('../dist/', import.meta.url));
const base = new URL('https://example.test/Mobile-Bimanual-Home-Assistant/');

async function filesIn(directory) {
  const entries = await readdir(directory, { withFileTypes: true });
  const nested = await Promise.all(
    entries.map((entry) => {
      const path = resolve(directory, entry.name);
      return entry.isDirectory() ? filesIn(path) : [path];
    }),
  );
  return nested.flat();
}

async function verifyReference(from, reference) {
  if (!reference || /^(?:data:|https?:|mailto:|#)/i.test(reference)) return;
  const resolved = new URL(reference, new URL(from, base));
  assert.ok(
    resolved.pathname.startsWith(base.pathname),
    `${from}: reference escapes project path: ${reference}`,
  );
  const file = resolve(
    output,
    decodeURIComponent(resolved.pathname.slice(base.pathname.length)) || 'index.html',
  );
  assert.ok((await stat(file)).isFile(), `${from}: missing file: ${reference}`);
}

let checked = 0;
for (const file of await filesIn(output)) {
  const from = relative(output, file).split(sep).join('/');
  const extension = extname(file);
  const patterns = {
    '.html': /(?:src|href)=["']([^"']+)["']/g,
    '.css': /(?:url\(\s*["']?|@import\s+["'])([^\s"')]+)["']?\s*\)?/g,
    '.js': /(?:\bfrom\s+|\bimport\s*)["']([^"']+)["']/g,
  };
  if (!patterns[extension]) continue;
  const content = await readFile(file, 'utf8');
  for (const match of content.matchAll(patterns[extension])) {
    await verifyReference(from, match[1]);
    checked += 1;
  }
}
const manifest = JSON.parse(await readFile(resolve(output, 'manifest.webmanifest'), 'utf8'));
assert.equal(manifest.start_url, './', 'The installed app must start under the repository URL.');
for (const icon of manifest.icons) await verifyReference('manifest.webmanifest', icon.src);
await stat(resolve(output, '.nojekyll'));
console.log(`Verified ${checked} references and app manifest under the GitHub Pages project path.`);
