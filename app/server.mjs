import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const args = process.argv.slice(2);
const appRoot = fileURLToPath(new URL('.', import.meta.url));
const root = path.resolve(appRoot, args.includes('--built') ? 'dist' : 'src');
const base = args.find((arg) => arg.startsWith('--base='))?.slice(7) || '/';
const port = Number(process.env.PORT || 4173);
if (!/^\/(?:[\w-]+\/)*$/.test(base))
  throw new Error('Use a base path with leading and trailing slashes.');
if (!Number.isInteger(port) || port < 1 || port > 65535) throw new Error('Invalid PORT.');
const types = {
  '.html': 'text/html; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.json': 'application/json',
  '.webmanifest': 'application/manifest+json',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.ttf': 'font/ttf',
  '.txt': 'text/plain; charset=utf-8',
};
http
  .createServer((req, res) => {
    let file;
    try {
      const pathname = new URL(req.url, 'http://localhost').pathname;
      if (base !== '/' && pathname === base.slice(0, -1)) {
        res.writeHead(302, { Location: base });
        res.end();
        return;
      }
      if (!pathname.startsWith(base)) {
        res.writeHead(404);
        res.end('Not found');
        return;
      }
      file = path.resolve(root, decodeURIComponent(pathname.slice(base.length)));
    } catch {
      res.writeHead(400);
      res.end();
      return;
    }
    if (file !== root && !file.startsWith(root + path.sep)) {
      res.writeHead(403);
      res.end();
      return;
    }
    if (file === root) file = path.join(root, 'index.html');
    fs.stat(file, (err, stat) => {
      if (err || !stat.isFile()) {
        res.writeHead(404);
        res.end('Not found');
        return;
      }
      res.writeHead(200, {
        'Content-Type': types[path.extname(file)] || 'application/octet-stream',
        'Cache-Control': 'no-cache',
      });
      fs.createReadStream(file).pipe(res);
    });
  })
  .listen(port, '127.0.0.1', () => console.log(`Local: http://127.0.0.1:${port}${base}`));
