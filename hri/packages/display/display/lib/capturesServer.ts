import fs from "fs";
import os from "os";
import path from "path";

// The docker containers mount the repo checkout at /workspace/src, so captures
// saved there survive `docker compose down` and are visible on the host.
const WORKSPACE_SRC = "/workspace/src";

// One folder per server boot: <root>/<task>_<BOOT_STAMP>/
const BOOT_STAMP = (() => {
  const d = new Date();
  const p = (n: number) => String(n).padStart(2, "0");
  return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
})();

function capturesRoot(): string {
  if (fs.existsSync(WORKSPACE_SRC)) {
    return path.join(WORKSPACE_SRC, "logs", "captures");
  }
  return path.join(os.homedir(), ".frida", "captures");
}

export function slugify(text: string): string {
  return (
    text
      .replace(/[^\w-]+/g, "_")
      .replace(/^_+|_+$/g, "")
      .slice(0, 40) || "capture"
  );
}

export function timeStamp(): string {
  const d = new Date();
  const p = (n: number) => String(n).padStart(2, "0");
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
}

export function runDir(task: string): string {
  const dir = path.join(capturesRoot(), `${slugify(task)}_${BOOT_STAMP}`);
  fs.mkdirSync(dir, { recursive: true });
  return dir;
}

export function appendLog(task: string, kind: string, text: string): void {
  const line = `${timeStamp()} [${kind.toUpperCase()}] ${text}\n`;
  fs.appendFileSync(path.join(runDir(task), "messages.log"), line);
}
