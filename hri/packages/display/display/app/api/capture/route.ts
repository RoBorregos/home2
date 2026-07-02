import fs from "fs";
import path from "path";
import { NextResponse } from "next/server";
import { appendLog, runDir, slugify, timeStamp } from "../../../lib/capturesServer";

// Single-frame endpoint of web_video_server (same process the /stream view uses).
const SNAPSHOT_URL = "http://localhost:8080/snapshot";
const SNAPSHOT_TIMEOUT_MS = 3000;

export async function POST(req: Request) {
  const body = await req.json().catch(() => ({}));
  const label: string = body.label || "capture";
  const topic: string = body.topic || "";
  const task: string = body.task || "default";

  let file: string | null = null;
  let dataUrl: string | null = null;

  if (topic) {
    try {
      const res = await fetch(
        `${SNAPSHOT_URL}?topic=${encodeURIComponent(topic)}&quality=90`,
        { signal: AbortSignal.timeout(SNAPSHOT_TIMEOUT_MS), cache: "no-store" }
      );
      if (res.ok) {
        const buf = Buffer.from(await res.arrayBuffer());
        file = `${timeStamp().replaceAll(":", "")}_${slugify(label)}.jpg`;
        fs.writeFileSync(path.join(runDir(task), file), buf);
        dataUrl = `data:image/jpeg;base64,${buf.toString("base64")}`;
      }
    } catch {
      // snapshot timed out (topic silent / web_video_server down) — logged below
    }
  }

  appendLog(task, "capture", file ? `${label} -> ${file}` : `${label} (no frame from ${topic || "?"})`);
  return NextResponse.json({ ok: file !== null, file, dataUrl });
}
