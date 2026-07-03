import { NextResponse } from "next/server";
import { appendLog } from "../../../lib/capturesServer";

export async function POST(req: Request) {
  const body = await req.json().catch(() => ({}));
  const task: string = body.task || "default";
  const kind: string = body.kind || "info";
  const text: string = body.text ?? "";
  appendLog(task, kind, text);
  return NextResponse.json({ ok: true });
}
