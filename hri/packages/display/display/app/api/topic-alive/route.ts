import http from "http";
import { NextResponse } from "next/server";

// web_video_server only listens on IPv4; 127.0.0.1 avoids Node resolving
// "localhost" to ::1 first.
const WVS_HOST = "127.0.0.1";
const WVS_PORT = 8080;
const CHECK_TIMEOUT_MS = 1500;

function checkAlive(topic: string, timeoutMs: number): Promise<boolean> {
  return new Promise((resolve) => {
    const req = http.get(
      {
        host: WVS_HOST,
        port: WVS_PORT,
        path: `/snapshot?topic=${topic}&quality=30`,
        timeout: timeoutMs,
      },
      (res) => {
        res.destroy(); // headers arriving is enough proof of life; skip the body
        resolve(res.statusCode === 200);
      }
    );
    req.setTimeout(timeoutMs, () => {
      req.destroy();
      resolve(false);
    });
    req.on("error", () => resolve(false));
  });
}

export async function GET(req: Request) {
  const { searchParams } = new URL(req.url);
  const topic = searchParams.get("topic") || "";
  if (!topic) return NextResponse.json({ alive: false });
  const alive = await checkAlive(topic, CHECK_TIMEOUT_MS);
  return NextResponse.json({ alive });
}
