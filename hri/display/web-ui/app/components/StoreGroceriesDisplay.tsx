"use client";

import { useState } from "react";
import dynamic from "next/dynamic";
import { useRouter } from "next/router";
import { MessageCircle } from "lucide-react";
import { AudioStateIndicator } from "./AudioStateIndicator";
import { useWebSocket } from "../hooks/useWebSocket";
import { useEffect } from "react";

const MjpegStream = dynamic(() => import("./video"), { ssr: false });

export default function StoringGroceriesVideoDisplay() {
  const router = useRouter();
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  const [task, setTask] = useState<string>("default");

  // Detect task from query param or environment variable
  useEffect(() => {
  if (router.isReady) {
    const taskFromQuery = router.query.task as string | undefined;
    setTask(taskFromQuery || process.env.NEXT_PUBLIC_DISPLAY_TASK || "default");
    }
  }, [router.isReady, router.query.task]);
  const [audioTopic, setAudioTopic] = useState(
    "/zed/zed_node/rgb/image_rect_color"
  );

  const { connected } = useWebSocket({
    onAddMessage: (_type: string, _data: any) => {},
    onAudioStateChange: (_event: string, _data: any) => {},
    onVideoTopicChange: setAudioTopic,
    onQuestionReceived: (_question: string) => {},
    onMapReceived: (_map: any) => {},
  });

  return (
    <div className="flex flex-col h-screen bg-[oklch(0.145_0_0)] text-[oklch(0.985_0_0)] overflow-hidden">

      {/* ---- HEADER COMPLETO COMO EN TU DISPLAY ORIGINAL ---- */}
      <div className="p-4 border-b border-[oklch(1_0_0/10%)] flex items-center justify-between">
        <h1 className="text-xl font-bold flex items-center">
          <MessageCircle className="mr-2 h-5 w-5" />
          Storing Groceries
        </h1>

        <div className="flex items-center gap-3">
          <AudioStateIndicator state={{ state: "idle", vadLevel: 0 }} />

          <div
            className={
              connected
                ? "px-2 py-1 rounded-full text-xs font-medium bg-[oklch(0.488_0.243_264.376/20%)] text-[oklch(0.488_0.243_264.376)]"
                : "px-2 py-1 rounded-full text-xs font-medium bg-[oklch(0.577_0.245_27.325/20%)] text-[oklch(0.704_0.191_22.216)]"
            }
          >
            {connected ? "Connected" : "Disconnected"}
          </div>
        </div>
      </div>

      {/* ---- CONTENIDO CENTRADO ---- */}
      <div className="flex flex-1 flex-col items-center justify-center p-6 text-center">

        {/* Botón Start */}
        <button
          className="mb-6 px-6 py-3 bg-[oklch(0.488_0.243_264.376)] text-white
          rounded-xl hover:bg-[oklch(0.488_0.243_264.376/80%)]
          transition-colors text-lg font-semibold shadow-md"
          onClick={() => void fetch("http://localhost:8001/send_button_press")}
        >
          Start 🔥
        </button>

        <p className="text-xl mb-4">Video feed at {audioTopic}</p>

        {/* VIDEO CENTRADO (max width) */}
        <div className="w-full max-w-3xl rounded-xl overflow-hidden shadow-xl border border-[oklch(1_0_0/15%)]">
          <MjpegStream
            streamUrl={`http://localhost:8080/stream?topic=${audioTopic}`}
          />
        </div>
      </div>

      {/* ---- FOOTER ---- */}
      <div className="p-3 border-t border-[oklch(1_0_0/10%)] bg-[oklch(0.205_0_0/40%)] text-center">
        <p className="text-sm text-[oklch(0.708_0_0)]">
          Video Display Ready
        </p>
      </div>
    </div>
  );
}
