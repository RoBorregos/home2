"use client";

import { useState, useEffect } from "react";
import dynamic from "next/dynamic";
import { MessageCircle } from "lucide-react";
import { AudioStateIndicator } from "./AudioStateIndicator";
import { useWebSocket } from "../hooks/useWebSocket";

const MjpegStream = dynamic(() => import("./video"), { ssr: false });

export default function StoringGroceriesVideoDisplay() {
  const [audioTopic, setAudioTopic] = useState("/zed/zed_node/rgb/image_rect_color");
  const [audioState, setAudioState] = useState<{ state: "idle" | "listening" | "saying"; vadLevel: number }>({
    state: "idle",
    vadLevel: 0,
  });

  const { connected } = useWebSocket({
    onAddMessage: () => {},
    onAudioStateChange: (type: string, content: string | number) => {
      if (type === "audioState") {
        setAudioState((prev) => ({ ...prev, state: content as "idle" | "listening" | "saying" }));
      } else if (type === "vad") {
        setAudioState((prev) => ({
          ...prev,
          vadLevel: typeof content === "number" ? content : Number.parseFloat(content as string),
        }));
      }
    },
    onVideoTopicChange: setAudioTopic,
    onQuestionReceived: () => {},
    onMapReceived: () => {},
  });

  return (
    <div className="flex flex-col h-screen bg-[oklch(0.145_0_0)] text-[oklch(0.985_0_0)] overflow-hidden">
      {/* HEADER */}
      <div className="p-4 border-b border-[oklch(1_0_0/10%)] flex items-center justify-between">
        <h1 className="text-xl font-bold flex items-center">
          <MessageCircle className="mr-2 h-5 w-5" />
          Storing Groceries
        </h1>

        <div className="flex items-center gap-3">
          <AudioStateIndicator state={audioState} />

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

      {/* CONTENIDO CENTRADO */}
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

        {/* VIDEO CENTRADO */}
        <div className="w-full max-w-3xl rounded-xl overflow-hidden shadow-xl border border-[oklch(1_0_0/15%)]">
          <MjpegStream streamUrl={`http://localhost:8080/stream?topic=${audioTopic}`} />
        </div>
      </div>

      {/* FOOTER */}
      <div className="p-3 border-t border-[oklch(1_0_0/10%)] bg-[oklch(0.205_0_0/40%)] text-center">
        <p className="text-sm text-[oklch(0.708_0_0)]">Video Display Ready</p>
      </div>
    </div>
  );
}