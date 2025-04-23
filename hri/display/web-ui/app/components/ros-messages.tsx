"use client";

import { useEffect, useState, useRef } from "react";
import { MessageCircle, Mic, Speaker, Star, VolumeX } from "lucide-react";

interface Message {
  type: "heard" | "spoken" | "keyword";
  content: string;
  timestamp: Date;
}

interface AudioState {
  state: "idle" | "listening" | "saying";
  vadLevel: number;
}

export default function RosMessagesDisplay() {
  const [messages, setMessages] = useState<Message[]>([]);
  const [connected, setConnected] = useState(false);
  const [audioState, setAudioState] = useState<AudioState>({
    state: "idle",
    vadLevel: 0,
  });
  const messagesStartRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const socket = new WebSocket("ws://localhost:8001/");

    socket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === "audioState" || data.type === "vad") {
        handleMic(data.type, data.data);
      } else {
        addMessage(data.type, data.data);
      }
    };

    socket.onopen = () => {
      console.log("WebSocket connected");
      setConnected(true);
    };

    socket.onclose = () => {
      console.log("WebSocket disconnected");
      setConnected(false);
    };

    socket.onerror = (error) => {
      console.error("WebSocket error:", error);
      setConnected(false);
    };

    return () => {
      socket.close();
    };
  }, []);

  const addMessage = (
    type: "heard" | "spoken" | "keyword",
    content: string
  ) => {
    let displayContent = content;

    if (type === "keyword") {
      const parts = content.split("'"); // Split by single quotes
      if (parts.length >= 4) {
        displayContent = parts[3]; // 3rd index = 4th quote-surrounded content
      } else {
        displayContent = "Unknown Keyword";
      }
    }

    setMessages((prev) => [
      { type, content: String(displayContent), timestamp: new Date() },
      ...prev,
    ]);
  };

  const handleMic = (type: string, content: string | number) => {
    if (type === "audioState") {
      setAudioState((prev) => ({
        ...prev,
        state: content as "idle" | "listening" | "saying",
      }));
    } else if (type === "vad") {
      // Assuming vad is a float between 0 and 1
      setAudioState((prev) => ({
        ...prev,
        vadLevel:
          typeof content === "number"
            ? content
            : Number.parseFloat(content as string),
      }));
    }
  };

  useEffect(() => {
    messagesStartRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  return (
    <div className="flex flex-col h-screen bg-[oklch(0.145_0_0)] text-[oklch(0.985_0_0)]">
      <div className="p-4 border-b border-[oklch(1_0_0/10%)] flex items-center justify-between">
        <h1 className="text-xl font-bold flex items-center">
          <MessageCircle className="mr-2 h-5 w-5" />
          ROS2 Messages
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

      <div className="flex-1 overflow-y-auto p-4 space-y-3">
        <div ref={messagesStartRef} />
        {messages.length === 0 ? (
          <div className="flex items-center justify-center h-full text-[oklch(0.708_0_0)]">
            <p>Waiting for messages...</p>
          </div>
        ) : (
          messages.map((msg, index) => (
            <div
              key={index}
              className={`
                p-3 rounded-lg animate-fadeIn transition-all duration-300
                ${
                  msg.type === "heard"
                    ? "bg-[oklch(0.488_0.243_264.376/20%)] border-l-4 border-l-[oklch(0.488_0.243_264.376)]"
                    : msg.type === "spoken"
                    ? "bg-[oklch(0.627_0.265_303.9/20%)] border-l-4 border-l-[oklch(0.627_0.265_303.9)]"
                    : "bg-[oklch(0.9_0.3_60/20%)] border-l-4 border-l-[oklch(0.9_0.3_60)]"
                }
                ${
                  index === 0
                    ? "ring-2 ring-offset-2 ring-offset-[oklch(0.145_0_0)] ring-[oklch(0.985_0_0/10%)]"
                    : ""
                }
              `}
            >
              <div className="flex items-start gap-2">
                {msg.type === "heard" ? (
                  <Mic className="h-5 w-5 text-[oklch(0.488_0.243_264.376)] mt-0.5 flex-shrink-0" />
                ) : msg.type === "spoken" ? (
                  <Speaker className="h-5 w-5 text-[oklch(0.627_0.265_303.9)] mt-0.5 flex-shrink-0" />
                ) : (
                  <Star className="h-5 w-5 text-[oklch(0.9_0.3_60)] mt-0.5 flex-shrink-0" />
                )}
                <div className="flex-1 min-w-0">
                  <p className="font-medium text-sm text-[oklch(0.708_0_0)]">
                    {msg.type === "heard"
                      ? "Heard"
                      : msg.type === "spoken"
                      ? "Spoken"
                      : "Keyword"}
                  </p>
                  <p className="text-lg font-bold break-words">{msg.content}</p>
                  <p className="text-xs text-[oklch(0.708_0_0)] mt-1">
                    {msg.timestamp.toLocaleTimeString([], {
                      hour: "2-digit",
                      minute: "2-digit",
                      second: "2-digit",
                    })}
                  </p>
                </div>
              </div>
            </div>
          ))
        )}
      </div>

      <div className="p-3 border-t border-[oklch(1_0_0/10%)] bg-[oklch(0.205_0_0/50%)] text-center">
        <p className="text-sm text-[oklch(0.708_0_0)]">
          {messages.length > 0
            ? `${messages.length} message${
                messages.length === 1 ? "" : "s"
              } received`
            : "No messages received"}
        </p>
      </div>
    </div>
  );
}

interface AudioStateIndicatorProps {
  state: AudioState;
}

function AudioStateIndicator({ state }: AudioStateIndicatorProps) {
  const { state: audioState, vadLevel } = state;

  // Calculate the number of bars to show based on VAD level (0-1)
  const maxBars = 5;
  const activeBars = Math.ceil(vadLevel * maxBars);

  if (audioState === "idle") {
    return (
      <div className="flex items-center gap-2 px-3 py-1.5 rounded-full bg-[oklch(0.3_0_0/30%)]">
        <VolumeX className="h-4 w-4 text-[oklch(0.708_0_0)]" />
        <span className="text-xs font-medium text-[oklch(0.708_0_0)]">
          Idle
        </span>
      </div>
    );
  }

  if (audioState === "saying") {
    return (
      <div className="flex items-center gap-2 px-3 py-1.5 rounded-full bg-[oklch(0.627_0.265_303.9/20%)]">
        <Speaker className="h-4 w-4 text-[oklch(0.627_0.265_303.9)] animate-pulse" />
        <span className="text-xs font-medium text-[oklch(0.627_0.265_303.9)]">
          Speaking
        </span>
      </div>
    );
  }

  // For listening state, show the mic with audio level bars
  return (
    <div className="flex items-center gap-2 px-3 py-1.5 rounded-full bg-[oklch(0.488_0.243_264.376/20%)]">
      <Mic className="h-4 w-4 text-[oklch(0.488_0.243_264.376)]" />
      <div className="flex items-end h-4 gap-0.5">
        {Array.from({ length: maxBars }).map((_, i) => (
          <div
            key={i}
            className={`w-1 rounded-sm transition-all duration-150 ${
              i < activeBars
                ? "bg-[oklch(0.488_0.243_264.376)]"
                : "bg-[oklch(0.488_0.243_264.376/30%)]"
            }`}
            style={{
              height: `${Math.max(30, (i + 1) * 20)}%`,
            }}
          />
        ))}
      </div>
      <span className="text-xs font-medium text-[oklch(0.488_0.243_264.376)]">
        Listening
      </span>
    </div>
  );
}
