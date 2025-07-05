"use client";

import { useEffect, useState, useRef } from "react";
import { MessageCircle, Mic, Speaker, Star, VolumeX } from "lucide-react";
import dynamic from "next/dynamic";

const MjpegStream = dynamic(() => import("./video"), { ssr: false });

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
  const [currentMessage, setCurrentMessage] = useState<Message | null>(null);
  const [connected, setConnected] = useState(false);
  const [audioState, setAudioState] = useState<AudioState>({
    state: "idle",
    vadLevel: 0,
  });
  const audioStateRef = useRef(audioState);
  useEffect(() => {
    audioStateRef.current = audioState;
  }, [audioState]);

  
  const [audioTopic, setAudioTopic] = useState<string>(
    "/zed/zed_node/rgb/image_rect_color"
  );
  const messagesStartRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const socket = new WebSocket("ws://localhost:8001/");

    socket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === "audioState" || data.type === "vad") {
        handleMic(data.type, data.data);
      } else if (data.type === "changeVideo") {
        setAudioTopic(data.data);
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
      try {
        // Replace single quotes with double quotes to make valid JSON
        const jsonString = content.replace(/'/g, '"');
        const parsedContent = JSON.parse(jsonString);

        if (parsedContent.score !== -1) {
          displayContent = parsedContent.keyword;
        } else {
          return;
        }
      } catch (error) {
        console.error("Error parsing keyword content:", error);
      }
    }
    const timestamp = new Date();

  if (type === "heard") {
    console.log("Heard message received ", audioStateRef.current.state);
    if (audioStateRef.current.state === "listening") {
      setCurrentMessage({ type, content: displayContent, timestamp });
    } else {
      setMessages((prev) => [
        { type, content: displayContent, timestamp },
      ...prev,
    ]);
  }
} else {
  setMessages((prev) => [
    {
      type,
        content: String(displayContent),
        timestamp,
      },
      ...prev,
    ]);
  }
};

 const handleMic = (type: string, content: string | number) => {
  console.log("handleMic received:", type, content);
  if (type === "audioState") {
    const newState = content as "idle" | "listening" | "saying";
    setAudioState(prev => ({ ...prev, state: newState }));
  } else if (type === "vad") {
    setAudioState(prev => ({
      ...prev,
      vadLevel: typeof content === "number" ? content : Number.parseFloat(content as string),
    }));
  }
};
useEffect(() => {
  if (audioState.state === "idle" && currentMessage) {
    setMessages(prevMsgs => [currentMessage, ...prevMsgs]);
    setCurrentMessage(null);
  }
}, [audioState.state, currentMessage]);


  useEffect(() => {
    messagesStartRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  return (
    <div className="flex flex-col h-screen bg-[oklch(0.145_0_0)] text-[oklch(0.985_0_0)] overflow-y-hidden">
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
      {audioState.state === "listening" && currentMessage && (
        <div
          key={currentMessage.timestamp.getTime()}
          className="fixed bottom-32 left-1/2 transform -translate-x-1/2 bg-[oklch(0.5_0.25_260)] text-white px-6 py-3 rounded-lg shadow-lg z-50 min-w-[300px] text-center animate-fadeIn"
        >
          <div className="flex items-center justify-center gap-4">
            <Mic className="h-8 w-8 animate-pulse text-white" />
            <p className="text-2x1 font-bold tracking-wide">{currentMessage.content}</p>
          </div>
        </div>
      )}
      <div className="grid grid-cols-2 h-full overflow-y-hidden">
        {/* Left column */}
        <div className="flex flex-col p-4 space-y-3 overflow-y-auto">
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
                    <p className="text-lg font-bold break-words">
                      {msg.content}
                    </p>
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
        {/* Right column */}
        <div className="sticky top-0 self-start h-[inherit] border-l border-[oklch(1_0_0/10%)] bg-[oklch(0.145_0_0)]">
          <div className="h-full flex flex-col items-center justify-center p-4">
            <p className="text-xl mb-4">Video feed at {audioTopic}</p>
            <MjpegStream
              streamUrl={`http://localhost:8080/stream?topic=${audioTopic}`}
            />
          </div>
        </div>
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
  const maxBars = 20;
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

  // For listening state, show the mic centered
  return (
  <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/10 pointer-events-none">
    <div className="relative">
      <div className="absolute inset-0 rounded-full bg-[oklch(0.5_0.25_260)] opacity-10 animate-[pulse_3s_infinite] scale-110" />
      <div className="absolute inset-0 rounded-full bg-[oklch(0.5_0.25_260)] opacity-15 animate-[pulse_3s_infinite_1s]scale-125" />
      <div className="absolute inset-0 rounded-full bg-[oklch(0.5_0.25_260)] opacity-20 animate-[pulse_3s_infinite_2s] scale-150" />
      
      <div className="relative z-10 h-24 w-24 rounded-full bg-[oklch(0.5_0.25_260)] shadow-lg flex items-center justify-center">
        <Mic className="h-24 w-24 text-white/90 drop-shadow-md" />
      </div>

      <div 
        className="absolute inset-0 rounded-full border-4 border-[oklch(0.5_0.25_260)] opacity-0 transition-all duration-300"
        style={{
          transform: `scale(${1.2 + (vadLevel || 0)})`,
          opacity: (vadLevel || 0) * 0.8
        }}
      />
      
    </div>
  </div>   
  );
}