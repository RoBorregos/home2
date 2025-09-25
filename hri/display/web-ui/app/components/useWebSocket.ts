"use client";

import { useEffect, useRef, useState } from "react";
import { Message } from "./types";

interface UseWebSocketProps {
  onAddMessage: (type: Message["type"], content: string) => void;
  onAudioStateChange: (type: string, content: string | number) => void;
  onVideoTopicChange: (topic: string) => void;
  onQuestionReceived: (question: string) => void;
  onMapReceived: (data: unknown) => void;
}

export function useWebSocket({
  onAddMessage,
  onAudioStateChange,
  onVideoTopicChange,
  onQuestionReceived,
  onMapReceived,
}: UseWebSocketProps) {
  const [connected, setConnected] = useState(false);
  const socketRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    const socket = new WebSocket("ws://localhost:8001/");
    socketRef.current = socket;

    socket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      
      if (data.type === "answer") return;
      
      if (data.type === "audioState" || data.type === "vad") {
        onAudioStateChange(data.type, data.data);
      } else if (data.type === "changeVideo") {
        onVideoTopicChange(data.data);
      } else if (data.type === "question") {
        onQuestionReceived(data.data);
      } else if (data.type === "map") {
        onMapReceived(data.data);
      } else {
        onAddMessage(data.type, data.data);
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
  }, [onAddMessage, onAudioStateChange, onVideoTopicChange, onQuestionReceived, onMapReceived]);

  const sendMessage = (message: object) => {
    if (socketRef.current?.readyState === WebSocket.OPEN) {
      socketRef.current.send(JSON.stringify(message));
    }
  };

  return { connected, sendMessage };
}
