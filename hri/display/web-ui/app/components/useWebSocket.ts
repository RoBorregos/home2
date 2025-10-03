"use client";

import { useEffect, useRef, useState } from "react";
import { Message } from "../types";

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
  
  // Use refs to store the latest callbacks
  const handlersRef = useRef({
    onAddMessage,
    onAudioStateChange,
    onVideoTopicChange,
    onQuestionReceived,
    onMapReceived,
  });

  // Update refs when callbacks change
  useEffect(() => {
    handlersRef.current = {
      onAddMessage,
      onAudioStateChange,
      onVideoTopicChange,
      onQuestionReceived,
      onMapReceived,
    };
  });

  useEffect(() => {
    const socket = new WebSocket("ws://localhost:8001/");
    socketRef.current = socket;

    const handleMessage = (event: MessageEvent) => {
      const data = JSON.parse(event.data);
      
      if (data.type === "answer") return;
      
      if (data.type === "audioState" || data.type === "vad") {
        handlersRef.current.onAudioStateChange(data.type, data.data);
      } else if (data.type === "changeVideo") {
        handlersRef.current.onVideoTopicChange(data.data);
      } else if (data.type === "question") {
        handlersRef.current.onQuestionReceived(data.data);
      } else if (data.type === "map") {
        handlersRef.current.onMapReceived(data.data);
      } else {
        handlersRef.current.onAddMessage(data.type, data.data);
      }
    };

    const handleOpen = () => {
      console.log("WebSocket connected");
      setConnected(true);
    };

    const handleClose = () => {
      console.log("WebSocket disconnected");
      setConnected(false);
    };

    const handleError = (error: Event) => {
      console.error("WebSocket error:", error);
      setConnected(false);
    };

    socket.onmessage = handleMessage;
    socket.onopen = handleOpen;
    socket.onclose = handleClose;
    socket.onerror = handleError;

    return () => {
      socket.close();
    };
  }, []); // Dependencias vacías - se conecta una vez al montar

  const sendMessage = (message: object) => {
    if (socketRef.current?.readyState === WebSocket.OPEN) {
      socketRef.current.send(JSON.stringify(message));
    }
  };

  return { connected, sendMessage };
}
