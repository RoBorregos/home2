"use client";

import { useEffect, useState, useRef } from "react";
import { MessageCircle, Mic, Speaker, Star, VolumeX } from "lucide-react";
import dynamic from "next/dynamic";

const MjpegStream = dynamic(() => import("./video"), { ssr: false });

interface Message {
  type: "heard" | "spoken" | "keyword" | "answer" | "user_message" | "text_spoken";
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
  const [question, setQuestion] = useState<string | null>(null);
  const [answer, setAnswer] = useState<string>("");
  const [modalOpen, setModalOpen] = useState(false);
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
  
  const socketRef = useRef<WebSocket | null>(null);
  useEffect(() => {
    const socket = new WebSocket("ws://localhost:8001/");
    socketRef.current = socket;
    socket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if(data.type === "answer")return;
      if (data.type === "audioState" || data.type === "vad") {
        handleMic(data.type, data.data);
      } else if (data.type === "changeVideo") {
        setAudioTopic(data.data);
      } else if (data.type === "question") {
        setModalOpen(true);
        setQuestion(data.data);
        addMessage("spoken", data.data);

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
    type: "heard" | "spoken" | "keyword" | "answer" | "user_message",
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
    const newMessage: Message = { type, content: displayContent, timestamp };

  if (type === "heard") {
    console.log("Heard message received ", audioStateRef.current.state);
    if (audioStateRef.current.state === "listening") {
      setCurrentMessage(newMessage);
    } else {
      setMessages((prev) => [newMessage, ...prev]);
  }
} else {
  setMessages(prev => [newMessage, ...prev]);
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

const sendAnswer = () => {
  if (!answer.trim()) return;
  addMessage("answer", answer);

  // 2. Send to WebSocket server
  if (socketRef.current?.readyState === WebSocket.OPEN) {
    socketRef.current.send(
      JSON.stringify({ 
        type: "answer", 
        answer: answer,
        isLocal: true 
      })
    );
  }

  // 3. Clear the state
  setModalOpen(false);
  setAnswer("");
};

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
    <>
      <div
        key={currentMessage.timestamp.getTime()}
        className="fixed bottom-32 inset-x-0 mx-auto bg-gray-500/20 text-white py-6 rounded-lg shadow-lg z-50 animate-fadeIn w-fit max-w-[90vw]"
      >
        <div className="flex items-center justify-center gap-4">
          <Mic className="h-20 w-20 animate-pulse text-white" />
          <p className="text-7xl font-medium tracking-wide">{currentMessage.content}</p>
        </div>
      </div>
    </>
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
      msg.type === "spoken" || msg.type === "text_spoken"
        ? "bg-[oklch(0.627_0.265_303.9/20%)] border-l-4 border-l-[oklch(0.627_0.265_303.9)]"
        : msg.type === "answer"
        ? "bg-[oklch(0.4_0.3_120/20%)] border-l-4 border-l-[oklch(0.4_0.3_120)]"
        : msg.type === "heard"
        ? "bg-[oklch(0.488_0.243_264.376/20%)] border-l-4 border-l-[oklch(0.488_0.243_264.376)]"
        : "bg-[oklch(0.9_0.3_60/20%)] border-l-4 border-l-[oklch(0.9_0.3_60)]"
    }
    ${index === 0 ? "ring-2 ring-offset-2 ring-offset-[oklch(0.145_0_0)] ring-[oklch(0.985_0_0/10%)]" : ""}
  `}
>
  <div className="flex items-start gap-2">
    {msg.type === "spoken" || msg.type === "text_spoken" ? (
      <Speaker className="h-5 w-5 text-[oklch(0.627_0.265_303.9)] mt-0.5 flex-shrink-0" />
    ) : msg.type === "answer" ? (
      <MessageCircle className="h-5 w-5 text-[oklch(0.4_0.3_120)] mt-0.5 flex-shrink-0" />
    ) : msg.type === "heard" ? (
      <Mic className="h-5 w-5 text-[oklch(0.488_0.243_264.376)] mt-0.5 flex-shrink-0" />
    ) : (
      <Star className="h-5 w-5 text-[oklch(0.9_0.3_60)] mt-0.5 flex-shrink-0" />
    )}
    <div className="flex-1 min-w-0">
      <p className="font-medium text-sm text-[oklch(0.708_0_0)]">
        {msg.type === "spoken" || msg.type === "text_spoken"
          ? "Spoken"
          : msg.type === "answer"
          ? "Answer"
          : msg.type === "heard"
          ? "Heard"
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
      {modalOpen && question && (
        <div className="fixed inset-0 z-70 flex items-center justify-center bg-black/50">
          <div className="bg-white rounded-lg p-6 w-11/12 max-w-md"
            onClick={(e) => e.stopPropagation()} // Prevent click from closing modal
          >
            <h2 className="text-xl font-bold mb-4 text-black">Question</h2>
            <p className="mb-4 text-black">{question}</p>
            <textarea
              value={answer}
              onChange={(e) => setAnswer(e.target.value)}
              onKeyDown={(e) => {
              
                if (e.key === "Enter" && e.shiftKey) {
                  // Allow new lines with  // 1. Create and add the message locally
  const newMessage: Message = {
    type: "answer",
    content: answer,
    timestamp: new Date()
  };
  setMessages(prev => [newMessage, ...prev]);
                  return;
                }
                if (e.key === "Enter") {
                  e.preventDefault(); // Prevent default Enter behavior
                  sendAnswer(); // Send answer on Enter
                }
              }}
              className="w-full h-24 p-2 border border-gray-300 rounded mb-4 text-gray-500"
              placeholder="Type your answer here... "
            />
            <button
              onClick={sendAnswer}
              className="w-full bg-blue-500 text-white py-2 rounded hover:bg-blue-600 transition-colors"
            >
              Send Answer
            </button>
            <button
              onClick={() => {
                setModalOpen(false);
                setQuestion(null);
                setAnswer("");
              }}
              className="mt-3 w-full bg-gray-300 text-gray-800 py-2 rounded hover:bg-gray-400 transition-colors"
            >
              Close
            </button>
          </div>
        </div>
      )}

    </div>
  );
}

interface AudioStateIndicatorProps {
  state: AudioState;
}

function AudioStateIndicator({ state }: AudioStateIndicatorProps) {
  const { state: audioState, vadLevel } = state;

  // Calculate the number of bars to show based on VAD level (0-1)
  // const maxBars = 20;
  // const activeBars = Math.ceil(vadLevel * maxBars);

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
  <div className="fixed inset-0 z-50 flex items-start justify-center bg-black/10 pointer-events-none pt-35 ">
    <div className="relative">
      <div className="absolute inset-0 rounded-full bg-[oklch(0.5_0.25_260)] opacity-10 animate-[pulse_3s_infinite] scale-110" />
      <div className="absolute inset-0 rounded-full bg-[oklch(0.5_0.25_260)] opacity-15 animate-[pulse_3s_infinite_1s] scale-125" />
      <div className="absolute inset-0 rounded-full bg-[oklch(0.5_0.25_260)] opacity-20 animate-[pulse_3s_infinite_2s] scale-150" />

      <div className="relative z-10 h-70 w-70 rounded-full bg-[oklch(0.5_0.25_260)] shadow-lg flex items-center justify-center">
        <Mic className="h-50 w-50 text-white/90 drop-shadow-md" />
      </div>

      <div 
        className="absolute inset-0 rounded-full border-4 border-[oklch(0.5_0.25_260)] opacity-0 transition-all duration-300"
        style={{
          transform: `scale(${1 + (vadLevel || 0)})`,
          opacity: (vadLevel || 0) * 0.8
        }}
      />
      
    </div>
  </div>   
  );
}